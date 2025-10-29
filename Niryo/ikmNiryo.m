clc; close all; clear all;

%% ----------------------------
%% Robot & symbolic declarations
%% ----------------------------
syms t1 t2 t3 t4 t5 t6

% link lengths (mm)
l1 = 183;
l2 = 210;
l3 = 30;
l4 = 221.5;

% DH table template (Niryo)
dhT = [ 0   pi/2  l1   t1;
        l2  0     0    t2 + pi/2;
        l3  pi/2  0    t3;
        0   0     l4   0;
        0  -pi/2  0    t4;
        0   pi/2  0    t5;
        0   0     0    t6;
        0   pi/2  23.7 pi/2;
        0   0    -5.5  pi/2;
        0   0     0    0 ];

% joint limits (degrees)
JointLimits = [ -175    175;    % t1
                -90     36.7;   % t2
                -80     90;     % t3
                -175    175;    % t4
                -100    110;    % t5
                -147.5  147.5]; % t6

% transform from w3 to End (fixed tail of DH chain)
Tw3E = [0  pi/2  23.7  pi/2;
        0   0   -5.5  pi/2;
        0   0    0     0];

%% ----------------------------
%% Input: use example or enter custom T
%% ----------------------------
disp('Input desired end-effector pose T.');
disp(' - Type "d" (or press Enter) to use example T from the original script.');
disp(' - Or enter 16 numbers (space or comma separated) in row-major order to specify a custom 4x4 transform.');

resp = input('Enter "d" or 16 numbers: ','s');

if isempty(resp) || strcmpi(strtrim(resp),'d')
    % Use example T from original script
    T = [0.8660    0.1294    0.4830   61.1698;
         0.1294   -0.9910    0.0335  -67.2961;
         0.4830    0.0335   -0.8750  626.9893;
         0         0         0      1.0000];
else
    % try to parse 16 numbers from the string
    nums = sscanf(resp, '%f');
    if numel(nums) == 16
        T = reshape(nums,4,4)'; % row-major to matrix
    else
        % if user entered nothing parseable, abort with helpful message
        error('Input must be either "d" or 16 numeric values (row-major).');
    end
end

%% ----------------------------
%% Compute wrist pose T0w0
%% ----------------------------
% Use previously defined dkm function (unchanged)
T0w0 = double(T * inv(dkm(dhT(8:10,:))));

disp('wrist pose:');
x = T0w0(1,4);  fprintf('x = %g\n', x);
y = T0w0(2,4);  fprintf('y = %g\n', y);
z = T0w0(3,4);  fprintf('z = %g\n', z);

%% ----------------------------
%% Solve for t1 (four possibilities via atan2 combinations)
%% ----------------------------
t1111 = [ atan2(y,x), atan2(-y,-x), atan2(-y,-x), atan2(+y,+x) ];

%% ----------------------------
%% Solve for t3 (up to 4 solutions computed below)
%% ----------------------------
t3333 = zeros(1,4);

% convenience terms used in algebra
a = x.*cos(t1111(1)) + y.*sin(t1111(2));
b = z - l1; % note l1 = 183

% discriminant-like term used in those formulas
term_inner = (a.^2 + b.^2 - l4^2 - l3^2 - l2^2);
temp = sqrt( (2*l3*l2)^2 + (2*l4*l2)^2 - term_inner.^2 );

% compute t3 candidate set (4 entries - duplicates later will be handled)
t3333(1) = atan2(term_inner, +temp) - atan2((2*l3*l2), (2*l4*l2));
t3333(2) = atan2(term_inner, -temp) - atan2((2*l3*l2), (2*l4*l2));
t3333(3) = atan2(term_inner, +temp) - atan2((2*l3*l2), (2*l4*l2));
t3333(4) = atan2(term_inner, -temp) - atan2((2*l3*l2), (2*l4*l2));

%% ----------------------------
%% Solve for t2 given each t1,t3 candidate (4 combos)
%% ----------------------------
t2222 = zeros(1,4);
for i = 1:4
    % build 2x2 linear system as in original code
    A = [ (l4*cos(t3333(i)) - l3*sin(t3333(i))),  (-l3*cos(t3333(i)) - l4*sin(t3333(i)) - l2);
          (l2 + l3*cos(t3333(i)) + l4*sin(t3333(i))), (l4*cos(t3333(i)) - l3*sin(t3333(i))) ];
    rhs = [ x*cos(t1111(i)) + y*sin(t1111(i)); z - l1 ];
    tempVec = A \ rhs;
    t2222(i) = atan2(tempVec(2), tempVec(1));
end

%% ----------------------------
%% Duplicate the first 4 solutions to make room for two wrist orientations each
%% (matching original script logic)
%% ----------------------------
t1111 = [t1111 t1111];
t2222 = [t2222 t2222];
t3333 = [t3333 t3333];

t4444 = zeros(1,8);
t5555 = zeros(1,8);
t6666 = zeros(1,8);

%% ----------------------------
%% For each of the 4 base-elbow solutions compute the wrist angles (t4,t5,t6)
%% using Tw0w3 and Tw3E as in original code
%% ----------------------------
for i = 1:4
    % plug in current t1,t2,t3 into the first 4 rows of dhT and compute Tw0w3
    t1 = t1111(i);
    t2 = t2222(i);
    t3 = t3333(i);
    
    % evaluate numeric DH for first 4 joints and compute Tw0w3
    Tw0w3 = inv(dkm(double(subs(dhT(1:4, :))))) * T * inv(dkm(Tw3E));
    
    % compute t5 two possibilities (+/- acos)
    t5555(i)   = acos( Tw0w3(3,3) );
    t5555(i+4) = -acos( Tw0w3(3,3) );
    
    % compute t6 and t4 according to original relationships (careful with sign usage)
    % note: indexing matches the original script: some lines use i and i+4 swapped
    t6666(i+4) = atan2( Tw0w3(3,2) / -sin(t5555(i)), Tw0w3(3,1) / sin(t5555(i)) );
    t6666(i)   = atan2( Tw0w3(3,2) / -sin(t5555(i+4)), Tw0w3(3,1) / sin(t5555(i+4)) );
    
    t4444(i+4) = atan2( Tw0w3(2,3) / -sin(t5555(i)), Tw0w3(1,3) / -sin(t5555(i)) );
    t4444(i)   = atan2( Tw0w3(2,3) / -sin(t5555(i+4)), Tw0w3(1,3) / -sin(t5555(i+4)) );
end

%% ----------------------------
%% Filter solutions according to joint limits (converted to degrees)
%% ----------------------------
t11 = []; t22 = []; t33 = [];
t44 = []; t55 = []; t66 = [];

for i = 1:8
    if rad2deg(t1111(i)) > JointLimits(1,1) && rad2deg(t1111(i)) < JointLimits(1,2) && ...
       rad2deg(t2222(i)) > JointLimits(2,1) && rad2deg(t2222(i)) < JointLimits(2,2) && ...
       rad2deg(t3333(i)) > JointLimits(3,1) && rad2deg(t3333(i)) < JointLimits(3,2) && ...
       rad2deg(t4444(i)) > JointLimits(4,1) && rad2deg(t4444(i)) < JointLimits(4,2) && ...
       rad2deg(t5555(i)) > JointLimits(5,1) && rad2deg(t5555(i)) < JointLimits(5,2) && ...
       rad2deg(t6666(i)) > JointLimits(6,1) && rad2deg(t6666(i)) < JointLimits(6,2)
   
        t11 = [t11; t1111(i)];
        t22 = [t22; t2222(i)];
        t33 = [t33; t3333(i)];
        t44 = [t44; t4444(i)];
        t55 = [t55; t5555(i)];
        t66 = [t66; t6666(i)];
    end
end

%% ----------------------------
%% Display valid solutions (in degrees)
%% ----------------------------
disp(' ');
disp('solution is:');
disp('t1 (deg):'); disp(rad2deg(t11));
disp('t2 (deg):'); disp(rad2deg(t22));
disp('t3 (deg):'); disp(rad2deg(t33));
disp('t4 (deg):'); disp(rad2deg(t44));
disp('t5 (deg):'); disp(rad2deg(t55));
disp('t6 (deg):'); disp(rad2deg(t66));
