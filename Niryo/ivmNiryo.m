clc;
clear;

%% ----------------------------
%% Symbolic declarations
%% ----------------------------
syms t1 t2 t3 t4 t5 t6 real;
syms DTH1 DTH2 DTH3 DTH4 DTH5 DTH6 real;

%% ----------------------------
%% User input for joint angles and end-effector velocities
%% ----------------------------
disp('Enter joint angles (in degrees):');
t1 = deg2rad(input('t1 = '));
t2 = deg2rad(input('t2 = '));
t3 = deg2rad(input('t3 = '));
t4 = deg2rad(input('t4 = '));
t5 = deg2rad(input('t5 = '));
t6 = deg2rad(input('t6 = '));

disp('Enter end-effector linear velocities (Vx, Vy, Vz in mm/s):');
Vx = input('Vx = ');
Vy = input('Vy = ');
Vz = input('Vz = ');

disp('Enter end-effector angular velocities (Wx, Wy, Wz in rad/s):');
Wx = input('Wx = ');
Wy = input('Wy = ');
Wz = input('Wz = ');

%% ----------------------------
%% DH Table definition (Niryo One)
%% ----------------------------
dhTable = [ 0   pi/2  183   t1;
            210 0     0     t2 + pi/2;
            30  pi/2  0     t3;
            0   0     221.5 0;
            0  -pi/2  0     t4;
            0   pi/2  0     t5;
            0   0     0     t6;
            0   pi/2  23.7  pi/2;
            0   0    -5.5   pi/2;
            0   0     0     0 ];

%% ----------------------------
%% Forward kinematics transformations
%% ----------------------------
T0_1 = dkm(dhTable(1,:));
T0_2 = T0_1 * dkm(dhTable(2,:));
T0_3 = T0_2 * dkm(dhTable(3:4,:));
T0_4 = T0_3 * dkm(dhTable(5,:));
T0_5 = T0_4 * dkm(dhTable(6,:));
T0_6 = T0_5 * dkm(dhTable(7:10,:));

%% ----------------------------
%% Compute Jacobian
%% ----------------------------
Z0 = [0; 0; 1];
O0 = [0; 0; 0];
Z1 = T0_1(1:3,3); O1 = T0_1(1:3,4);
Z2 = T0_2(1:3,3); O2 = T0_2(1:3,4);
Z3 = T0_3(1:3,3); O3 = T0_3(1:3,4);
Z4 = T0_4(1:3,3); O4 = T0_4(1:3,4);
Z5 = T0_5(1:3,3); O5 = T0_5(1:3,4);
Z6 = T0_6(1:3,3); OE = T0_6(1:3,4);

J1 = [cross(Z0, OE - O0); Z0];
J2 = [cross(Z1, OE - O1); Z1];
J3 = [cross(Z2, OE - O2); Z2];
J4 = [cross(Z3, OE - O3); Z3];
J5 = [cross(Z4, OE - O4); Z4];
J6 = [cross(Z5, OE - O5); Z5];

J = [J1 J2 J3 J4 J5 J6];

%% ----------------------------
%% Check for singularity
%% ----------------------------
if rank(J) < size(J,1)
    warning('?? Singularity detected: Jacobian is rank deficient.');
end

%% ----------------------------
%% Inverse velocity kinematics
%% ----------------------------
Velocities = [Vx; Vy; Vz; Wx; Wy; Wz];
J_inv = pinv(J); % pseudo-inverse of Jacobian
DTH_velocities = J_inv * Velocities;

%% ----------------------------
%% Display results
%% ----------------------------
disp(' ');
disp('==================== Results ====================');
disp('Joint Velocities (rad/s):');
disp(DTH_velocities);
disp('=================================================');
