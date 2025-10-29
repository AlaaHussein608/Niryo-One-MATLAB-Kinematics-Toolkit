clc;
clear;

% --- Symbolic Declarations ---
syms t1 t2 t3 t4 t5 t6 real;
syms DTH1 DTH2 DTH3 DTH4 DTH5 DTH6 real;

% --- User Input ---
disp('Enter joint angles (in degrees):');
t1 = input('t1 = ');
t2 = input('t2 = ');
t3 = input('t3 = ');
t4 = input('t4 = ');
t5 = input('t5 = ');
t6 = input('t6 = ');

disp('Enter joint velocities (in rad/s):');
DTH1 = input('DTH1 = ');
DTH2 = input('DTH2 = ');
DTH3 = input('DTH3 = ');
DTH4 = input('DTH4 = ');
DTH5 = input('DTH5 = ');
DTH6 = input('DTH6 = ');

% --- DH Table Definition ---
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

% --- Forward Kinematics Transformations ---
T0_1 = dkm(dhTable(1,:));
T0_2 = T0_1 * dkm(dhTable(2,:));
T0_3 = T0_2 * dkm(dhTable(3:4,:));
T0_4 = T0_3 * dkm(dhTable(5,:));
T0_5 = T0_4 * dkm(dhTable(6,:));
T0_6 = T0_5 * dkm(dhTable(7:10,:));

% --- Extract Z and O Vectors ---
Z0 = [0; 0; 1];
O0 = [0; 0; 0];
Z1 = T0_1(1:3,3); O1 = T0_1(1:3,4);
Z2 = T0_2(1:3,3); O2 = T0_2(1:3,4);
Z3 = T0_3(1:3,3); O3 = T0_3(1:3,4);
Z4 = T0_4(1:3,3); O4 = T0_4(1:3,4);
Z5 = T0_5(1:3,3); O5 = T0_5(1:3,4);
Z6 = T0_6(1:3,3); OE = T0_6(1:3,4);

% --- Compute Jacobian Columns ---
J1 = [cross(Z0, OE - O0); Z0];
J2 = [cross(Z1, OE - O1); Z1];
J3 = [cross(Z2, OE - O2); Z2];
J4 = [cross(Z3, OE - O3); Z3];
J5 = [cross(Z4, OE - O4); Z4];
J6 = [cross(Z5, OE - O5); Z5];

% --- Assemble the Jacobian ---
J = [J1 J2 J3 J4 J5 J6];

% --- Joint Velocities Vector ---
DTH = [DTH1; DTH2; DTH3; DTH4; DTH5; DTH6];

% --- Compute End-Effector Velocity ---
V = J * DTH;

% --- Display Results ---
disp('Jacobian Matrix (J):');
disp(J);
disp('End-Effector Velocity (V):');
disp(V);
