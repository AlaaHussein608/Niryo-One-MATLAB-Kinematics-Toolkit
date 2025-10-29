clc;
clear;

% --- Symbolic joint variables ---
syms t1 t2 t3 t4 t5 t6

% --- Robot link parameters (in mm) ---
l1 = 183;
l2 = 210;
l3 = 30;
l4 = 221.5;

% --- DH Table Definition ---
dhT = [ 0   pi/2   l1   t1;
        l2  0      0    t2 + pi/2;
        l3  pi/2   0    t3;
        0   0      l4   0;
        0  -pi/2   0    t4;
        0   pi/2   0    t5;
        0   0      0    t6;
        0   pi/2   23.7 pi/2;
        0   0     -5.5  pi/2;
        0   0      0    0 ];

% --- User Input for Joint Angles (in radians) ---
disp('Enter joint angles in radians:');
theta1 = input('t1 = ');
theta2 = input('t2 = ');
theta3 = input('t3 = ');
theta4 = input('t4 = ');
theta5 = input('t5 = ');
theta6 = input('t6 = ');

% --- Substitute input angles into DH table ---
dh_table = subs(dhT, {t1, t2, t3, t4, t5, t6}, {theta1, theta2, theta3, theta4, theta5, theta6});

% --- Compute and display the Direct Kinematics Matrix ---
T = double(dkm(dh_table));

disp('Direct Kinematics Matrix:');
disp(T);
