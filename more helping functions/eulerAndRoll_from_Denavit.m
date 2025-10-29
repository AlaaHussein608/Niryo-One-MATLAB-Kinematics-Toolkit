clc
clear

syms t1 t2 t3 d3
dhTable = [0 -pi/2 0 pi/3;
           0 pi/2 0 pi/4;
           0 0 200 pi/6];
       
DKM = dkm(dhTable);

Ex = DKM(1,4);
Ey = DKM(2,4);
Ez = DKM(3,4);

eulerAngles(DKM(1:3,1:3))
rollPitchYawAngles(DKM(1:3,1:3))