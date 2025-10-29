clc
clear

syms t1 t2 t3 t4 t5 t6 l1 l2 l3 l4

l1 = 183;
l2 = 210;
l3 = 30;
l4 = 221.5;


dhT= [0 pi/2 l1 t1;
      l2 0 0 t2+pi/2;
      l3 pi/2 0 t3;
      0 0 l4 0];
       
make_readable_transform(simplify(dkm(dhT)))