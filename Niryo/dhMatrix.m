function [ T ] = dhMatrix( a,alpha,d,theta )
    T(1,:) = [cos(theta) -cos(alpha)*sin(theta) sin(theta)*sin(alpha) a*cos(theta)];
    T(2,:) = [sin(theta) cos(alpha)*cos(theta) -cos(theta)*sin(alpha) a*sin(theta)];
    T(3,:) = [0 sin(alpha) cos(alpha) d];
    T(4,:) = [0 0 0 1];
end

