function [ psi,theta,phi ] = eulerAngles( R )
    if abs(R(3,3)) == 1
        psi = NaN;
        theta = NaN;
        phi = NaN;
    else
       psi = atan2(R(1,3),-R(2,3));
       theta = acos(R(3,3));
       phi = atan2(R(3,1), R(3,2));
    end
end

