function [ alpha,beta,gamma ] = rollPitchYawAngles( R )
    if abs(R(3,1)) == 1
        alpha = NaN;
        beta = NaN;
        gamma = NaN;
    else
       alpha = atan2(R(2,1),R(1,1));
       beta = asin(-R(3,1));
       gamma = atan2(R(3,2), R(3,3));
    end
end

