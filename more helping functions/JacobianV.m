function [ Jv ] = JacobianV( dhTable )

    n = size(dhTable, 1);
    J = zeros(6, n);    
    O = zeros(3, n+1);    
    Z = zeros(3, n+1);       
    
    % Base frame
    O(:,1) = [0 0 0]';
    Z(:,1) = [0 0 1]';
    
    T_accum = eye(4);
    for i = 1:n
        T_i = dkm(dhTable(i,:));
        T_accum = T_accum * T_i;
        
        Z(:,i+1) = T_accum(1:3,3);
        O(:,i+1) = T_accum(1:3,4);
    end
    
    % End effector position
    OE = O(:,end);
    
    % Compute Jacobian columns
    for j = 1:n
        Jv(1:3,j) = cross(Z(:,j), OE - O(:,j));
        Jv(4:6,j) = Z(:,j);
    end
    
end