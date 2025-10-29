function [ T ] = dkm( dhTable )
    T = eye(4);
    for i = 1:size(dhTable,1)
            T = T*dhMatrix(dhTable(i,1),dhTable(i,2),dhTable(i,3),dhTable(i,4));
%             T = simplify(T);
    end 
end

