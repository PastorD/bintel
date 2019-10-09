function [ O ] = obsvk( A, C, k )
O = [];
for i = 1:k
    O = [O;C*A^(i-1)]; % Inefficient
end

