function diff = num_diff(X, deltaT)
    
    diff = zeros(size(X));
    for k = 1 : size(X,1)
        diff(k,:) = movingslope(X(k,:),4,2,deltaT);
    end
end