function [X_norm, X_mean, X_std] = normalize_data(X)
    
    %Make sure each row corresponds to a time step:
    if size(X,1)>size(X,2)
        X = X';
    end
    
    X_mean = mean(X,2);
    X_std = std(X,0,2);
    X_norm = (X-X_mean)./X_std;
end