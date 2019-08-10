function res = phifun_mat(phifun,X)
    [n,m] = size(X);
    res = [];
    for i = 1 : m
        res = [res phifun(X(:,i))];
    end 
end