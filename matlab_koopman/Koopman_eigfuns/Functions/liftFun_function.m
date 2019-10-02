function out = liftFun_function(x,phi)
out = zeros(numel(phi),size(x,2));
for i = 1:numel(phi)
    out(i,:) = phi{i}(x);
end
end

