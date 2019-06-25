function phiVector =  phiFunction (phi_fun,x)

    phiVector = zeros(length(phi_fun),size(x,2));
    for i=1:length(phi_fun)
        phiVector(i,:) =  phi_fun{i}(x);
    end
end