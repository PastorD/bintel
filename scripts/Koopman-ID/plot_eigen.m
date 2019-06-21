function [] = plot_eigen(phi_fun)

% Evaluate the Eigenfunction

%x = -1:0.05:1;

[X,Y] = meshgrid(-1:0.1:1,-1:0.1:1);
Z = zeros(size(X));

cmap = hsv(length(phi_fun));

afigure
hold all 
for k=1:length(phi_fun)    
    for i=1:size(X,1)
        for j=1:size(X,2)
            Z(i,j) = phi_fun{12}([X(i,j),Y(i,j)]);
        end
    end
    surf(X,Y,Z)
    alpha 0.3
end

xlabel('x')
ylabel('y')

end