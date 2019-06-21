
function [C] = get_c(traj,phi_fun, y_reg)

    
    [X,Y] = meshgrid(-0.5:0.1:0.5,-0.5:0.1:0.5);
    phi_grid = zeros(length(phi_fun),size(X,1)*size(X,2));
    y_reg_grid = zeros(1,size(X,1)*size(X,2));
    
    % Evaluate phi over the grid
    for k=1:length(phi_fun)    
        ik=0;
        for i=1:size(X,1)
            for j=1:size(X,2)
                ik = ik+1;
                phi_grid(k,ik) = phi_fun{12}([X(i,j),Y(i,j)]);
            end
        end
    end

    % Evaluate y_reg over the grid
    ik=0;
    for i=1:size(X,1)
        for j=1:size(X,2)
            ik = ik+1;
            y_reg_grid(ik) = y_reg([X(i,j),Y(i,j)]);
        end
    end
    
    C = y_reg_grid/phi_grid;

end