
function [C] = get_c(traj,phi_fun, y_reg)

    

    method = 'training_data';

    switch method
        case 'grid'
            
            [X,Y] = meshgrid(-0.5:0.2:0.5,-0.5:0.2:0.5);
            phi_grid = zeros(length(phi_fun),size(X,1)*size(X,2));
            yout = y_reg([0;0]);
            y_reg_grid = zeros(length(yout),size(X,1)*size(X,2));

            x_flat = reshape(X,[1,size(X,1)*size(X,2)]);
            y_flat = reshape(Y,[1,size(X,1)*size(X,2)]);
            for k=1:length(phi_fun)    
                phi_grid(k,:) = phi_fun{k}([x_flat;y_flat]);
            end

            % Evaluate y_reg over the grid
            ik=0;
            for i=1:size(X,1)
                for j=1:size(X,2)
                    ik = ik+1;
                    y_reg_grid(:,ik) = y_reg([X(i,j),Y(i,j)]);
                end
            end
        case 'training_data'
            % Get sizes
            Ns = size(traj,1); % number of states
            Ntraj = size(traj,2); % number of discontinous trajectories
            Nt = size(traj,3); % number of timesteps per trajectory 
            
            traj_flat = reshape(traj,[Ns,Nt*Ntraj]);
            
            %for i=1:size(X,1)
            
    end
    
    C = y_reg_grid/phi_grid;

end