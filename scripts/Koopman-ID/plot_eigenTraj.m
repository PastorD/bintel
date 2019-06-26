function [] = plot_eigenTraj(phi_grid,traj_flat,plotType)

    %phi_grid = zeros(Nphi,Nt*Ntraj);
    %traj_flat = zeros(Ns,Nt*Ntraj);
    
    
    nPlot = 1; %length(phi_grid) 
    cmap = hsv(nPlot);
    
    
    switch plotType 
        case 'scatter3'
            
            afigure
            hold all 
            for k=1:nPlot 
                s(k) = scatter3(traj_flat(1,:),traj_flat(2,:),real(phi_grid(k,:)),1,cmap(k,:));
                alpha(s(k),0.3)
            end

            xlabel('x')
            ylabel('y')
        case 'complex'
            afigure
            hold all 
            for k=1:nPlot 
                s(k) = plot(phi_grid(k,:),'.','color',cmap(k,:));
                alpha(s(k),0.3)
            end
            axis equal
            xlabel('Real')
            ylabel('Imaginary')
    end
            
            

end