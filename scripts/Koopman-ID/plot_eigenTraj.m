function [] = plot_eigenTraj(phi_grid,traj_flat,plotType,indexPlot)

    %phi_grid = zeros(Nphi,Nt*Ntraj);
    %traj_flat = zeros(Ns,Nt*Ntraj);
    
    if(~exist('indexPlot','var') || isempty(indexPlot))
        indexPlot = 1;
    end
    %nPlot = 1; %length(phi_grid) 
    cmap = hsv(length(indexPlot));
    
    
    switch plotType 
        case 'scatter3'
            
            afigure
            hold all 
            ki = 1;
            for k=indexPlot                
                s(k) = scatter3(traj_flat(1,:),traj_flat(2,:),real(phi_grid(k,:)),5,imag(phi_grid(k,:)));
                alpha(s(k),0.3)
                ki = ki +1;
            end
            title(['Eigenvalue ',num2str(indexPlot)])
            xlabel('x')
            ylabel('y')
        case 'complex'
            afigure
            hold all 
            for k=indexPlot 
                s(k) = plot(phi_grid(k,:),'.','color',cmap(k,:));
                alpha(s(k),0.3)
            end
            axis equal
            xlabel('Real')
            ylabel('Imaginary')
    end
            
            

end