function f_ud = sim_timestep(deltaT, f_u, t, x, u)
    %Runge-Kutta 4 one step evolution of dynamics
    
    %Inputs:
    %   deltaT  - Length of time step
    %   f_u     - System dynamics (nonlinear)
    
    %Outputs:
    %   f_ud    - Discrete time model (state values at next time step)

    
    k1 = @(t,x,u) (  f_u(t,x,u) );
    k2 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT/2,u) );
    k3 = @(t,x,u) ( f_u(t,x + k2(t,x,u)*deltaT/2,u) );
    k4 = @(t,x,u) ( f_u(t,x + k1(t,x,u)*deltaT,u) );
    f_ud = ( x + (deltaT/6) * ( k1(t,x,u) + 2*k2(t,x,u) + 2*k3(t,x,u) + k4(t,x,u)  )   );
end