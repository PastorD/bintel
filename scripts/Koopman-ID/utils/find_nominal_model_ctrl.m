function [A_nom, B_nom, C_nom, K_nom] = find_nominal_model_ctrl(n, m, f_u, Q, R, x0)
    %Linearize around origin and design linear feedback control law using LQR
    
    %Inputs:
    %   f_u - System dynamics (nonlinear)
    %   Q   - State penalty matrix for LQR
    %   R   - Actuation penalty matrix for LQR
    
    %Outputs:
    %   A_nom - Linearized state space model passive dynamics matrix
    %   B_nom - Linearized state space model actuation matrix
    %   K_nom - Linear feedback gain matrix
    
    
    x = sym('x',[n;1]); u = sym('u',[m;1]);
    A_nom = double(subs(jacobian(f_u(0,x,u),x),[x;u],[x0;0]));
    B_nom = double(subs(jacobian(f_u(0,x,u),u),[x;u],[x0;0]));
    C_nom = eye(n);
    K_nom = -lqr(A_nom,B_nom,Q,R);
end