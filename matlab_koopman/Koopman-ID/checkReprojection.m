function [] = checkReprojection(phi_fun)
    X = 3 + sin(t); Y = 2 + cos(t); Z = X*0; 
h = patch(X,Y,Z,'g') 
axis([-10 10 -10 10]); 
% Then make it movable 
    moveit2(h); 
end

