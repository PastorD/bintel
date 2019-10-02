
clear all
x = -2:0.01:2;
C = 0;

afigure

hold on
plot(x,rbf( x,C, 'thinplate'))
plot(x,rbf( x,C,'gauss'))
plot(x,rbf( x,C, 'invquad'))
plot(x,rbf( x,C, 'invmultquad'))
xlabel('x')
ylabel('RBF')
legend('thinplate','gauss','invquad','invmultquad')



 [X,Y] = meshgrid(-1:0.1:1,-1:0.1:1);
 x_flat = reshape(X,[1,size(X,1)*size(X,2)]);
 y_flat = reshape(Y,[1,size(X,1)*size(X,2)]);
 xy_flat = [x_flat;y_flat];
 
 rbf_flat = rbf( xy_flat,[0;0], 'invmultquad');
 rbf_pack = reshape(rbf_flat,[size(X,1),size(X,2)]);
 
 afigure
 surf(X,Y,rbf_pack)
 xlabel('x')
 ylabel('y')
  
