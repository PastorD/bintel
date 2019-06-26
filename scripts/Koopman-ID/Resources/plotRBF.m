
clear all
x = -1:0.01:1;
%x = x'
C = zeros(size(x));

afigure
xlabel('x')
ylabel('RBF')
hold on

subplot(2,2,1)
plot(x,rbf( x,C, 'thinplate'))

subplot(2,2,2)
plot(x,rbf( x,C,'gauss'))

subplot(2,2,3)
plot(x,rbf( x,C, 'invquad'))

subplot(2,2,4)
plot(x,rbf( x,C, 'invmultquad'))
%legend('thinplate','gauss','invquad','invmultquad')
