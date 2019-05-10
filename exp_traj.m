clear all, close all
t = 0:0.01:3;
afigure
plot(t,t.*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x-x_0, x_f=1')
title('x=x_0+(x_f-x_0)\lambda e^{1-\lambda}')


afigure
plot(t,exp(1-t)-t.*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x dot, x_f=1')
title('x dot=(x_f-x_0)(e^{1-\lambda}-\lambda e^{1-\lambda})')

afigure
plot(t,t.*exp(1-t)-2*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x dot dot, x_f=1')
title('x dot dot=\lambda dot (x_f-x_0)(\lambda e^{1-\lambda}-2*e^{1-\lambda})')


%% Smooth Step

clear all, close all
t = 0:0.01:1;
afigure
plot(t,6*t.^5-15*t.^4+10*t.^3)
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x-x_0, x_f=1')
title('6*t.^5-15*t.^4+10*t.^3')
saveas(gcf,'smooth_setp.png')

afigure
plot(t,6*5*t.^4-15*4*t.^3+10*3*t.^2)
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x dot, x_f=1')
title('6*5*t.^4-15*4*t.^3+10*3*t.^2')
saveas(gcf,'smooth_setp_dt.png')

afigure
plot(t,6*5*4*t.^3-15*4*3*t.^2+10*3*2*t)
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x dot dot, x_f=1')
title('6*5*4*t.^3-15*4*3*t.^2+10*3*2*t2')
saveas(gcf,'smooth_setp_dtt.png')