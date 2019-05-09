t = 0:0.01:1;
afigure
plot(t,t.*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x-x_0, x_f=1')
title('x=x_0+(x_f-x_0)\lambda e^{1-\lambda}')


t = 0:0.01:1;
afigure
plot(t,exp(1-t)-t.*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x-x_0, x_f=1')
title('x dot=(x_f-x_0)(e^{1-\lambda}-\lambda e^{1-\lambda})')

t = 0:0.01:1;
afigure
plot(t,t.*exp(1-t)-2*exp(1-t))
xlabel('\lambda, ({t-t_0})/({t_f-t_0})')
ylabel('x-x_0, x_f=1')
title('x dot dot=\lambda dot (x_f-x_0)(2*\lambda e^{1-\lambda}-e^{1-\lambda})')