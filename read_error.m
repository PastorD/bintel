filename = 'csv_data.txt';
fileraw = csvread(filename,0,0); %initial reading of all values
time = fileraw(:,1);
pos = fileraw(:,1:3);
pos_d = fileraw(:,4:6);
error = pos - pos_d;

afigure
plot3([pos(:,1),pos_d(:,1)],[pos(:,2),pos_d(:,2)],[pos(:,3),pos_d(:,3)])
legend('Estimated', 'Desired')
axis equal
saveas(gcf,'pos_3d.png')


afigure
subplot(3,1,1)
plot(time,pos(:,1),time,pos_d(:,1))
xlabel('Time(s)')
ylabel('Position X (m)')
legend('Estimated', 'Desired')

subplot(3,1,2)
plot(time,pos(:,2),time,pos_d(:,3))
xlabel('Time(s)')
ylabel('Position Y (m)')
legend('Estimated', 'Desired')

subplot(3,1,3)
plot(time,pos(:,3),time,pos_d(:,3))
xlabel('Time(s)')
ylabel('Position Z (m)')
legend('Estimated', 'Desired')
saveas(gcf,'pos.png')


afigure
plot(time,error)
xlabel('Time(s)')
ylabel('Error(m)')
legend('x','y','z')
saveas(gcf,'error.png')

