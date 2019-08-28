clear all; close all

foldername = 'dataexp2019-05-03_12-13-56';

dinfo = dir([foldername,'/*.csv']);
time = cell(length(dinfo),1);
error = cell(length(dinfo),1);
for j = 1:length(dinfo)
    thisfilename = dinfo(j).name;  %just the name
    fileraw = csvread(thisfilename,0,0); %initial reading of all values
    time{j} = fileraw(:,1);
    pos = fileraw(:,2:4);
    pos_d = fileraw(:,5:7);
    error{j} = pos - pos_d;
end

timeStamp = 0:1.0/60:8;

error_sync = zeros(length(timeStamp),3,length(dinfo));
for j = 1:length(dinfo) 
    error_sync(:,:,j) = syncTime(error{j} , time{j}, timeStamp);
end



error_mean = mean(error_sync,3);
error_std = std(error_sync,3);

% Plot TRPO Reward
afigure;
hold on
plot(0,0,'r'); plot(0,0,'b--');
plot(timeStamp,error_mean)
%shadedErrorBar(1:length(error_mean), error_mean, error_std,'lineProps','b--')
