
function [newData, timeStamp] = syncTime(data, dataTimeStamp, timeStamp)

for i = 1:length(data(1,:))
    newData(:,i) = interp1(dataTimeStamp, data(:,i), timeStamp, 'pchip');
end

end