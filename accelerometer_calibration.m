%% Accelerometer calibration

% Reading LSM6DS3H accelerometer data being transmitted over serial from arduino.
% Data has format "65481,815,15479,65397,64697,602,1858", corresponding to
% Xacc,Yacc,Zacc,WX,WY,WZ,dt [us]



close all; clear all;
s=serialport("COM5",115200);
configureTerminator(s,"CR/LF"); %Carriage return/Linefeed ("Serial.write(13);Serial.write(10);")
data = [];
N=30000; %40000
AVGrawdata = 1;
rawDataBoxCarN = 1;
boxCarN = 1; %Complementary filter output averaging
X = zeros(N,1);
Y = zeros(N,1);
Z = zeros(N,1)+16000;
Xavg_array = zeros(N,1);
Yavg_array = zeros(N,1);
Zavg_array = zeros(N,1);
WX = zeros(N,1);
WY = zeros(N,1);
WZ = zeros(N,1);
Xavg = 0;
Yavg = 0;
Zavg = 0;
WXavg = 0;
WYavg = 0;
WZavg = 0;

phi_gyro_array = zeros(N,1);
theta_gyro_array = zeros(N,1);
phi_compl_array = zeros(N,1);
theta_compl_array = zeros(N,1);
phi_accelerometer_array = zeros(N,1);
theta_accelerometer_array = zeros(N,1);
readbytes = [0,0]
figure(); title('data');

allReadouts=[];
pause(1);

counter=0;



while 1
 
 data=readline(s);

 try
 allReadouts=str2double(regexp(data,'\d+','match'));
 isNegative = int16(bitget(allReadouts,16));
 signedReadouts=int16(bitset(allReadouts,16,0)) + (-2^15)*isNegative;
 X = circshift(X,-1);
 X(end)=signedReadouts(1)-Xavg;
 Y = circshift(Y,-1);
 Y(end)=signedReadouts(2)-Yavg;
 Z = circshift(Z,-1);
 Z(end)=signedReadouts(3)-Zavg;
 WX = circshift(WX,-1);
 WX(end)=signedReadouts(4)-WXavg;
 WY = circshift(WY,-1);
 WY(end)=signedReadouts(5)-WYavg;
 WZ = circshift(WZ,-1);
 WZ(end)=signedReadouts(6)-WZavg;
 dt = double(signedReadouts(7))/1e6;
 catch 'no data received'
 end
 

 
 if AVGrawdata
     Xavg_array = movmean(X,rawDataBoxCarN);
     Yavg_array = movmean(Y,rawDataBoxCarN);
     Zavg_array = movmean(Z,rawDataBoxCarN);
 end


 
 
 if counter == 500
     
 % Accelerometer data only
 subplot(3,1,1);
 plot(Xavg_array);
 subplot(3,1,2);
 plot(Yavg_array);
 subplot(3,1,3);
 plot(Zavg_array);
 
  % Accelerometer data only
%  subplot(3,1,1);
%  plot(Xavg_array);
%  subplot(3,1,2);
%  plot(Yavg_array);
%  subplot(3,1,3);
%  plot(Zavg_array);
     

 counter=0;
 end
 counter=counter+1;
 drawnow;
 
end

%Save calibration data:
XYZ = [X,Y,Z]; 
clear s

