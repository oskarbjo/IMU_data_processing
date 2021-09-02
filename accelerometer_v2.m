%% Accelerometer v2 (cleaned up)

% Reading LSM6DS3H accelerometer data being transmitted over serial from arduino.
% Data has format "65481,815,15479,65397,64697,602,1858", corresponding to
% Xacc,Yacc,Zacc,WX,WY,WZ,dt [us]



clear all; close all;

%CALIBRATION DATA CAPTURED WITH ROBOT ARM: 
load calibration_data/sensor2/calmatrix.mat;


s=serialport("COM6",115200);
configureTerminator(s,"CR/LF"); %Carriage return/Linefeed ("Serial.write(13);Serial.write(10);")
data = [];
N=80000;

%OPTIONS
doCalibration=1;
doStaticCalibration=0;
calibrationDone=0;
highPassFilterGyro = 0;
AVGrawdata = 0;
rawDataBoxCarN = 1;
boxCarN = 50; %Complementary filter output averaging
 gain1 = 0.98;
 gain2 = 0.02;




[X,Y,Z,WX,WY,WZ,phi_gyro_array,theta_gyro_array, ...
    phi_compl_array,theta_compl_array,theta_accelerometer_array,phi_accelerometer_array] = initarrays(N);


X_cal=0;
Y_cal=0;
Z_cal=0;
WX_cal=0;
WY_cal=0;
WZ_cal=0;



figure();
pause(0.1);
avg=1;

phi_compl=0;
theta_compl=0;
phi_gyro=0;
theta_gyro=0;

dt=0;
phi_compl = 0;
theta_compl = 0;
counter=0;

P = getcubepoints();%Cube animation data

while 1
tic
    for i=1:1 %read N lines of data
    data=readline(s);
  end

 try
 allReadouts=str2double(regexp(data,'\d+','match'));
 isNegative = int16(bitget(allReadouts,16));
 signedReadouts=int16(bitset(allReadouts,16,0)) + (-2^15)*isNegative;
 signedReadouts(1:3)=matrixCalibration(double(signedReadouts(1:3)),calMatrix);
 X = circshift(X,-1);
 X(end)=signedReadouts(1)-X_cal;
 Y = circshift(Y,-1);
 Y(end)=signedReadouts(2)-Y_cal;
 Z = circshift(Z,-1);
 Z(end)=signedReadouts(3)-Z_cal;
 WX = circshift(WX,-1);
 WX(end)=signedReadouts(4)-WX_cal;
 WY = circshift(WY,-1);
 WY(end)=signedReadouts(5)-WY_cal;
 WZ = circshift(WZ,-1);
 WZ(end)=signedReadouts(6)-WZ_cal;
 dt = double(signedReadouts(7))/1e6;
 catch 'no data received'
 end
 

 

 if doCalibration
 if avg && sum(X ~= zeros(length(X),1))>0.2*N
     if doStaticCalibration==1
%         X_cal=3.980552000000000e+02;
%         Y_cal=1.380102000000000e+02;
%         Z_cal=1.680260140000000e+04-2^16/4;
        WX_cal=7.980355000000000e+02;
        WY_cal=-1.104364100000000e+03;
        WZ_cal=-1.122500000000000e+02;
     elseif doStaticCalibration==0
%         X_cal = mean(X(100:end));
%         Y_cal = mean(Y(100:end));
%         Z_cal = mean(Z(100:end))-2^16/4;
        WX_cal = mean(WX(end-5000:end));
        WY_cal = mean(WY(end-5000:end));
        WZ_cal = mean(WZ(end-5000:end));
     end
     avg=0;
    
    %Reset data
    phi_gyro = 0;
    theta_gyro = 0;
    phi_compl = 0;
    theta_compl = 0;
    phi_gyro_array = zeros(N,1);
    theta_gyro_array = zeros(N,1);
%     phi_compl_array = zeros(N,1);
%     theta_compl_array = zeros(N,1);
%     phi_accelerometer_array = zeros(N,1);
%     theta_accelerometer_array = zeros(N,1);
    doCalibration=0;
 end
 end
 
 
 %Pure accelerometer data
 phi_accelerometer = atan2(Y(end),sqrt(X(end)^2+Z(end)^2))*180/pi;
 theta_accelerometer = atan2(-X(end),Z(end))*180/pi;
 phi_accelerometer_array = circshift(phi_accelerometer_array,-1);
 phi_accelerometer_array(end)=phi_accelerometer;
 theta_accelerometer_array = circshift(theta_accelerometer_array,-1);
 theta_accelerometer_array(end)=theta_accelerometer;
 

 %Pure gyro data
 gyro_scaling = 250/2^15; %see datasheet
 
 phi_gyro = phi_gyro + gyro_scaling*WX(end)*dt;
 theta_gyro = theta_gyro + gyro_scaling*WY(end)*dt;
 phi_gyro_array = circshift(phi_gyro_array,-1);
 phi_gyro_array(end)=phi_gyro;
 theta_gyro_array = circshift(theta_gyro_array,-1);
 theta_gyro_array(end)=theta_gyro;
 
 %Complementary filter data
 phi_compl = gain1 * (phi_compl + gyro_scaling*WX(end)*dt) + gain2*phi_accelerometer;
 theta_compl = gain1 * (theta_compl + gyro_scaling*WY(end)*dt) + gain2*theta_accelerometer;
 phi_compl_array = circshift(phi_compl_array,-1);
 phi_compl_array(end)=phi_compl;
 theta_compl_array = circshift(theta_compl_array,-1);
 theta_compl_array(end)=theta_compl;

 
 
 if counter == 3000
     
 % Accelerometer data only
%  subplot(2,1,1);
%  plot(X);
%  hold on
%  plot(Y);
%  plot(Z);
%  legend('wx','wy','wz');
%  hold off
     
     
 %Complementary filter plot:
 subplot(2,2,1);
 plot(movmean(phi_compl_array,boxCarN));
 subplot(2,2,3);
 plot(movmean(theta_compl_array,boxCarN));

%Individual sensor data plots:
%  subplot(2,2,1);
%  plot(movmean(WX,boxCarN));
%  hold on;
%  plot(movmean(WY,boxCarN));
%  plot(movmean(X,boxCarN));
%  hold on
%  plot(movmean(Y,boxCarN));
%  legend('Gyro phi','Gyro theta','Acc phi','Acc theta');
%  hold off;

 %Angles from gyro data only:
%  subplot(2,2,1);
%  plot(movmean(phi_gyro_array,boxCarN));
%  subplot(2,2,3);
%  plot(movmean(theta_gyro_array,boxCarN));
%  ylabel('Degrees')

%  %Angles from acc data only:
%  subplot(2,2,1);
%  plot(movmean(phi_accelerometer_array,boxCarN));
%  subplot(2,2,3);
%  plot(movmean(theta_accelerometer_array,boxCarN));
%  ylabel('Degrees')

%Animate cube
subplot(2,2,[2 4]);
P = getcubepoints();%Cube animation data
yaw=0;
dcm = angle2dcm(yaw, -phi_compl*pi/180, theta_compl*pi/180);
P = P*dcm;
plot3(P(:,1),P(:,2),P(:,3)) % rotated cube
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);
 
drawnow;
 counter=0;
 toc
 end
 counter=counter+1;

end


