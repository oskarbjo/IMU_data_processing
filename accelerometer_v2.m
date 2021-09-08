%% Accelerometer v2 (cleaned up)

% Reading LSM6DS3H accelerometer data being transmitted over serial from arduino.
% Data has format "65481,815,15479,65397,64697,602,1858", corresponding to
% Xacc,Yacc,Zacc,WX,WY,WZ,dt [us]



clear all; close all;

%CALIBRATION DATA CAPTURED WITH ROBOT ARM: 
load calibration_data/Arduino_onboard_sensor_1/calmatrix.mat;


s=serialport("COM5",115200);
configureTerminator(s,"CR/LF"); %Carriage return/Linefeed ("Serial.write(13);Serial.write(10);")
data = [];
N=50000;
calN=3000;

%OPTIONS
doCalibration=1;
calibrationDone=0;
doActiveGainCorr=0;
lowPassAcc = 1;
accMovingAvgN = 25;
boxCarN = 10; %Complementary filter output averaging N
gain1 = 0.98;
gain2 = 0.02;
deltaT = 104; %samples/s


IMU = IMU_data_class(N);


X_cal=0;
Y_cal=0;
Z_cal=0;
WX_cal=0;
WY_cal=0;
WZ_cal=0;



f=figure(1);
gain1_slider = uicontrol('Parent',f,'Style','slider','Position',[1000,100,200,23],...
              'value',gain1, 'min',0, 'max',1);
gain2_slider = uicontrol('Parent',f,'Style','slider','Position',[1000,50,200,23],...
  'value',gain2, 'min',0, 'max',1);
value_gain1 = uicontrol('Parent',f,'Style','text','Position',[1100,75,43,23],...
                'String',num2str(gain1_slider.Value),'BackgroundColor',f.Color);
value_gain2 = uicontrol('Parent',f,'Style','text','Position',[1100,25,43,23],...
                'String',num2str(gain2_slider.Value),'BackgroundColor',f.Color);
pause(0.1);
avg=1;

phi_compl=0;
theta_compl=0;
phi_gyro=0;
theta_gyro=0;

phi_compl = 0;
theta_compl = 0;
counter=0;

P = getcubepoints();%Cube animation data

while 1

gain1 = gain1_slider.Value;
gain2=1-gain1;
gain2_slider.Value=gain2;
value_gain1.String = num2str(gain1);
value_gain2.String = num2str(gain2);

    
tic
    for i=1:1 %read N lines of data
    data=readline(s);
  end

 try
 allReadouts=str2double(regexp(data,'\d+','match'));
 isNegative = int16(bitget(allReadouts,16));
 signedReadouts=int16(bitset(allReadouts,16,0)) + (-2^15)*isNegative;
 signedReadouts(1:3)=matrixCalibration(double(signedReadouts(1:3)),calMatrix);
 IMU.X = circshift(IMU.X,-1);
 IMU.X(end)=signedReadouts(1)-X_cal;
 IMU.Y = circshift(IMU.Y,-1);
 IMU.Y(end)=signedReadouts(2)-Y_cal;
 IMU.Z = circshift(IMU.Z,-1);
 IMU.Z(end)=signedReadouts(3)-Z_cal;
 IMU.WX = circshift(IMU.WX,-1);
 IMU.WX(end)=signedReadouts(4)-WX_cal;
 IMU.WY = circshift(IMU.WY,-1);
 IMU.WY(end)=signedReadouts(5)-WY_cal;
 IMU.WZ = circshift(IMU.WZ,-1);
 IMU.WZ(end)=signedReadouts(6)-WZ_cal;
 IMU.dt = double(signedReadouts(7))/1e6;
 IMU.dt=1/deltaT;
 catch 'no data received'
 end
 
 if doActiveGainCorr
     [gain1,gain2]=activeGainCorrection(IMU.X_avg(end),IMU.Y_avg(end),IMU.Z_avg(end));
 end

 if lowPassAcc
     IMU.lowpassAcc(accMovingAvgN);
 end
 
 if doCalibration
 if avg && sum(IMU.X ~= zeros(length(IMU.X),1))>calN*1.2
%         X_cal = mean(X(100:end));
%         Y_cal = mean(Y(100:end));
%         Z_cal = mean(Z(100:end))-2^16/4;
        WX_cal = mean(IMU.WX(end-calN:end));
        WY_cal = mean(IMU.WY(end-calN:end));
        WZ_cal = mean(IMU.WZ(end-calN:end));

     avg=0;
    
    %Reset data
    phi_gyro = 0;
    theta_gyro = 0;
    phi_compl = 0;
    theta_compl = 0;
    IMU.phi_gyro_array = zeros(N,1);
    IMU.theta_gyro_array = zeros(N,1);
%     phi_compl_array = zeros(N,1);
%     theta_compl_array = zeros(N,1);
%     phi_accelerometer_array = zeros(N,1);
%     theta_accelerometer_array = zeros(N,1);
    doCalibration=0;
 end
 end
 
 
 %Pure accelerometer data
 phi_accelerometer = atan2(IMU.Y_avg(end),sqrt(IMU.X_avg(end)^2+IMU.Z_avg(end)^2))*180/pi;
 theta_accelerometer = atan2(-IMU.X_avg(end),IMU.Z_avg(end))*180/pi;
 IMU.phi_accelerometer_array = circshift(IMU.phi_accelerometer_array,-1);
 IMU.phi_accelerometer_array(end)=phi_accelerometer;
 IMU.theta_accelerometer_array = circshift(IMU.theta_accelerometer_array,-1);
 IMU.theta_accelerometer_array(end)=theta_accelerometer;
 

 %Pure gyro data
 gyro_scaling = 250/2^15; %see datasheet
 
 phi_gyro = phi_gyro + gyro_scaling*IMU.WX(end)*IMU.dt;
 theta_gyro = theta_gyro + gyro_scaling*IMU.WY(end)*IMU.dt;
 IMU.phi_gyro_array = circshift(IMU.phi_gyro_array,-1);
 IMU.phi_gyro_array(end)=phi_gyro;
 IMU.theta_gyro_array = circshift(IMU.theta_gyro_array,-1);
 IMU.theta_gyro_array(end)=theta_gyro;
 
 %Complementary filter data
 phi_compl = gain1 * (phi_compl + gyro_scaling*IMU.WX(end)*IMU.dt) + gain2*phi_accelerometer;
 theta_compl = gain1 * (theta_compl + gyro_scaling*IMU.WY(end)*IMU.dt) + gain2*theta_accelerometer;
 IMU.phi_compl_array = circshift(IMU.phi_compl_array,-1);
 IMU.phi_compl_array(end)=phi_compl;
 IMU.theta_compl_array = circshift(IMU.theta_compl_array,-1);
 IMU.theta_compl_array(end)=theta_compl;

 
 
 if counter == 100

     
     
 %Complementary filter plot:
 subplot(2,2,1);
 plot(movmean(IMU.phi_compl_array,boxCarN));
 subplot(2,2,3);
 plot(movmean(IMU.theta_compl_array,boxCarN));


%Animate cube
subplot(2,2,2);
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
toc;
 end
 counter=counter+1;

end


