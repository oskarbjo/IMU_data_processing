%% Preparing calibration data
close all;
load Arduino_onboard_sensor_1/XYZ.mat;
% load sensor2/XYZ.mat;
% load sensor2/WXYZ.mat;
numSamples = 499;
startingPoint=14700;
step=30000;
figure();
plot(XYZ(:,1));
hold on;
plot(XYZ(:,2));
plot(XYZ(:,3));
legend('x','y','z');


% 104 samples/s
p1 = startingPoint;
p2 = p1+2400;
p3=p2+2200;
p4=p3+2000;
p5=p4+3100;
p6=p5+2000;

% 208 samples/s
% p1 = startingPoint;
% p2 = p1+3500;
% p3=p2+4000;
% p4=p3+4200;
% p5=p4+6400;
% p6=p5+4000;

patch([p1,p1,p1+numSamples,p1+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);
patch([p2,p2,p2+numSamples,p2+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);
patch([p3,p3,p3+numSamples,p3+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);
patch([p4,p4,p4+numSamples,p4+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);
patch([p5,p5,p5+numSamples,p5+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);
patch([p6,p6,p6+numSamples,p6+numSamples],[-2^14,2^14,2^14,-2^14],'red','FaceAlpha',.3);

xyz_001 = [XYZ(p1:p1+numSamples,:),ones(numSamples+1,1)];
xyz_m100 = [XYZ(p2:p2+numSamples,:),ones(numSamples+1,1)];
xyz_00m1 = [XYZ(p3:p3+numSamples,:),ones(numSamples+1,1)];
xyz_100 = [XYZ(p4:p4+numSamples,:),ones(numSamples+1,1)];
xyz_0m10 = [XYZ(p5:p5+numSamples,:),ones(numSamples+1,1)];
xyz_010 = [XYZ(p6:p6+numSamples,:),ones(numSamples+1,1)];

B=[xyz_100;xyz_010;xyz_001;xyz_m100;xyz_0m10;xyz_00m1];

g=2^14;
v_100=[g,0,0,1];
v_010=[0,g,0,1];
v_001=[0,0,g,1];
v_m100=[-g,0,0,1];
v_0m10=[0,-g,0,1];
v_00m1=[0,0,-g,1];

v1=repmat(v_100,numSamples+1,1);
v2=repmat(v_010,numSamples+1,1);
v3=repmat(v_001,numSamples+1,1);
v4=repmat(v_m100,numSamples+1,1);
v5=repmat(v_0m10,numSamples+1,1);
v6=repmat(v_00m1,numSamples+1,1);

A = [v1;v2;v3;v4;v5;v6];

x=A\B;
calMatrix = inv(x);
Ap = B*calMatrix;

figure();
subplot(3,1,1);
plot(B(:,1));
hold on;
plot(Ap(:,1));
subplot(3,1,2);
plot(B(:,2));
hold on;
plot(Ap(:,2));
subplot(3,1,3);
plot(B(:,3));
hold on;
plot(Ap(:,3));