
close all;
%Head motion acceleration estimated to max ~0.1g when moving visibly,
%~0.004g when attempting to lie still
load typical_head_motion_data/moving_head_motion_data.mat;
IMU_moving=IMU;
load typical_head_motion_data/still_head_motion_data.mat;
IMU_still=IMU;

t1_moving=1;
t2_moving=16500;
t1_still=1;
t2_still=18000;

figure(1);
subplot(3,1,1);
plot(100*IMU_moving.X(t1_moving:end)/2^14);
subplot(3,1,2);
plot(100*IMU_moving.Y(t1_moving:end)/2^14);
subplot(3,1,3);
plot(100*IMU_moving.Z(t1_moving:end)/2^14);

figure(2);
subplot(3,1,1);
plot(100*IMU_still.X(t1_still:end)/2^14);
subplot(3,1,2);
plot(100*IMU_still.Y(t1_still:end)/2^14);
subplot(3,1,3);
plot(100*IMU_still.Z(t1_still:end)/2^14);


