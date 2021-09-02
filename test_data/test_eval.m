% Test data eval

load compl_output_gain1_0_gain2_1_N_1.mat;
phi_compl_array_gain1_0_gain2_1_N_1 = phi_compl_array;
theta_compl_array_gain1_0_gain2_1_N_1 = theta_compl_array;
load compl_output_gain1_0p05_gain2_0p95_N_1.mat;
phi_compl_array_gain1_0p05_gain2_0p95_N_1 = phi_compl_array;
theta_compl_array_gain1_0p05_gain2_0p95_N_1 = theta_compl_array;
load compl_output_gain1_0p5_gain2_0p5_N_1.mat;
phi_compl_array_gain1_0p5_gain2_0p5_N_1 = phi_compl_array;
theta_compl_array_gain1_0p5_gain2_0p5_N_1 = theta_compl_array;
load compl_output_gain1_0p95_gain2_0p05_N_1.mat;
phi_compl_array_gain1_0p95_gain2_0p05_N_1 = phi_compl_array;
theta_compl_array_gain1_0p95_gain2_0p05_N_1 = theta_compl_array;
load compl_output_gain1_1_gain2_0_N_1.mat;
phi_compl_array_gain1_1_gain2_0_N_1 = phi_compl_array;
theta_compl_array_gain1_1_gain2_0_N_1 = theta_compl_array;

load compl_output_gain1_0_gain2_1_N_50.mat;
phi_compl_array_gain1_0_gain2_1_N_50 = phi_compl_array;
theta_compl_array_gain1_0_gain2_1_N_50 = theta_compl_array;
load compl_output_gain1_0p05_gain2_0p95_N_50.mat;
phi_compl_array_gain1_0p05_gain2_0p95_N_50 = phi_compl_array;
theta_compl_array_gain1_0p05_gain2_0p95_N_50 = theta_compl_array;
load compl_output_gain1_0p5_gain2_0p5_N_50.mat;
phi_compl_array_gain1_0p5_gain2_0p5_N_50 = phi_compl_array;
theta_compl_array_gain1_0p5_gain2_0p5_N_50 = theta_compl_array;
load compl_output_gain1_0p95_gain2_0p05_N_50.mat;
phi_compl_array_gain1_0p95_gain2_0p05_N_50 = phi_compl_array;
theta_compl_array_gain1_0p95_gain2_0p05_N_50 = theta_compl_array;
load compl_output_gain1_1_gain2_0_N_50.mat;
phi_compl_array_gain1_1_gain2_0_N_50 = phi_compl_array;
theta_compl_array_gain1_1_gain2_0_N_50 = theta_compl_array;


figure();
plot(circshift(phi_compl_array_gain1_0_gain2_1_N_1,0));
hold on;
title('No averaging');
plot(circshift(phi_compl_array_gain1_0p05_gain2_0p95_N_1,1400));
plot(circshift(phi_compl_array_gain1_0p5_gain2_0p5_N_1,100));
plot(circshift(phi_compl_array_gain1_0p95_gain2_0p05_N_1,1420));
plot(circshift(phi_compl_array_gain1_1_gain2_0_N_1,-470));
legend('Gain1 = 0, Gain2 = 1','Gain1 = 0.05, Gain2 = 0.95','Gain1 = 0.5, Gain2 = 0.5','Gain1 = 0.95, Gain2 = 0.05','Gain1 = 1, Gain2 = 0')
xlim([7000,15000]);

figure();
plot(movmean(circshift(phi_compl_array_gain1_0_gain2_1_N_1,0),50));
hold on;
title('50 Samples/100 ms averaging');
plot(movmean(circshift(phi_compl_array_gain1_0p05_gain2_0p95_N_1,1400),50));
plot(movmean(circshift(phi_compl_array_gain1_0p5_gain2_0p5_N_1,100),50));
plot(movmean(circshift(phi_compl_array_gain1_0p95_gain2_0p05_N_1,1420),50));
plot(movmean(circshift(phi_compl_array_gain1_1_gain2_0_N_1,-470),50));
legend('Gain1 = 0, Gain2 = 1','Gain1 = 0.05, Gain2 = 0.95','Gain1 = 0.5, Gain2 = 0.5','Gain1 = 0.95, Gain2 = 0.05','Gain1 = 1, Gain2 = 0')
xlim([7000,15000]);

figure();
% plot(circshift(phi_compl_array_gain1_0p95_gain2_0p05_N_1,1420));
hold on;
% plot(circshift(theta_compl_array_gain1_0p95_gain2_0p05_N_1,1420));
plot(movmean(circshift(phi_compl_array_gain1_0p95_gain2_0p05_N_1,1420),[50,0]));
hold on;
plot(movmean(circshift(theta_compl_array_gain1_0p95_gain2_0p05_N_1,1420),[50,0]));
legend('Phi','Theta');
xlim([6000,19000]);
grid on;
