% Motion test data analysis
clear all
close all
load arduino1_10vel.mat;


gain1=0.8;
gain2=1-gain1;
accAvgN=20;
IMU.lowpassAcc(accAvgN);
IMU.recalculateAngles(gain1,gain2);

phi_peak_ind = findPeakIndices(IMU.phi_compl_array);
theta_peak_ind = findPeakIndices(IMU.theta_compl_array);


figure();
for i=1:numel(phi_peak_ind)
    plot(circshift(IMU.theta_compl_array,-theta_peak_ind(i)),'color','r');
    hold on;
end
xlim([0,1000]);