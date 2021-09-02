% Motion test data analysis
clear all
close all

load compl_output_100.mat;
load compl_output_75.mat;
load compl_output_50.mat;

phi_ind_50 = findPeakIndices(phi_compl_array_50);



figure(1);
for i=1:length(phi_ind_50)
plot(circshift(phi_compl_array_50,-phi_ind_50(i)),'color','red');
hold on;
end
xlim([0,phi_ind_50(2)-phi_ind_50(1)])