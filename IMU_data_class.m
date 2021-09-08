classdef IMU_data_class < handle
    %IMU_DATA_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N;
        X;
        Y;
        Z;
        X_avg;
        Y_avg;
        Z_avg;
        WX;
        WY;
        WZ;
        phi_gyro_array;
        theta_gyro_array;
        phi_compl_array;
        theta_compl_array;
        theta_accelerometer_array;
        phi_accelerometer_array;
        dt=1;
    end
    
    methods
        function obj = IMU_data_class(N)
            obj.N = N;
            obj.X=zeros(N,1);
            obj.Y=zeros(N,1);
            obj.Z=zeros(N,1);
            obj.X_avg=zeros(N,1);
            obj.Y_avg=zeros(N,1);
            obj.Z_avg=zeros(N,1);
            obj.WX=zeros(N,1);
            obj.WY=zeros(N,1);
            obj.WZ=zeros(N,1);
            obj.phi_gyro_array=zeros(N,1);
            obj.theta_gyro_array=zeros(N,1);
            obj.phi_compl_array=zeros(N,1);
            obj.theta_compl_array=zeros(N,1);
            obj.theta_accelerometer_array=zeros(N,1);
            obj.phi_accelerometer_array=zeros(N,1);
        end
        
        function lowpassAcc(obj,Nsamples)
            obj.X_avg = movmean(obj.X,[Nsamples,0]);
            obj.Y_avg = movmean(obj.Y,[Nsamples,0]);
            obj.Z_avg = movmean(obj.Z,[Nsamples,0]);
        end
        
        function recalculateAngles(obj,gain1,gain2) %Recalculate the angles using raw data to change gain retrospectively
            phi_accelerometer = 0;
            theta_accelerometer = 0;
            phi_gyro = 0;
            theta_gyro = 0;
            phi_compl = 0;
            theta_compl = 0;
            
            for i=1:numel(obj.X)
                 %Accelerometer data
                 phi_accelerometer = atan2(obj.Y_avg(i),sqrt(obj.X_avg(i)^2+obj.Z_avg(i)^2))*180/pi;
                 theta_accelerometer = atan2(-obj.X_avg(i),obj.Z_avg(i))*180/pi;
                 obj.phi_accelerometer_array = circshift(obj.phi_accelerometer_array,-1);
                 obj.phi_accelerometer_array(end)=phi_accelerometer;
                 obj.theta_accelerometer_array = circshift(obj.theta_accelerometer_array,-1);
                 obj.theta_accelerometer_array(end)=theta_accelerometer;
                 
                 %Gyro data
                 gyro_scaling = 250/2^15; %see datasheet 
                 phi_gyro = phi_gyro + gyro_scaling*obj.WX(i)*obj.dt;
                 theta_gyro = theta_gyro + gyro_scaling*obj.WY(i)*obj.dt;
                 obj.phi_gyro_array = circshift(obj.phi_gyro_array,-1);
                 obj.phi_gyro_array(end)=phi_gyro;
                 obj.theta_gyro_array = circshift(obj.theta_gyro_array,-1);
                 obj.theta_gyro_array(end)=theta_gyro;
                 
                  %Complementary filter data
                  phi_compl = gain1 * (phi_compl + gyro_scaling*obj.WX(i)*obj.dt) + gain2*phi_accelerometer;
                  theta_compl = gain1 * (theta_compl + gyro_scaling*obj.WY(i)*obj.dt) + gain2*theta_accelerometer;
                  obj.phi_compl_array = circshift(obj.phi_compl_array,-1);
                  obj.phi_compl_array(end)=phi_compl;
                  obj.theta_compl_array = circshift(obj.theta_compl_array,-1);
                  obj.theta_compl_array(end)=theta_compl;
                 
             end
        end
        
        function averageComplFilterOutput(obj) %TO IMPLEMENT!!
        end
    end
    

end

