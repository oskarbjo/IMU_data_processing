% Completmentary filter

s=serialport("COM6",57600);

data = [];
Nsamples=10;
X = zeros(Nsamples,1);
Y = zeros(Nsamples,1);
Z = zeros(Nsamples,1);
WX = zeros(Nsamples,1);
WY = zeros(Nsamples,1);
WZ = zeros(Nsamples,1);

dT = ones(Nsamples,1)*0.01;
T = cumsum(dT);

allReadouts=[];
pause(1);
t=0;
compFilt = complementaryFilter('HasMagnetometer', false);

tuner = HelperOrientationFilterTuner(compFilt);
tic;
while true
     data=readline(s);
     t=toc;
     tic;
     dT = circshift(dT,-1);
     dT(end)=t;
     T = cumsum(dT);
     
     allReadouts=str2double(regexp(data,'\d+','match'));
     isNegative = int16(bitget(allReadouts,16));
     signedReadouts=int16(bitset(allReadouts,16,0)) + (-2^15)*isNegative;

     X = circshift(X,-1);
     X(end)=signedReadouts(1);
     Y = circshift(Y,-1);
     Y(end)=signedReadouts(2);
     Z = circshift(Z,-1);
     Z(end)=signedReadouts(3);
     signedReadouts(3)
     WX = circshift(WX,-1);
     WX(end)=signedReadouts(4);
     WY = circshift(WY,-1);
     WY(end)=signedReadouts(5);
     WZ = circshift(WZ,-1);
     WZ(end)=signedReadouts(6);
     
     
     allAccel = [X,Y,Z];
     allGyro = [WX,WY,WZ];
     allT = [T,T,T]; 
     
        accel = allAccel(1:Nsamples,:);
        gyro = allGyro(1:Nsamples,:);
        t = allT(1:Nsamples,:);
%         overrun = allOverrun(overrunIdx,:);
        
%         idx = idx + samplesPerRead;
%         overrunIdx = overrunIdx + 1;
%         pause(samplesPerRead/Fs)

    
%     if (isVerbose && overrun > 0)
%         fprintf('%d samples overrun ...\n', overrun);
%     end
    
    q = compFilt(accel, gyro);
    update(tuner, q);
    
%     if useHW
%         if toc >= runTime
%             break;
%         end
%     else
%         if idx(end) > numSamplesAccelGyroMag
%             break;
%         end
%     end
end