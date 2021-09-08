function [gain1,gain2] = activeGainCorrection(x,y,z)
correctionLimit = 1.01;
correctionGain=2;
ampl=sqrt(x^2 + y^2 + z^2);
if ampl > 2^14 * correctionLimit;
    
    a=ampl/2^14;
    gain2=0.02-0.02*(a-1)*correctionGain;
    gain1=1-gain2;
    'Gain corrected'
    
    
else
    gain1=0.98;
    gain2=1-gain1;
    
end


end

