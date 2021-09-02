function out=matrixCalibration(v,calMatrix)

    out=[v,1]*calMatrix;
    out=out(1:3);
end