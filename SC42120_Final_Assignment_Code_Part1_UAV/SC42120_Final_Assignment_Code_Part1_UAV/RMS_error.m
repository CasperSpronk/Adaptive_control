%Check wind conditions before starting

error = zeros(6,3);
for i=0:0       % Straight-line or orbit
    for k=0:2   % Method
        disp('Bixler UAV simulation start.');
        switch k
            case 0
                disp('APF Method');
            case 1
                disp('Standard Method');
            case 2
                disp('Ideal Method');
        end
        Guid.Method = k;
        Guid.Strategy = i;
        sim('Bixler');
        
        error(i+k+1,1) = i;
        error(i+k+1,2) = k;
        plot_VF;
        error(i+k+1,3) = rms(SimOut.VF.xtrack_erorr.Data(SimOut.VF.xtrack_erorr.Time...
            >= 100 & SimOut.VF.xtrack_erorr.Time <= 200)); % Compute steady error
        disp(['RMS = ', num2str(error(i+k+1,3))]);
    end
end

clear k i