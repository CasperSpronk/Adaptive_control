%% PLOTS OF THE FLIGHT DATA %%

close all
warning('off','all')

% Main figure (4 subplots) with the 12 states
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,2,1)
plot(SimOut.PlantData.xyh);
grid minor; xlabel('[s]'); ylabel('[m]'); legend('North', 'East', 'altitude')
title('POSITION (in Earth frame)')
subplot(2,2,2)
plot(SimOut.PlantData.Velocity__inertial_);
grid minor; xlabel('[s]'); ylabel('[m/s]'); legend('u', 'v', 'w')
title('VELOCITY (body frame)')
subplot(2,2,3)
plot(SimOut.PlantData.Euler_angles__rad_.Time, rad2deg(SimOut.PlantData.Euler_angles__rad_.Data));
grid minor; xlabel('[s]'); ylabel('[deg]'); legend('R', 'P', 'Y');
title('ATTITUDE (roll, pitch, yaw)')
subplot(2,2,4)
plot(SimOut.PlantData.Angular_rates__body___rad_s_.Time, rad2deg(SimOut.PlantData.Angular_rates__body___rad_s_.Data));
grid minor; xlabel('[s]'); ylabel('[deg/s]'); legend('p', 'q', 'r');
title('ANGULAR RATES (body frame)')

% Trajectory
figure;
plot(SimOut.PlantData.xyh.Time, SimOut.PlantData.xyh.Data(:,3));
xlabel('[s]'); ylabel('[m]');grid minor;
title('Bixler Height')

% Angle of attack
figure
plot(SimOut.DerCond.alpha.Time, rad2deg(SimOut.DerCond.alpha.Data))
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Angle of Attack (alpha)')

% Sideslip angle
figure
plot(SimOut.DerCond.beta.Time, rad2deg(SimOut.DerCond.beta.Data))
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Sideslip angle (beta)')

% Airspeed
figure
plot(SimOut.DerCond.Va)
grid minor; xlabel('[s]'); ylabel('[m/s]')
title('Airspeed (Va)')

% Course command
figure
plot(SimOut.DerCond.Course.Time, rad2deg(SimOut.DerCond.Course.Data))
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Course angle (chi)')
% 
% % Vector field error
% figure
% plot(Out.time, Out.xtrack_error)
% grid minor; xlabel('[s]'); ylabel('[m]')
% title('Crosstrack error')

warning('on','all')


%% END OF CODE
disp('Done.');