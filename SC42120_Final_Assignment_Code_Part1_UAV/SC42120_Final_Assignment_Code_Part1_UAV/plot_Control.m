%% Autopilot plots

%% Roll Control
figure
subplot(2,1,1)
plot(SimOut.ControlBus.Out_references.Roll_Reference,'r','LineWidth',1.5)
hold on
plot(SimOut.PlantData.Euler_angles__rad_.Time, rad2deg(SimOut.PlantData.Euler_angles__rad_.Data(:,1)),'LineWidth',1.5);
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Roll Control')

subplot(2,1,2)
plot(SimOut.ControlBus.ailerons,'LineWidth',1.5)
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Ailerons demand')


%% Pitch Control
figure
subplot(2,1,1)
plot(SimOut.ControlBus.Out_references.Pitch_Reference,'r', 'LineWidth',1.5)
hold on
plot(SimOut.PlantData.Euler_angles__rad_.Time, rad2deg(SimOut.PlantData.Euler_angles__rad_.Data(:,2)),'LineWidth',1.5);
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Pitch Control')

subplot(2,1,2)
plot(SimOut.ControlBus.elevator,'LineWidth',1.5)
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Elevator demand')


%% Yaw Control

figure
subplot(2,1,1)
plot(SimOut.DerCond.beta.Time, rad2deg(SimOut.DerCond.beta.Data),'LineWidth',1.5)
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Side-slip angle')
subplot(2,1,2)
plot (SimOut.ControlBus.rudder,'LineWidth',1.5);
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Rudder demand')


%% Steady state comparison

% figure
% plot(Out.time, Out.throttledemand,'LineWidth',1.5)
% hold on
% hline = refline([0 Trim.throttle]);
% hline.Color = 'r';
% grid minor; xlabel('[s]'); ylabel('[%]')
% title('Throttle Demand'), legend('Throttle', 'Trimmed Throttle')
% 
% figure
% plot(Out.time, rad2deg(Out.attitude(:,2)),'LineWidth',1.5)
% hold on
% hline = refline([0 Trim.pitch]);
% hline.Color = 'r';
% grid minor; xlabel('[s]'); ylabel('[deg]')
% title('Pitch Demand'), legend('Pitch', 'Trimmed Pitch')
% 
% figure
% subplot(2,1,1)
% plot(Out.time,Out.throttledemand-Trim.throttle,'LineWidth',1.5)
% grid minor; xlabel('[s]'); ylabel('[%]')
% title('Throttle Error wrt trimmed model')
% 
% subplot(2,1,2)
% plot(Out.time,rad2deg(Out.attitude(:,2))-Trim.pitch,'LineWidth',1.5)
% grid minor; xlabel('[s]'); ylabel('[deg]')
% title('Pitch error wrt trimmed model')
% 

%% TECS
figure
subplot(2,1,1)
plot(SimOut.ControlBus.Target_height, 'r', 'LineWidth',1.5)
hold on
plot(SimOut.PlantData.xyh.Time, SimOut.PlantData.xyh.Data(:,3),'LineWidth',1.5);
grid minor; xlabel('[s]'); ylabel('[m/s]')
title('Height Control Loop')
subplot(2,1,2)
plot(SimOut.ControlBus.Out_references.Pitch_Reference,'LineWidth',1.5)
grid minor; xlabel('[s]'); ylabel('[deg]')
title('Pitch demand')

figure
subplot(2,1,1)
plot(SimOut.ControlBus.Target_airspeed, 'r', 'LineWidth',1.5)
hold on
plot(SimOut.DerCond.Va,'LineWidth',1.5);
grid minor; xlabel('[s]'); ylabel('[m/s]')
title('Airspeed Control Loop')
subplot(2,1,2)
plot(SimOut.ControlBus.throttle,'LineWidth',1.5)
grid minor; xlabel('[s]'); ylabel('[%]')
title('Throttle demand')
