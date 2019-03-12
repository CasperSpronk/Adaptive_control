% Assignments
Out.xyh = OutState.Data(:, 1:3);
Out.vel_body = OutState.Data(:, 4:6);
Out.ang_rates_body = OutState.Data(:, 7:9);
Out.attitude = OutState.Data(:, 10:12);
Out.rollreference = OutState.Data(:,13);
Out.pitchreference = OutState.Data(:,14);
Out.yawreference = OutState.Data(:,15);
Out.throttledemand = OutState.Data(:,16);
Out.aildemand = OutState.Data(:,17);
Out.elevdemand = OutState.Data(:,18);
Out.rudddemand = OutState.Data(:,19);
Out.target_height = OutState.Data(:,20);
Out.target_airspeed = OutState.Data(:,21);
Out.xtrack_error = OutState.Data(:,22);
Out.d_tilde = OutState.Data(:,23);

Out.time = OutState.Time;

% Out.Fx = OutAeroFM.Data(:,1);
% Out.Fy = OutAeroFM.Data(:,2);
% Out.Fz = OutAeroFM.Data(:,3);
% Out.L = OutAeroFM.Data(:,4);
% Out.M = OutAeroFM.Data(:,5);
% Out.N = OutAeroFM.Data(:,6);
Out.alpha = OutDerCond.alpha.Data(:,1);
Out.beta = OutDerCond.beta.Data(:,1);
Out.Va = OutDerCond.Va.Data(:,1);