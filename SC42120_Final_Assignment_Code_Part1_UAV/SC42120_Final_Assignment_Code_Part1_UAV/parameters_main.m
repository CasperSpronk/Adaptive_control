% clearvars
% addpath('datcom/drawDatcom','datcom/bixler_total','RealTime_Paser',...
%     'common','RealTime_Paser','images','trimmed','FlightGear_Interface');
% open Bixler.slx
% set_param('Bixler', 'DecoupleContODEIntegFromDiscRates', 'off')
% 
% simtime = 210;

%% Load aerodynamic model parameters
parameters_aero
disp([newline, 'Aero:     OK']);

%% Load trim point
parameters_trim
disp('Trim:     OK');

%% Load controllers parameters
parameters_pid
parameters_ctrlReference
disp('PID:      OK');

%% Load TECS parameters
parameters_TECS
disp('TECS:     OK');
% APlane.throttle_slewrate = 4;

%% Load APF parameters
parameters_vf
disp('APF:      OK');

%% Load Waypoints
parameters_WP
disp('WP:       OK');

%% Utilities
Utility.x = datestr(datetime('now','Format','hh:mm'));
Utility.DummyLP.cuttingFreq = 1000;                    % [Hz]
Utility.DummyLP.ome = 2*pi*Utility.DummyLP.cuttingFreq;   % [rad/s]
Utility.DummyLP.den = [1/Utility.DummyLP.ome 1];
Utility.DummyLP.num = 1;
disp(['Utility:  OK', newline]);

%% END OF CODE
disp(['Parameters loaded.       ', Utility.x]);
