%% Implementation of the the PID parameters %%

Ctr.TS = 0.0025;      % 400 Hz

%% 
%%%%%%%%%%%%%%%%%% GENERAL %%%%%%%%%%%%%%
APlane.scaling_speed = Aero.AirspeedRef;  % [m/s]
APlane.airspeed_min = 6;                  % [m/s]
APlane.EAS2TAS = 1;
APlane.roll_limit = 84;
APlane.kff_throttle_to_pitch = 0.1920;

%%
%%%%%%%%%%%%%%%%%% ROLL CONTROL %%%%%%%%%%%%%%
% Param: TCONST
% DisplayName: Roll Time Constant
% Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
% Range: 0.4 1.0
% Units: seconds
Ctr.Roll.RLL2SRV_TCONST = 0.45;              % This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.

% Param: P
% DisplayName: Proportional Gain
% Description: This is the gain from bank angle error to aileron.
% Range: 0.1 4.0
Ctr.Roll.RLL2SRV_P = 0.7;                   % From DOCUMENTATION (for Bixler).

% Param: I
% DisplayName: Integrator Gain
% Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
% Range: 0 1.0
Ctr.Roll.RLL2SRV_I = 0.1;                  % From DOCUMENTATION (common value for most airframes)

% Param: D
% DisplayName: Damping Gain
% Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
% Range: 0 0.1
Ctr.Roll.RLL2SRV_D = 0.01;

% Param: FF
% DisplayName: Feed forward Gain
% Description: This is the gain from demanded rate to aileron output. 
% Range: 0.1 4.0
Ctr.Roll.RLL2SRV_FF = 0;                    % GUESS

% Param: RMAX
% DisplayName: Maximum Roll Rate
% Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
% Range: 0 180
% Units: degrees/second
Ctr.Roll.RLL2SRV_RMAX = 75;                 % From DOCUMENTATION [max input for ailerons][deg/s]

% Param: IMAX
% DisplayName: Integrator limit
% Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 2/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
% Range: 0 4500
Ctr.Roll.RLL2SRV_IMAX = 3000;               % From CODE
Ctr.Roll.RLL2SRV_IMAX_SCALED = Ctr.Roll.RLL2SRV_IMAX * 0.01;

Ctr.Roll.OMEGA = 1/ Ctr.Roll.RLL2SRV_TCONST;

Ctr.Roll.K_P = (Ctr.Roll.RLL2SRV_P-Ctr.Roll.RLL2SRV_I*Ctr.Roll.RLL2SRV_TCONST)*Ctr.Roll.RLL2SRV_TCONST-Ctr.Roll.RLL2SRV_D;
Ctr.Roll.K_I = Ctr.Roll.RLL2SRV_I*Ctr.Roll.RLL2SRV_TCONST;
%%
%%%%%%%%%%%%%%%%%% PITCH CONTROL %%%%%%%%%%%%%%

% Param: TCONST
% DisplayName: Pitch Time Constant
% Description: This controls the time constant in seconds from demanded to achieved pitch angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
% Range: 0.4 1.0
% Units: seconds
Ctr.Pitch.PTCH2SRV_TCONST = 0.45;            % From DOCUMENTATION (normal parameter, useless to change)

% Param: P
% DisplayName: Proportional Gain
% Description: This is the gain from pitch angle to elevator. This gain works the same way as PTCH2SRV_P in the old PID controller and can be set to the same value.
% Range: 0.1 3.0
Ctr.Pitch.PTCH2SRV_P = 1.055762;                

% Param: I
% DisplayName: Integrator Gain
% Description: This is the gain applied to the integral of pitch angle. It has the same effect as PTCH2SRV_I in the old PID controller and can be set to the same value. Increasing this gain causes the controller to trim out constant offsets between demanded and measured pitch angle.
% Range: 0 0.5
Ctr.Pitch.PTCH2SRV_I = max(0.08798018, 0.15);

% Param: D
% DisplayName: Damping Gain
% Description: This is the gain from pitch rate to elevator. This adjusts the damping of the pitch control loop. It has the same effect as PTCH2SRV_D in the old PID controller and can be set to the same value, but without the spikes in servo demands. This gain helps to reduce pitching in turbulence. Some airframes such as flying wings that have poor pitch damping can benefit from increasing this gain term. This should be increased in 0.01 increments as too high a value can lead to a high frequency pitch oscillation that could overstress the airframe.
% Range: 0 0.1
Ctr.Pitch.PTCH2SRV_D = 0.07918216;            

% Param: RMAX_UP
% DisplayName: Pitch up max rate
% Description: This sets the maximum nose up pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
% Range: 0 100
% Units: degrees/second
Ctr.Pitch.PTCH2SRV_RMAX_UP = 75;   % GUESS

% Param: RMAX_DN
% DisplayName: Pitch down max rate
% Description: This sets the maximum nose down pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
% Range: 0 100
% Units: degrees/second
Ctr.Pitch.PTCH2SRV_RMAX_DN = 75;   % GUESS

% Param: RLL
% DisplayName: Roll compensation
% Description: This is the gain term that is applied to the pitch rate offset calculated as required to keep the nose level during turns. The default value is 1 which will work for all models. Advanced users can use it to correct for height variation in turns. If height is lost initially in turns this can be increased in small increments of 0.05 to compensate. If height is gained initially in turns then it can be decreased.
% Range: 0.7 1.5
Ctr.Pitch.PTCH2SRV_RLL = 1;                 % From DOCUMENTATION (default value)
% 
% Param: IMAX
% DisplayName: Integrator limit
% Description: This limits the number of centi-degrees of elevator over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 degrees, so the default value represents a 2/3rd of the total control throw which is adequate for most aircraft unless they are severely out of trim or have very limited elevator control effectiveness.
% Range: 0 4500
Ctr.Pitch.PTCH2SRV_IMAX = 3000;

% Param: FF
% DisplayName: Feed forward Gain
% Description: This is the gain from demanded rate to elevator output.
% Range: 0.1 4.0
Ctr.Pitch.PTCH2SRV_FF = 0;

Ctr.Pitch.OMEGA = 1/Ctr.Pitch.PTCH2SRV_TCONST; 

Ctr.Pitch.Trimmed = Trim.pitch/Trim.throttle;

Ctr.Pitch.K_P = (Ctr.Pitch.PTCH2SRV_P - Ctr.Pitch.PTCH2SRV_I*Ctr.Pitch.PTCH2SRV_TCONST)*Ctr.Pitch.PTCH2SRV_TCONST - Ctr.Pitch.PTCH2SRV_D;

%%
%%%%%%%%%%%%%%%%%% YAW CONTROL %%%%%%%%%%%%%%

% Param: SLIP
% DisplayName: Sideslip control gain
% Description: This is the gain from measured lateral acceleration to demanded yaw rate. It should be set to zero unless active control of sideslip is desired. This will only work effectively if there is enough fuselage side area to generate a measureable lateral acceleration when the model sideslips. Flying wings and most gliders cannot use this term. This term should only be adjusted after the basic yaw damper gain YAW2SRV_DAMP is tuned and the YAW2SRV_INT integrator gain has been set. Set this gain to zero if only yaw damping is required.
% Range: 0 4
% Increment: 0.25
% User: Advanced
Ctr.Yaw.YAW2SRV_SLIP = 0;

% Param: INT
% DisplayName: Sideslip control integrator
% Description: This is the integral gain from lateral acceleration error. This gain should only be non-zero if active control over sideslip is desired. If active control over sideslip is required then this can be set to 1.0 as a first try.
% Range: 0 2
Ctr.Yaw.YAW2SRV_INT = 1;

% Param: DAMP
% DisplayName: Yaw damping
% Description: This is the gain from yaw rate to rudder. It acts as a damper on yaw motion. If a basic yaw damper is required, this gain term can be incremented, whilst leaving the YAW2SRV_SLIP and YAW2SRV_INT gains at zero. Note that unlike with a standard PID controller, if this damping term is zero then the integrator will also be disabled.
%     Range: 0 2
%     Increment: 0.25
%     User: Advanced
Ctr.Yaw.YAW2SRV_DAMP = 1.5;

% Param: RLL
% DisplayName: Yaw coordination gain
% Description: This is the gain term that is applied to the yaw rate offset calculated as required to keep the yaw rate consistent with the turn rate for a coordinated turn. The default value is 1 which will work for all models. Advanced users can use it to correct for any tendency to yaw away from or into the turn once the turn is established. Increase to make the model yaw more initially and decrease to make the model yaw less initially. If values greater than 1.1 or less than 0.9 are required then it normally indicates a problem with the airspeed calibration.
% Range: 0.8 1.2
% Increment: 0.05
% User: Advanced
Ctr.Yaw.YAW2SRV_RLL = 1;

% /*
% Note: index 4 should not be used - it was used for an incorrect
% AP_Int8 version of the IMAX in the 2.74 release
% */


% Param: IMAX
% DisplayName: Integrator limit
% Description: This limits the number of centi-degrees of rudder over which the integrator will operate. At the default setting of 1500 centi-degrees, the integrator will be limited to +- 15 degrees of servo travel. The maximum servo deflection is +- 45 degrees, so the default value represents a 1/3rd of the total control throw which is adequate for most aircraft unless they are severely out of trim or have very limited rudder control effectiveness.
% Range: 0 4500
% Increment: 1
% User: Advanced
Ctr.Yaw.YAW2SRV_IMAX = 1500;

Ctr.Yaw.KFF_RDDRMIX = 0.5;

%%
%%%%%%%%%%%%%%%%%% BANK ANGLE CONTROL %%%%%%%%%%%%%%
% Ctr.Hdg2Roll.NAV_ROLL_P = 0.9;
% % Ctr.Hdg2Roll.NAV_ROLL_P = 0.3;
% % Ctr.Hdg2Roll.NAV_ROLL_I = 0.01;
% % Ctr.Hdg2Roll.NAV_ROLL_I = 0.1;
% Ctr.Hdg2Roll.NAV_ROLL_I = 0.3;
% % Ctr.Hdg2Roll.NAV_ROLL_I = 0.0;
% Ctr.Hdg2Roll.NAV_ROLL_D = 0.01;
% Ctr.Hdg2Roll.IMAX = 5;

% Ctr.Hdg2Roll.f_cut = 20;
% [ Ctr.Hdg2Roll.lowpass.num, Ctr.Hdg2Roll.lowpass.den ] = tfdata(c2d(tf(...
%     2*pi*Ctr.Hdg2Roll.f_cut, [ 1 2*pi*Ctr.Hdg2Roll.f_cut]),Ctr.TS,'tustin'),...
%     'v');

% Dopo taratura
Ctr.Hdg2Roll.NAV_ROLL_P = 0.7;
Ctr.Hdg2Roll.NAV_ROLL_D = 0;


% load controller2
% [Ctr.Hdg2Roll.idealNum, Ctr.Hdg2Roll.idealDen] = tfdata(tf(controller2), 'v');

%% END OF CODE
