Ctr.TECS.TS = 1/50;                 % [Hz]

% Param: CLMB_MAX
% DisplayName: Maximum Climb Rate (metres/sec)
% Description: This is the best climb rate that the aircraft can achieve with the throttle set to THR_MAX and the airspeed set to the default value. 
% For electric aircraft make sure this number can be achieved towards the end of flight when the battery voltage has reduced. The setting of this 
% parameter can be checked by commanding a positive altitude change of 100m in loiter, RTL or guided mode. If the throttle required to climb is close to 
% THR_MAX and the aircraft is maintaining airspeed, then this parameter is set correctly. If the airspeed starts to reduce, then the parameter is set
% to high, and if the throttle demand require to climb and maintain speed is noticeably less than THR_MAX, then either CLMB_MAX should be increased or THR_MAX reduced.
% Increment: 0.1
% Range: 0.1 20.0
% User: Standard
Ctr.TECS.CLMB_MAX = 5;

% Param: SINK_MIN
% DisplayName: Minimum Sink Rate (metres/sec)
% Description: This is the sink rate of the aircraft with the throttle set to THR_MIN and the same airspeed as used to measure CLMB_MAX.
% Increment: 0.1
% Range: 0.1 10.0
% User: Standard
Ctr.TECS.SINK_MIN = 2;

% Param: TIME_CONST
% DisplayName: Controller time constant (sec)
% Description: This is the time constant of the TECS control algorithm. Smaller values make it faster to respond, large values make it slower to respond.
% Range: 3.0 10.0
% Increment: 0.2
% User: Advanced
Ctr.TECS.TIME_CONST = 5;

% Param: THR_DAMP
% DisplayName: Controller throttle damping
% Description: This is the damping gain for the throttle demand loop. Increase to add damping  to correct for oscillations in speed and height.
% Range: 0.1 1.0
% Increment: 0.1
% User: Advanced
Ctr.TECS.THR_DAMP = 0.5;

% Param: INTEG_GAIN
% DisplayName: Controller integrator
% Description: This is the integrator gain on the control loop. Increase to increase the rate at which speed and height offsets are trimmed out
% Range: 0.0 0.5
% Increment: 0.02
% User: Advanced
Ctr.TECS.INTEG_GAIN = 0.1;

% Param: VERT_ACC
% DisplayName: Vertical Acceleration Limit (metres/sec^2)
% Description: This is the maximum vertical acceleration either up or down that the  controller will use to correct speed or height errors.
% Range: 1.0 10.0
% Increment: 0.5
% User: Advanced
Ctr.TECS.VERT_ACC = 7;

% Param: HGT_OMEGA
% DisplayName: Height complementary filter frequency (radians/sec)
% Description: This is the cross-over frequency of the complementary filter used to fuse vertical acceleration and baro alt to obtain an estimate of height rate and height.
% Range: 1.0 5.0
% Increment: 0.05
% User: Advanced
Ctr.TECS.HGT_OMEGA = 3;

% Param: SPD_OMEGA
% DisplayName: Speed complementary filter frequency (radians/sec)
% Description: This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.
% Range: 0.5 2.0
% Increment: 0.05
% User: Advanced
Ctr.TECS.SPD_OMEGA = 2;

% Param: RLL2THR
% DisplayName: Bank angle compensation gain
% Description: Increasing this gain turn increases the amount of throttle that will be used to compensate for the additional drag created by turning. Ideally this should be set to approximately 10 x the extra sink rate in m/s created by a 45 degree bank turn. Increase this gain if the aircraft initially loses energy in turns and reduce if the aircraft initially gains energy in turns. Efficient high aspect-ratio aircraft (eg powered sailplanes) can use a lower value, whereas inefficient low aspect-ratio models (eg delta wings) can use a higher value.
% Range: 5.0 30.0
% Increment: 1.0
% User: Advanced
Ctr.TECS.RLL2THR = 10;

% Param: SPDWEIGHT
% DisplayName: Weighting applied to speed control
% Description: This parameter adjusts the amount of weighting that the pitch control applies to speed vs height errors. Setting it to 0.0 will cause the pitch control to control height and ignore speed errors. This will normally improve height accuracy but give larger airspeed errors. Setting it to 2.0 will cause the pitch control loop to control speed and ignore height errors. This will normally reduce airsped errors, but give larger height errors.	A value of 1.0 gives a balanced response and is the default.
% Range: 0.0 2.0
% Increment: 0.1
% User: Advanced
Ctr.TECS.SPDWEIGHT = 1;

% Param: PTCH_DAMP
% DisplayName: Controller pitch damping
% Description: This is the damping gain for the pitch demand loop. Increase to add damping  to correct for oscillations in speed and height.
% Range: 0.1 1.0
% Increment: 0.1
% User: Advanced
Ctr.TECS.PTCH_DAMP = 0;

% Param: SINK_MAX
% DisplayName: Maximum Descent Rate (metres/sec)
% Description: This sets the maximum descent rate that the controller will use.  If this value is too large, the aircraft will reach the pitch angle limit first and be unable to achieve the descent rate. This should be set to a value that can be achieved at the lower pitch angle limit.
% Increment: 0.1
% Range: 0.0 20.0
% User: User
Ctr.TECS.SINK_MAX = 5;

% Param: PITCH_MAX
% DisplayName: Maximum pitch in auto flight
% Description: This controls maximum pitch up in automatic throttle modes. If this is set to zero then LIM_PITCH_MAX is used instead. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode.
% Range: 0 45
% Increment: 1
% User: Advanced
Ctr.TECS.PITCH_MAX = 20;

% Param: PITCH_MIN
% DisplayName: Minimum pitch in auto flight
% Description: This controls minimum pitch in automatic throttle modes. If this is set to zero then LIM_PITCH_MIN is used instead. The purpose of this parameter is to allow the use of a smaller pitch range when in automatic flight than what is used in FBWA mode. Note that TECS_PITCH_MIN should be a negative number.
% Range: -45 0
% Increment: 1
% User: Advanced
Ctr.TECS.PITCH_MIN = -20;

Ctr.TECS.STEdot_max = Ctr.TECS.CLMB_MAX * C.g;
Ctr.TECS.STEdot_min = Ctr.TECS.SINK_MIN * C.g;
Ctr.TECS.K_STE2Thr = 1 / ((Ctr.TECS.STEdot_max-Ctr.TECS.STEdot_min) * max(0.1,Ctr.TECS.TIME_CONST));

Ctr.TECS.throttle_cruise = 45;