%% Adaptive Vector Field parameters
% Selector
Guid.Strategy = 0;                      % 1 for SL, 0 for Orbit
Guid.Method = 0;                        % 0=APF, 1=Standard, 2=Ideal

%% %%%%%%%%%%%%%%%% GENERAL %%%%%%%%%%%%%%
% Wind
Guid.Wind.W = sqrt(Init.windConst(1)^2+Init.windConst(2)^2);    % Constant wind amplitude [m/s]
Guid.Wind.phiw = deg2rad(Init.windAngle);   % Constant wind direction [rad]

% Alpha
Guid.Est.alpha = Ctr.Hdg2Roll.NAV_ROLL_P * C.g / Guid.Course2Roll.SteadyVG;

%Vg0
% Guid.Vg0 = sqrt (Init.uvw(1)^2 + Init.uvw(2)^2 );
Guid.EstMax = 25;                       % Max possible value for the Vg estimation [m/s]

%% %%%%%%%%%%%%%%%% STRAIGHT LINE %%%%%%%%
Guid.SL.Line_Slope = [1 0];             % Reference line versors [m]
                                        % (only if pathmanager not active)
Guid.SL.b = 50;                      
Guid.SL.Line_Origin = [0 Guid.SL.b];    % Reference line origin [m]
                                        % (only if pathmanager not active)

% VF
Guid.SL.chi_inf = deg2rad(90);          % Course angle far away from path (rad)
Guid.SL.k = 0.1;                        % Positive constant influence the rate of the transition from
                                        % x_inf to zero, also control the slope of the
                                        % sliding surface near the origin(m^-1)
Guid.SL.kk = pi/2;                      % Gain parameter controls the shape of the trajectories onto
                                        % the sliding surface.(rad^2/s)
Guid.SL.eps = 1;                        % Width of the transition region around the sliding surface
                                        % which is used to reduce chattering in the control.(rad)

% Estimator
Guid.SL.gamma = 0.5;                    % Estimator gain for straight line
Guid.SL.sigma = 0;                      % Gain of the sigma modification

% Init. cond.
Guid.SL.chi_q_init = atan2(Guid.SL.Line_Slope(2), Guid.SL.Line_Slope(1));    
                                        % Initial Course of the straight line [rad]
Guid.SL.epy = -sin(Guid.SL.chi_q_init)*(Init.xyz(1)-Guid.SL.Line_Origin(1))...
    + cos(Guid.SL.chi_q_init)*(Init.xyz(2)-Guid.SL.Line_Origin(2));
                                        % Initial error

%% %%%%%%%%%%%%%%%% LOITER %%%%%%%%
Guid.Loiter.R = 50;                     % R_min = 25 [m]
Guid.Loiter.C = [250 500];               % Orbt center [m] 
                                        % (only of pathmanager not active)
Guid.Loiter.j = -1;                      % Counterclockwise (-1), clockwise (+1)
                                        % (only of pathmanager not active)

% VF
% Guid.Loiter.k = 0.2650;                 % Optimized
Guid.Loiter.kk = pi/2;                  % Optimised
% Guid.Loiter.eps = 1.5;                  % Optimised

Guid.Loiter.k = 0.1;
Guid.Loiter.eps = 1;

% Estimator
Guid.Loiter.sigma = 0;
Guid.Loiter.gamma = 0.1;

% Init. cond.
Guid.Loiter.d_int = sqrt((Init.xyz(1)-Guid.Loiter.C(1))^2 + (Init.xyz(2)-Guid.Loiter.C(2))^2)-0*Guid.Loiter.R;
Guid.Loiter.gamma_int = atan2(Init.xyz(2)-Guid.Loiter.C(2),Init.xyz(1)-Guid.Loiter.C(1));
