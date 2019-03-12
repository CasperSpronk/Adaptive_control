%% %%%%%%%%% Kinematic initial conditions %%%%%%%%%%%%%%

Init.xyz = [0 490 -50];                 % Initial position (inertial) [m]
Init.uvw = [15 0 0];                    % Initial velocity (inertial) [m/s]
Init.attitude(1) = 0;                   % Initial roll angle [deg]
Init.attitude(2) = 0;                   % Initial pitch angle [deg]
Init.attitude(3) = wrapTo180(-360);     % Initial yaw angle [deg]
Init.pqr = [0 0 0];                     % Initial angular rates (p,q,r) [deg/s]

Guid.Course2Roll.SteadyVG = ...
    hypot(Init.uvw(1), Init.uvw(2));    % Target ground speed [m/s]

Aero.AirspeedRef = Init.uvw(1);         % Target Airspeed [m/s]
Aero.HeightRef = 50;                    % Target height [m]

%%%%%%%%%%%%%%  WIND  %%%%%%%%%%%
Init.windAmp = 0;                       % Wind amplitude (inertial NED) [m/s]
Init.windAngle = 240;                   % Wind angle (inertial NED) [deg]
Init.windConst = Init.windAmp*[cosd(Init.windAngle),...
                               sind(Init.windAngle),...
                               0];      % Constant wind (inertial NED) [m/s]
Init.DryOnOff = 0;                      % Turn on or off Dryden turbulences [0,1]
Init.WTimevar = 1;                      % Turn on or off slowly time-varying wind [0,1]

%% Common constants

C.g = 9.81;                             % Gravity acceleration [m/s^2]
C.rho = 1.225;                          % Air density [kg/m^3]

%% Common properties of the aircraft
    
Bxl.mass = 1;                           % Total mass [kg]
% Bxl.J = [0.028  0           0;
%          0      0.026       0;
%          0      0           0.053];     % Inertia tensor [kg*m^2]
     
Bxl.J = [0.02   0           0;
         0      0.026       0;
         0      0           0.053];     % Inertia tensor [mg*m^2]
     
Bxl.S = 0.228;                          % Planform area of the wing [m^2]   (from DATCOM estimate) (OK wrt datasheet)
Bxl.b = 1.31;                           % Wingspan [m]                      (from DATCOM estimate) (OK wrt datasheet)
Bxl.c = 0.175;                          % Mean chord of the wing [m]        (from DATCOM estimate) (OK wrt datasheet)
Bxl.COG = [0, 0, 0];                    % Distance of the center of gravity from the body axes [m]
Bxl.propDist = 0.06;                    % Approx. distance of the COG from the propeller force direction [m]
Bxl.k_motor = 11.39;                    % Motor constant { Ome = k*d_t + q } [rad/s]
Bxl.q_motor = 239;                      % Motor constant { Ome = k*d_t + q } [rad/s]
Bxl.r_prop = 0.1;                       % Propeller radius [m]
Bxl.S_prop = Bxl.r_prop^2*pi;           % Area swept by the propeller [m^2]
Bxl.rotor_thrust_coeff = 0.12;          % Rotor thrust coefficient [-]

%% DATCOM check
% Draw the aircraft (comment this section to speed up)
% 
% datcom_input = 'datcom/bixler_total/Bixler_total.dcm';
% drawDATCOMaircraft(datcom_input);


%% Aerodynamics (import from DATCOM)

Aero.alpha_vector = [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,...
             8.0, 9.0, 10.0, 12.0, 14.0, 16.0, 18.0, 19.0, 20.0, 25.0];
Aero.delta_elev_vector = [-20.0,-10.0,-5.0,-1.0,0.0,1.0,5.0,10.0,20.0];
Aero.delta_ail_vector = [0.0,1.0,3.0,5.0,8.0,10.0,15.0,20.0,25.0];
Aero.mach = 0.03;

% Define DATCOM+ CSV file where to find the coefficients
outputCSV = 'datcom/bixler_total/Bixler_total.csv';

%%%%%%%%%%%%%%  CD  %%%%%%%%%%%%%%
Aero.CD_Basic = vectorParser(outputCSV,5,24);                % CD

%%%%%%%%%%%%%%  Cy  %%%%%%%%%%%%%%
Aero.Cy_Beta =  vectorParser(outputCSV,71,71);               % Cyb
Aero.Cy_RollRate = vectorParser(outputCSV,313,332);          % Cyp

%%%%%%%%%%%%%%  CL  %%%%%%%%%%%%%%      
Aero.CL_Basic = vectorParser(outputCSV,27,46);               % CL
Aero.CL_PitchRate = vectorParser(outputCSV,203,203);         % CL pitch rate
%CL_AlphaDot = vectorParser(outputCSV,247,266);              % CL_alphadot

%%%%%%%%%%%%%%  Cl (Roll)  %%%%%%%
Aero.Cl_Beta = vectorParser(outputCSV,115,134);
Aero.Cl_Beta = vectorParser(outputCSV,115,134) .*0.8;        % Clb
Aero.Cl_RollRate = vectorParser(outputCSV,291,310);
Aero.Cl_RollRate = vectorParser(outputCSV,291,310) .* 1.5;   % Clp
Aero.Cl_YawRate = vectorParser(outputCSV,379,398);           % Clr
                                    
%%%%%%%%%%%%%%  Cm (pitch)  %%%%%%%
Aero.Cm_Basic = vectorParser(outputCSV,49,68);               % Cm
Aero.Cm_PitchRate = vectorParser(outputCSV,225,225);         % Cmq
%Cm_AlphaDot = vectorParser(outputCSV,269,288);              % Cmad

%%%%%%%%%%%%%%  Cn (yaw)  %%%%%%%%%
Aero.Cn_Beta = vectorParser(outputCSV,93,93);                % Cnb
Aero.Cn_RollRate = vectorParser(outputCSV,335,354);          % Cnp
Aero.Cn_YawRate = vectorParser(outputCSV,357,376);           % Cnr

                          
%%%%%%%%%%%%%%  AILERONS  %%%%%%%%%
Aero.Cn_ail = matrixParser(outputCSV,457,476);               % Cn_ail
Aero.Cl_ail = vectorParser(outputCSV,480,488);               % Cl_ail                        

%%%%%%%%%%%%%%  ELEVATOR  %%%%%%%%%
Aero.CD_elev = matrixParser(outputCSV,496,515);              % CD_deltaE
Aero.CL_elev = vectorParser(outputCSV,523,523);              % CL_deltaE
Aero.Cm_elev = vectorParser(outputCSV,534,534);              % Cm_deltaE

%%%%%%%%%%%%%%  RUDDER  %%%%%%%%%%%
% Taken from Bixler 2 experimental results
Aero.Cy_rudd = 0.001570;
Aero.Cl_rudd = 0.000055;
Aero.Cn_rudd = -0.000567;

%%%%%%%%%%%%%%  PROPELLER  %%%%%%%%%
% Experimental data
Aero.thrust_vect = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]; % Adimentional
Aero.thrust_force_vect = C.g.*[0, 0.039, 0.057, 0.098, 0.137,...
    0.181,0.223, 0.263, 0.3, 0.4, 0.58];
Aero.thrust_freq = [0, 50, 130, 180, 237, 280, 318, 345, 370, 400, 450];

clear outputCSV

%% Draw polar curves (comment this section to speed up)
% 
% figure;
% plot(Aero.alpha_vector,Aero.CL_Basic)
% grid;xlabel('Angle of Attack (deg)');ylabel('CL');title('Lift Curve');
% 
% figure;
% plot(Aero.alpha_vector,Aero.CD_Basic)
% grid;xlabel('Angle of Attack (deg)');ylabel('CD');title('Drag Curve');
% 
% figure;
% plot(Aero.alpha_vector,Aero.Cm_Basic)
% grid;xlabel('Angle of Attack (deg)');ylabel('Cm');title('Pitch Curve');
% 
% figure;
% plot(Aero.CD_Basic,Aero.CL_Basic)
% grid;xlabel('CD');ylabel('CL');title('CL vs CD');
% 
% %% Motor thrust
% figure
% plot(Aero.thrust_vect, Aero.thrust_force_vect)
% grid minor; xlabel('\delta_t (%)'); ylabel('Thrust (N)');
% title('Motor thrust characteristic');

