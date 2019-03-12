
%%
clear all
clc
close all

%%
% Bixler (Fixed-wing UAV parameters Declaration)
run('parameters_main.m')

%% Parameters Declaration (Ref.Model)
I0 = [0.02 0 -0.01;
    0 0.026 0;
    -0.01 0 0.053];
M0 = 10;


G = 9.80665;
C_rho = 1.225;

%% Wind Initial Condition
InitWind= [0;0;0];


%% Initial Conditions of the reference model
init_pos0 = [0 500 -50];


init_vel0 = [15 0 0];


init_eul0 = [0 0 0];


init_ang0 = [0 0 0];



%% Reference Model inverse dynamics controller (Simple PD controller)
Proportional_x = 50;
Derivative_gain_x = 50;

Proportional_y = 50;
Derivative_gain_y = 50;

Proportional_z = 50;
Derivative_gain_z = 50;

Proportional_phi = 50;
Derivative_gain_phi = 50;

Proportional_theta = 50;
Derivative_gain_theta = 50;

Proportional_psi = 50;
Derivative_gain_psi = 50;

P = [Proportional_x Proportional_y Proportional_z Proportional_phi Proportional_theta Proportional_psi];
Kp = diag(P);

V = [Derivative_gain_x Derivative_gain_y Derivative_gain_z Derivative_gain_phi Derivative_gain_theta Derivative_gain_psi];
Kv = diag(V);


%% Euler-Lagrange Parameters Ref. model
D0 = [M0 0 0 0 0 0;
    0 M0 0 0 0 0;
    0 0 M0 0 0 0;
    0 0 0 I0(1) 0 I0(3);
    0 0 0 0 I0(5) 0;
    0 0 0 I0(7) 0 I0(9)];

g0 = [0;0;-M0*G;0;0;0];







