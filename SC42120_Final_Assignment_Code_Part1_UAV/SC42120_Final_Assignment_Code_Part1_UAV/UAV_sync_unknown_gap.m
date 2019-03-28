clc
clear all
close all
%%
run SC42120_UAV_Preconditions.m

%% setup agent 1
M1 = 20;
% initial condition [x,y,z,phi,thetha,zi]'
init_pos1 = [-50, 750, -75]';
init_eul1 = [0.5, 0.05, 0.5]';
% initial condition [u,v,w,p,q,r]'
init_vel1 = [25, 0, 0]';
init_ang1 = [0, 0, 0]';
% moment
I1 = [0.1 0 -0.01;
      0 0.05 0;
      -0.01 0 0.1];
D1 = [eye(3)*M1 zeros(3,3);
      zeros(3,3) I1];
g1 = [0;0;-M1*G;0;0;0];

  
%% setup agent 2
M2 = 30;
% initial condition [x,y,z,phi,thetha,psi]'
init_pos2 = [-100, 1000, -100]';
init_eul2 = [1, 0.1, 1]';
% initial condition [u,v,w,p,q,r]'
init_vel2 = [5, 0, 0]';
init_ang2 = [0, 0, 0]';
% moment
I2 = 2*[0.1 0 -0.01;
      0 0.05 0;
      -0.01 0 0.1];
D2 = [eye(3)*M2 zeros(3,3);
      zeros(3,3) I2];
g2 = [0;0;-M2*G;0;0;0];

%% setup agent 3 
M3 = 40;
% initial condition [x,y,z,phi,thetha,psi]'
init_pos3 = [-50, 250, -150]';
init_eul3 = [-0.5, -0.05, -0.5]';
% initial condition [u,v,w,p,q,r]'
init_vel3 = [20, 0, 0]';
init_ang3 = [0, 0, 0]';
% moment
I3 = 4*[0.1 0 -0.01;
      0 0.05 0;
      -0.01 0 0.1];
D3 = [eye(3)*M3 zeros(3,3);
      zeros(3,3) I3];
g3 = [0;0;-M3*G;0;0;0];


%% setup agent 4
M4 = 50;
% initial condition [x,y,z,phi,thetha,psi]'
init_pos4 = [-100, 0, -200]';
init_eul4 = [-1, -0.1, -1]';
% initial condition [u,v,w,p,q,r]'
init_vel4 = [10, 0, 0]';
init_ang4 = [0, 0, 0]';
% moment
I4 = 8*[0.1 0 -0.01;
      0 0.05 0;
      -0.01 0 0.1];
D4 = [eye(3)*M4 zeros(3,3);
      zeros(3,3) I4];
g4 = [0;0;-M4*G;0;0;0];
%% Adaptive part

Am = [zeros(6,6) eye(6); -Kp -Kv];
Bm = [zeros(6,6); eye(6)];
Q = 20*eye(12);
P = lyap(Am',Q);

%% Gaps
y_gap = 100;
x_gap = 50;
%% Simulate
tic
sim('UAV_sync_unknown_gap_sim.slx');
runtime=toc
%% Plots
figure(1)
title("UAV's trajectories")
plot3(inertial_pos_node_0.Data(:,2),inertial_pos_node_0.Data(:,1),-inertial_pos_node_0.Data(:,3))
grid on
hold on
plot3(inertial_pos_node_1.Data(:,2),inertial_pos_node_1.Data(:,1),-inertial_pos_node_1.Data(:,3))
plot3(inertial_pos_node_2.Data(:,2),inertial_pos_node_2.Data(:,1),-inertial_pos_node_2.Data(:,3))
plot3(inertial_pos_node_3.Data(:,2),inertial_pos_node_3.Data(:,1),-inertial_pos_node_3.Data(:,3))
plot3(inertial_pos_node_4.Data(:,2),inertial_pos_node_4.Data(:,1),-inertial_pos_node_4.Data(:,3))

t=g1_11.time;
%%  Estimated parameters from \Theta_{D_1}
figure(2)
title('Estimate of m1')
xlabel('time [s]')
grid on, hold on
plot(t,m1.signals.values(:,:),'b')
plot(t,M1*ones(1,length(t)),'g-.')
%plot(t,m3.signals.values(:,:),'r--')
%plot(t,M3*ones(1,length(t)),'c-.')

figure(3)
grid on, hold on
title('\Theta_{D_1}(4,4)')
plot(t,Ix1.signals.values(:,:),'b')
plot(t,I1(1,1)*ones(1,length(t)),'g-.')

figure(4)
grid on, hold on
title('\Theta_{D_1}(6,6)')
plot(t,Iz1.signals.values(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')

%%

figure(5)
title('\Theta_{C_1}(4,4)')
xlabel('time [s]')
grid on, hold on
plot(t,c1_44.signals.values(:,:),'b')
plot(t,I1(1,1)*ones(1,length(t)),'g-.')

figure(6)
grid on, hold on
title('\Theta_{C_1}(4,10)')
plot(t,c1_410.signals.values(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')

figure(7)
grid on, hold on
title('\Theta_{C_1}(6,12)')
plot(t,c1_612.signals.values(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')





