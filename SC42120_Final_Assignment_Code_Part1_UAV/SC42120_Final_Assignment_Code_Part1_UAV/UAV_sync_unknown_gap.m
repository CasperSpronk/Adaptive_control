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
init_pos2 = [-100, -1000, -100]';
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
%%
t=D1_hat.time;

%% Plots
close all

as = 10;
figure_num = 1;
figure("Name","UAV's trajectories",'NumberTitle','off')
plot3(inertial_pos_node_0(:,2),inertial_pos_node_0(:,1),-inertial_pos_node_0(:,3))
grid on
hold on
plot3(inertial_pos_node_1(:,2),inertial_pos_node_1(:,1),-inertial_pos_node_1(:,3))
plot3(inertial_pos_node_2(:,2),inertial_pos_node_2(:,1),-inertial_pos_node_2(:,3))
plot3(inertial_pos_node_3(:,2),inertial_pos_node_3(:,1),-inertial_pos_node_3(:,3))
plot3(inertial_pos_node_4(:,2),inertial_pos_node_4(:,1),-inertial_pos_node_4(:,3))
legend("Node 0","Node 1","Node 2","Node 3","Node 4")
title("UAV's trajectories")
xlabel({'x location','[m]'})
ylabel({'y location','[m]'})
zlabel({'z location','[m]'})


figure_num = figure_num + 1;
figure("Name","error x with respect to node 0",'NumberTitle','off')
len = length(inertial_pos_node_0);
error_node1 = inertial_pos_node_0(:,2) - inertial_pos_node_1(:,2) + 100 * ones(len,1);
error_node2 = inertial_pos_node_0(:,2) - inertial_pos_node_2(:,2) + 200 * ones(len,1);
error_node3 = inertial_pos_node_0(:,2) - inertial_pos_node_3(:,2) - 100 * ones(len,1);
error_node4 = inertial_pos_node_0(:,2) - inertial_pos_node_4(:,2) - 200 * ones(len,1);
plot(t,error_node1(:,:),'g')
hold on
grid on
plot(t,error_node2(:,:),'r')
plot(t,error_node3(:,:),'b')
plot(t,error_node4(:,:),'y')
title("error x with respect to node 0")
ylim([-as,as])
legend("Node 1","Node 2","Node 3","Node 4")
xlabel({'time','[s]'})
ylabel({'error','[m]'})

figure_num = figure_num + 1;
figure("Name","error y with respect to node 0",'NumberTitle','off')
len = length(inertial_pos_node_0);
error_node1 = inertial_pos_node_0(:,1) - inertial_pos_node_1(:,1) - 50 * ones(len,1);
error_node2 = inertial_pos_node_0(:,1) - inertial_pos_node_2(:,1) - 100 * ones(len,1);
error_node3 = inertial_pos_node_0(:,1) - inertial_pos_node_3(:,1) - 50 * ones(len,1);
error_node4 = inertial_pos_node_0(:,1) - inertial_pos_node_4(:,1) - 100 * ones(len,1);
plot(t,error_node1(:,:),'g')
hold on
grid on
plot(t,error_node2(:,:),'r')
plot(t,error_node3(:,:),'b')
plot(t,error_node4(:,:),'y')
ylim([-as,as])
legend("Node 1","Node 2","Node 3","Node 4")
title("error y with respect to node 0")
xlabel({'time','[s]'})
ylabel({'error','[m]'})


figure_num = figure_num + 1;
figure("Name","error z with respect to node 0",'NumberTitle','off')
len = length(inertial_pos_node_0);
error_node1 = inertial_pos_node_0(:,3) - inertial_pos_node_1(:,3);
error_node2 = inertial_pos_node_0(:,3) - inertial_pos_node_2(:,3);
error_node3 = inertial_pos_node_0(:,3) - inertial_pos_node_3(:,3); 
error_node4 = inertial_pos_node_0(:,3) - inertial_pos_node_4(:,3);
plot(t,error_node1(:,:),'g')
hold on
grid on
plot(t,error_node2(:,:),'r')
plot(t,error_node3(:,:),'b')
plot(t,error_node4(:,:),'y')
ylim([-as,as])
legend("Node 1","Node 2","Node 3","Node 4")
title("error z with respect to node 0")
xlabel({'time','[s]'})
ylabel({'error','[m]'})

figure_num = figure_num + 1;
figure("Name","UAV's trajectories",'NumberTitle','off')
plot(inertial_pos_node_0(:,2),inertial_pos_node_0(:,1))
grid on
hold on
plot(inertial_pos_node_1(:,2),inertial_pos_node_1(:,1))
plot(inertial_pos_node_2(:,2),inertial_pos_node_2(:,1))
plot(inertial_pos_node_3(:,2),inertial_pos_node_3(:,1))
plot(inertial_pos_node_4(:,2),inertial_pos_node_4(:,1))
legend("Node 0","Node 1","Node 2","Node 3","Node 4")
title("UAV's trajectories")
xlabel({'x location','[m]'})
ylabel({'y location','[m]'})
zlabel({'z location','[m]'})

figure_num = figure_num + 1;
figure("Name","UAV's jaw",'NumberTitle','off')
plot(t,angular_pos_node_0(:,1))
grid on
hold on
plot(t,angular_pos_node_1(:,1))
plot(t,angular_pos_node_2(:,1))
plot(t,angular_pos_node_3(:,1))
plot(t,angular_pos_node_4(:,1))
legend("Node 0","Node 1","Node 2","Node 3","Node 4")
title("UAV's jaw angles")
xlabel({'time','[s]'})
ylabel({'jaw','[rad]'})

figure_num = figure_num + 1;
figure("Name","UAV's pitch",'NumberTitle','off')
plot(t,angular_pos_node_0(:,2))
grid on
hold on
plot(t,angular_pos_node_1(:,2))
plot(t,angular_pos_node_2(:,2))
plot(t,angular_pos_node_3(:,2))
plot(t,angular_pos_node_4(:,2))
legend("Node 0","Node 1","Node 2","Node 3","Node 4")
title("UAV's pitch angles")
xlabel({'time','[s]'})
ylabel({'pitch','[rad]'})

figure_num = figure_num + 1;
figure("Name","UAV's roll",'NumberTitle','off')
plot(t,angular_pos_node_0(:,3))
grid on
hold on
plot(t,angular_pos_node_1(:,3))
plot(t,angular_pos_node_2(:,3))
plot(t,angular_pos_node_3(:,3))
plot(t,angular_pos_node_4(:,3))
legend("Node 0","Node 1","Node 2","Node 3","Node 4")
title("UAV's roll angles")
xlabel({'time','[s]'})
ylabel({'roll','[rad]'})

%%  Estimated parameters from \Theta_{D_1}

figure_num = figure_num + 1;
figure(figure_num)
title('Estimate of m1')
xlabel('time [s]')
grid on, hold on
m1=D1_hat.signals.values(1,1,:);
plot(t,m1(:,:),'b')
plot(t,M1*ones(1,length(t)),'g')

%plot(t,m3.signals.values(:,:),'r--')
%plot(t,M3*ones(1,length(t)),'c-.')

figure_num = figure_num + 1;
figure(figure_num)
grid on, hold on
title('\Theta_{D_1}(4,4)')
ix1 = D1_hat.signals.values(4,4,:);
plot(t,ix1(:,:),'b')
plot(t,I1(1,1)*ones(1,length(t)),'g-.')

figure_num = figure_num + 1;
figure(figure_num)
grid on, hold on
title('\Theta_{D_1}(6,6)')
iz1 = D1_hat.signals.values(6,6,:);
plot(t,iz1(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')

%%

figure_num = figure_num + 1;
figure(figure_num)
title('\Theta_{C_1}(4,4)')
xlabel('time [s]')
grid on, hold on
c1_44 = C1_hat.signals.values(4,4,:);
plot(t,c1_44(:,:),'b')
plot(t,I1(1,1)*ones(1,length(t)),'g-.')
%%
figure_num = figure_num + 1;
figure(figure_num)
grid on, hold on
title('\Theta_{C_1}(4,10)')
c1_410 = C1_hat.signals.values(4,10,:);
plot(t,c1_410(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')

figure_num = figure_num + 1;
figure(figure_num)
grid on, hold on
title('\Theta_{C_1}(6,12)')
c1_612 = C1_hat.signals.values(6,12,:);
plot(t,c1_612(:,:),'b')
plot(t,I1(3,3)*ones(1,length(t)),'g-.')





