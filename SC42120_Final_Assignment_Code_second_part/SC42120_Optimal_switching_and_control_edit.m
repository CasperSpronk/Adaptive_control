%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Optimal Switching and Control of Nonlinear Switching Systems Using
%%% Approximate Dynamic Programming 
%%% 
%%% Reference paper: Optimal Switching and Control of Nonlinear Switching Systems Using Approximate Dynamic Programming 
%%% Authors of the paper: Ali Heydari and S.N. Balakrishnan
%%% Journal: Applied Soft Computing, Vol. 24, 2014, pp. 291�303
%%% 
%%% Copyright 2013 by Ali Heydari (heydari.ali@gmail.com)
%%% Author Webpage: http://webpages.sdsmt.edu/~aheydari/
%%%
%%% Refer to the paper and the notations used therein for understanding this code
%%%
%%% Author of the Code: 
%%% Vittorio Giammarino
%%% DSCS, Tu Delft, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;

%Exercise 1
S = [10 0; 0 10]; % matrix to determine the phi final cost 

% The function subject to minimization
% psi is the final cost at t_f
c = 10;
psi = @(S,xtf)(xtf'*S*xtf);             %DONE

%% Systems Dynamics in continuous 

% System 1 dynamics
f_1 = @(x1,x2)[x2;x1^2-x2^2];           %Done
g_1 = [0;1];                            %DONE

% System 2 dynamics
f_2 = @(x1,x2)[x2;-x2];                 %DONE
g_2 = [0;0.5];                          %DONE

f_1 = @(x1,x2)[x2;x1^2-x2^2];           %DONE
g_1 = [0;1];                            %DONE


% Quadratic Lyapunov Design 
Q = eye(2);
R = 1;

n = 2; m = 1; %system order and number of inputs
s = 2; % number of systems 


%% Initialization

dt = 2e-3;  %4e-4; %Sampling time of the discrete system
tf = 3;     %The simulation will be performed from t=0 to this value
t_bar = 1;  %Eq, (18) of the paper, constant switching time in the new time indipendent variable 
t_bar_f = tf;
K = s-1;
N = (K+1)/dt;   %Number of steps during each simulation 

X_k  =  (rand(n,1)*2-1); %Initial states selection
t_switch = (tf*rand(s-1,1));

% Basis functions

% Critic basis
phi = @(x_1, x_2, t_1) [
                          (t_1)^0*(x_1)^2; (t_1)^0*(x_2)^2; (t_1)^0*(x_1)*(x_2); (t_1)^0*(x_1)^3; (t_1)^0*(x_2)^3; (t_1)^0*(x_1)*(x_2)^2; (t_1)^0*(x_1)^2*(x_2); (t_1)^0*(x_1)^4; (t_1)^0*(x_2)^4; (t_1)^0*(x_1)*(x_2)^3; (t_1)^0*(x_1)^3*(x_2); (t_1)^0*(x_1)^2*(x_2)^2;
                          (t_1)^1*(x_1)^2; (t_1)^1*(x_2)^2; (t_1)^1*(x_1)*(x_2); (t_1)^1*(x_1)^3; (t_1)^1*(x_2)^3; (t_1)^1*(x_1)*(x_2)^2; (t_1)^1*(x_1)^2*(x_2); (t_1)^1*(x_1)^4; (t_1)^1*(x_2)^4; (t_1)^1*(x_1)*(x_2)^3; (t_1)^1*(x_1)^3*(x_2); (t_1)^1*(x_1)^2*(x_2)^2;
                          (t_1)^3*(x_1)^2; (t_1)^3*(x_2)^2; (t_1)^3*(x_1)*(x_2); (t_1)^3*(x_1)^3; (t_1)^3*(x_2)^3; (t_1)^3*(x_1)*(x_2)^2; (t_1)^3*(x_1)^2*(x_2); (t_1)^3*(x_1)^4; (t_1)^3*(x_2)^4; (t_1)^3*(x_1)*(x_2)^3; (t_1)^3*(x_1)^3*(x_2); (t_1)^3*(x_1)^2*(x_2)^2;
                          (t_1)^4*(x_1)^2; (t_1)^4*(x_2)^2; (t_1)^4*(x_1)*(x_2); (t_1)^4*(x_1)^3; (t_1)^4*(x_2)^3; (t_1)^4*(x_1)*(x_2)^2; (t_1)^4*(x_1)^2*(x_2); (t_1)^4*(x_1)^4; (t_1)^4*(x_2)^4; (t_1)^4*(x_1)*(x_2)^3; (t_1)^4*(x_1)^3*(x_2); (t_1)^4*(x_1)^2*(x_2)^2;
                          (t_1)^5*(x_1)^2; (t_1)^5*(x_2)^2; (t_1)^5*(x_1)*(x_2); (t_1)^5*(x_1)^3; (t_1)^5*(x_2)^3; (t_1)^5*(x_1)*(x_2)^2; (t_1)^5*(x_1)^2*(x_2); (t_1)^5*(x_1)^4; (t_1)^5*(x_2)^4; (t_1)^5*(x_1)*(x_2)^3; (t_1)^5*(x_1)^3*(x_2); (t_1)^5*(x_1)^2*(x_2)^2;
                          (t_1)^6*(x_1)^2; (t_1)^6*(x_2)^2; (t_1)^6*(x_1)*(x_2); (t_1)^6*(x_1)^3; (t_1)^6*(x_2)^3; (t_1)^6*(x_1)*(x_2)^2; (t_1)^6*(x_1)^2*(x_2); (t_1)^6*(x_1)^4; (t_1)^6*(x_2)^4; (t_1)^6*(x_1)*(x_2)^3; (t_1)^6*(x_1)^3*(x_2); (t_1)^6*(x_1)^2*(x_2)^2;
                          (t_1)^7*(x_1)^2; (t_1)^7*(x_2)^2; (t_1)^7*(x_1)*(x_2); (t_1)^7*(x_1)^3; (t_1)^7*(x_2)^3; (t_1)^7*(x_1)*(x_2)^2; (t_1)^7*(x_1)^2*(x_2); (t_1)^7*(x_1)^4; (t_1)^7*(x_2)^4; (t_1)^7*(x_1)*(x_2)^3; (t_1)^7*(x_1)^3*(x_2); (t_1)^7*(x_1)^2*(x_2)^2;
                          (t_1)^8*(x_1)^2; (t_1)^8*(x_2)^2; (t_1)^8*(x_1)*(x_2); (t_1)^8*(x_1)^3; (t_1)^8*(x_2)^3; (t_1)^8*(x_1)*(x_2)^2; (t_1)^8*(x_1)^2*(x_2); (t_1)^8*(x_1)^4; (t_1)^8*(x_2)^4; (t_1)^8*(x_1)*(x_2)^3; (t_1)^8*(x_1)^3*(x_2); (t_1)^8*(x_1)^2*(x_2)^2;
                       ];  

% Gradient of NN critic basis phi
dphi_dx = @(x_1, x_2, t_1) [(t_1)^0*2*(x_1) 0 (t_1)^0*(x_2) (t_1)^0*3*(x_1)^2 0 (t_1)^0*(x_2)^2 (t_1)^0*2*(x_1)*(x_2) (t_1)^0*4*(x_1)^3 0 (t_1)^0*(x_2)^3 (t_1)^0*3*(x_1)^2*(x_2) (t_1)^0*2*(x_1)*(x_2)^2, ...
                            (t_1)^1*2*(x_1) 0 (t_1)^1*(x_2) (t_1)^1*3*(x_1)^2 0 (t_1)^1*(x_2)^2 (t_1)^1*2*(x_1)*(x_2) (t_1)^1*4*(x_1)^3 0 (t_1)^1*(x_2)^3 (t_1)^1*3*(x_1)^2*(x_2) (t_1)^1*2*(x_1)*(x_2)^2, ...
                            (t_1)^3*2*(x_1) 0 (t_1)^3*(x_2) (t_1)^3*3*(x_1)^2 0 (t_1)^3*(x_2)^2 (t_1)^3*2*(x_1)*(x_2) (t_1)^3*4*(x_1)^3 0 (t_1)^3*(x_2)^3 (t_1)^3*3*(x_1)^2*(x_2) (t_1)^3*2*(x_1)*(x_2)^2, ...
                            (t_1)^4*2*(x_1) 0 (t_1)^4*(x_2) (t_1)^4*3*(x_1)^2 0 (t_1)^4*(x_2)^2 (t_1)^4*2*(x_1)*(x_2) (t_1)^4*4*(x_1)^3 0 (t_1)^4*(x_2)^3 (t_1)^4*3*(x_1)^2*(x_2) (t_1)^4*2*(x_1)*(x_2)^2, ...
                            (t_1)^5*2*(x_1) 0 (t_1)^5*(x_2) (t_1)^5*3*(x_1)^2 0 (t_1)^5*(x_2)^2 (t_1)^5*2*(x_1)*(x_2) (t_1)^5*4*(x_1)^3 0 (t_1)^5*(x_2)^3 (t_1)^5*3*(x_1)^2*(x_2) (t_1)^5*2*(x_1)*(x_2)^2, ...
                            (t_1)^6*2*(x_1) 0 (t_1)^6*(x_2) (t_1)^6*3*(x_1)^2 0 (t_1)^6*(x_2)^2 (t_1)^6*2*(x_1)*(x_2) (t_1)^6*4*(x_1)^3 0 (t_1)^6*(x_2)^3 (t_1)^6*3*(x_1)^2*(x_2) (t_1)^6*2*(x_1)*(x_2)^2, ...
                            (t_1)^7*2*(x_1) 0 (t_1)^7*(x_2) (t_1)^7*3*(x_1)^2 0 (t_1)^7*(x_2)^2 (t_1)^7*2*(x_1)*(x_2) (t_1)^7*4*(x_1)^3 0 (t_1)^7*(x_2)^3 (t_1)^7*3*(x_1)^2*(x_2) (t_1)^7*2*(x_1)*(x_2)^2, ...
                            (t_1)^8*2*(x_1) 0 (t_1)^8*(x_2) (t_1)^8*3*(x_1)^2 0 (t_1)^8*(x_2)^2 (t_1)^8*2*(x_1)*(x_2) (t_1)^8*4*(x_1)^3 0 (t_1)^8*(x_2)^3 (t_1)^8*3*(x_1)^2*(x_2) (t_1)^8*2*(x_1)*(x_2)^2;
                            0 (t_1)^0*2*(x_2) (t_1)^0*(x_1) 0 (t_1)^0*3*(x_2)^2 (t_1)^0*(x_1)*2*(x_2) (t_1)^0*(x_1)^2 0 (t_1)^0*4*(x_2)^3 (t_1)^0*(x_1)*3*(x_2)^2 (t_1)^0*(x_1)^3 (t_1)^0*(x_1)^2*2*(x_2), ...
                            0 (t_1)^1*2*(x_2) (t_1)^1*(x_1) 0 (t_1)^1*3*(x_2)^2 (t_1)^1*(x_1)*2*(x_2) (t_1)^1*(x_1)^2 0 (t_1)^1*4*(x_2)^3 (t_1)^1*(x_1)*3*(x_2)^2 (t_1)^1*(x_1)^3 (t_1)^1*(x_1)^2*2*(x_2), ...
                            0 (t_1)^3*2*(x_2) (t_1)^3*(x_1) 0 (t_1)^3*3*(x_2)^2 (t_1)^3*(x_1)*2*(x_2) (t_1)^3*(x_1)^2 0 (t_1)^3*4*(x_2)^3 (t_1)^3*(x_1)*3*(x_2)^2 (t_1)^3*(x_1)^3 (t_1)^3*(x_1)^2*2*(x_2), ...
                            0 (t_1)^4*2*(x_2) (t_1)^4*(x_1) 0 (t_1)^4*3*(x_2)^2 (t_1)^4*(x_1)*2*(x_2) (t_1)^4*(x_1)^2 0 (t_1)^4*4*(x_2)^3 (t_1)^4*(x_1)*3*(x_2)^2 (t_1)^4*(x_1)^3 (t_1)^4*(x_1)^2*2*(x_2), ...
                            0 (t_1)^5*2*(x_2) (t_1)^5*(x_1) 0 (t_1)^5*3*(x_2)^2 (t_1)^5*(x_1)*2*(x_2) (t_1)^5*(x_1)^2 0 (t_1)^5*4*(x_2)^3 (t_1)^5*(x_1)*3*(x_2)^2 (t_1)^5*(x_1)^3 (t_1)^5*(x_1)^2*2*(x_2), ... 
                            0 (t_1)^6*2*(x_2) (t_1)^6*(x_1) 0 (t_1)^6*3*(x_2)^2 (t_1)^6*(x_1)*2*(x_2) (t_1)^6*(x_1)^2 0 (t_1)^6*4*(x_2)^3 (t_1)^6*(x_1)*3*(x_2)^2 (t_1)^6*(x_1)^3 (t_1)^6*(x_1)^2*2*(x_2), ...
                            0 (t_1)^7*2*(x_2) (t_1)^7*(x_1) 0 (t_1)^7*3*(x_2)^2 (t_1)^7*(x_1)*2*(x_2) (t_1)^7*(x_1)^2 0 (t_1)^7*4*(x_2)^3 (t_1)^7*(x_1)*3*(x_2)^2 (t_1)^7*(x_1)^3 (t_1)^7*(x_1)^2*2*(x_2), ... 
                            0 (t_1)^8*2*(x_2) (t_1)^8*(x_1) 0 (t_1)^8*3*(x_2)^2 (t_1)^8*(x_1)*2*(x_2) (t_1)^8*(x_1)^2 0 (t_1)^8*4*(x_2)^3 (t_1)^8*(x_1)*3*(x_2)^2 (t_1)^8*(x_1)^3 (t_1)^8*(x_1)^2*2*(x_2)]' ;
    
% Actor Basis
sigma = @(x_1, x_2, t_1) [(t_1)^0*(x_1) (t_1)^0*(x_2) (t_1)^0*(x_1)^2 (t_1)^0*(x_2)^2 (t_1)^0*(x_1)*(x_2) (t_1)^0*(x_1)^3 (t_1)^0*(x_2)^3 (t_1)^0*(x_1)*(x_2)^2 (t_1)^0*(x_1)^2*(x_2), ...
                          (t_1)^1*(x_1) (t_1)^1*(x_2) (t_1)^1*(x_1)^2 (t_1)^1*(x_2)^2 (t_1)^1*(x_1)*(x_2) (t_1)^1*(x_1)^3 (t_1)^1*(x_2)^3 (t_1)^1*(x_1)*(x_2)^2 (t_1)^1*(x_1)^2*(x_2), ...
                          (t_1)^3*(x_1) (t_1)^3*(x_2) (t_1)^3*(x_1)^2 (t_1)^3*(x_2)^2 (t_1)^3*(x_1)*(x_2) (t_1)^3*(x_1)^3 (t_1)^3*(x_2)^3 (t_1)^3*(x_1)*(x_2)^2 (t_1)^3*(x_1)^2*(x_2), ... 
                          (t_1)^4*(x_1) (t_1)^4*(x_2) (t_1)^4*(x_1)^2 (t_1)^4*(x_2)^2 (t_1)^4*(x_1)*(x_2) (t_1)^4*(x_1)^3 (t_1)^4*(x_2)^3 (t_1)^4*(x_1)*(x_2)^2 (t_1)^4*(x_1)^2*(x_2), ... 
                          (t_1)^5*(x_1) (t_1)^5*(x_2) (t_1)^5*(x_1)^2 (t_1)^5*(x_2)^2 (t_1)^5*(x_1)*(x_2) (t_1)^5*(x_1)^3 (t_1)^5*(x_2)^3 (t_1)^5*(x_1)*(x_2)^2 (t_1)^5*(x_1)^2*(x_2), ...
                          (t_1)^6*(x_1) (t_1)^6*(x_2) (t_1)^6*(x_1)^2 (t_1)^6*(x_2)^2 (t_1)^6*(x_1)*(x_2) (t_1)^6*(x_1)^3 (t_1)^6*(x_2)^3 (t_1)^6*(x_1)*(x_2)^2 (t_1)^6*(x_1)^2*(x_2), ... 
                          (t_1)^7*(x_1) (t_1)^7*(x_2) (t_1)^7*(x_1)^2 (t_1)^7*(x_2)^2 (t_1)^7*(x_1)*(x_2) (t_1)^7*(x_1)^3 (t_1)^7*(x_2)^3 (t_1)^7*(x_1)*(x_2)^2 (t_1)^7*(x_1)^2*(x_2), ... 
                          (t_1)^8*(x_1) (t_1)^8*(x_2) (t_1)^8*(x_1)^2 (t_1)^8*(x_2)^2 (t_1)^8*(x_1)*(x_2) (t_1)^8*(x_1)^3 (t_1)^8*(x_2)^3 (t_1)^8*(x_1)*(x_2)^2 (t_1)^8*(x_1)^2*(x_2)]'; 

% Initializing memory
FinalW = zeros(length(phi(X_k(1), X_k(2), t_switch)),N);
FinalV = zeros(length(sigma(X_k(1), X_k(2), t_switch)),N); %Scalar control, for now

%% NN Training

MaxEpochNo= 10; % Unlike Algorithm 1 of the paper, where "beta" was defined for evaluating the convergence of "Step 6", we're setting a fixed number of iterations for that successive approximation
NoOfEquations = 500; % Number of sample "x" selected for training using least squares, in the referenced paper this value is called "n"
StateSelectionWidth = 1.25; % the states will be selected from interval (-StateSelectionWidth, StateSelectionWidth)


Q1_bar = dt*Q;                              % Discretized Q1        %DONE                
Q2_bar = dt*Q;                              % Discretized Q2        %DONE
R1_bar = dt*R;                              % Discretized R1        %DONE
R2_bar = dt*R;                              % Discretized R2        %DONE
f_1_bar = @(x1,x2)([x1;x2]+dt*f_1(x1,x2)) ; % Discretized f_1       %DONE
f_2_bar = @(x1,x2)([x1;x2]+dt*f_2(x1,x2));  % Discretized f_2       %DONE
g_1_bar = @(x1,x2)(dt*g_1);                 % Discretized g_1       %DONE
g_2_bar = @(x1,x2)(dt*g_2);                 % Discretized g_2       %DONE


tic
W = zeros(length(phi(X_k(1), X_k(2), t_switch)),N,MaxEpochNo);

% the next matrix stores the "evolution" of the weight of the actor.  
Vi_1 = zeros(length(sigma(X_k(1), X_k(2), t_switch)),N,MaxEpochNo); %Scalar control, for now

% Variables defined for least squares calculation, refer to the Appendix in the paper
RHS_J = zeros(NoOfEquations,1);
RHS_U = zeros(NoOfEquations,m);
LHS_J = zeros(NoOfEquations,length(phi(X_k(1), X_k(2), t_switch)));
LHS_U = zeros(NoOfEquations,length(sigma(X_k(1), X_k(2), t_switch)));

V_k_tolerance = 0.1; % The preset tolerance for evaluating the convergence of the weights of the actor, as used in Step 8 of Algorithm 2

diverged = 0;

% Training process based on Algorithm 1 of the paper
for t = 0:N-1 % time goes from 0 to tf ----> N number of sample 
    k = N - t; %Changing from N to 1
    W(:,k,1) = FinalW(:,k);
    if diverged == 0
        if mod(k,50)==0
            fprintf('Current time = %g\n',k);
        end 
        if k == N % Step 1: least square to initialize the final critic weight W
            for j=1:NoOfEquations                
                X_k =  (rand(n,1)*2-1)* StateSelectionWidth; %Initial states selection
                t_switch = (tf*rand(s-1,1));
                
                J_k_t = psi(S,X_k); %DONE
                % J_k_t is the target. So, W_N'*phi(X_k) is supposed to approximate J_k_t. Let's store them in the following matrices for least squares
                RHS_J(j,:) = J_k_t;
                LHS_J(j,:) = phi(X_k(1), X_k(2), t_switch)';
            end
            if det(LHS_J'*LHS_J)==0
                fprintf('det phi = 0\n');
                break;
            end
            FinalW(:,k) = inv(LHS_J' * LHS_J) * LHS_J' * RHS_J; %DONE
            
        else % Step 2, k=N-1 to k=0
            % Step 3, i=1 and select a guess on V_0
            Vi_1(:,k,1) = FinalV(:,k+1); 
            for i=1:MaxEpochNo-1
                for j = 1:NoOfEquations 
                    % Step 4
                    X_k =  (rand(n,1)*2-1)* StateSelectionWidth; %Initial states selection
                    t_switch = (tf*rand(s-1,1)); %Initial states selection
                    % Step 5
                    temp_U_k = Vi_1(:,k,i)'*sigma(X_k(1), X_k(2), t_switch);
                    % Step 6
                    if k*dt < t_bar 
                        X_k_plus_1 = dt*f_1_bar(X_k(1),X_k(2)) + dt*g_1_bar(X_k(1),X_k(2))*temp_U_k; %DONE
                        U_k = -dt*R1_bar^-1 * dt*g_1_bar(X_k(1),X_k(2))'*dphi_dx(X_k(1),X_k(2),t_switch)'*FinalW(:,k+1);%TODO
                    else
                        X_k_plus_1 = dt*f_2_bar(X_k(1),X_k(2)) + dt*g_2_bar(X_k(1),X_k(2))*temp_U_k; %DONE
                        U_k = -dt*R2_bar^-1 * dt*g_2_bar(X_k(1),X_k(2))'*dphi_dx(X_k(1),X_k(2),t_switch)'*FinalW(:,k+1);%TODO
                    end
                    RHS_U(j,:) = U_k';
                    LHS_U(j,:) = sigma(X_k(1), X_k(2), t_switch)';
                end
                if det(LHS_U'*LHS_U)==0
                    fprintf('det sigma = 0\n');
                    break;
                end
                % Step 7
                Vi_1(:,k,i+1) = inv(LHS_U' * LHS_U) * LHS_U' * RHS_U;%DONE
                
                % Step 8
                % Check to see if the weights of the actors in fixed point
                % iterations has convereged
                if norm (Vi_1(:,k,i+1) - Vi_1(:,k,i)) < V_k_tolerance
                     if (i+1) < MaxEpochNo
                            Vi_1(:,k,i+2:end) = NaN(length(sigma(X_k(1), X_k(2), t_switch)),MaxEpochNo-(i+1));                          
                     end
                     break;
                end
            end
            %Store the result of V, Step 9
            FinalV(:,k) = Vi_1(:,k,i+1);
             
            % Step 10
            for j= 1:NoOfEquations
                X_k =  (rand(n,1)*2-1)* StateSelectionWidth; %Initial states selection
                t_switch = (tf*rand(s-1,1)); %Initial states selection
                temp_U_k = FinalV(:,k)'*sigma(X_k(1), X_k(2), t_switch);
                if k*dt < t_bar 
                   X_k_plus_1 = X_k + dt*f_1_bar(X_k(1),X_k(2)) + dt*g_1_bar(X_k(1),X_k(2)) * temp_U_k; %DONE
                   J_k_plus_1 = FinalW(:,k+1)' * phi(X_k_plus_1(1),X_k_plus_1(2),t_switch); %DONE
                   J_k_t = 0.5*dt*Q1_bar(1,1)+0.5*sigma(X_k(1), X_k(2), t_switch)'*FinalV(:,k)*dt*R1_bar*FinalV(:,k)'*sigma(X_k(1), X_k(2), t_switch)+J_k_plus_1;%TODO
                else
                   X_k_plus_1 =  X_k + dt*f_2_bar(X_k(1),X_k(2)) + dt*g_2_bar(X_k(1),X_k(2)) * temp_U_k;%DONE
                   J_k_plus_1 = FinalW(:,k+1)' * phi(X_k_plus_1(1),X_k_plus_1(2),t_switch);%DONE
                   J_k_t = 0.5*dt*Q2_bar(2,2)+0.5*sigma(X_k(1), X_k(2), t_switch)'*FinalV(:,k)*dt*R2_bar*FinalV(:,k)'*sigma(X_k(1), X_k(2), t_switch)+J_k_plus_1;%TODO
                end
                RHS_J(j,:) = J_k_t;
                LHS_J(j,:) = phi(X_k(1), X_k(2), t_switch)';
            end
            if det(LHS_J'*LHS_J)==0
                fprintf('det phi = 0\n');
                break;
            end
            FinalW(:,k) = inv(LHS_J' * LHS_J) * LHS_J' * RHS_J;%DONE
            if isnan(FinalW(:,k))
                fprintf('Training W is diverging...\n');
                diverged = 1;
                break;
            end
        end
    else
        fprint('Diverged\n');
    end
end
toc
 
%% plot cost function 

x0 = [-1 0.5]';
%x0 = [0 -1]';

dt_switch = 0.001;

t_switch = 0:dt_switch:tf;
J0 = zeros(1,length(t_switch));

for j=1:length(t_switch)
J0(1,j) = FinalW(:,1)'*phi(x0(1),x0(2),t_switch(1,j));
end

[x,t] = min(J0);
l = t*dt_switch;

figure()
plot(t_switch, J0)
xlabel('t_{switch}')
ylabel('J0')

[M,I]=min(J0(1,1:end));
t_1 = I*dt_switch;
Xd_k = zeros(n,N);
Ud_k = zeros(1,N);
Xd_k(:,1) = x0;


time = dt:dt:2;


for k = 1:N-1
    
    if k*dt < 1
    u_k = FinalV(:,k)'*sigma(Xd_k(1,k),Xd_k(2,k),t_1);
    Ud_k(:,k)=u_k;
    Xd_k(:,k+1) = Xd_k(:,k) + dt*f_1(Xd_k(1,k),Xd_k(2,k)) + dt*g_1*u_k;
    else
    u_k = FinalV(:,k)'*sigma(Xd_k(1,k),Xd_k(2,k),t_1);
    Ud_k(:,k)=u_k;
    Xd_k(:,k+1) = Xd_k(:,k) + dt*f_2(Xd_k(1,k),Xd_k(2,k)) + dt*g_2*u_k;
    end
    
end

figure()
plot(time, Xd_k(1,:), 'r', time, Xd_k(2,:), 'k')
xlabel('time')
ylabel('X')
legend('x_1','x_2')
title('states with time transformed')

%transformation to indipendent time variable to real time 
t=zeros(1,N);
for k = 1:N
    if k*dt < 1
        t(1,k)= t_1*k*dt;
    else
        t(1,k)=t_1 + (tf-t_1)*(k*dt-1);
    end
end

figure()
plot(t, Xd_k(1,:), 'r', t, Xd_k(2,:), 'k')
xlabel('time')
ylabel('X')
legend('x_1','x_2')
title('states with real time')

samples = 1:N;

Conv_check=zeros(N,MaxEpochNo-1);
for k = 1:N
    for i=1:MaxEpochNo-1
        Conv_check(k,i) = norm (Vi_1(:,k,i+1)-Vi_1(:,k,i));
    end
end

figure()
plot(samples, Conv_check(:,1))
xlabel('samples')
ylabel('convergence')

