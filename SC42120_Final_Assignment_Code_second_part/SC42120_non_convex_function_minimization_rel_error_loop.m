%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Single Variable Non-convex Function Minimization
%%% 
%%% Source code for paper: Global optimality of approximate dynamic programming and its usein non-convex function minimization
%%% Authors: Ali Heydari and S.N. Balakrishnan
%%% Journal: Applied Soft Computing, Vol. 24, 2014, pp. 291???303
%%% 
%%% Copyright 2013 by Ali Heydari (heydari.ali@gmail.com)
%%% Author Webpage: http://webpages.sdsmt.edu/~aheydari/
%%%
%%% Refer to the paper and the notations used therein for understanding this code
%%%
%%% Author of the code: 
%%% Vittorio Giammarino
%%% DSCS, Tu Delft, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%% Initialization
for h=1:10
% The function subject to minimization
c = 100;
psi = @(x) c*[2.1333*x.^4 + 0.9333*x.^3 - 2.1333*x.^2 - 0.9333*x + 1]; % A non-convex psi to optimze

dt = 1e-4; %Sampling time of the discreet system
tf =  1; %The simulation will be performed from t=0 to this value
N = tf/dt; %Number of steps during each simulation 

% System dynamics
f = @(x) [0];
g = [1];

Q = @(x) [0*dt*x^2];
R = 1;

n = 1; m = 1; %system order and number of inputs

X_k =  (rand(n,1)*2-1); %Initial states selection

% Basis functions
phi = @(x) [1 x x.^2 x.^3 x.^4 x.^5 x.^6 x.^7]';
dphi_dx = @(x) [0 1 2*x 3*x.^2 4*x.^3 5*x.^4 6*x.^5 7*x.^6]';
sigma = @(x) [1 x x.^2 x.^3 x.^4 x.^5 x.^6]';

% Initializing memory
FinalW = zeros(length(phi(X_k)),N);
FinalV = zeros(length(sigma(X_k)),N); %Scalar control, for now

%% NN Training

MaxEpochNo= 3; % Unlike Algorithm 1 of the paper, where "beta" was defined for evaluating the convergence of "Step 6", we're setting a fixed number of iterations for that successive approximation
NoOfEquations = 100; % Number of sample "x" selected for training using least squares
StateSelectionWidth = 2; % the states will be selected from interval (-StateSelectionWidth, StateSelectionWidth)

f_bar = @(x) dt*f(x); % Discretized R
g_bar = dt*g; % Discretized g
Q_bar =@(x) dt*Q(x); %Discretized Q
R_bar = dt*R; % Discretized R

tic
W = zeros(length(phi(X_k)),N,MaxEpochNo);
V = zeros(length(sigma(X_k)),N,MaxEpochNo); %Scalar control, for now

% Variables defined for least squares calculation, refer to the Appendix in the paper
RHS_J = zeros(NoOfEquations,1); %Right Half Square J
RHS_U = zeros(NoOfEquations,m); %Right Half Square U
LHS_J = zeros(NoOfEquations,length(phi(X_k))); %Left Half Square J
LHS_U = zeros(NoOfEquations,length(sigma(X_k))); %Left Half Square U

diverged = 0;

% Training process based on Algorithm 1 of the paper
for t = 0:N-1
    k = N - t; %Changing from N to 1
    W(:,k,1) = FinalW(:,k); V(:,k,1) = FinalV(:,k);
    if diverged == 0
        if mod(k,50)==0
            %fprintf('Current time = %g\n',k);
        end 
        if k == N % Step 2
            for i=1:NoOfEquations
                X_k =  (rand(1,1)*2-1) * StateSelectionWidth; %Initial states selection
                J_k_t = psi(X_k);
                RHS_J(i,:) = J_k_t;
                LHS_J(i,:) = phi(X_k)';
                RHS_U(i,:) = zeros(m,1)'; %Just to assign some values
                LHS_U(i,:) = sigma(X_k)';
            end
            if det(LHS_J'*LHS_J)==0
                fprintf('det phi = 0\n');
                break;
            end
            FinalW(:,k) = inv(LHS_J' * LHS_J) * LHS_J' * RHS_J;%DONE
            
        else % Step 3
            for i=1:NoOfEquations % Step 4
                U_k(i) = 0;
                % Step 5
                X_k =  (rand(1,1)*2-1) * StateSelectionWidth; %Initial states selection
                
                % Steps 6 and 7 of Algorithm 1 (conducting a fixed number
                % of iterations instead of using a convergence tolerance
                % beta
                % like in the paper)
                for j = 1:MaxEpochNo-1 
                    %disp("here")
                    X_k_plus_1 = X_k + f_bar(X_k) + g_bar * U_k(i);%DONE
                    %disp("X_k = " + X_k_plus_1)
                    U_k(i) = -0.5 * R_bar^-1 * g_bar * dphi_dx(X_k_plus_1)' * FinalW(:,k+1);%DONE
                    %disp("U_k(i) = "+ U_k(i))
                end
                RHS_U(i,:) = U_k(i)';
                LHS_U(i,:) = sigma(X_k)';
                % Generate target for updating W
                X_k_plus_1 = X_k + f_bar(X_k) + g_bar * U_k(i);%DONE
                J_k_plus_1 = FinalW(:,k+1)' * phi(X_k_plus_1);%DONE
                J_k_t = Q_bar(X_k) + U_k(i)' * R_bar * U_k(i) + J_k_plus_1;%DONE
                RHS_J(i,:) = J_k_t;
                LHS_J(i,:) = phi(X_k)';
            end
            if det(LHS_U'*LHS_U)==0
                fprintf('det sigma = 0\n');
                break;
            end
            % Step 8
            FinalV(:,k) = inv(LHS_U' * LHS_U) * LHS_U' * RHS_U;%pinv(sigma(X_k)) * U_k(i);%DONE

            if det(LHS_J'*LHS_J)==0
                fprintf('det phi = 0\n');
                break;
            end
            % Step 9   
            FinalW(:,k) = inv(LHS_J' * LHS_J) * LHS_J' * RHS_J;%DONE
            %Q_bar(X_k) + U_k(i)' * R_bar * U_k(i) + FinalW(:,k+1)' * phi(X_k_plus_1);%DONE
        end

        if isnan(FinalW(:,k))
            fprintf('Training W is diverging...\n');
            diverged = 1;
            break;
        end
        if isnan(FinalV(:,k))
            fprintf('Training V is diverging...\n');
            diverged = 1;
            break;
        end
    end
end
toc
 

%% Plot Results

x=-StateSelectionWidth:dt:StateSelectionWidth;

%figure
%plot(x,psi(x))
%figure

for l = 1:length(x)
    J0(l) = FinalW(:,1)'* phi(x(l));
end
%plot(x,J0)

[~,indexJ0] = min(J0);
x_j0 = x(indexJ0);
[~,psiX0] = min(psi(x));
x_psix0 = x(psiX0);

rel_error(h) = (x_j0 - x_psix0)/x_psix0;
end

rel_error_avg=mean(rel_error)
rel_error_std=std(rel_error)