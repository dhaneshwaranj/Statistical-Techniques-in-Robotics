% clean up the matlab environment
clear; clc; close all;

% run initialization of some paths and variables
init_setup;
load('lab3.mat');
% contains A, B, C, LQR_Kss, target_hover_state, clipping_distance
% C = eye(size(x))

H = 2000;

% actual state, not observable, do not reference
x(:,1) = target_hover_state;

% initial state estimate (given)
mu_x(:,1) = x(:,1);
% initial state observation (given)
y(:,1) = x(:,1);

% initial control
dx = compute_dx(target_hover_state, mu_x(:,1));
u(:,1) = LQR_Kss* dx;

% default noise parameters
sigmaY = 0.5;
sigmaX = 0.1;

% My Co-Variance Matrices
sigma = 5*eye(21);
Q = (sigmaX)^2*eye(21);
R = (sigmaY)^2*eye(21);

for t=2:H    
	% add noise to motion model:
    noise_F_T = randn(6,1)*sigmaX;
    
    % Simulate helicopter, do not change:
    x(:,t) = f_heli(x(:,t-1), u(:,t-1), dt, model, idx, noise_F_T);
    
    % add state observation noise
    v = randn(size(C*x(:,t)))*sigmaY;
    
    % observe noisy state
    y(:,t) = C*x(:,t) + v;    
    
    % use Kalman filter to calculate mean state
    % from mu_x(:,t-1), y(t) ,u(t-1)
    
    % My Code 
    % Prediction
    mu_x(:,t) = A*mu_x(:,t-1) + B*u(:,t-1);
    sigma = A*sigma*A' + Q;
    
    % Correction
    % Kalman Gain calculation
    if sigmaY==0.0
        K = eye(21);
    else
        K = sigma*C'/(C*sigma*C' + R);
    end
    
    mu_x(:,t) = mu_x(:,t) + K*(y(:,t) - C*mu_x(:,t));
    sigma = sigma - K*C*sigma;
    
%     mu_x(:,t) = y(:,t); % dumb take observation as estimate
    
    % LQR controller generates control for next step.  u(t-1) takes x(:,t-1)
    % to x(:,t).  do not change (only change mu_x value above)
	dx = compute_dx(target_hover_state, mu_x(:,t));
    dx(idx.ned) = max(min(dx(idx.ned), clipping_distance),-clipping_distance);
	u(:,t) = LQR_Kss* dx;   
    
end

% figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('Hover position');
% figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('Hover quaternion');
% figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('Hover trim');

fig1 = figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('Hover quaternion');
fig2 = figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('Hover trim');
fig3 = figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('Hover position');

% saveas(fig1, 'C:\Users\dhane\Desktop\MS ECE\Spring 2018\Statistical Techniques in Robotics\Labs\Lab3\code\code\images\4\Y\quatKY.png', 'png');
% saveas(fig2, 'C:\Users\dhane\Desktop\MS ECE\Spring 2018\Statistical Techniques in Robotics\Labs\Lab3\code\code\images\4\Y\trimKY.png', 'png');
% saveas(fig3, 'C:\Users\dhane\Desktop\MS ECE\Spring 2018\Statistical Techniques in Robotics\Labs\Lab3\code\code\images\4\Y\posKY.png', 'png');

