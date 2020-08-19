%Path invarient Control - single input
clc
clear
close all
%% =========== Set the paramters =======
T=0.01; % Sampling Time
k=2; % Sampling counter
x(k-1)=1; % initilize the state x
y(k-1)=1; % initilize the state y
theta(k-1)=0; % initilize the state theta
v_trans(k-1)=0;
tfinal=100; % final simulation time
t(k-1)=0; % intilize the time
%=====================================
%% =========== The main loop ==========
while(t<=tfinal) 
t(k)=t(k-1)+T; % increase the time
V=1; %constant linear velocity

% transversal control gains
k1 = -1; 
k2 = -1;

% Xi States - Diffeomorphism
xi_1(k-1) = x(k-1)^2+y(k-1)^2-9;
xi_2(k-1) = 2*V*(x(k-1)*cos(theta(k-1))+y(k-1)*sin(theta(k-1)));

% Auxiliary control input - transversal controller
v_trans(k-1) = k1*xi_1(k-1)+k2*xi_2(k-1);  

%Lie Derivatives
 Lf2_alpha=2*(V^2); % second lie derivative
 
 LgLfalpha(k-1) = 2*V*(y(k-1)*cos(theta(k-1))-x(k-1)*sin(theta(k-1))); % second lie derivative
     
 u(k) = (-Lf2_alpha+v_trans(k-1))/(LgLfalpha(k-1)); % Controller
 
% kinematics
theta(k)=u(k)*T+theta(k-1); % calculating theta
x(k)=V*cos(theta(k))*T+x(k-1); % calculating x
y(k)=V*sin(theta(k))*T+y(k-1); % calculating y


k=k+1; % increase the sampling counter
end

%% =========== Plot the results =======
figure(1)
plot(x,y,'b','LineWidth',1)
xlabel('x(m)')
ylabel('y(m)')

figure(2)
plot(t,x,'b','LineWidth',1)
xlabel('t(s)')
ylabel('x(m)')
xlim([0 100])

figure(3)
plot(t,y,'b','LineWidth',1)
xlabel('t(s)')
ylabel('y(m)')
xlim([0 100])

figure(4)
plot(t,u,'b','LineWidth',1)
xlabel('t(s)')
ylabel('u(rad/s)')
xlim([0 100])
%=====================================
