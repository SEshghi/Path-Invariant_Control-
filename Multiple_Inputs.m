%Path invarient Control - Multiple Inputs
clc
clear
close all
%% =========== Set the paramters =======
T=0.01; % Sampling Time
k=2; % Sampling counter
x(k-1)=1; % initilize the state x
y(k-1)=1; % initilize the state y
theta(k-1)=0; % initilize the state theta
x4(k-1)=0; % initilize the derivative of linear velocity
tfinal=100; % final simulation time
t(k-1)=0; % intilize the time
%=====================================
%% =========== The main loop ==========
while(t<=tfinal) 
t(k)=t(k-1)+T; % increase the time
v = 1;
% Desired velocity profile
    if t(k)>=0 && t(k) <10
        eta_ref_2 = -0.5;
    
    else 
        eta_ref_2 = -1.5;
    end



% Xi and eta States - Diffeomorphism
    eta_1(k-1)= atan(y(k-1)/x(k-1));
    eta_2(k-1)= x(k-1)*cos(theta(k-1))+y(k-1)*sin(theta(k-1));

    xi_1(k-1) = x(k-1)^2+y(k-1)^2-9;
    xi_2(k-1) = 2*v*(x(k-1)*cos(theta(k-1))+y(k-1)*sin(theta(k-1))); 

% Auxiliary controller
    % Tangential controller
        % Control gains
            k3 = -0.2;
        % Auxiliary control input
            v_par(k-1) = k3*(eta_1(k-1) - eta_ref_2);
            
    % Transversal controller
        % Control gains
            k1 = -1; 
            k2 = -1;
    % Auxiliary control input
            v_trans(k-1) = k1*xi_1(k-1)+k2*xi_2(k-1);  

% Lie Derivatives
    d11(k-1) = -(y(k-1)*cos(theta(k-1)) - x(k-1)*sin(theta(k-1)))/(x(k-1)^2 + y(k-1)^2);
    d12(k-1) = ((v + x4(k-1))*(x(k-1)*cos(theta(k-1)) + y(k-1)*sin(theta(k-1))))/(x(k-1)^2 + y(k-1)^2);
    d21(k-1) = 2*x(k-1)*cos(theta(k-1)) + 2*y(k-1)*sin(theta(k-1));
    d22(k-1) = 2*(v + x4(k-1))*(y(k-1)*cos(theta(k-1)) - x(k-1)*sin(theta(k-1)));
 
    D=[d11(k-1) d12(k-1);d21(k-1) d22(k-1)];
    L2f_alpha(k-1) = 2*(v + x4(k-1))^2;
    L2f_pi(k-1) = (2*(v + x4(k-1))^2*((sin(2*theta(k-1))*y(k-1)^2)/2 + x(k-1)*cos(2*theta(k-1))*y(k-1)...
        -(x(k-1)^2*sin(2*theta(k-1)))/2))/(x(k-1)^2 + y(k-1)^2)^2;

u=inv(D)*[-L2f_pi(k-1)+v_par(k-1);-L2f_alpha(k-1)+v_trans(k-1)];
u1(k-1) =u(1,1);
u2(k-1) = u(2,1);

    
% Kinematics with dynamic extension
    theta(k)=u2(k-1)*T+theta(k-1); % calculating theta
    x4(k)=u1(k-1)*T+x4(k-1); % calculating x4
    x(k)=v*cos(theta(k))*T+x(k-1); % calculating x
    y(k)=v*sin(theta(k))*T+y(k-1); % calculating y
    


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
