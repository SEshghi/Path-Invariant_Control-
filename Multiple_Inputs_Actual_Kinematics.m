%Path invarient Control - Multiple Inputs
clc
clear
close all
%% =========== Set the paramters =======
dt=0.01; % Sampling Time
tfinal=100; % final simulation time
time = 0:dt:tfinal;
%% =========== Initialize the states =======
x_1 = 0.1; % initilize the position in x direction       - represented as x1
x_2 = 0.3; % initilize the position in y direction       - represented as x2
x_3 = pi/2; % initilize the heading                       - represented as x3
x_4 = 0.1; % initilize the derivative of linear velocity - represented as x4

%% =========== velocity profile =======
v = 0.2;   % constant linear velocity                   
%=====================================

x_1_plot = zeros(1,length(time)-1);
x_2_plot = zeros(1,length(time)-1);
x_3_plot = zeros(1,length(time)-1);
x_4_plot = zeros(1,length(time)-1);
eta_1_plot = zeros(1,length(time)-1);
eta_2_plot = zeros(1,length(time)-1);
xi_1_plot = zeros(1,length(time)-1);
xi_2_plot = zeros(1,length(time)-1);
u_2_plot = zeros(1,length(time)-1);
%% =========== The main loop ==========
for k = 1:tfinal/dt

% Desired velocity profile
    if k>=0 && k <10
        eta_ref_2 = -0.5;
    
    else 
        eta_ref_2 = -1.5;
    end

% Xi and eta States %%%%%%%%%%%%%%%%%%%
    eta_1= atan2(x_2,x_1);
    eta_2= -((v + x_4)*(x_2*cos(x_3) - x_1*sin(x_3)))/(x_1^2 + x_2^2);

    xi_1 = x_1^2+x_2^2-1;
    xi_2 = 2*(v+x_4)*(x_1*cos(x_3)+x_2*sin(x_3)); 

% Auxiliary controller %%%%%%%%%%%%%%%%%%%

    % Tangential controller
        % Control gains
            k3 = -20;
        % Auxiliary control input
            v_par = k3*(eta_2 - eta_ref_2);%eta_ref_2);
            
    % Transversal controller
        % Control gains
            k1 = -50; 
            k2 = -10;
    % Auxiliary control input
            v_trans = k1*xi_1+k2*xi_2;  

% Lie Derivatives

    d11 = -(x_2*cos(x_3) - x_1*sin(x_3))/(x_1^2 + x_2^2);
    d12 = ((v + x_4)*(x_1*cos(x_3) + x_2*sin(x_3)))/(x_1^2 + x_2^2);
    d21 = 2*x_1*cos(x_3) + 2*x_2*sin(x_3);
    d22 = 2*(v + x_4)*(x_2*cos(x_3) - x_1*sin(x_3));

    D=[d21,d22;d11,d21];
  
    M=inv(D);

    L2f_alpha = 2*cos(x_3)^2*(v + x_4)^2 + 2*sin(x_3)^2*(v + x_4)^2;
    %cos(x_3)*(v+x_4)*((sin(x_3)*(v+x_4))/(x_1^2 + x_2^2)+(2*x_1*(v + x_4)*(x_2*cos(x_3)-x_1*sin(x_3)))/((x_1^2 + x_2^2)^2));

    L2f_pi = (2*(v + x_4)^2*((sin(2*x_3)*x_2^2)/2 + x_1*cos(2*x_3)*x_2 - (x_1^2*sin(2*x_3))/2))/(x_1^2 + x_2^2)^2;
    %2*cos(x_3)^2*(v+x_4)^2+2*(sin(x_3)^2)*(v+x_4)^2;
    


u = M*[-L2f_pi+  v_trans;-L2f_alpha+v_par];
u_1 = u(1,1);
u_2 =  u(2,1);

% input commands
w = u_2;
vel = x_4 + v;
% Kinematics with dynamic extension
    x_1 = vel*cos(x_3)*dt+x_1; % calculating x
    x_2 = vel*sin(x_3)*dt+x_2; % calculating y
    x_3 = w*dt+x_3; % calculating theta // u_2 = w angular velocity
    
    
    x_1_plot(k) = x_1;
    x_2_plot(k) = x_2;
    x_3_plot(k) = x_3;
    x_4_plot(k) = x_4;
    eta_1_plot(k) = eta_1;
    eta_2_plot(k) = eta_2;
    xi_1_plot(k) = xi_1;
    xi_2_plot(k) = xi_1;
    u_2_plot(k) = u_2;
  
end

%% =========== Plot the results =======

figure(1)
plot(x_1_plot,x_2_plot,'b--','LineWidth',2)
xlabel('$x(m)$','FontSize',16,'Interpreter','latex')
ylabel('$y (m)$','FontSize',16,'Interpreter','latex')
hold on
h = desired_path(0,0,1);


figure(2)
plot(time(1:end-1),xi_1_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\xi_1$','FontSize',16,'Interpreter','latex')

figure(3)
plot(time(1:end-1),xi_2_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\xi_2$','FontSize',16,'Interpreter','latex')

figure(4)
plot(time(1:end-1),eta_1_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_1$','FontSize',16,'Interpreter','latex')

figure(5)
plot(time(1:end-1),eta_2_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_2$','FontSize',16,'Interpreter','latex')


figure(6)
plot(time(1:end-1),eta_2_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\eta_2$','FontSize',16,'Interpreter','latex')

figure(7)
plot(time(1:end-1),u_2_plot,'b','LineWidth',1)
xlabel('$t(s)$','FontSize',16,'Interpreter','latex')
ylabel('$\omega$','FontSize',16,'Interpreter','latex')
% %=====================================
