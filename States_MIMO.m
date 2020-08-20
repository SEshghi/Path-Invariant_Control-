clc
clear all
close all
syms x1 x2 x3 x4 v u%L u d1
syms a0 a1 a2 a3 k1 k2
X=[x1 x2 x3 x4];

%% System
fx=[(v+x4)*cos(x3);(v+x4)*sin(x3);0;0];
g1x=[0;0;0;1];
g2x=[0;0;1;0];


%% Path: Circular Path
alpha = x1^2+x2^2-1;
pi = atan(x2/x1);

%% Lie Derivative
L_alpha=jacobian(alpha,X);
L_pi = simplify(jacobian(pi,X));

% alpha derivatives
Lf_alpha = simplify(L_alpha*fx);

Lg1_Lf_alpha = simplify(jacobian(Lf_alpha,X))*g1x;

Lg2_Lf_alpha = simplify(jacobian(Lf_alpha,X))*g2x;


% pi derivatives
Lf_pi = simplify(L_pi*fx);

Lg1_Lf_pi = simplify(jacobian(Lf_pi,X))*g1x;

Lg2_Lf_pi = simplify(jacobian(Lf_pi,X))*g2x;


%%
d11=simplify(Lg1_Lf_pi);
d12=simplify(Lg2_Lf_pi);
d21=simplify(Lg1_Lf_alpha);
d22=simplify(Lg2_Lf_alpha );


D=[d11 d12;d21 d22];
M=inv(D);

%% second derivative 
L2f_alpha = simplify(jacobian(Lf_alpha,X)*fx)
L2f_pi = simplify(jacobian(Lf_pi,X)*fx)

u1 = [-L2f_pi+v_par]/D;
u2 = [-L2f_alpha+v_tra]/D;

