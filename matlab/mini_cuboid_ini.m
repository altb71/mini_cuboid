clc, clear all
addpath app/
%% Open Gui

% run GPA_nucleo_UART_exported.m

% You might want to use the following code later.

% fprintf('K << %1.4ff, %1.4ff, %1.4ff, %1.4ff;\n', K(1), K(2), K(3), K(4));


%%

load G_est_00.mat % save G_est_00 G_est

Kp = 0.3;
G0 = Kp * G_est;
Gcl = feedback(G0, 1);

figure(1)
subplot(121)
margin(G0, 2*pi*G_est.Frequency), grid on
subplot(122)
bode(Gcl, 1-Gcl, 2*pi*G_est.Frequency), grid on


%% Modellbildung und Zustandsregelung Mini-Cuboid

J_geh = 7.66e-4;
R = 0.066;
m = 0.816;
g = 9.81;
J_ges = J_geh + R^2 * m;

A = [0          , 1;...
     m*g*R/J_ges, 0];
B = [0; -1/J_ges];

% Qs = ctrb(A,B); % Steuerbarkeitsmatrix
% K = place(A,B,10*[-.707+.707i -.707-.707i])

J_sb = 2.81e-4;
A3 = [A zeros(2,1);zeros(1,3)];
B3 = [B;1/J_sb];
C3 = [1 0 0;0 1 0;0 -1 1];
s3 = ss(A3,B3,C3,0);
T = C3; % verwende alte C-Matrix also TrafoMatrix
s3_ = ss2ss(s3,T); % Trafo in Sensorkoordinaten

% Erweiterung um Integratorzustand:
A_ = [s3_.a zeros(3,1);-[0 0 1] 0];
B_ = [s3_.b;0];
K = place(A_,B_,10*[-.707+.707i -.707-.707i -1 -.1]);
fprintf('K << %1.4ff, %1.4ff, %1.4ff, %1.4ff;\n', K(1), K(2), K(3), K(4));

Q = diag([1 0.01 0.01 100]);
K =lqr(A_, B_, Q, 1e5)
% Matrix<float, 1, 3> Kx(-2.1929f, -0.2016f, -0.0042f);
fprintf('float Ki(%1.4ff);\n', K(4));
fprintf('Matrix<float, 1, 3> Kx(%1.4ff, %1.4ff, %1.4ff);\n', K(1), K(2), K(3));


%%

load data_00.mat % save data_00 data

figure(1)
subplot(311)
plot(data.time, data.values(:,[3])), grid on
subplot(312)
plot(data.time, data.values(:,[1 4 5 6])), grid on, legend
subplot(313)
plot(data.time, data.values(:,2)), grid on


%%

syms s fcut z Ts

tau = 1/(2*pi*fcut)
Glp = 1 / (tau*s + 1);
Gdlp = s * Glp

Glp_d = collect( simplify( ...
    subs(Glp, s, 2/Ts * (1 - z^-1) / (1 + z^-1)) ...
    ), z)
a0 = pi*Ts*fcut + 1;
b0 = pi*Ts*fcut;
b1 = b0;
a1 = pi*Ts*fcut - 1;
Glp_d_ = (b0 + b1*z^-1) / (a0 + a1*z^-1)
simplify(Glp_d - Glp_d_)

Gdlp_d = collect( simplify( ...
    subs(Gdlp, s, 2/Ts * (1 - z^-1) / (1 + z^-1)) ...
    ), z)
a0 = pi*Ts*fcut + 1;
b0 = 2*pi*fcut;
b1 = -b0;
a1 = pi*Ts*fcut - 1;
Gdlp_d_ = (b0 + b1*z^-1) / (a0 + a1*z^-1)
simplify(Gdlp_d - Gdlp_d_)
