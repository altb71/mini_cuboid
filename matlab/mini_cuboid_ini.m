clc, clear all
addpath app/
%% Open Gui

% run GPA_nucleo_UART_exported.m

% You might want to use the following code later.

% fprintf('Matrix<float, 1, 2> K2(%1.4ff, %1.4ff);\n', K2(1), K2(2));
% fprintf('Matrix<float, 1, 2> K4(%1.4ff, %1.4ff, %1.4ff, %1.4ff);\n', K4(1), K4(2), K4(3), K4(4));

% Parameters
% J1 = 7.66e-4;
% J2 = 2.81e-4;
% m = 0.816;
% R = 0.066;
% g = 9.81;
% km = 36.9e-3;

% % Parameters for Simulation
% phi1_0 = 0;
% param = get_parameter();
% [A_sim, B_sim, C_sim, D_sim] = linmod('mini_cuboid_simscape_sim');
% sys_sim = minreal( ss(A_sim, B_sim, C_sim, D_sim) );
%
% phi1_0 = 30 * pi/180;


%% Symbolic Toolbox for discretization

syms s fcut z Ts tau

% tau = 1/(2*pi*fcut)
Glp = 1 / (tau*s + 1)
Gdlp = s * Glp

Glp_d = collect( simplify( ...
    subs(Glp, s, 2/Ts * (1 - z^-1) / (1 + z^-1) ) ), z)
% a1 = pi*Ts*fcut + 1;
% b1 = pi*Ts*fcut;
% b0 = b1;
% a0 = pi*Ts*fcut - 1;
a1 = Ts + 2*tau;
b1 = Ts;
b0 = b1;
a0 = Ts - 2*tau;
Glp_d_ = (b1 + b0*z^-1) / (a1 + a0*z^-1)
simplify(Glp_d - Glp_d_) % double check solution

Gdlp_d = collect( simplify( ...
    subs(Gdlp, s, 2/Ts * (1 - z^-1) / (1 + z^-1) ) ), z)
% a1 = pi*Ts*fcut + 1;
% b1 = 2*pi*fcut;
% b0 = -b1;
% a0 = pi*Ts*fcut - 1;
a1 = Ts + 2*tau;
b1 = 2;
b0 = -b1;
a0 = Ts - 2*tau;
Gdlp_d_ = (b1 + b0*z^-1) / (a1 + a0*z^-1)
simplify(Gdlp_d - Gdlp_d_) % double check solution


%% IIR-Filter implementation

% --- AUFGABE 2.4 ---
Ts = 1/1e3;
tau = 0.1;

b1 = Ts / (Ts + 2*tau);
b0 = b1;
a0 = (Ts - 2*tau) / (Ts + 2*tau);
u_kmin1 = 0;
y_kmin1 = 0;

t = (0:Ts:1).';
u = zeros(size(t));
y = zeros(size(t));
u(t > 0.1) = 1;
for i = 1:length(t)
    % y(k) = b1*u(k) + b0*u(k-1) - a0*y(k-1)
    y(i) = b1 * u(i) + b0 * u_kmin1 - a0 * y_kmin1;
    u_kmin1 = u(i);
    y_kmin1 = y(i);
end

figure(1)
stairs(t, [u, y]), grid on
xlabel('Time (sec)')

s = tf('s');
Gc = 1 / (tau*s + 1);
Ts = 1e-3;
z = tf('z', Ts);
Gd = (b1*z + b0) / (z + a0);

figure(2)
bode(Gc, Gd), grid on
legend('Time Continuous', ...
    'Time Discrete', ...
    'Location', 'best')


%% Sensor-Fusion, Complementary filter

% --- AUFGABE 3.2 ---
figure(3)
load data_00.mat % save data_00 data
plot(data.time, data.values(:,1) * 180/pi), hold on
load data_01.mat % save data_01 data
plot(data.time, data.values(:,1) * 180/pi)
load data_02.mat % save data_02 data
plot(data.time, data.values(:,1) * 180/pi), hold off, grid on
xlabel('Time (sec)'), ylabel('Cube Angle (deg)')
legend('Only Acc', ...
    'Filtered Acc', ...
    'Filtered Acc and filtered Gyro', ...
    'Location', 'best')


