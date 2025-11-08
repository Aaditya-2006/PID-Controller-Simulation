% MATLAB SIMULATION %

clc;
clear all;
close all;

% Define the system
% G(s) = 5 / (s^2 + 11s + 10)
Gs = tf(5, [1, 11, 10]);

% Defining time (0 to 5 seconds)
t = 0:0.01:5; % 501 datapoints

% System 1: Uncontrolled (open-loop)
[y_un, t_un] = step(Gs, t);
info_un = stepinfo(Gs);

% System 2: Ziegler-Nichols Tune
Kp_zn = 14.52;
Ki_zn = 48.56;
Kd_zn = 1.085;
C_zn = pid(Kp_zn, Ki_zn, Kd_zn); % using the built in pid function
sys_zn = feedback(C_zn * Gs, 1) % creating closed loop 
[y_zn, t_zn] = step(sys_zn, t);
info_zn = stepinfo(sys_zn, 'SettlingTimeThreshold', 0.02) % 2% threshold

% System 3: Manual tune
Kp_man = 10.0;
Ki_man = 40.0;
Kd_man = 1.5;
C_man = pid(Kp_man, Ki_man, Kd_man);
sys_man = feedback(C_man * Gs, 1);
[y_man, t_man] = step(sys_man, t);
info_man = stepinfo(sys_man, 'SettlingTimeThreshold', 0.02);

% Plotting Final Comparison
figure('Name', 'Final Comparison');
hold on;

% Plot 1: Uncontrolled
plot(t_un, y_un, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);

% Plot 2: Ziegler-Nichols Tuned
plot(t_zn, y_zn, 'LineWidth', 1.5);

% Plot 3: Manual Tuned
plot(t_man, y_man, 'g-', 'LineWidth', 2);

% Plot 4: Setpoint
plot(t, ones(size(t)), 'r--', 'LineWidth', 1);

% adding plot details
title('Final Comparison: Manual Tune vs Ziegler-Nichols');
xlabel('Time (seconds)');
ylabel('Speed');
grid on;
legend(...
    'Uncontrolled System', ...
    sprintf('Ziegler-Nichols (OS: %.1f%%)', info_zn.Overshoot), ...
    sprintf('Manual Tune (OS: %.1f%%)', info_man.Overshoot), ...
    'Setpoint', ...
    'Location', 'best' ...
);
hold off;

% printing metrics
fprintf('--- Performance Metrics ---\n');
fprintf('Uncontrolled    | Ess=%.3f\n', (1 - y_un(end)));
fprintf('Ziegler-Nichols | OS=%.1f%% | Ts=%.3fs | Ess=%.3f\n', info_zn.Overshoot, info_zn.SettlingTime, (1 - y_zn(end)));
fprintf('Manual Tune     | OS=%.1f%% | Ts=%.3fs | Ess=%.3f\n', info_man.Overshoot, info_man.SettlingTime, (1 - y_man(end)));