%% Space actuator model parameters

% Electrical
R  = 1.0;      % Ohm
L  = 0.5;      % H
Ke = 0.1;      % V/(rad/s)
Kt = 0.1;      % Nm/A

% Mechanical
J  = 0.02;     % kg*m^2
b  = 0.2;      % N*m*s/rad

% Disturbance
tau_dist_nom = 0.0;   % N*m

% Controller
Kp_pos = 2.5;
Ki_pos = 1.0;
Kd_pos = 0.05;

% Command
theta_step = 0.2;     % rad
step_time = 0.5;      % s
enable_val = 1;

