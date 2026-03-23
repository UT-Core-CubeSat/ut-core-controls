% CubeSat Reaction Wheel Control - MATLAB Script
function [delta_t] = tau2PWM(tau)

% Define angles
theta = deg2rad(180-105);  % Elevation angle from base
phi = deg2rad([45, 135, 225, 315]);  % Azimuth angles

% Compute unit vectors for reaction wheels
% A = [cos(theta) * cos(phi);
%      cos(theta) * sin(phi);
%      sin(theta) * ones(1, 4)];
A = [cos(theta) * ones(1, 4);
     sin(theta) * sin(phi);
     sin(theta) * cos(phi)];

R = 1.5*6.7; %Ohm
% Define reaction wheel moment of inertia 
J_r =  1.643E-5;  % [kg*m^2]
K_e = sqrt(3)*(68/60*2*pi)^-1;
J_m = 0;
%Battery voltage
e_bat = 8.4; %Volts

% Compute the pseudoinverse of A
A_pseudo = (A' * inv(A * A'));

% Compute individual reaction wheel torques
tau_wheels = A_pseudo * tau;
Q = tau_wheels;

% % hall effect sensor
% omega = ones(4,1) * 1000;
% 
% % Throttle Speed
% delta_t = (Q * (J_m+J_r)/J_r + K_e^2 / R * omega) * R / (K_e * e_bat);


% Define reaction wheel moment of inertia 
I_w = 1.643E-5;  % [kg*m^2]

% Compute angular acceleration of each wheel
alpha_wheels = tau_wheels / I_w;  % [rad/s^2]

% Assume a control update period (time step for integration)
dt = 0.01;  % [s], adjust based on your control loop frequency

% Compute final angular velocity (assuming zero initial velocity)
omega_wheels = alpha_wheels * dt;  % [rad/s]

% Convert to RPM
RPM = (omega_wheels / (2 * pi)) * 60;

% hall effect sensor
omega = tau2RPM(tau); % Have readings from motors hall effect sensors

% Throttle Speed
delta_t = (Q * (J_m+J_r)/J_r + K_e^2 / R * omega) * R / (K_e * e_bat);

end