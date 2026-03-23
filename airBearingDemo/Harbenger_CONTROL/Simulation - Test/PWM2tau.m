function [tau] = PWM2tau(delta_t)

% Define angles
theta = deg2rad(180-105);  % Elevation angle from base
phi = deg2rad([45, 135, 225, 315]);  % Azimuth angles

% Compute unit vectors for reaction wheels
A = [cos(theta) * ones(1, 4);
     sin(theta) * sin(phi);
     sin(theta) * cos(phi)];

R = 1.5*6.7; % Ohm
% Define reaction wheel moment of inertia 
J_r =  1.643E-5;  % [kg*m^2]
K_e = sqrt(3)*(68/60*2*pi)^-1;
J_m = 0;
%Battery voltage
e_bat = 8.4; % Volts

omega = ones(4,1) * 1000;

% Compute individual reaction wheel torques from throttle input
Q = (J_r / (J_m + J_r))*(delta_t * (K_e * e_bat) / R) - (K_e^2 / R * omega);


% Compute torque output using forward transformation
tau = A * Q;

end
