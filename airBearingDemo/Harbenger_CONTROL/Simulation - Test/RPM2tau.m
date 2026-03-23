% CubeSat Reaction Wheel Control - Inverse Function (RPM to Torque)
function [tau] = RPM2tau(RPM)

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

% Compute the pseudoinverse of A
A_pseudo = (A' * inv(A * A'));

% Define reaction wheel moment of inertia 
I_w = 1.643E-5; % [kg*m^2]

% Assume a control update period (time step used in the original function)
dt = 0.01; % [s]

% Convert RPM to angular velocity (rad/s)
omega_wheels = (RPM / 60) * (2 * pi);

% Compute angular acceleration of each wheel
alpha_wheels = omega_wheels / dt; % [rad/s^2]

% Compute individual reaction wheel torques
tau_wheels = I_w * alpha_wheels; % [Nm]

% Compute control torques by inverting A_pseudo
tau = A * tau_wheels;

end
