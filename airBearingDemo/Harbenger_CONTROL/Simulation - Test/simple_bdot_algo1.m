% B-dot Controller Implementation for a Satellite in Polar Orbit
% This simulation models a satellite with B-dot control to detumble after deployment
% Includes realistic orbital mechanics, magnetic field modeling, and hardware limitations

clear all;
close all;
clc;

%% Simulation Parameters
dt = 0.2;                   % Time step [s]
T = 3*90*60;                % Total simulation time [s] (3 orbits)
t = 0:dt:T;                 % Time vector
numSteps = length(t);

%% Orbital Parameters
altitude = 500e3;           % Orbit altitude [m]
Re = 6371.2e3;              % Earth radius [m]
Rc = Re + altitude;         % Orbital radius [m]
mu = 3.986004418e14;        % Earth's gravitational parameter [m^3/s^2]
orbital_period = 2*pi*sqrt(Rc^3/mu); % Orbital period [s]
mean_motion = 2*pi/orbital_period;   % Mean motion [rad/s]
inclination = 98 * pi/180;  % Inclination for sun-synchronous orbit [rad]

%% Satellite Parameters
% Inertia tensor [kg·m^2] - Realistic values for a 3U CubeSat
I = [36857.5, .06,     -7.6;
     .06,     36771.47, 42.7;
     -7.6,  42.7,     7705.886].*10^-6;

% Initial conditions
omega_0 = [0.1; -1.5; 0.3];  % Initial angular velocity [rad/s]
q_0 = [0; 0; 0; 1];          % Initial quaternion (scalar last)

% Hardware constraints
max_dipole = 0.001;            % Maximum magnetic dipole moment [A·m^2]
B_noise_sigma = 50e-9;       % Magnetometer noise standard deviation [T]
omega_noise_sigma = 0.002;   % Angular velocity noise [rad/s]

%% Preallocate arrays
omega = zeros(3, numSteps);          % Angular velocity
omega(:,1) = omega_0;

q = zeros(4, numSteps);              % Quaternion
q(:,1) = q_0;

B_body = zeros(3, numSteps);         % Magnetic field in body frame
B_ECI = zeros(3, numSteps);          % Magnetic field in ECI frame
magnetic_dipole = zeros(3, numSteps); % Control magnetic dipole moment
control_torque = zeros(3, numSteps);  % Control torque

position_ECI = zeros(3, numSteps);    % Position in ECI frame
velocity_ECI = zeros(3, numSteps);    % Velocity in ECI frame

%% B-dot Controller Parameters
K_bdot = 2e5;                        % B-dot controller gain [N·m·s/T]

%% Initialize orbit
% Initial position and velocity for polar orbit
position_ECI(:,1) = [Rc; 0; 0];
velocity_ECI(:,1) = [0; sqrt(mu/Rc); 0];

% Rotate initial state for inclination
R_inc = [1, 0, 0;
         0, cos(inclination), -sin(inclination);
         0, sin(inclination), cos(inclination)];
     
position_ECI(:,1) = R_inc * position_ECI(:,1);
velocity_ECI(:,1) = R_inc * velocity_ECI(:,1);

%% Main simulation loop
for i = 1:numSteps-1
    % Current state
    omega_current = omega(:,i);
    q_current = q(:,i);
    
    % Update orbit position using simple orbit propagation
    theta = mean_motion * t(i);  % True anomaly
    
    % Simple circular orbit propagation in ECI frame
    position_ECI(:,i) = Rc * [cos(theta); sin(theta)*cos(inclination); sin(theta)*sin(inclination)];
    velocity_ECI(:,i) = sqrt(mu/Rc) * [-sin(theta); cos(theta)*cos(inclination); cos(theta)*sin(inclination)];
    
    % Get satellite position in spherical coordinates (r, lat, lon)
    r = norm(position_ECI(:,i));
    lat = asin(position_ECI(3,i)/r) * 180/pi;  % Latitude in degrees
    lon = atan2(position_ECI(2,i), position_ECI(1,i)) * 180/pi;  % Longitude in degrees
    
    % Get magnetic field in ECI frame - Simulated IGRF model
    % This is a simplified magnetic dipole model; replace with actual IGRF for accuracy
    B_ECI(:,i) = simulate_magnetic_field(lat, lon, altitude, t(i));
    
    % Add magnetometer noise (simulating sensor noise)
    B_ECI_noisy = B_ECI(:,i) + B_noise_sigma * randn(3,1);
    
    % Convert magnetic field to body frame using current attitude
    B_body(:,i) = quat_to_dcm(q_current)' * B_ECI_noisy;
    
    % Add noise to angular velocity measurement
    omega_measured = omega_current + omega_noise_sigma * randn(3,1);
    
    % Apply B-dot control law
    % m = -K * (ω × B)/|B|
    B_norm = norm(B_body(:,i));
    if B_norm > 1e-9  % Avoid division by zero
        m_desired = -K_bdot * cross(omega_measured, B_body(:,i)) / B_norm;
        
        % Apply hardware constraints (saturation)
        m_magnitude = norm(m_desired);
        if m_magnitude > max_dipole
            m_desired = m_desired * (max_dipole / m_magnitude);
        end
    else
        m_desired = zeros(3,1);
    end
    
    magnetic_dipole(:,i) = m_desired;
    
    % Calculate control torque: T = m × B
    control_torque(:,i) = cross(magnetic_dipole(:,i), B_body(:,i));
    
    % Compute angular acceleration: α = I⁻¹(T - ω × (I·ω))
    gyroscopic_torque = cross(omega_current, I * omega_current);
    angular_acceleration = I \ (control_torque(:,i) - gyroscopic_torque);
    
    % Update angular velocity (integration)
    omega(:,i+1) = omega_current + angular_acceleration * dt;
    
    % Update quaternion
    q(:,i+1) = update_quaternion(q_current, omega(:,i+1), dt);
end

%% Plot results
figure(1);
subplot(3,1,1);
plot(t/60, omega(1,:), 'LineWidth', 1.5);
ylabel('\omega_x [rad/s]');
title('Angular Velocity Components');
grid on;

subplot(3,1,2);
plot(t/60, omega(2,:), 'LineWidth', 1.5);
ylabel('\omega_y [rad/s]');
grid on;

subplot(3,1,3);
plot(t/60, omega(3,:), 'LineWidth', 1.5);
xlabel('Time [minutes]');
ylabel('\omega_z [rad/s]');
grid on;

figure(2);
subplot(3,1,1);
plot(t/60, magnetic_dipole(1,:), 'LineWidth', 1.5);
ylabel('m_x [A·m^2]');
title('Control Magnetic Dipole Moment');
grid on;

subplot(3,1,2);
plot(t/60, magnetic_dipole(2,:), 'LineWidth', 1.5);
ylabel('m_y [A·m^2]');
grid on;

subplot(3,1,3);
plot(t/60, magnetic_dipole(3,:), 'LineWidth', 1.5);
xlabel('Time [minutes]');
ylabel('m_z [A·m^2]');
grid on;

% Plot 3D trajectory
figure(3);
plot3(position_ECI(1,:), position_ECI(2,:), position_ECI(3,:), 'b', 'LineWidth', 1.5);
hold on;
% Plot Earth
[X, Y, Z] = sphere(50);
surf(Re*X, Re*Y, Re*Z, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal;
grid on;
title('Satellite Orbit');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

%% Helper Functions
function dcm = quat_to_dcm(q)
    % Convert quaternion to direction cosine matrix
    % Input: q = [q1; q2; q3; q4] (scalar last)
    
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    
    dcm = [1-2*(q2^2+q3^2), 2*(q1*q2-q3*q4), 2*(q1*q3+q2*q4);
           2*(q1*q2+q3*q4), 1-2*(q1^2+q3^2), 2*(q2*q3-q1*q4);
           2*(q1*q3-q2*q4), 2*(q2*q3+q1*q4), 1-2*(q1^2+q2^2)];
end

function q_new = update_quaternion(q, omega, dt)
    % Update quaternion using angular velocity
    % First-order quaternion kinematics
    
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);
    wx = omega(1); wy = omega(2); wz = omega(3);
    
    omega_matrix = 0.5 * [
        0,  -wx, -wy, -wz;
        wx,   0,  wz, -wy;
        wy, -wz,   0,  wx;
        wz,  wy, -wx,   0
    ];
    
    q_dot = omega_matrix * q;
    q_new = q + q_dot * dt;
    
    % Normalize quaternion
    q_new = q_new / norm(q_new);
end

function B = simulate_magnetic_field(lat, lon, altitude, time)
    % Simulate Earth's magnetic field based on position
    % This is a simplified tilted dipole model
    % Replace with actual IGRF model for better accuracy
    
    % Convert to radians
    lat_rad = lat * pi/180;
    lon_rad = lon * pi/180;
    
    % Earth's magnetic dipole parameters
    m_dipole = 7.94e22;  % Earth's magnetic dipole moment [A·m²]
    tilt_angle = 11 * pi/180;  % Dipole tilt angle
    
    % Earth rotation
    earth_rotation_rate = 7.2921150e-5;  % [rad/s]
    lon_rotated = lon_rad + earth_rotation_rate * time;
    
    % Compute magnetic field vector in local NED frame
    r = 6371.2e3 + altitude;  % Distance from Earth's center [m]
    
    % Magnetic field magnitude at the given altitude (dipole model)
    B_magnitude = (mu0 / (4*pi)) * m_dipole / r^3;
    
    % Components in spherical coordinates
    B_r = -2 * B_magnitude * sin(lat_rad);
    B_theta = B_magnitude * cos(lat_rad);
    B_phi = 0;  % Simplified model
    
    % Convert to cartesian coordinates
    B_x = B_r * cos(lat_rad) * cos(lon_rotated) - B_theta * sin(lat_rad) * cos(lon_rotated);
    B_y = B_r * cos(lat_rad) * sin(lon_rotated) - B_theta * sin(lat_rad) * sin(lon_rotated);
    B_z = B_r * sin(lat_rad) + B_theta * cos(lat_rad);
    
    % Apply dipole tilt
    R_tilt = [cos(tilt_angle), 0, sin(tilt_angle);
              0, 1, 0;
              -sin(tilt_angle), 0, cos(tilt_angle)];
    
    B = R_tilt * [B_x; B_y; B_z];
    
    % Add small time-varying component to simulate field variations
    B = B + 50e-9 * sin(0.001 * time) * [1; 0; 0];
end

% Physical constant for magnetic field calculation
function mu0 = mu0()
    % Magnetic permeability of vacuum [N/A²]
    mu0 = 4*pi*1e-7;
end