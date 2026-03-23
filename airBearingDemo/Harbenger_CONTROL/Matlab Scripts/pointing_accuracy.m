clear all, close all, clc
% Pointing Accuracy Calculation for a 3U CubeSat

% Define parameters
mass = 627.2; % mass of the CubeSat in grams
moment_of_inertia = [0.001, 0.001, 0.001]; % moment of inertia (Ixx, Iyy, Izz) in kg*m^2
sensor_noise = 0.01; % sensor noise in radians
actuator_noise = 0.02; % actuator noise in radians
control_system_gain = 1; % control system gain
sampling_time = 0.1; % sampling time in seconds
num_samples = 100; % number of samples for simulation

% Initialize arrays to store results
pointing_errors = zeros(1, num_samples);
attitude_changes = zeros(1, num_samples);

% Simulation loop
for i = 1:num_samples
    % Simulate the attitude change (random noise)
    attitude_change = randn(1, 3) * sensor_noise; % random attitude change
    attitude_change = attitude_change .* control_system_gain; % apply control gain
    
    % Calculate the pointing error
    pointing_error = norm(attitude_change) + randn * actuator_noise; % total error
    pointing_errors(i) = pointing_error; % store the error
    
    % Update the attitude (could be more complex in a real scenario)
    attitude_changes(i) = norm(attitude_change);
end

% Calculate statistics
mean_pointing_error = mean(pointing_errors);
std_pointing_error = std(pointing_errors);

% Display results
fprintf('Mean Pointing Error: %.4f radians\n', mean_pointing_error);
fprintf('Standard Deviation of Pointing Error: %.4f radians\n', std_pointing_error);

% Plot results
figure;
plot(1:num_samples, pointing_errors, 'b-', 'LineWidth', 1.5);
xlabel('Sample Number');
ylabel('Pointing Error (radians)');
title('Pointing Error over Time');
grid on;
