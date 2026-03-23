clear();
clc();
close();
close all;

main();

function main()

% Load parameters
parameters = getParameters();
observer = Observer(parameters);
sensors = Sensors(parameters);
time = 0:parameters.sample:30;

% Initialize state
x = zeros(7,1);
x(1:4) = [1,0,0,0];
u = [0,0,0]';

% Initialize reference
r = zeros(7,1);
r(1:4) = [0.5, 0.75, -0.5, -0.5]';
r(1:4) = r(1:4) ./ norm(r(1:4));

% Store states for plotting
x_history = zeros(7, length(time));
u_history = zeros(3, length(time));

for i = 1:length(time)
    % Get sensor measurements
    measurements = sensors.update(x, 0, time(i), false);
    
    % Update observer
    x_hat = observer.update(u, measurements);
    
    % Calculate control input
    [u, parameters] = controller(x_hat, r, time(i), parameters);
    
    % Saturate inputs
    RPM = tau2RPM(u);
    RPM(RPM > 2000) = 2000;
    RPM(RPM < -2000) = -2000;
    u = RPM2tau(RPM);
    
    % Propagate model
    x = rk4(@(x) kinetics(x, u, parameters.I), x, parameters.sample);
    
    % Normalize quaternion
    x(1:4) = x(1:4) ./ norm(x(1:4));
    
    % Store data
    x_history(:,i) = x;
    u_history(:,i) = u;
end

% Plot results (optional)
figure;
subplot(2,1,1);
plot(time, x_history(1:4,:));
legend('q_0', 'q_1', 'q_2', 'q_3');
title('Quaternions');
xlabel('Time (s)');

subplot(2,1,2);
plot(time, x_history(5:7,:));
legend('\omega_x', '\omega_y', '\omega_z');
title('Angular Velocities');
xlabel('Time (s)');

end

% All functions must appear after the script code
function parameters = getParameters()
    parameters.sample = 0.01;
    parameters.length = 150;
   % parameters.title = "CubeSat";
   % parameters.plots = [3,3];
    parameters.time = 0;
    parameters.controlledPrinted = false;
    parameters.C_m = [ 1, 0, 0, 0, 0, 0, 0;
                       0, 1, 0, 0, 0, 0, 0;
                       0, 0, 1, 0, 0, 0, 0;	
                       0, 0, 0, 1, 0, 0, 0;];
    parameters.D_m = 0;
    parameters.C_r     = [1,0,0,0,0,0;
                         0,1,0,0,0,0;
                         0,0,1,0,0,0;
                         ];
    parameters.D_r     = zeros(3, 3);
    % Observer Tuning Parameters
    parameters.P_0 = diag([.1, .1, .1, 1, 1, 1]);
    parameters.G   = diag([1, 1, 1, 1, 1, 1]);
    parameters.Q1   = diag([0, 0, 0, 1, 1, 1]);
    parameters.R1   = diag([.1, .1, .1]);
    parameters.R = diag([1, 5, 5]) .* 20;
    parameters.Q = diag([1,10,10,0,0,0]) .* 1;
    % Control Tuning Parameters
    parameters.Q_a = diag([1,1,1,1,1,1,1,1,1]);
    parameters.R_a = 5;
    parameters.N_a = zeros(9, 3);
    
    parameters.I = [36857.5, .06,     -7.6;
                    .06,     36771.47, 42.7;
                    -7.6,  42.7,     7705.886].*10^-6;
    
    % Gyroscope noise standard deviations
    parameters.sigma_gyro_x = 0.05;
    parameters.sigma_gyro_y = 0.05;
    parameters.sigma_gyro_z = 0.05;
    
    % Magnetometer noise standard deviations
    parameters.sigma_mag_x = 0.02;
    parameters.sigma_mag_y = 0.02;
    parameters.sigma_mag_z = 0.02;
    
    % Magnetic field parameters
    parameters.declination = deg2rad(5);
    parameters.inclination = deg2rad(60);
    parameters.field_strength = 4.2e-5;
    
    % Star tracker noise
    parameters.sigma_star_quaternion = 0.001;
    
    % Star tracker sample time
    parameters.Ts_star_tracker = 1.0;
    
    % Load gains (you'll need to make sure "gains.mat" exists in your directory)
    try
        parameters.gains = load("gains.mat").p;
    catch
        % If gains file doesn't exist, create dummy gains
        warning('gains.mat not found. Using dummy gains.');
        parameters.gains = zeros(100, 31);  % Adjust size as needed
    end
end

% Dynamics functions
function x_dot = kinetics(x, u, I)
    q = x(1:4);
    omega = x(5:7);
    q_dot = 0.5 .* quatmultiply([0;omega]', q')';
    omega_dot = I \ (u - cross(omega, I * omega));
    x_dot = [q_dot'; omega_dot];
end

function [u, parameters] = controller(x, r, time, parameters)
    q_e = invquatmultiply(x(1:4), r(1:4));
    quaternion_error = sqrt(sum((x(1:4) - r(1:4)).^2));
    
    if abs(quaternion_error) < 0.001 && ~parameters.controlledPrinted
        fprintf('Controlled - Time: %.3f seconds\n', time);
        parameters.controlledPrinted = true;
    end
    
    if q_e(1) < 0
        r(1:4) = -r(1:4);
    end
    
    MRP = quat2MRP(x(1:4));
    MRP_r = quat2MRP(r(1:4));
    
    q_table = parameters.gains(:, 1:4);
    Q_e_table = zeros(size(q_table,1), 1);
    for j = 1:size(q_table,1)
        Q_e_temp = invquatmultiply(x(1:4), q_table(j,:)');
        Q_e_table(j) = Q_e_temp(1);
    end
    [~, nearest_idx] = max(abs(Q_e_table));
    
    x_e = parameters.gains(nearest_idx, 1:4);
    K_x = reshape(parameters.gains(nearest_idx, 5:22), 3, []);
    K_r = reshape(parameters.gains(nearest_idx, 23:31), 3, []);
    
    omega = x(5:7);
    MRP_e = quat2MRP(x_e(:));
    u = -K_x * [MRP-MRP_e; omega] + K_r * (MRP_r-MRP_e);
end

% Utility functions
function x1 = rk4(fun,x,time)
    F1 = fun(x);
    F2 = fun(x + time/2*F1);
    F3 = fun(x + time/2*F2);
    F4 = fun(x + time*F3);
    x1 = x + time/6 * (F1 + 2*F2 + 2*F3 + F4);
end

function p = quatmultiply(q, r)
    if size(q, 2) ~= 4 || size(r, 2) ~= 4
        error('Expecting quaternions as rows.');
    elseif size(q, 1) ~= size(r, 1)
        error('Number of quaternions don''t match.');
    end
    p = [q(1), -q(2), -q(3), -q(4);
         q(2), q(1), q(4), -q(3);
         q(3),-q(4), q(1),  q(2);
         q(4), q(3),-q(2), q(1)] * r';
end

function p = invquatmultiply(q, r)
    p = [q(1), -q(2),  -q(3),  -q(4);
         q(2), q(1), q(4),  -q(3);
         q(3), -q(4),  q(1), q(2);
         q(4), q(3),  -q(2), q(1)]^-1 * r;
end

function mrp = quat2MRP(q)
    q = q(:);
    p = q'*q;
    if(p < 0.99999 || p > 1.00001)
        disp('Warning: quat2mrp: quaternion is not of unit norm');
        disp(1-p);
    end
    q = q./sqrt(p);
    mrp = q(2:4)./(1 + q(1));
end

% function quat = MRP2quat(mrp)
%     mrp_norm = norm(mrp);
%     s_squared = mrp_norm^2;
%     q0 = (1 - s_squared) / (1 + s_squared);
%     qv = (2 * mrp) / (1 + s_squared);
%     quat = [q0; qv];
%     quat = quat / norm(quat);
% end

function [RPM] = tau2RPM(tau)
    theta = deg2rad(180-105);
    phi = deg2rad([45, 135, 225, 315]);
    A = [cos(theta) * ones(1, 4);
         sin(theta) * sin(phi);
         sin(theta) * cos(phi)];
    A_pseudo = (A' * inv(A * A'));
    tau_wheels = A_pseudo * tau;
    I_w = 1.643E-5;
    alpha_wheels = tau_wheels / I_w;
    dt = 0.01;
    omega_wheels = alpha_wheels * dt;
    RPM = (omega_wheels / (2 * pi)) * 60;
end

function [tau] = RPM2tau(RPM)
    theta = deg2rad(180-105);
    phi = deg2rad([45, 135, 225, 315]);
    A = [cos(theta) * ones(1, 4);
         sin(theta) * sin(phi);
         sin(theta) * cos(phi)];
    %A_pseudo = (A' * inv(A * A'));
    I_w = 1.643E-5;
    dt = 0.01;
    omega_wheels = (RPM / 60) * (2 * pi);
    alpha_wheels = omega_wheels / dt;
    tau_wheels = I_w * alpha_wheels;
    tau = A * tau_wheels;
end

function A = cross_mat(v)
    A = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

% function [A,B,C,D] = modrod_state_space_fast(x,u,I)
%     % Simple placeholder - you'll need to implement this based on your actual system
%     q = x(1:4);
%     omega = x(5:7);
%     sigma = quat2MRP(q);
%     
%     % Simplified A matrix computation
%     A = zeros(6,6);
%     A(1:3,4:6) = 1/4*((1-norm(sigma)^2)*eye(3) + 2*cross_mat(sigma) + 2*(sigma*sigma'));
%     A(4:6,4:6) = -inv(I)*cross_mat(I*omega);
%     
%     % Simplified B matrix
%     B = [zeros(3,3); inv(I)];
%     
%     % Measurement matrices
%     C = eye(6);
%     D = zeros(6,3);
% end