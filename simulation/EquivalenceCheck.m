%% Equivalence Check / Playback Tool
clear; clc; close all;

% Select newest available CSV to avoid plotting stale output.
candidate_files = {
    'build/simulation_data.csv', ...
    'build/Debug/simulation_data.csv', ...
    'simulation_data.csv', ...
    '../simulation/simulation_data.csv'
};

existing_files = {};
existing_times = [];
for i = 1:numel(candidate_files)
    f = candidate_files{i};
    if isfile(f)
        d = dir(f);
        existing_files{end+1} = f; %#ok<AGROW>
        existing_times(end+1) = d.datenum; %#ok<AGROW>
    end
end

if isempty(existing_files)
    error('Could not find simulation_data.csv in expected locations. Run PlantSim first.');
end

[~, newest_idx] = max(existing_times);
filename = existing_files{newest_idx};
fprintf('Using data file: %s\n', filename);

opts = detectImportOptions(filename);
opts.VariableNamingRule = 'preserve';
data_table = readtable(filename, opts);
data = table2array(data_table);
[numRows, numCols] = size(data);
fprintf('Loaded %s with %d rows and %d columns.\n', filename, numRows, numCols);

switch numCols
    case 67
        data_format = 'airbearing';
        fprintf('Detected Air Bearing format.\n');

        time_log     = data(:, 1);
        states_log   = data(:, 2:12)';     % 11 states
        est_log      = data(:, 13:23)';    % 11 estimates
        ref_log      = data(:, 24:33)';    % 10 reference
        input_log    = data(:, 34:40)';    % 7 inputs
        meas_log     = data(:, 41:53)';    % 13 measurements
        model_log    = data(:, 54:60)';    % 7 model states
        tau_grav_log = data(:, 61:63)';    % 3 gravity torque
        tau_dist_log = data(:, 64:66)';    % 3 disturbance torque
        mode_log     = data(:, 67)';       % 1 mode
        innov_log    = [];                 % No innovations in this format

    case 75
        % Extended format with per-face MTQ data
        data_format = 'airbearing';
        fprintf('Detected Air Bearing format with MTQ per-face data.\n');

        time_log         = data(:, 1);
        states_log       = data(:, 2:12)';     % 11 states
        est_log          = data(:, 13:23)';    % 11 estimates
        ref_log          = data(:, 24:33)';    % 10 reference
        input_log        = data(:, 34:40)';    % 7 inputs
        meas_log         = data(:, 41:53)';    % 13 measurements
        model_log        = data(:, 54:60)';    % 7 model states
        tau_grav_log     = data(:, 61:63)';    % 3 gravity torque
        tau_dist_log     = data(:, 64:66)';    % 3 disturbance torque
        mtq_current_log  = data(:, 67:70)';    % 4 per-face MTQ currents [A]
        mtq_b_ref_log    = data(:, 71:74)';    % 4 per-face B-field references [T]
        mode_log         = data(:, 75)';       % 1 mode
        innov_log        = [];                 % No innovations in this format
        
    case 83
        % Extended format with innovations (accel and mag)
        data_format = 'airbearing';
        fprintf('Detected Air Bearing format with innovation diagnostics.\n');

        time_log         = data(:, 1);
        states_log       = data(:, 2:12)';     % 11 states
        est_log          = data(:, 13:23)';    % 11 estimates
        ref_log          = data(:, 24:33)';    % 10 reference
        input_log        = data(:, 34:40)';    % 7 inputs
        meas_log         = data(:, 41:53)';    % 13 measurements
        model_log        = data(:, 54:60)';    % 7 model states
        tau_grav_log     = data(:, 61:63)';    % 3 gravity torque
        tau_dist_log     = data(:, 64:66)';    % 3 disturbance torque
        mtq_current_log  = data(:, 67:70)';    % 4 per-face MTQ currents [A]
        mtq_b_ref_log    = data(:, 71:74)';    % 4 per-face B-field references [T]
        accel_innov_log  = data(:, 75:77)';    % 3 accel innovation components
        accel_innov_norm = data(:, 78);        % 1 accel innovation norm
        mag_innov_log    = data(:, 79:81)';    % 3 mag innovation components
        mag_innov_norm   = data(:, 82);        % 1 mag innovation norm
        mode_log         = data(:, 83)';       % 1 mode
        innov_log        = [accel_innov_log; accel_innov_norm'; mag_innov_log; mag_innov_norm'];
        
    otherwise
        error('Unknown simulation_data.csv column count: %d. Expected 67, 75, or 83.', numCols);
end

switch data_format
    case 'airbearing'
        Parameters;
        animation = Animation(Param);
        plotter   = Plotter(Param);
    case 'fororbit'
        orbit_Parameters;
        animation = orbit_Animation(Param);
        plotter   = orbit_Plotter(Param);
end

% Initialize innovation log for backward compatibility
if ~exist('innov_log', 'var') || isempty(innov_log)
    innov_log = [];
end

mode_map = containers.Map({0, 1, 2}, {'off', 'safe', 'bearing'});

% Initialize MTQ logs if they don't exist (for backward compatibility)
if ~exist('mtq_current_log', 'var') || isempty(mtq_current_log)
    mtq_current_log = zeros(4, length(time_log));
    mtq_b_ref_log = zeros(4, length(time_log));
end

figure(Param.active_figure);

fprintf('Starting Playback of %d data points...\n', length(time_log));

for k = 1:length(time_log)
    t = time_log(k);

    states_k = states_log(:, k);
    est_k    = est_log(:, k);
    ref_k    = ref_log(:, k);
    input_k  = input_log(:, k);
    meas_k   = meas_log(:, k);
    model_k  = model_log(:, k);

    if strcmp(data_format, 'airbearing')
        tau_grav_k = tau_grav_log(:, k);
        tau_dist_k = tau_dist_log(:, k);
    end

    mode_int = mode_log(k);
    if isKey(mode_map, mode_int)
        mode_str = mode_map(mode_int);
    else
        mode_str = 'off';
    end

    animation.update(states_k, ref_k, mode_str);
    plotter.update(t, states_k, est_k, ref_k, model_k, input_k, meas_k, mode_str);
    %drawnow;
end

fprintf('Playback Complete.\n');

if strcmp(data_format, 'airbearing')
    figure('Name', 'Air Bearing Disturbance Analysis');

    subplot(3,1,1);
    plot(time_log, tau_grav_log');
    xlabel('Time [s]'); ylabel('Torque [Nm]');
    title('Gravity Torque (CG Offset)');
    legend('X', 'Y', 'Z'); grid on;

    subplot(3,1,2);
    plot(time_log, tau_dist_log');
    xlabel('Time [s]'); ylabel('Torque [Nm]');
    title('External Disturbance Torque');
    legend('X', 'Y', 'Z'); grid on;

    subplot(3,1,3);
    q_log = states_log(1:4, :);
    z_body = [0; 0; 1];
    pointing_error_deg = zeros(1, length(time_log));
    for k = 1:length(time_log)
        q = q_log(:, k);
        q = q / norm(q);

        z_inertial = quatrotate(q, z_body')';
        c = dot(z_inertial, [0;0;1]);
        c = max(-1, min(1, c));

        pointing_error_deg(k) = acosd(c);
    end
    plot(time_log, pointing_error_deg);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title('Pointing Error (Body Z vs Inertial Z)');
    grid on;
end

% ============================================================================
% MTQ Health & EKF Analysis (if MTQ data available)
% ============================================================================
if exist('mtq_current_log', 'var') && ~isempty(mtq_current_log) && any(mtq_current_log(:) ~= 0)
    figure('Name', 'MTQ Health & EKF Estimation Analysis');
    
    % Per-face MTQ currents
    subplot(3,2,1);
    plot(time_log, mtq_current_log');
    xlabel('Time [s]'); ylabel('Current [A]');
    title('Per-Face MTQ Currents');
    legend('Xpos', 'Xneg', 'Ypos', 'Yneg'); grid on;
    
    % Per-face B-field references
    subplot(3,2,2);
    plot(time_log, mtq_b_ref_log');
    xlabel('Time [s]'); ylabel('Field [T]');
    title('Per-Face B-Field References');
    legend('Xpos', 'Xneg', 'Ypos', 'Yneg'); grid on;
    
    % EKF quaternion error (estimated vs true)
    subplot(3,2,3);
    q_true = states_log(1:4, :);
    q_est = est_log(1:4, :);
    q_error_deg = zeros(1, length(time_log));
    
    for k = 1:length(time_log)
        qt = q_true(:, k) / norm(q_true(:, k));
        qe = q_est(:, k) / norm(q_est(:, k));
        
        % Quaternion error magnitude (from quatconj and multiply)
        q_err = quatMultiply(quatconj(qt), qe);
        angle_rad = 2 * acos(max(-1, min(1, abs(q_err(1)))));
        q_error_deg(k) = rad2deg(angle_rad);
    end
    
    plot(time_log, q_error_deg);
    xlabel('Time [s]'); ylabel('Angle [deg]');
    title('EKF Quaternion Estimation Error');
    grid on;
    
    % Angular velocity estimation error
    subplot(3,2,4);
    omega_true = states_log(5:7, :);
    omega_est = est_log(5:7, :);
    omega_error = vecnorm(omega_true - omega_est);
    
    plot(time_log, omega_error);
    xlabel('Time [s]'); ylabel('Error [rad/s]');
    title('Angular Velocity Estimation Error');
    grid on;
    
    % Wheel speed estimation error
    subplot(3,2,5);
    rw_true = states_log(8:11, :);
    rw_est = est_log(8:11, :);
    rw_error = vecnorm(rw_true - rw_est);
    
    plot(time_log, rw_error);
    xlabel('Time [s]'); ylabel('Error [rad/s]');
    title('Wheel Speed Estimation Error');
    grid on;
    
    % Overall EKF convergence metric
    subplot(3,2,6);
    ekf_health = sqrt(q_error_deg'.^2 + omega_error'.^2 + rw_error'.^2);
    plot(time_log, ekf_health);
    xlabel('Time [s]'); ylabel('Combined Error');
    title('EKF Overall Health Metric');
    grid on;
end

% ============================================================================
% Innovation Diagnostics (if available)
% ============================================================================
if ~isempty(innov_log) && any(innov_log(:) ~= 0)
    figure('Name', 'EKF Innovation Diagnostics');
    
    % Extract innovation data
    accel_innov_x = innov_log(1, :);
    accel_innov_y = innov_log(2, :);
    accel_innov_z = innov_log(3, :);
    accel_innov_norm_plot = innov_log(4, :);
    mag_innov_x = innov_log(5, :);
    mag_innov_y = innov_log(6, :);
    mag_innov_z = innov_log(7, :);
    mag_innov_norm_plot = innov_log(8, :);
    
    % Accel innovation components
    subplot(2,2,1);
    plot(time_log, [accel_innov_x; accel_innov_y; accel_innov_z]');
    xlabel('Time [s]'); ylabel('Innovation');
    title('Accel Innovation Components');
    legend('x', 'y', 'z'); grid on;
    
    % Accel innovation norm
    subplot(2,2,2);
    plot(time_log, accel_innov_norm_plot);
    xlabel('Time [s]'); ylabel('Norm');
    title('Accel Innovation Norm');
    grid on;
    
    % Mag innovation components
    subplot(2,2,3);
    plot(time_log, [mag_innov_x; mag_innov_y; mag_innov_z]');
    xlabel('Time [s]'); ylabel('Innovation');
    title('Mag Innovation Components');
    legend('x', 'y', 'z'); grid on;
    
    % Mag innovation norm
    subplot(2,2,4);
    plot(time_log, mag_innov_norm_plot);
    xlabel('Time [s]'); ylabel('Norm');
    title('Mag Innovation Norm');
    grid on;
end

function v_rot = quatrotate(q, v)
    q = q(:);
    q = q / norm(q);
    qv = [0; v(:)];
    v_rot = quatMultiply(quatMultiply(q, qv), quatconj(q));
    v_rot = v_rot(2:4);
end

function q = quatMultiply(q1, q2)
    w1 = q1(1); v1 = q1(2:4);
    w2 = q2(1); v2 = q2(2:4);
    w = w1*w2 - dot(v1,v2);
    v = w1*v2 + w2*v1 + cross(v1,v2);
    q = [w; v];
end

function q_conj = quatconj(q)
    q_conj = [q(1); -q(2:4)];
end