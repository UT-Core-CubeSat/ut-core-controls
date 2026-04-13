%% Equivalence Check / Playback Tool
clear; clc; close all;

filename = 'simulation_data.csv';
if ~isfile(filename)
    error('Could not find %s. Make sure you ran the C++ simulation and the file is in the current directory.', filename);
end

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

    case 89
        data_format = 'fororbit';
        fprintf('Detected ForOrbit format.\n');

        time_log   = data(:, 1);
        states_log = data(:, 2:18)';
        est_log    = data(:, 19:35)';
        ref_log    = data(:, 36:45)';
        input_log  = data(:, 46:52)';
        meas_log   = data(:, 53:81)';
        model_log  = data(:, 82:88)';
        mode_log   = data(:, 89)';

    otherwise
        error('Unknown simulation_data.csv column count: %d. Expected 67 or 89.', numCols);
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

mode_map = containers.Map({0, 1, 2}, {'off', 'detumble', 'point'});

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
        tau_grav_k = tau_grav_log(:, k); %#ok<NASGU>
        tau_dist_k = tau_dist_log(:, k); %#ok<NASGU>
    end

    mode_int = mode_log(k);
    if isKey(mode_map, mode_int)
        mode_str = mode_map(mode_int);
    else
        mode_str = 'off';
    end

    animation.update(states_k, ref_k, mode_str);
    plotter.update(t, states_k, est_k, ref_k, model_k, input_k, meas_k, mode_str);
    % drawnow;
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