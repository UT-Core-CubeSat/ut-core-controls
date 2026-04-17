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

% ============================================================================
% MTQ Desaturation Effectiveness (Air Bearing)
% ============================================================================
if strcmp(data_format, 'airbearing')
    if isfield(Param, 'S') && isfield(Param, 'I_wheel') && isfield(Param, 'm_max')
        S = Param.S;
        I_wheel = Param.I_wheel;
        m_max = Param.m_max;

        % True wheel momentum in body frame: h_w = S * (I_wheel * omega_w)
        omega_w_true = states_log(8:11, :);                % [rad/s]
        h_w = S * (I_wheel * omega_w_true);                % [N*m*s]

        % Measured magnetic field in body frame (what controller has available)
        B_body = meas_log(7:9, :);                         % [T]
        B_norm = vecnorm(B_body, 2, 1);
        B_hat = zeros(size(B_body));
        valid_B = B_norm > 1e-10;
        B_hat(:, valid_B) = B_body(:, valid_B) ./ B_norm(valid_B);

        % Decompose wheel momentum into uncontrollable (parallel to B)
        % and controllable (perpendicular to B) components.
        h_par_scalar = sum(h_w .* B_hat, 1);
        h_par = B_hat .* h_par_scalar;
        h_perp = h_w - h_par;

        h_norm = vecnorm(h_w, 2, 1);
        h_par_norm = vecnorm(h_par, 2, 1);
        h_perp_norm = vecnorm(h_perp, 2, 1);

        % MTQ command and expected torque authority
        m_cmd = input_log(5:7, :);                         % [A*m^2]
        m_cmd_norm = vecnorm(m_cmd, 2, 1);
        tau_mtq = cross(m_cmd.', B_body.').';              % [N*m]
        tau_mtq_norm = vecnorm(tau_mtq, 2, 1);
        tau_mtq_max = m_max .* B_norm;                     % Max |m x B| with |m| <= m_max
        tau_util = tau_mtq_norm ./ max(tau_mtq_max, 1e-12);
        tau_util = min(max(tau_util, 0), 1);

        % Optional smoothing for slope visibility (roughly 1 second window)
        dt_med = median(diff(time_log));
        win = max(3, round(1 / max(dt_med, 1e-3)));
        h_perp_smooth = movmean(h_perp_norm, win);
        dh_perp_dt = gradient(h_perp_smooth, time_log');

        actionable_frac = h_perp_norm ./ max(h_norm, 1e-12);
        actionable_frac = min(max(actionable_frac, 0), 1);

        bearing_mask = (mode_log == 2);
        bearing_idx = find(bearing_mask);
        mtq_active_mask = m_cmd_norm >= 0.1 * m_max;
        active_mask = bearing_mask & mtq_active_mask;

        figure('Name', 'MTQ Desaturation Effectiveness');

        subplot(2,2,1);
        plot(time_log, h_norm, 'k', 'LineWidth', 1.2); hold on;
        plot(time_log, h_perp_norm, 'b', 'LineWidth', 1.2);
        plot(time_log, h_par_norm, 'r', 'LineWidth', 1.2);
        xlabel('Time [s]'); ylabel('|h| [N*m*s]');
        title('Wheel Momentum Decomposition');
        legend('|h|', '|h_{\perp B}| (MTQ-controllable)', '|h_{\parallel B}| (MTQ-limited)', 'Location', 'best');
        grid on;

        subplot(2,2,2);
        yyaxis left;
        plot(time_log, actionable_frac, 'm', 'LineWidth', 1.2);
        ylabel('Controllable Fraction');
        ylim([0 1]);
        yyaxis right;
        plot(time_log, B_norm, 'g', 'LineWidth', 1.0);
        ylabel('|B| [T]');
        xlabel('Time [s]');
        title('Available MTQ Leverage');
        legend('|h_{\perp B}| / |h|', '|B|', 'Location', 'best');
        grid on;

        subplot(2,2,3);
        yyaxis left;
        plot(time_log, m_cmd_norm, 'k', 'LineWidth', 1.2);
        ylabel('|m_{cmd}| [A*m^2]');
        ylim([0, max(m_max * 1.05, 1e-6)]);
        yyaxis right;
        plot(time_log, tau_util, 'c', 'LineWidth', 1.2);
        ylabel('MTQ Torque Utilization');
        ylim([0 1]);
        xlabel('Time [s]');
        title('MTQ Command and Authority Use');
        legend('|m_{cmd}|', '|m x B| / (m_{max}|B|)', 'Location', 'best');
        grid on;

        subplot(2,2,4);
        plot(time_log, h_perp_smooth, 'b', 'LineWidth', 1.2); hold on;
        yyaxis right;
        plot(time_log, dh_perp_dt, 'r', 'LineWidth', 1.0);
        ylabel('d|h_{\perp B}|/dt [N*m*s^2]');
        yyaxis left;
        ylabel('|h_{\perp B}| (smoothed) [N*m*s]');
        xlabel('Time [s]');
        title('Controllable Momentum Trend');
        legend('|h_{\perp B}|', 'd|h_{\perp B}|/dt', 'Location', 'best');
        grid on;

        if ~isempty(bearing_idx)
            t_row = time_log';
            t_b_start = t_row(bearing_idx(1));
            t_b_end = t_row(bearing_idx(end));

            start_win = 5.0;  % seconds after BEARING entry
            end_win = 10.0;   % final seconds in BEARING

            start_mask = bearing_mask & (t_row >= t_b_start) & (t_row <= (t_b_start + start_win));
            end_mask = bearing_mask & (t_row >= (t_b_end - end_win)) & (t_row <= t_b_end);

            % Fallback for short bearing windows
            if nnz(start_mask) < 3
                start_mask = bearing_mask;
            end
            if nnz(end_mask) < 3
                end_mask = bearing_mask;
            end

            h0 = median(h_norm(start_mask));
            hf = median(h_norm(end_mask));
            hperp0 = median(h_perp_norm(start_mask));
            hperpf = median(h_perp_norm(end_mask));
            hpar0 = median(h_par_norm(start_mask));
            hparf = median(h_par_norm(end_mask));

            red_total = 100 * (h0 - hf) / max(h0, 1e-12);
            red_perp = 100 * (hperp0 - hperpf) / max(hperp0, 1e-12);
            red_par = 100 * (hpar0 - hparf) / max(hpar0, 1e-12);

            if any(active_mask)
                mean_slope_active = mean(dh_perp_dt(active_mask));
                frac_decreasing_active = 100 * mean(dh_perp_dt(active_mask) < 0);
                mean_tau_util_active = 100 * mean(tau_util(active_mask));
            else
                mean_slope_active = NaN;
                frac_decreasing_active = NaN;
                mean_tau_util_active = NaN;
            end

            fprintf('\n=== MTQ Desaturation Effectiveness (BEARING) ===\n');
            fprintf('BEARING window: %.2f s to %.2f s (%.2f s)\n', t_b_start, t_b_end, t_b_end - t_b_start);
            fprintf('|h| median start/end:      %.3e -> %.3e N*m*s  (reduction %.1f%%)\n', h0, hf, red_total);
            fprintf('|h_perp| start/end:        %.3e -> %.3e N*m*s  (reduction %.1f%%)\n', hperp0, hperpf, red_perp);
            fprintf('|h_parallel| start/end:    %.3e -> %.3e N*m*s  (reduction %.1f%%)\n', hpar0, hparf, red_par);
            fprintf('Mean controllable fraction in BEARING: %.1f%%\n', 100 * mean(actionable_frac(bearing_mask)));

            if ~isnan(mean_slope_active)
                fprintf('When MTQ active in BEARING:\n');
                fprintf('  mean d|h_perp|/dt = %.3e N*m*s^2\n', mean_slope_active);
                fprintf('  time with decreasing |h_perp| = %.1f%%\n', frac_decreasing_active);
                fprintf('  mean MTQ torque utilization = %.1f%% of max m_max|B|\n', mean_tau_util_active);
            else
                fprintf('MTQ-active BEARING samples were not detected with current threshold.\n');
            end
        else
            fprintf('\nNo BEARING samples found in mode log. Skipping BEARING-only desat summary.\n');
        end
    else
        fprintf('\nMissing Param fields (S, I_wheel, or m_max). Skipping desaturation effectiveness analysis.\n');
    end
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