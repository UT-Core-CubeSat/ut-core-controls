%% MATLAB Simulation -> CSV Export (Corrected t=0 Start)
clear; clc; close all;

% Seed RNG for deterministic comparison with C++
rng(42);

% 1. Setup & Parameters
Parameters
filename = 'matlab_data.csv';
fid = fopen(filename, 'w');

if fid == -1
    error('Could not open file %s for writing.', filename);
end

% 2. Write Header (Matches C++)
header_str = [...
    't,', ...
    'rx,ry,rz,vx,vy,vz,q0,q1,q2,q3,wx,wy,wz,rw1,rw2,rw3,rw4,', ...
    'rx_est,ry_est,rz_est,vx_est,vy_est,vz_est,q0_est,q1_est,q2_est,q3_est,wx_est,wy_est,wz_est,rw1_est,rw2_est,rw3_est,rw4_est,', ...
    'q0_ref,q1_ref,q2_ref,q3_ref,wx_ref,wy_ref,wz_ref,ax_ref,ay_ref,az_ref,', ...
    'tau_w1,tau_w2,tau_w3,tau_w4,m_x,m_y,m_z,', ...
    'meas_ax,meas_ay,meas_az,meas_gx,meas_gy,meas_gz,', ...
    'meas_st_q0,meas_st_q1,meas_st_q2,meas_st_q3,', ...
    'meas_css1,meas_css2,meas_css3,meas_css4,meas_css5,meas_css6,', ...
    'meas_mx,meas_my,meas_mz,', ...
    'meas_gps_r1,meas_gps_r2,meas_gps_r3,meas_gps_v1,meas_gps_v2,meas_gps_v3,', ...
    'meas_rw1,meas_rw2,meas_rw3,meas_rw4,', ...
    'mod1,mod2,mod3,mod4,mod5,mod6,mod7,', ...
    'mode\n'];

fprintf(fid, header_str);

%% 3. Instantiate Classes
dynamics     = Dynamics(Param);
sensors      = Sensors(Param);
observer     = Observer(Param);
controller   = Controller(Param);
refGen       = ReferenceGenerator(Param);

% 4. INITIALIZATION (t=0)
t            = Param.t_start;
t_next_plot  = 0;

% Initialize Variables to match C++ 'Zero' initialization at start
states_true  = Param.states_init; 
states_hat   = Param.states_init; 
reference    = zeros(10,1);
tau          = zeros(7,1);
measurements = zeros(29,1); % C++ uses Vector29::Zero() initially
states_m     = zeros(7,1);
mode         = 0;           % OFF

fprintf('Running MATLAB Simulation & Exporting...\n');

% --- LOG INITIAL STATE (Row 1: t=0) ---
% This was missing before!
fprintf(fid, '%.6f,', t);
fprintf(fid, '%.6f,', states_true);
fprintf(fid, '%.6f,', states_hat);
fprintf(fid, '%.6f,', reference);
fprintf(fid, '%.6f,', tau);
fprintf(fid, '%.6f,', measurements);
fprintf(fid, '%.6f,', states_m);
fprintf(fid, '%d\n', mode);

%% 5. Main Loop
while t < Param.t_end
    
    % --- Inner Physics Loop ---
    while t <= t_next_plot
        
        % A. Reference
        [reference, mode_out] = refGen.update(dynamics.states, t);
        
        % Convert Mode String to Int
        if strcmpi(mode_out, 'point')
            mode = 2;
        elseif strcmpi(mode_out, 'detumble')
            mode = 1;
        else
            mode = 0;
        end

        % B. Sensors
        measurements = sensors.measurements(dynamics.states_dot, dynamics.states, t);
        
        % C. Observer (Ensure this is enabled to match C++)
        states_hat = observer.update(measurements);
        %states_hat = dynamics.states; % Keeping your override for now if C++ has it

        % D. Controller
        [tau, states_m] = controller.update(states_hat, reference, measurements, mode_out);

        % E. Dynamics
        dynamics.update(tau, mode_out);
        
        % F. Time Step
        t = t + Param.Ts;
    end
    
    % --- Outer Loop (Log Data) ---
    
    % 1. Time
    fprintf(fid, '%.9f,', t);
    
    % 2. True States (17)
    fprintf(fid, '%.9f,', dynamics.states);
    
    % 3. Estimates (17)
    fprintf(fid, '%.9f,', states_hat);
    
    % 4. Reference (10)
    fprintf(fid, '%.9f,', reference);
    
    % 5. Inputs (7)
    fprintf(fid, '%.9f,', tau);
    
    % 6. Measurements (29)
    fprintf(fid, '%.9f,', measurements);
    
    % 7. Model States (7)
    fprintf(fid, '%.9f,', states_m);
    
    % 8. Mode (Int)
    fprintf(fid, '%d\n', mode);
    
    % Update Plot Timer
    t_next_plot = t + Param.t_plot;
end

fclose(fid);
fprintf('Done! Saved to %s\n', filename);