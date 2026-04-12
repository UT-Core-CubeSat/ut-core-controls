classdef Observer < handle
    % =========================================================================
    % MODERN MULTIPLICATIVE EXTENDED KALMAN FILTER (MEKF)
    % Combined formulation based on:
    %   [CJ04] Crassidis & Junkins, "Optimal Estimation of Dynamic Systems"
    %   (2004)Table 7.1, Eqs. (7.18–7.48)
    %   [MC14] Markley & Crassidis, "Fundamentals of Spacecraft Attitude
    %   Determination and Control" (2014) Sec. 6.3–6.4, Eqs. (6.15–6.23)
    % =========================================================================

    properties
        q_hat       % Attitude quaternion estimate [4x1]
        beta_hat    % Gyro bias estimate [3x1]
        P           % Error covariance [6x6]
        G
        Q

        sigma_v     % Gyro measurement noise std [rad/s]
        sigma_u     % Gyro bias random walk std [rad/s/sqrt(s)]
        R_star      % Star tracker covariance [3x3]
        R_quest
        Ts          % Sample time [s]
        Helpers
        DateTime
        omega_earth

        use_star_tracker
        use_magnetometer
        use_sun_sensor

        last_star_update_time
        T_star

        S
        I_wheel
        I
        I_max
    end

    methods
        % ---------------------------------------------------------------------
        function self = Observer(Param)
            % === Initialization ===
            self.q_hat = Param.states_init(7:10);
            self.beta_hat = Param.beta_gyro;
            self.P = Param.P_0;
            self.G = Param.G;
            self.Q = Param.Q;

            % Process noise — [CJ04] Eq. (7.37c)
            self.sigma_v = Param.sigma_gyro;
            self.sigma_u = Param.sigma_bias_walk;

            % Measurement noise — [MC14] Eq. (6.23)
            self.R_star = Param.R_star;
            self.R_quest = Param.R_quest;

            % Helpers
            self.Helpers = HelperFunctions;
            self.DateTime = Param.DateTime;
            self.omega_earth = Param.omega_earth;

            self.Ts = Param.Ts;
            self.DateTime = Param.DateTime;

            self.use_star_tracker  = true;
            self.use_magnetometer  = false;
            self.use_sun_sensor    = false;

            self.last_star_update_time = 0;
            self.T_star = Param.T_star;
            self.S = Param.S;
            self.I_wheel = Param.I_wheel;
            self.I = Param.I;
            self.I_max = Param.I_max;
        end

        % ---------------------------------------------------------------------
        function propagate(self, omega_meas)
            % ================================================================
            % PROPAGATION STEP
            % [CJ04] Eqs. (7.18–7.20), (7.32–7.37)
            % [MC14] Eqs. (6.15–6.20)
            % ================================================================
            % ω̂ = ω̃ − β̂ — [MC14] Eq. (6.15)
            omega_hat = omega_meas - self.beta_hat;
          
            % Quaternion kinematics — [CJ04] Eq. (7.18)
            q_vec = self.q_hat(2:4);   % vector part (x,y,z)
            q0    = self.q_hat(1);     % scalar part
            
            q_dot_vec = 0.5*( q0*omega_hat + cross(q_vec, omega_hat) );
            q_dot0    = -0.5*(q_vec.'*omega_hat);

            q_dot = [q_dot0; q_dot_vec];

            % Integrate — [CJ04] Eq. (7.49)
            self.q_hat = self.q_hat + self.Ts*q_dot;
            self.q_hat = self.q_hat / norm(self.q_hat);

            % Covariance propagation — [CJ04] Eq. (7.37a–c)
            F = [-skew(omega_hat), -eye(3);
                zeros(3), zeros(3)];
            self.P = self.P + self.Ts * (F*self.P + self.P*F' + self.G*self.Q*self.G');
            self.P = 0.5*(self.P + self.P'); % Symmetrization
        end

        % ---------------------------------------------------------------------
        function star_tracker_update(self, q_meas)
            % ================================================================
            % STAR TRACKER UPDATE
            % [CJ04] Eqs. (7.43–7.48)
            % [MC14] Eqs. (6.22b–6.23)
            % ================================================================

            % δq = q̂⁻⁻¹ ⊗ q_meas — [CJ04] Eq. (7.47)
            q_hat_inv = [self.q_hat(1); -self.q_hat(2:4)];
            delta_q = quatMultiply(q_hat_inv, q_meas);

            % Positive scalar part — [MC14] note after Eq. (6.22b)
            if delta_q(1) < 0, delta_q = -delta_q; end

            % Measurement residual ỹ = 2 * vec(δq) — [CJ04] Eq. (7.43)
            y_tilde = 2 * delta_q(2:4);

            % Linearized model H = [I 0] — [CJ04] Eq. (7.43)
            H = [eye(3), zeros(3)];
            S = H*self.P*H' + self.R_star; %#ok<*PROPLC>
            K = self.P*H'/S;

            % Kalman correction δx = K*ỹ — [CJ04] Eq. (7.44)
            delta_x = K * y_tilde;
            delta_alpha = delta_x(1:3);
            delta_beta  = delta_x(4:6);

            % Joseph covariance update — [CJ04] Eq. (7.45)
            I_KH = eye(6) - K*H;
            self.P = I_KH*self.P*I_KH' + K*self.R_star*K';
            self.P = 0.5*(self.P + self.P');

            % Quaternion correction — [CJ04] Eq. (7.47), [MC14] Eq. (6.23)
            dq_plus = [1; 0.5*delta_alpha];
            dq_plus = dq_plus / norm(dq_plus);
            self.q_hat = quatMultiply(self.q_hat, dq_plus);
            self.q_hat = self.q_hat / norm(self.q_hat);

            % Bias update — [CJ04] Eq. (7.46)
            self.beta_hat = self.beta_hat + delta_beta;
        end

        function quest_update(self, q_meas)

            q_hat_inv = [self.q_hat(1); -self.q_hat(2:4)];
            delta_q = quatMultiply(q_hat_inv, q_meas);
            if delta_q(1) < 0, delta_q = -delta_q; end
            
            y_tilde = 2 * delta_q(2:4);
            H = [eye(3), zeros(3)];
            
            S = H*self.P*H' + self.R_quest;
            K = self.P*H'/S;
            
            delta_x = K * y_tilde;
            delta_alpha = delta_x(1:3);
            delta_beta  = delta_x(4:6);
            
            % Joseph form update
            I_KH = eye(6) - K*H;
            self.P = I_KH*self.P*I_KH' + K*self.R_quest*K';
            self.P = 0.5*(self.P + self.P');
            
            % Apply correction
            dq_plus = [1; 0.5*delta_alpha];
            dq_plus = dq_plus / norm(dq_plus);
            self.q_hat = quatMultiply(self.q_hat, dq_plus);
            self.q_hat = self.q_hat / norm(self.q_hat);
            
            self.beta_hat = self.beta_hat + delta_beta;
        end

        function q_quest = quest_algorithm(self, JD, ~, r_eci, b_sun, b_mag)
            helper = self.Helpers;

            % === Inertial Reference Vectors ===
            % From helper functions (ECI frame)
            r_sun = helper.earth2sun(JD);     % ECI sun vector
            r_mag = compute_r_mag(helper, JD, r_eci);  % magnetic field ref
        
            % === Normalize body-frame sensor vectors ===
            b_sun = b_sun / norm(b_sun);
            b_mag = b_mag / norm(b_mag);
        
            % === Define weights ===
            a_sun = 0.6;
            a_mag = 0.4;
        
            % Construct B matrix [MC14] Eq. 5.11 
            B = a_sun * (b_sun * r_sun') + a_mag * (b_mag * r_mag');
        
            % Compute K matrix [MC14] Eq. 5.12
            S = B + B';
            sigma = trace(B);
            Z = [B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];
            K = [sigma, Z';
                 Z, S - sigma*eye(3)];
        
            % === Compute eigenvector of K with max eigenvalue ===
            [V,D] = eig(K);
            [~, idx] = max(diag(D));
            q_meas = V(:, idx);
        
            % Ensure positive scalar part
            if q_meas(1) < 0
                q_meas = -q_meas;
            end
        
            q_meas = q_meas / norm(q_meas);
            q_quest = q_meas;
        end
        % ---------------------------------------------------------------------
        function states_hat = update(self, measurements)
    % FULL MEKF CYCLE
    % [CJ04] Table 7.1 — “Continuous-discrete MEKF implementation”

    self.DateTime = self.DateTime + seconds(self.Ts);
    omega_gyro = measurements(4:6);
    q_star     = measurements(7:10);
    y_css      = measurements(11:16);
    gps_ecef   = measurements(20:25);
    omega_w    = measurements(26:29);

    % ================================================================
    % 1. Propagate attitude and covariance
    % ================================================================
    self.propagate(omega_gyro);

    % ================================================================
    % 2. Determine which attitude source to use
    % ================================================================
    persistent last_q_star
    if isempty(last_q_star)
        last_q_star = q_star;   % Initialize tracker memory
    end

    q_diff = norm(q_star - last_q_star);
    new_star_update = (q_diff > 1e-6) && all(~isnan(q_star));

    % --- Sun sensor processing ---
    b_sun = synthesize_sun_vector(y_css, self.I_max); %#ok<*NASGU>
    sun_current_sum = sum(y_css);
    I_threshold = 0.05 * self.I_max * 6;   % Eclipse threshold
    in_eclipse = (sun_current_sum < I_threshold);

    % --- Attitude update logic ---
    if self.use_star_tracker && new_star_update
        % New star tracker measurement available
        self.star_tracker_update(q_star);
        last_q_star = q_star;
        self.last_star_update_time = posixtime(self.DateTime);

    else 
        % Sun visible → QUEST update
        jd      = self.Helpers.julianDate(self.DateTime);
        decYear = 2025.8;
        b_mag   = measurements(17:19);

   

         
        C_eci2ecef = self.Helpers.dcmeci2ecef(jd);
        r_ecef = gps_ecef(1:3);
        r_eci = C_eci2ecef' * r_ecef;  % Transform ECEF → ECI
        
        b_sun = synthesize_sun_vector(y_css, self.I_max);
        
        q_quest = self.quest_algorithm(jd, decYear, r_eci, b_sun, b_mag);
        self.quest_update(q_quest);  % Still needs R_quest fix!

    end

    % ================================================================
    % 3. Transform position/velocity to ECI for output
    % ================================================================
    jd = self.Helpers.julianDate(self.DateTime);
    C_eci2ecef = self.Helpers.dcmeci2ecef(jd);
    r_ecef = gps_ecef(1:3);
    v_ecef = gps_ecef(4:6);

    R_hat = C_eci2ecef' * r_ecef;
    V_hat = C_eci2ecef' * v_ecef + cross(self.omega_earth, R_hat);

    omega_b_hat = omega_gyro - self.beta_hat;
    q_hat = self.q_hat / norm(self.q_hat);

    states_hat = [R_hat; V_hat; q_hat; omega_b_hat; omega_w];
end
    end
end

% ==========================
% Helper Functions
% ==========================
function S = skew(v)
% [MC14] Eq. (3.157)
S = [  0   -v(3)  v(2);
    v(3)   0   -v(1);
    -v(2)  v(1)   0 ];
end

function q_out = quatMultiply(q1, q2)
% Quaternion multiplication, scalar-first
% [MC14] Eq. (3.168)
q0 = q1(1); qv = q1(2:4);
p0 = q2(1); pv = q2(2:4);
q_out = [ q0*p0 - dot(qv,pv);
    q0*pv + p0*qv + cross(qv,pv) ];
end
function r_mag = compute_r_mag(helper, JD, r_eci)

    decYear = 2025.8;
    % Convert ECI position → ECEF
    r_ecef = helper.eci2ecef(r_eci, JD);

    % Get lat/lon/alt
    [lat, lon, alt] = helper.ecef2lla(r_ecef);

    % Get magnetic vector in NED frame
    [B_NED, ~] = helper.wrldmagm(lat, lon, alt, decYear);

    % Convert NED → ECEF
    C_ned2ecef = helper.dcmecef2ned(lat, lon)';
    B_ecef = C_ned2ecef * B_NED;

    % Convert ECEF → ECI
    C_eci2ecef = helper.dcmeci2ecef(JD);
    B_eci = C_eci2ecef' * B_ecef;

    % Normalize
    r_mag = B_eci / norm(B_eci);
end
function b_sun = synthesize_sun_vector(y_css, I_max)
% =========================================================================
% COARSE SUN SENSOR VECTOR RECONSTRUCTION
% Markley & Crassidis (2014), Sec. 4.3, Eqs. (4.18–4.22)
% Converts 6 CSS currents (+X,+Y,+Z,−X,−Y,−Z) → body-frame sun vector
% =========================================================================

    n = [eye(3); -eye(3)];  % Sensor normals (+X,+Y,+Z,−X,−Y,−Z)

    % Eq. (4.20): Opposing face differencing
    I_diff = [ ...
        y_css(1) - y_css(4);
        y_css(2) - y_css(5);
        y_css(3) - y_css(6)];

    % Eq. (4.21): Linear relation ΔI = I_max * n^T * s
    s_est = I_diff / I_max;

    % Eq. (4.22): Normalize → unit vector
    if norm(s_est) > 1e-8
        b_sun = s_est / norm(s_est);
    else
        b_sun = [NaN; NaN; NaN];  % Eclipse / dark condition
    end
end