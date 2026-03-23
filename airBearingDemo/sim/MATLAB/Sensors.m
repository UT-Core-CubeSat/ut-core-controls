classdef Sensors < handle
    properties
        % place properties here

        % current datetime
        DateTime
        Ts

        mu_E
        % accelerometer
        beta_a
        sigma_a

        %gyro
        sigma_gyro
        beta_gyro

        % star tracker
        star_update
        y_star_previous
        sigma_star
        beta_star
        small_angle_tol
        T_star
        previous_star_update

        % sun sensor
        I_max
        beta_css
        sigma_css

        % magnetometer
        %reduction
        beta_mag
        sigma_mag

        % gps
        omega_earth
        K_gps
        T_gps
        nu
        sigma_cep
        sigma_V
        y_gps_previous
        previous_gps_update

        % Wheels
        omega_w_previous
        omega_w_meas
        sigma_w
        alpha_w

        % Helpers
        Helpers
        r_E
    end
    methods
        %==========================================================================
        %                         Methods Section
        %==========================================================================
        function self = Sensors(Param)
            % constructor method
            % current datetime
            self.Helpers = HelperFunctions;
            self.DateTime = Param.DateTime; % from param or from simulation?
            self.Ts       = Param.Ts;
            self.mu_E = Param.mu_E;
            % accelerometer
            self.beta_a  = Param.beta_a;
            self.sigma_a = Param.sigma_a;
            self.r_E = Param.r_E;

            %gyro
            self.sigma_gyro = Param.sigma_gyro;
            self.beta_gyro  = Param.beta_gyro ;

            % star tracker
            self.star_update     = Param.star_update    ;
            self.sigma_star      = Param.sigma_star     ;
            self.beta_star       = Param.beta_star      ;
            self.small_angle_tol = Param.small_angle_tol;
            self.T_star          = Param.T_star         ;

            self.y_star_previous = Param.states_init(7:10);

            self.previous_star_update = 0;

            % sun sensor
            self.I_max     = Param.I_max    ;
            self.beta_css  = Param.beta_css ;
            self.sigma_css = Param.sigma_css;

            % magnetometer
            %self.reduction = Param.reduction;
            self.beta_mag  = Param.beta_mag ;
            self.sigma_mag = Param.sigma_mag;

            % gps
            self.omega_earth = Param.omega_earth;
            self.K_gps       = Param.K_gps      ;
            self.T_gps       = Param.T_gps      ;
            self.nu          = [0; 0; 0];  % GPS position error (Gauss-Markov state)
            self.sigma_cep   = Param.sigma_gps_cep;
            self.sigma_V     = Param.sigma_V    ;

            % Initialize GPS in ECEF frame (not ECI) to match gps() output
            jd_init = self.Helpers.julianDate(self.DateTime);
            C_eci2ecef_init = self.Helpers.dcmeci2ecef(jd_init);
            r_eci_init = Param.states_init(1:3);
            v_eci_init = Param.states_init(4:6);
            r_ecef_init = C_eci2ecef_init * r_eci_init;
            v_ecef_init = C_eci2ecef_init * v_eci_init - cross(self.omega_earth, r_ecef_init);
            self.y_gps_previous = [r_ecef_init; v_ecef_init];

            self.previous_gps_update = 0;

            % wheels 
            self.omega_w_previous = Param.states_init(14:17);
            self.omega_w_meas = self.omega_w_previous;          
            self.sigma_w = Param.sigma_wheel;
            self.alpha_w = Param.alpha_wheel;
        end
        %-------------------------------------------------------------------------
        function y = measurements(self,states_dot,states,t)
            % update states with sensor noise

            % update datetime to current timestep
            self.DateTime = self.DateTime + seconds(self.Ts);
            %------------------------------
            % update accelerometer:
            y_a = self.accelerometer(states_dot,states);


            %------------------------------
            % update gyro
            y_gyro = self.gyroscope(states);


            %------------------------------
            % check if star tracker can update
            if t - self.previous_star_update >= self.T_star

                % update star tracker reading
                y_star = self.star_tracker(states);
                self.y_star_previous = y_star;
                self.previous_star_update = t;
            else
                % return previous star tracker reading
                y_star = self.y_star_previous;
            end


            %------------------------------
            % update sun sensor
            y_css = self.sun_sensor(states);
            %------------------------------
            % update magnetometer
            y_B = self.magnetometer(states);


            %------------------------------
            % check if gps can update
            if t - self.previous_gps_update >= self.T_gps

                % update gps reading
                y_gps = self.gps(states);
                self.y_gps_previous = y_gps;
                self.previous_gps_update = t;
            else
                % return previous gps reading
                y_gps = self.y_gps_previous;
            end

            %------------------------------
            %------------------------------
            % update wheels
            omega_w = self.reaction_wheels(states);
            %------------------------------

            % Pack measurements vector
            % [3;3;4;6;3;6;4] (29x1)
            y = [y_a;y_gyro;y_star;y_css;y_B;y_gps;omega_w];
        end
        %-------------------------------------------------------------------------
        function states_hat = measurements2states(self,measurements)
            % unpack measurements
            %y_a    = measurements(1:3);
            y_gyro = measurements(4:6);
            y_star = measurements(7:10);
            %y_css  = measurements(11:13);
            %y_B    = measurements(14:19);
            y_gps  = measurements(20:25);

            C_eci2ecef = self.Helpers.dcmeci2ecef(self.Helpers.julianDate(self.DateTime));

            % find R_hat in ECI frame
            R_hat = C_eci2ecef'*y_gps(1:3);

            % find V_hat in ECI frame
            V_hat = C_eci2ecef'*y_gps(4:6) + cross(self.omega_earth,R_hat); % reynolds transport theorem

            % find q_hat
            q_hat = y_star / norm(y_star + 1e-12);

            % find omega_hat in body frame
            omega_hat = y_gyro;

            % pack states_hat vector
            states_hat = [R_hat;V_hat;q_hat;omega_hat];
        end
        %-------------------------------------------------------------------------
        function y_a = accelerometer(self,states_dot,states)
            % model accelerometer
            R = states(1:3);
            q = states(7:10);
            a_i = states_dot(4:6);
            % calculate acceleration due to gravity in ECI frame
            g_i = two_body(self.mu_E, self.r_E,R);

            % calculate the acceleration the sensors will be able to measure
            a_mi = a_i - g_i;

            % passive rotation into body frame (assuming quaternion scalar
            % in front)
            q_star = self.Helpers.quatconj(q);
            a_b = self.Helpers.quatrotate(q_star,a_mi);

            % add bias and noise
            y_a = a_b + self.beta_a + self.sigma_a .* randn(3,1);

        end
        %-------------------------------------------------------------------------
        function y_gyro = gyroscope(self,states)
            % model gyroscope
            omega_b    = states(11:13);   % true angular velocities

            % simulate sensor noise
            eta_gyro = self.sigma_gyro .* randn(3,1);

            % gyro outputs
            y_gyro = omega_b + self.beta_gyro + eta_gyro; %[x;y;z]
        end
        %-------------------------------------------------------------------------
        function y_star = star_tracker(self,states)
        % model a star tracker
        q_true = states(7:10);

        % determine if startracker will update
        % Use randn (not rand) for consistency with other sensors
        normal_sample = randn();
        random_val = 0.5 * (1 + erf(normal_sample / sqrt(2)));
        
        if self.star_update < random_val
            y_star = self.y_star_previous;

        else
            % check if we can apply small angle approximation for noise
            if norm(self.sigma_star) < self.small_angle_tol

                q_noise_vec = self.sigma_star/2;
                q_noise_scl = 1;
            else
                % no small angle approximation
                angle = norm(self.sigma_star);
                n_hat = self.sigma_star/angle;

                q_noise_vec = n_hat .* sin(angle/2);
                q_noise_scl = cos(angle/2);
            end
            q_eta = [q_noise_scl;q_noise_vec];

            % bias is fixed so no conditional needed
            % bias quaternion - convert bias angles to quaternion
            if norm(self.beta_star) < self.small_angle_tol
                q_beta = [1; self.beta_star/2];
            else
                beta_angle = norm(self.beta_star);
                beta_axis = self.beta_star / beta_angle;
                q_beta = [cos(beta_angle/2); beta_axis * sin(beta_angle/2)];
            end

            % multiply true quaternion by bias and noise error
            % quaternions
            y_star = self.Helpers.quatMultiply(q_eta,self.Helpers.quatMultiply(q_beta,q_true));
            y_star = y_star / norm(y_star);
        end
    end
        %-------------------------------------------------------------------------
        function y_css = sun_sensor(self,states)
            % model sun sensor eye
            q = states(7:10);
            R_sat = states(1:3);

            % get current datetime and convert to juliandate
            jd = self.Helpers.julianDate(self.DateTime);

            % calculate position of sun relative to earth
            R_sun = self.Helpers.earth2sun(jd); % normalized position vector
            R_sat = R_sat/norm(R_sat); % normalized position of satellite relative to earth

            % calculate position vector from satellite to sun
            S       = R_sun - R_sat; % make sure matching units
            s_i_hat = S/norm(S + 1e-12);     % inertial sun unit vector

            % rotate sun unit vector to body frame
            q_star = self.Helpers.quatconj(q);
            s_b_hat = self.Helpers.quatrotate(q_star,s_i_hat);

            % 6 faces: +x,+y,+z and -x,-y,-z (normals)
            n_p = eye(3);
            n_n = -eye(3);

            I_p = zeros(3,1);
            I_n = zeros(3,1);

            for i = 1:3
                dp_pos = n_p(i,:)*s_b_hat;      % scalar
                dp_neg = n_n(i,:)*s_b_hat;      % scalar
                I_p(i) = self.I_max * max(dp_pos, 0);
                I_n(i) = self.I_max * max(dp_neg, 0);
            end

            I = [I_p; I_n];

            beta = self.beta_css;               % 6x1
            eta  = self.sigma_css .* randn(6,1);
            y_css = I + beta + eta;
        end
        %-------------------------------------------------------------------------
        function y_B = magnetometer(self,states)
            % model magnetometers
            R = states(1:3);
            q = states(7:10);

            % get utc time
            dt = self.Helpers.julianDate(self.DateTime);

            % convert position to ecef
            R_ecef = self.Helpers.eci2ecef(R, dt);

            % convert position to geodetic coord.
            [lat,lon,h] = self.Helpers.ecef2lla(R_ecef');

            % calc magnetic field vector
            B_NED = self.Helpers.wrldmagm(lat,lon,h,2025);

            % Rotation matrix from NED to ecef
            C_ned2ecef = self.Helpers.dcmecef2ned(lat,lon)';

            % Rotation matrix from ecef to eci
            C_ecef2eci = self.Helpers.dcmeci2ecef(dt)';

            % rotate magnetic field from NED to ECI
            B_ECI = C_ecef2eci*(C_ned2ecef*B_NED);

            % passive rotation (i-->b) of magnetic vector
            q_star = self.Helpers.quatconj(q);
            B_b    = self.Helpers.quatrotate(q_star,B_ECI);
            %B_b    = self.Helpers.quatrotate(q,B_ECI);
            % add uncertainty to body vector
            y_B = B_b + self.beta_mag + self.sigma_mag.*randn(3,1);
        end
        %-------------------------------------------------------------------------
        function y_gps = gps(self,states)
            % model gps unit
            R = states(1:3);
            V = states(4:6);
            dt = self.Helpers.julianDate(self.DateTime);

            % rotate from eci to ecef
            C_eci2ecef = self.Helpers.dcmeci2ecef(dt);

            R_ecef = C_eci2ecef*R;
            V_ecef = C_eci2ecef*V - cross(self.omega_earth,R_ecef); % reynolds transport theorem

            % Vector 1st-order Gauss–Markov for position error (per-axis)
            % self.nu is 3x1; self.sigma_cep is 3x1 std devs (per axis)
            phi = exp(-self.K_gps * self.T_gps);                         % scalar or 3x1
            next_nu = phi .* self.nu + sqrt(1 - (phi.^2)) .* (self.sigma_cep .* randn(3,1));
            self.nu = next_nu;

            y_R = R_ecef + next_nu;

            % Velocity noise (3x1)
            y_V = V_ecef + (self.sigma_V .* randn(3,1));

            y_gps = [y_R; y_V];
        end
        %-------------------------------------------------------------------------
        function y_wheel = reaction_wheels(self, states)
            omega_w = states(14:17);
            noise = self.sigma_w .* randn(4,1);
            meas_raw = omega_w + noise;
            self.omega_w_meas = self.alpha_w.*meas_raw + (1 - self.alpha_w).*self.omega_w_meas;
            y_wheel = self.omega_w_meas;
        end
        %-------------------------------------------------------------------------
        function g = two_body(~, mu_E,R)
            % this function calculates the translational acceleration of a satellite
            % relative to a ECI frame.
            % Currently the function uses newtons two body problem, but should be
            % expanded to account of earth's oblateness.
            % INPUTS
            % R    - satellite position vector [x;y;z]
            % mu_E - earth's gravitational parameter
            % OUTPUTS
            % g - vector acceleration due to gravity [x;y;z]
            %--------------------------------------------------------------------------
            g = -mu_E .* R ./ norm(R)^3;
        end
        %-------------------------------------------------------------------------
        %==========================================================================
        %                    end of methods
        %==========================================================================
    end
end