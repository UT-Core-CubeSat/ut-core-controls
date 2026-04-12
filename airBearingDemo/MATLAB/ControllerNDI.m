classdef ControllerNDI < handle
    properties

        % Inertia
        I
        % Wheel mapping
        S
        S_pseudo
        N
        I_wheel

        % Wheel Saturation limits
        omega_w_max
        alpha_w_max
        tau_w_min
        tau_w_max
        lambda_min_model

        % MTQ for desaturation
        k_null
        k_desat
        m_max
        m_min

        % Reference model
        x_m

        % Time Step
        Ts
        t_end

        % Error dynamics gains for model and plant
        a0_model
        a1_model
        a0_plant
        a1_plant
    end
    methods

        function self = ControllerNDI(Param)

            % Instantiate above properties
            self.I = Param.I;

            self.I_wheel = Param.I_wheel;
            self.S = Param.S;
            self.S_pseudo = Param.S_pseudo;
            self.N = Param.N;

            self.omega_w_max = Param.RPM_max*2*pi/60;
            self.alpha_w_max = Param.alpha_max;
            self.tau_w_min = -Param.tau_max;
            self.tau_w_max = Param.tau_max;
            self.lambda_min_model = Param.lambda_min_model;
            % Desaturation for MTQ's -> RW's
            self.k_null = Param.k_null;
            self.k_desat = Param.k_desat;
            self.m_max = Param.m_max;
            self.m_min = -Param.m_max;

            % Time Step
            self.Ts = Param.Ts;
            self.t_end = Param.t_end;

            % Gains
            self.calculate_gains(Param.t_s_model, ...
                Param.zeta_model, ...
                Param.t_s_plant, ...
                Param.zeta_plant);

            % Initialize Control States
            q_init = [Param.states_init(8:10); Param.states_init(7)];
            omega_init = Param.states_init(11:13);
            self.x_m = [q_init; omega_init];
        end

        function [tau_wheel, tau_mtq, states_m] = update(self, states, reference, measurements)

            % Reference
            q_r   = [reference(2:4); reference(1)];
            q_r = q_r / norm(q_r);
            
            % States
            q     = [states(8:10); states(7)];
            q = q / norm(q);
            omega = states(11:13);
            omega_w = states(14:17);


            omega_r = reference(5:7);
            alpha_r = reference(8:10);

            % Outer Loop (Model -> Reference)
            x_model = self.update_reference_model(q_r, omega_r, alpha_r);
            q_m = x_model(1:4);
            omega_m = x_model(5:7);

            % Slow model down if we're about to saturate
            x_model_dot = self.reference_model_dif_eq(x_model, q_r, omega_r, alpha_r);
            omega_m_dot = x_model_dot(5:7);

            % REGULARIZE model reference IF plant → model quaternion error is too large
            [q_m_reg, omega_m_reg, omega_m_dot_reg] = ...
                self.regularize_reference(q, q_m, omega_m, omega_m_dot);

            q_m = q_m_reg;
            omega_m = omega_m_reg;
            omega_m_dot = omega_m_dot_reg;

            % Inner Loop (Plant -> Model)
            [E, C_BB, a] = self.compute_BP_Toolbox(q, omega, q_m, omega_m, "inner");

            % Compute Errors
            e_q = 2*E'*q_m;
            e_omega = omega - C_BB*omega_m;
            e_omega_dot = a*e_q - self.a1_plant*e_omega;

            omega_dot = C_BB*omega_m_dot ...
                + cross(C_BB*omega_m, omega) ...
                + e_omega_dot;

            % Compute torque from Euler's Rigid Body EQ
            tau_NDI = self.I*omega_dot + cross(omega, self.I*omega);

            % Get desat torque
            h_w = self.S * (self.I_wheel * omega_w);
            B_meas = measurements(17:19);
            [~, tau_mtq_expected] = self.wheel_desaturate(B_meas, h_w);
            
            % Convert to Wheel Torques, plus desat
            tau_tilde = self.allocateActuators(tau_NDI, tau_mtq_expected, omega_w);
            tau_wheel = self.applyWheelSaturation(tau_tilde, omega_w);
            tau_mtq = tau_mtq_expected;
            % Pack up model
            q_m_out = [q_m(4); q_m(1); q_m(2); q_m(3)];
            states_m = [q_m_out;omega_m];
        end

        function x_m_dot = reference_model_dif_eq(self, x_m, q_ref, omega_ref, alpha_ref)
            % Reference model based off linear error dynamics from Bach
            % Pailli Paper.
            q_m = x_m(1:4);
            q_m = q_m / norm(q_m);
            omega_m = x_m(5:7);

            [q_ref, omega_ref, alpha_ref] = ...
                self.regularize_reference(q_m, q_ref, omega_ref, alpha_ref);

            [E_m, C_BB_m, a_m] = self.compute_BP_Toolbox(q_m, omega_m, q_ref, omega_ref, "outer");

            e_q = 2*E_m'*q_ref;
            e_omega = omega_m - C_BB_m*omega_ref;
            e_omega_dot = a_m*e_q - self.a1_model*e_omega;
            x_m_dot_q = (E_m*omega_m)/2;
            x_m_dot_omega = C_BB_m*alpha_ref + cross(C_BB_m*omega_ref,omega_m) + e_omega_dot;

            x_m_dot = [x_m_dot_q;x_m_dot_omega];
        end

        function x_m = update_reference_model(self, q_ref, omega_ref, alpha_ref)
            % Integrate ODE using Runge-Kutta RK4 algorithm
            x_m_dot_1 = self.reference_model_dif_eq(self.x_m, q_ref, omega_ref, alpha_ref);
            x_m_dot_2 = self.reference_model_dif_eq(self.x_m + self.Ts/2*x_m_dot_1, q_ref, omega_ref, alpha_ref);
            x_m_dot_3 = self.reference_model_dif_eq(self.x_m + self.Ts/2*x_m_dot_2, q_ref, omega_ref, alpha_ref);
            x_m_dot_4 = self.reference_model_dif_eq(self.x_m + self.Ts  *x_m_dot_3, q_ref, omega_ref, alpha_ref);
            self.x_m = self.x_m + self.Ts/6 * (x_m_dot_1 + 2*x_m_dot_2 + 2*x_m_dot_3 + x_m_dot_4);
            self.x_m(1:4) = self.x_m(1:4) / norm(self.x_m(1:4));
            x_m = self.x_m;
        end

        function E = compute_E(~, q)
            % From Bach Pailli Paper
            E = [q(4), q(3), -q(2), -q(1);
                -q(3), q(4), q(1), -q(2);
                q(2), -q(1), q(4), -q(3)]';
        end

        function calculate_gains(self, t_s_model, zeta_model, t_s_plant, zeta_plant)
            % These eq's are based off of SETTLING time, rather than rise
            % time. t_s is defined as the time it takes to reach 10% of the
            % original error.

            % Nat frequencies
            omega_n_model = 4/(zeta_model*t_s_model);
            omega_n_plant = 4/(zeta_plant*t_s_plant);

            % Error gains
            self.a1_model = 2*zeta_model*omega_n_model;
            self.a0_model = omega_n_model^2;
            self.a1_plant = 2*zeta_plant*omega_n_plant;
            self.a0_plant = omega_n_plant^2;
        end

        function [E_m, C_BB, a] = compute_BP_Toolbox(self, q, omega, q_desired, omega_desired, loop)
            % Bach & Paielli Toolbox. Creates linearized error dynamics
            % from weird properties of quaternion kinematics.

            % This "toolbox" will compute all quantities necessary to
            % linearly drive q and omega to q_desired and omega_desired.
            % Pretty cool!

            lambda = q'*q_desired;

            % This "E" actually just defines quaternion multiplication
            % directly. Look in the Bach Pailli paper for explanation.
            E_ref = self.compute_E(q_desired);
            E_m = self.compute_E(q);

            % Auxiliary error vector and matrix
            q_aux = E_ref'*q;
            Q = E_ref'*E_m;
            Q_inv = Q' + q_aux*q_aux'/lambda;

            % Rotation matrix to get from omega to omega desired
            C_BB = Q_inv*Q';
            e_omega = omega - C_BB*omega_desired;

            % If we're in the outer loop (model -> reference), we need to
            % treat a, the most important gain quantity in the toolbox, as
            % the MODEL rather than the PLANT. The Controls White Paper
            % explains this in detail.
            if loop == "outer"
                a = (self.a0_model - (e_omega'*e_omega)/4)/lambda;
            else
                a = (self.a0_plant - (e_omega'*e_omega)/4)/lambda;
            end

        end

        function tau_w = allocateActuators(self, tau_req, tau_mtq_expected, omega_w)
            tau_to_wheels = tau_mtq_expected - tau_req;
            tau_nom = self.S_pseudo * tau_to_wheels;

            omega_avg = mean(omega_w);
            omega_err = omega_avg - omega_w;

            tau_null = self.k_null * (self.N * omega_err);

            tau_w = tau_nom + tau_null;
        end

        function [m_cmd, tau_mtq_expected] = wheel_desaturate(self, B_meas, h_w)
            B_norm = norm(B_meas);
            if B_norm < 1e-7
                m_cmd = zeros(3,1); 
                tau_mtq_expected = zeros(3,1);
                return;
            end

            % Cross product Control Law 
            m_cmd = self.k_desat * cross(h_w, B_meas) / B_norm;
            m_cmd = Saturate(m_cmd, self.m_min, self.m_max);
            tau_mtq_expected = cross(m_cmd, B_meas);
        end

        function tau_w_sat = applyWheelSaturation(self, tau_w_cmd, omega_w)
            tau_w_sat = Saturate(tau_w_cmd, self.tau_w_min, self.tau_w_max);

            for i = 1:4
                w = omega_w(i);
                u = tau_w_sat(i);

                if (w >= self.omega_w_max) && (u > 0)
                    tau_w_sat(i) = 0;
                elseif (w <= -self.omega_w_max) && (u < 0)
                    tau_w_sat(i) = 0;
                end
            end               
        end

        function [q_ref_eff, omega_ref_eff, alpha_ref_eff] = ...
                regularize_reference(self, q_m, q_ref, omega_ref, alpha_ref)

            % Ensure unit quaternions
            q_m   = q_m / norm(q_m);
            q_ref = q_ref / norm(q_ref);

            % Make sure we use the short way: if dot < 0, flip reference
            lambda = q_m' * q_ref;

            % Clamp numerical noise
            lambda = min(max(lambda, -1.0), 1.0);

            % Current error angle
            theta_err = 2 * acos(lambda);

            % Max error angle allowed in model (derived from lambda_min_model)
            lambda_min = self.lambda_min_model;
            lambda_min = min(max(lambda_min, 0.0), 1.0);
            theta_max  = 2 * acos(lambda_min);

            % If error is already within allowed range, do nothing
            if theta_err <= theta_max
                q_ref_eff     = q_ref;
                omega_ref_eff = omega_ref;
                alpha_ref_eff = alpha_ref;
                return;
            end

            % Otherwise, move the effective reference part-way along the geodesic
            % from q_m to q_ref, so that the new error is exactly theta_max.

            % Fraction of the total geodesic length we are allowed to "see":
            % sigma in (0,1)
            sigma = theta_max / theta_err;

            % Slerp from q_m to q_ref with parameter sigma
            if theta_err < 1e-6
                % practically identical - avoid division by zero
                q_ref_eff = q_ref;
            else
                % Slerp formula for unit quaternions
                half_theta    = theta_err / 2;
                sin_half_th   = sin(half_theta);
                % We want the new error angle to be theta_max, so we move
                % by sigma of the total error along the geodesic.
                half_theta_new = half_theta * sigma;

                w0 = sin(half_theta - half_theta_new) / (sin_half_th + 1e-12);
                w1 = sin(half_theta_new) / (sin_half_th + 1e-12);

                q_ref_eff = w0 * q_m + w1 * q_ref;
                q_ref_eff = q_ref_eff / norm(q_ref_eff);
            end

            % Rough but effective: scale rates/accelerations by the same sigma.
            % This keeps the "aggressiveness" of the reference consistent with
            % the reduced angle step.
            omega_ref_eff = sigma * omega_ref;
            alpha_ref_eff = sigma * alpha_ref;

        end

    end
end