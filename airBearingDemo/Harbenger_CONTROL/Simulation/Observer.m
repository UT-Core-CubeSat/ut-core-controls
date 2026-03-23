classdef Observer < handle
	properties
        % =====================================================================
        % EKF STATE VARIABLES
        % =====================================================================
        states_hat      % Estimated state: [MRP(1:3); omega(4:6)]
        P               % Covariance matrix (6x6) - uncertainty in state estimates

        % =====================================================================
        % SYSTEM MODEL MATRICES
        % =====================================================================
        C_m             % Measurement matrix for gyroscopes: H = [0, I] extracts omega
        D_m             % Direct feedthrough (unused, set to 0)
        G               % Process noise distribution matrix
        Q               % Process noise covariance (models gyro drift, dynamics uncertainty)
        R               % Measurement noise covariance (gyroscope noise)
        I               % Satellite inertia tensor
        Ts              % Sample time for integration
		
	end
	methods
        function self = Observer(parameters)
        % Observer States
            self.states_hat = [0;0;0;0;0;0];
            self.P          = parameters.P_0;

             % Model
            self.C_m = [zeros(3,3),eye(3)];
            self.D_m = parameters.D_m;

        % Observer gains
            self.G = parameters.G;
            self.Q = parameters.Q1;
            self.R = parameters.R1;

            self.Ts = parameters.sample;
            self.I = parameters.I;
			
		end
        function x = update(self, inputs, measurements)
			% =================================================================
            % MAIN EKF UPDATE CYCLE
            % ================================================================= 
            % INPUTS:
            %   inputs: Control torques applied to satellite [Tx, Ty, Tz]
            %   measurements: Sensor readings [gyro_x, gyro_y, gyro_z, optional_star_tracker]
            % 
            % OUTPUT:
            %   x: Estimated state as quaternion + angular velocity [q0,q1,q2,q3,wx,wy,wz]
            % =================================================================

			% STEP 1: PREDICTION PHASE
            % Use satellite dynamics and gyroscope measurements to predict next state
            self.propigate_ekf(inputs, measurements);

			% STEP 2: CORRECTION PHASE (when star tracker data available)
            % Star tracker provides absolute attitude measurement to
            % correct drift
            if length(measurements)>3   % Check if star tracker data included
                % measurements(4:end) contains quaternion from star tracker
                self.star_update_ekf(inputs, measurements(4:end));
            end
			
            % STEP 3: OUTPUT CONVERSION
            % Convert internal MRP representation to quaternion for external use
            x = [MRP2quat(self.states_hat(1:3));self.states_hat(4:6)];

        end
        function [x_hat_dot,P_dot] = ekf_dif_eq(self, x, P, inputs, measurements)

            % =================================================================
            % EKF DIFFERENTIAL EQUATIONS
            % =================================================================
            % Computes the time derivatives for both state estimate and covariance
            % This is the core of the continuous-discrete EKF formulation
            %
            % THEORY: EKF equations are:
            %   x_hat_dot = f(x_hat, u) + K*(y - h(x_hat))    [State prediction + correction]
            %   P_dot = A*P + P*A' - P*C'*R^-1*C*P + G*Q*G'  [Covariance evolution]
            %
            % WHERE:
            %   f(): Nonlinear dynamics function
            %   h(): Measurement function  
            %   A: Jacobian of dynamics (linearization)
            %   K: Kalman gain
            % =================================================================

            % Get linearized system matrices around current state estimate
            % This automatically computes Jacobians for MRP-based dynamics
            [A,~,~,~]= modrod_state_space_fast([MRP2quat(x(1:3));x(4:6)],inputs,self.I);
            
            % Extract control input
            u = inputs;
            

            %%%%%%%  START HERE %%%%%%%%%


            
             % Nonlinear differential equations to get a more accurate
            % propigation phase.
			q = MRP2quat(x(1:3));   % Current attitude as quaternion
			omega = x(4:6);         % Current angular velocity
            sigma = x(1:3);         % Current attitude as MRP

			% Quaternion kinematics: q_dot = 0.5 * Omega(omega) * q
            % where Omega(omega) is the quaternion multiplication matrix
            q_dot = 0.5 .* quatmultiply([0;omega]',q');
			
            % Euler's equation for rigid body: I*omega_dot = u - omega x (I*omega)
            I = self.I;
			omega_dot = inv(I) * (u - cross(omega, I * omega));
           
            % Convert quaternion derivative to MRP derivative
            % This uses the relationship: sigma_dot = B(sigma) * omega
            % where B(sigma) is the MRP kinematic matrix
            sigma_dot = 1/4*((1-norm(sigma)^2)*eye(3) + 2*cross_mat(sigma) + 2*(sigma*sigma'))*omega;
            
            % Combined state derivative in MRP coordinates
            f = [sigma_dot;omega_dot];

            % ===========================
            % KALMAN FILTER EQUATIONS
            % ===========================
            
            % Current measurement (gyroscopes)
            y_m = measurements(1:3);

            % Kalman Gain: K = P*C'*R^-1
            K_K = P*self.C_m'*self.R^-1;

            % Kalman differential equations.
            y_hat = self.C_m*x;% Technically X_hat

            % EKF state derivative: prediction + correction
            % x_hat_dot = f(x_hat, u) + K*(y_measured - y_predicted)
            x_hat_dot = f + K_K*(y_m - y_hat);  % Extract angular velocity from state

            % EKF covariance derivative (Riccati equation):
            % P_dot = A*P + P*A' - P*C'*R^-1*C*P + G*Q*G'
            %         ^^^^^^^^^^   ^^^^^^^^^^^^^^^^   ^^^^^^^
            %         Uncertainty  Measurement        Process
            %         growth       correction         noise
            P_dot = A*P + P*A' - P*self.C_m'*self.R^-1*self.C_m*P + self.G*self.Q*self.G';

        end
        function [x_hat,P] = star_update_ekf(self, inputs, measurements)
           
            % =================================================================
            % STAR TRACKER MEASUREMENT UPDATE
            % =================================================================
            % This implements the discrete measurement update when star tracker
            % data is available. Star tracker provides absolute attitude reference
            % to correct for gyroscope drift and improve long-term accuracy.
            %
            % PROCESS:
            % 1. Convert star tracker quaternion to MRP for consistency
            % 2. Apply standard Kalman measurement update equations
            % 3. Update both state estimate and covariance
            % =================================================================

            % Use current covariance estimate
            P = self.P;

            %R = self.R(7:end,7:end); % Only pull off Star Tracker 
            R = self.R;

            % Convert star tracker quaternion measurement to MRP
            y_m = quat2MRP(measurements);

            % Measurement matrix for star tracker: we observe MRP directly
            % State: [MRP(1:3); omega(4:6)] -> Measurement: MRP(1:3)
            C = [eye(3),zeros(3,3)];

            y_hat = self.states_hat(1:3);

            P = P - P*C'*(C*P*C'+R)^(-1)*C*P;
            K_L = P*C'*R^-1;

            e_y =  y_m - y_hat;

            x_hat = self.states_hat + K_L * e_y;

            self.states_hat = x_hat;
            self.P = P;

            

        end
        function propigate_ekf(self, inputs, measurements)
            %% Propogate EKF Forwared
			%[x_hat,P] = Integator.integrate([x_hat_dot,P_dot],self.ts);
            
            [x_hat_dot_1, P_dot_1] = self.ekf_dif_eq(self.states_hat,                         self.P,                     inputs, measurements);
            [x_hat_dot_2, P_dot_2] = self.ekf_dif_eq(self.states_hat + self.Ts/2*x_hat_dot_1, self.P + self.Ts/2*P_dot_1, inputs, measurements);
            [x_hat_dot_3, P_dot_3] = self.ekf_dif_eq(self.states_hat + self.Ts/2*x_hat_dot_2, self.P + self.Ts/2*P_dot_2, inputs, measurements);
            [x_hat_dot_4, P_dot_4] = self.ekf_dif_eq(self.states_hat + self.Ts  *x_hat_dot_3, self.P + self.Ts  *P_dot_3, inputs, measurements);
            self.states_hat = self.states_hat + self.Ts/6 * (x_hat_dot_1 + 2*x_hat_dot_2 + 2*x_hat_dot_3 + x_hat_dot_4);
            self.P     = self.P     + self.Ts/6 * (    P_dot_1 + 2*    P_dot_2 + 2*    P_dot_3 +     P_dot_4);
        end
	end
end

