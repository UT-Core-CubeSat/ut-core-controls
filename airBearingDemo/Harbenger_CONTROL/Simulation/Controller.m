classdef Controller < handle
	properties
		R
		Q
        K
        J
        gains     
	end
	methods
		function self = Controller(parameters)

			self.R = parameters.R;

			self.Q = parameters.Q;

            self.J = parameters.I;

            self.gains = parameters.gains;
        end
        function [K_x,K_r,x_e] = find_nearest_gains(self, quaternion)
            % GAIN SCHEDULING:
            % Purpose: Find pre-computed gains for the closest quaternion in the lookup table
            % Input: quaternion - current attitude quaternion [q0, q1, q2, q3]
            % Output: K_x - state feedback gain, K_r - reference gain, x_e - equilibrium quaternion
             
            % Extract only the quaternion columns from the lookup table
            % gains table structure: [q0, q1, q2, q3, Kx(18 elements), Kr(9 elements)]
            q_table = self.gains(:, 1:4); 

            % Calculate quaternion error between current quaternion and all table entries
            % invquatmultiply gives us the relative rotation between quaternions
            Q_e = invquatmultiply(quaternion, q_table');

            % Find the index of the nearest match
            % We use the scalar part (q0) of the error quaternion - closer to 1 means smaller error
                [~, nearest_idx] = max(abs(Q_e(1,:)));
                x_e = self.gains(nearest_idx,1:4);

             % Retrieve the corresponding gains [Kx, Kr] from the table
                K_x = reshape(self.gains(nearest_idx, 5:22),3,[]); % [Kx]
                K_r = reshape(self.gains(nearest_idx, 23:31),3,[]); % [Kr]

        end
           
        function [K_x,K_r] = calculate_gains(self,x_e)
            % REAL-TIME GAIN CALCULATION: Alternative to gain scheduling
            % Purpose: Calculate LQR gains for a specific equilibrium point
            % Input: x_e - equilibrium quaternion around which to linearize
            % Output: K_x - state feedback gain, K_r - reference feedforward gain
            
            % Set equilibrium control input (typically zero for attitude control)
            inputs_e = zeros(3,1);
            
            % LINEARIZATION: Get state-space matrices around equilibrium point x_e
            % This function linearizes the nonlinear satellite dynamics
            [A,B,C,D] = modrod_state_space_fast(x_e, inputs_e, self.J);
            
            % LQR DESIGN: Calculate optimal feedback gain
            % Solves: min∫(x'Qx + u'Ru)dt subject to ẋ = Ax + Bu
            [K_x,~,~] = lqr(A,B,self.Q, self.R);
            % Alternative pole placement approach (commented out):
            %[K_x,~,~] = place(A,B,[-1,-1,-1,-2,-2,-2]);
            
            % Get system dimensions
            n_x = size(A,1);  % Number of states
            n_y = size(C,1);  % Number of outputs  
            n_u = size(B,2);  % Number of control inputs
            
            % FEEDFORWARD DESIGN: Calculate reference tracking gain K_r
            % For non-zero setpoint tracking, we need feedforward gain
            % This ensures zero steady-state error to step references
            
            % Form the augmented system matrix for steady-state analysis
            QPM = [A, B;     % State equation: ẋ = Ax + Bu
                   C, D];    % Output equation: y = Cx + Du
            
            % Solve for steady-state gains
            P = QPM^-1;  % Inverse of augmented system matrix
            
            % Extract relevant blocks from the inverse
            P12 = P(1:n_x, end-n_y+1:end);           % State-to-output block
            P22 = P(end-n_u+1, end-n_y+1:end);      % Input-to-output block
            
            % Calculate reference feedforward gain
            % This ensures the steady-state output follows the reference
            K_r = [K_x * P12 + P22];
        end        
		function u = update(self,x,r)
      
            % QUATERNION ERROR CALCULATION
            % Calculate error quaternion: q_error = q_current^(-1) * q_reference
            q_e = invquatmultiply(x(1:4), r(1:4));
            
            % QUATERNION AMBIGUITY RESOLUTION
            % Quaternions have double coverage: q and -q represent same rotation
            % Choose the "short way" rotation (q0 > 0) for better performance
            if q_e(1) < 0
                % Negate the reference quaternion to take shorter path
                r(1:4) = -r(1:4);
            end

            % COORDINATE CONVERSION: Quaternion to Modified Rodrigues Parameters (MRP)
            % MRP provides better linearization properties near identity
            MRP = quat2MRP(x(1:4));      % Current attitude in MRP
            MRP_r = quat2MRP(r(1:4));    % Reference attitude in MRP

            % Extract angular velocity
            omega = x(5:7);

            % ========================================================================
            % GAIN CALCULATION: Currently using simple LQR - FUTURE ENHANCEMENT AREA
            % ========================================================================
            % TODO: Implement gain scheduling for improved performance
            % - Add mission mode detection (normal, precision, safe_mode, etc.)
            % - Switch between lookup table and real-time calculation based on:
            %   * Mission phase requirements
            %   * Computational resources available  
            %   * Distance from pre-computed operating points
            %   * Control performance metrics
            %
            % Future implementation structure:
            % if (mission_mode == "precision") || (attitude_outside_table_coverage)
            %     [K_x, K_r] = self.calculate_gains(x(1:4));  % Real-time LQR
            %     x_e = x(1:4);
            % else
            %     [K_x, K_r, x_e] = self.find_nearest_gains(x(1:4)); % Lookup table
            % end

 

            % CURRENT IMPLEMENTATION: Simple LQR for all conditions
            % Using identity quaternion as equilibrium point for simplicity
            x_e = x(1:4);  % Use current attitude as linearization point
            [K_x, K_r] = self.calculate_gains(x_e);
            
            % CONTROL LAW: LQR with reference feedforward
            % Note: Control law uses MRP for better linearization properties
            % u = -K_x * (x_mrp - x_equilibrium_mrp) + K_r * (r_mrp - x_equilibrium_mrp)
            % State error: [MRP_error; angular_velocity_error]
            state_error = [MRP - quat2MRP(x_e); omega];
            reference_error = MRP_r - quat2MRP(x_e);
            
            % Calculate control torques
            u = -K_x * state_error + K_r * reference_error;
            
            % ACTUATOR SATURATION: Convert to RPM and saturate
            RPM = tau2RPM(u);               % Convert torque to reaction wheel RPM
            RPM(RPM > 2000) = 2000;         % Upper saturation limit
            RPM(RPM < -2000) = -2000;       % Lower saturation limit
            u = RPM2tau(RPM);               % Convert back to torque


        end
	end
end
