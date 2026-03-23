classdef Controller < handle
	properties
		R
		Q
        K
        J
        gains
        controlledPrinted
	end
	methods
		function self = Controller(parameters)

			self.R = parameters.R;

			self.Q = parameters.Q;

            self.J = parameters.I;

            self.gains = parameters.gains;

            self.controlledPrinted = false; 
        end
        function [K_x,K_r,x_e] = find_nearest_gains(self, quaternion)
            
             % Extract only the quaternion columns from the table
             q_table = self.gains(:, 1:4); 

            Q_e = invquatmultiply(quaternion, q_table');

             % Find the index of the nearest match
                [~, nearest_idx] = max(abs(Q_e(1,:)));
                x_e = self.gains(nearest_idx,1:4);

             % Retrieve the corresponding gains [Kx, Kr] from the table
                K_x = reshape(self.gains(nearest_idx, 5:22),3,[]); % [Kx]
                K_r = reshape(self.gains(nearest_idx, 23:31),3,[]); % [Kr]

         end
        function [K_x,K_r] = calculate_gains(self,x_e)
            inputs_e = zeros(3,1);
            [A,B,C,D] = modrod_state_space_fast(x_e, inputs_e, self.J);

            [K_x,~,~] = lqr(A,B,self.Q, self.R);
            
            n_x=size(A,1);
            n_y=size(C,1);
            n_u=size(B,2);
            %Non-Zero Set Point
            QPM = [A,B;C,D];

            P = QPM^-1;

            P12= P(1:n_x,end-n_y+1:end);
            P22 = P(end-n_u+1,end-n_y+1:end);

            K_r = [K_x * P12 + P22];
           
        end       
		function u = update(self,x,r,time)
      
            % State is in Quaterinon   
            % Calculate error Quaternion
                % state Quaternion Matrix inverse multiplied by refrence
            q_e = invquatmultiply(x(1:4),r(1:4));
            
            quaternion_error = sqrt(sum((x(1:4) - r(1:4)).^2));
                
                 if abs(quaternion_error ) < 0.001 && ~self.controlledPrinted % Use tolerance for floating point comparison
                    fprintf('Controlled - Time: %.3f seconds\n', time);
                    self.controlledPrinted = true;
                end

            % Negate Refrence if the quaternion long way(q0 < -1)
            if q_e(1) < 0
				% Negate the MRP refrence
                r(1:4) = -r(1:4);
            end

            % Convert Refrence to MRP
            MRP = quat2MRP(x(1:4));
            MRP_r = quat2MRP(r(1:4));

            % Feed Refrence into Gains with refrence gain too NZP
            omega = x(5:7);

            [K_x,K_r, x_e] = self.find_nearest_gains(x(1:4));
           
            u = -K_x * [MRP-quat2MRP(x_e); omega] + K_r * (MRP_r-quat2MRP(x_e));
            
            RPM = tau2RPM(u);

            RPM(RPM > 2000) = 2000;
            RPM(RPM < -2000) = -2000;

            u = RPM2tau(RPM);


        end
	end
end
