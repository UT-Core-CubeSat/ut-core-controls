classdef Observer < handle
	properties
        % Observer States
        states_hat
        P

        % Model
        C_m
        D_m
        G
        Q
        R
        I
        Ts
		
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
			

			% Propigate the ekf forward in time
            self.propigate_ekf(inputs, measurements);

			% Check if there is any new sensor data
            if length(measurements)>3
                self.star_update_ekf(inputs, measurements(4:end));
            end
			 
            x = [MRP2quat(self.states_hat(1:3));self.states_hat(4:6)];

        end
        function [x_hat_dot,P_dot] = ekf_dif_eq(self, x, P, inputs, measurements)
%             % Unpack
%             q1 = x(1);
% 			q2 = x(2);
% 			q3 = x(3);
% 			q4 = x(4);
% 
% 			wx = x(5);
% 			wy = x(6);
% 			wz = x(7);
% 
%             Ixx = self.I(1,1);
% 			Iyy = self.I(2,2);
% 			Izz = self.I(3,3);

%             A = [...
% 			0, -wx/2, -wy/2, -wz/2,                 -q2/2,                -q3/2,                 -q4/2;
%             wx/2,     0,  wz/2, -wy/2,                  q1/2,                -q4/2,                  q3/2;
%             wy/2, -wz/2,     0,  wx/2,                  q4/2,                 q1/2,                 -q2/2;
%             wz/2,  wy/2, -wx/2,     0,                 -q3/2,                 q2/2,                  q1/2;
%             0,     0,     0,     0,                     0, (wz*(Iyy - Izz))/Ixx,  (wy*(Iyy - Izz))/Ixx;
%             0,     0,     0,     0, -(wz*(Ixx - Izz))/Iyy,                    0, -(wx*(Ixx - Izz))/Iyy;
%             0,     0,     0,     0,  (wy*(Ixx - Iyy))/Izz, (wx*(Ixx - Iyy))/Izz,                     0;
%             ];
            [A,~,~,~]= modrod_state_space_fast([MRP2quat(x(1:3));x(4:6)],inputs,self.I);
      
            u = inputs;
             % Nonlinear differential equations to get a more accurate
            % propigation phase.
			q = MRP2quat(x(1:3));
			omega = x(4:6);
            sigma = x(1:3);

			q_dot = 0.5 .* quatmultiply([0;omega]',q');
			I = self.I;
%             inputs_e = zeros(3,1);
%             x_e = zeros(7,1);
%             x_e(1) = 1;
%             [A,B,C,D] = modrod_state_space(x_e, inputs_e, I);

			omega_dot = inv(I) * (u - cross(omega, I * omega));
            
            %f = [q_dot(1:4);omega_dot];
            %q_dot to mrp_dot and add omega_dot to end
            %f = [((1 + q(1)) * q_dot(2:4) - q_dot(1) * q(2:4))/(1 + q(1))^2; omega_dot];
            sigma_dot = 1/4*((1-norm(sigma)^2)*eye(3) + 2*cross_mat(sigma) + 2*(sigma*sigma'))*omega;
            f = [sigma_dot;omega_dot];

            y_m = measurements(1:3);

            % Kalman Gain
            K_K = P*self.C_m'*self.R^-1;

            % Kalman differential equations.
            y_hat = self.C_m*x;% Technically X_hat

            x_hat_dot = f + K_K*(y_m - y_hat);

            P_dot = A*P + P*A' - P*self.C_m'*self.R^-1*self.C_m*P + self.G*self.Q*self.G';

        end
        function [x_hat,P] = star_update_ekf(self, inputs, measurements)
           

            P = self.P;
            %R = self.R(7:end,7:end); % Only pull off Star Tracker 
            R = self.R;

            y_m = quat2MRP(measurements);

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

