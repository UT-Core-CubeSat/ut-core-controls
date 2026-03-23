% classdef Parameters < handle
% 	properties
% 
%     end
% end

parameters.sample = 0.01;
parameters.length = 150;
parameters.title = "CubeSat";
parameters.plots = [3,3];
parameters.time = 0;


parameters.C_m = [ 1, 0, 0, 0, 0, 0, 0;
				   0, 1, 0, 0, 0, 0, 0;
				   0, 0, 1, 0, 0, 0, 0;	
				   0, 0, 0, 1, 0, 0, 0;];
parameters.D_m = 0;
parameters.C_r     = [1,0,0,0,0,0;
                     0,1,0,0,0,0;
                     0,0,1,0,0,0;
                     ];

parameters.D_r     = zeros(3, 3);



% Observer Tuning Parameters
parameters.P_0 = diag([.1, .1, .1, 1, 1, 1]);
parameters.G   = diag([1, 1, 1, 1, 1, 1]);
parameters.Q1   = diag([0, 0, 0, 1, 1, 1]);
 parameters.R1   = diag([.1, .1, .1]);

parameters.R = diag([1, 5, 5]) .* 20;

parameters.Q = diag([1,10,10,0,0,0]) .* 1;

% Control Tuning Parameters
parameters.Q_a = diag([1,1,1,1,1,1,1,1,1]);
parameters.R_a = 5;
parameters.N_a = zeros(9, 3);

            q1 = 1;
			q2 = 0;
			q3 = 0;
			q4 = 0;
			wx = 0;
			wy = 0;
			wz = 0;


            parameters.I = [36857.5, .06,     -7.6;
                            .06,     36771.47, 42.7;
                            -7.6,  42.7,     7705.886].*10^-6;

            %parameters.I = diag([1/3, 1/3, 1/3]);

            Ixx = parameters.I(1,1);
			Iyy = parameters.I(2,2);
			Izz = parameters.I(3,3);

            % Gyroscope noise standard deviations
            parameters.sigma_gyro_x = 0.05;
            parameters.sigma_gyro_y = 0.05;
            parameters.sigma_gyro_z = 0.05;
            
            % Magnetometer noise standard deviations
            parameters.sigma_mag_x = 0.02;
            parameters.sigma_mag_y = 0.02;
            parameters.sigma_mag_z = 0.02;
            
            % Magnetic field parameters
            parameters.declination = deg2rad(5);      % radians
            parameters.inclination = deg2rad(60);     % radians
            parameters.field_strength = 4.2e-5;       % Tesla (example value)
            
            % Star tracker noise
            parameters.sigma_star_quaternion = 0.001;
            
            % Star tracker sample time
            parameters.Ts_star_tracker = 1.0;         % seconds


			%Calculate A
			%Non Linearized

			

		parameters.A = [...
			0, -wx/2, -wy/2, -wz/2,                 -q2/2,                -q3/2,                 -q4/2;
            wx/2,     0,  wz/2, -wy/2,                  q1/2,                -q4/2,                  q3/2;
            wy/2, -wz/2,     0,  wx/2,                  q4/2,                 q1/2,                 -q2/2;
            wz/2,  wy/2, -wx/2,     0,                 -q3/2,                 q2/2,                  q1/2;
               0,     0,     0,     0,                     0, (wz*(Iyy - Izz))/Ixx,  (wy*(Iyy - Izz))/Ixx;
               0,     0,     0,     0, -(wz*(Ixx - Izz))/Iyy,                    0, -(wx*(Ixx - Izz))/Iyy;
               0,     0,     0,     0,  (wy*(Ixx - Iyy))/Izz, (wx*(Ixx - Iyy))/Izz,                     0;
               ];

			% Calculate B
			parameters.B = [
	                        0,     0,     0;
                            0,     0,     0;
                            0,     0,     0;
                            0,     0,     0;
                        1/Ixx,     0,     0;
                            0, 1/Iyy,     0;
                            0,     0, 1/Izz;
	                        ];

            parameters.gains = load("gains.mat").p;
%     % Gain Scheduling 
%        psi_min =   0;
%        psi_max =   315;
%        theta_min = -90;
%        theta_max = 90;
%        phi_min =   0;
%        phi_max =   315;
%        num_psi =   8;
%        num_theta = 5;
%        num_phi =   8;
%        parameters.gains = zeros(num_phi*num_theta*num_psi,31);
%         i=1;
%        for psi = psi_min:(psi_max-psi_min)/(num_psi-1):psi_max
%            for theta = theta_min:(theta_max-theta_min)/(num_theta-1):theta_max
%                for phi = phi_min:(phi_max-phi_min)/(num_phi-1):phi_max
%                    rpy = [deg2rad(psi), deg2rad(theta), deg2rad(phi)];
%                    c = cos(rpy/2);
%                     s = sin(rpy/2);
%         
%                  % Compute quaternion elements
%                     qw = c(3)*c(2)*c(1)+s(3)*s(2)*s(1);
%                     qx = c(3)*c(2)*s(1)-s(3)*s(2)*c(1);
%                     qy = c(3)*s(2)*c(1)+s(3)*c(2)*s(1);
%                     qz = s(3)*c(2)*c(1)-c(3)*s(2)*s(1);
% 
%                   % Assemble quaternion
%                     q = [qw; qx; qy; qz];

% 
%                    i = i+1;
% 
% 
% 
%                end
%            end
%        end





