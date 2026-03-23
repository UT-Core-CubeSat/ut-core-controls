classdef Sensors < handle
    properties
        % Standard Deviation of Sensor Noise
        % IMU noise parameters
        sigma_gyro_x
        sigma_gyro_y
        sigma_gyro_z
        
        % Magnetometer noise parameters
        sigma_mag_x
        sigma_mag_y
        sigma_mag_z

        declination
        inclination
        field_strength
        
        % Star tracker noise parameters
        sigma_star_quaternion
        
        % Parameters
        Ts_star_tracker  % Star tracker update period
        
        % States for noise modeling
        nu_quat_w
        nu_quat_x
        nu_quat_y
        nu_quat_z
        
        % Timing
        last_star_tracker_update_t
        
        % Parameters for plotting
        last_star_tracker_update

    end
    methods
        function self = Sensors(parameters)
            % Standard Deviation of Sensor Noise
            self.sigma_gyro_x = parameters.sigma_gyro_x;
            self.sigma_gyro_y = parameters.sigma_gyro_y;
            self.sigma_gyro_z = parameters.sigma_gyro_z;
            
            self.sigma_mag_x = parameters.sigma_mag_x;
            self.sigma_mag_y = parameters.sigma_mag_y;
            self.sigma_mag_z = parameters.sigma_mag_z;

            self.declination = parameters.declination;
            self.inclination = parameters.inclination;
            self.field_strength = parameters.field_strength;

            
            self.sigma_star_quaternion = parameters.sigma_star_quaternion;
            
            % Parameters
            self.Ts_star_tracker = parameters.Ts_star_tracker;
            
            % States
            self.nu_quat_w = 0;
            self.nu_quat_x = 0;
            self.nu_quat_y = 0;
            self.nu_quat_z = 0;
            self.last_star_tracker_update_t = -Inf;
            
            % Plotting
            % Initial attitude as quaternion [w, x, y, z]
            self.last_star_tracker_update = [1; 0; 0; 0];
        end
        
        function y = update(self, states, magnetic_field, t, force_star_tracker)
            % Measure with sensors
            % states format: [qw; qx; qy; qz; wx; wy; wz]
            
            % Get gyroscope measurements
            [y_gyro_x, y_gyro_y, y_gyro_z] = self.gyroscope(states);
            
            % Get magnetometer measurements
           % [y_mag_x, y_mag_y, y_mag_z] = self.magnetometer(states, magnetic_field);
            
            % Create measurement vector
            %y = [y_gyro_x; y_gyro_y; y_gyro_z;
            %     y_mag_x; y_mag_y; y_mag_z];
            y = [y_gyro_x; y_gyro_y; y_gyro_z];
            
            % Only update the star tracker at a slower rate
            if ((t - self.last_star_tracker_update_t) >= self.Ts_star_tracker) || force_star_tracker
                [y_quat_w, y_quat_x, y_quat_y, y_quat_z] = self.star_tracker(states);
                y = [y; y_quat_w; y_quat_x; y_quat_y; y_quat_z];
                
                if ~force_star_tracker
                    self.last_star_tracker_update_t = t;
                    self.last_star_tracker_update = [y_quat_w; y_quat_x; y_quat_y; y_quat_z];
                end
            end
        end
        
        function [y_accel_x, y_accel_y, y_accel_z] = accelerometer(self, states, states_dot, ~)

            % Unpack
            qw = states(1);
            qx = states(2);
            qy = states(3);
            qz = states(4);
            wx = states(5);
            wy = states(6);
            wz = states(7);

            u_dot = states_dot(4);
            v_dot = states_dot(5);
            w_dot = states_dot(6);

           
            % Calculate the accelerometer measurments
            eta_accel_x = randn()*self.sigma_accel_x;
            eta_accel_y = randn()*self.sigma_accel_y;
            eta_accel_z = randn()*self.sigma_accel_z;

            y_accel_x= u_dot + q*w - r*v + self.g*sin(theta)+eta_accel_x;
            y_accel_y= v_dot + r*u - p*w - self.g*cos(theta)*sin(phi) + eta_accel_y;
            y_accel_z=w_dot +p*v-q*u-self.g*cos(theta)*cos(phi) + eta_accel_z;
        end
        function [y_mag_x, y_mag_y, y_mag_z] = magnetometer(self, states,~,~)
            % Unpack
            qw = states(1);
            qx = states(2);
            qy = states(3);
            qz = states(4);

           
            % Rotation Vehicle to Body
            Rv2b = 0;


            % Get declination and inclination at the location we are at
            delta = self.declination;
            i = self.inclination;

            %Magnetic feild in the body frame
            m_i = [cos(delta), -sin(delta), 0;
                    sin(delta), cos(delta), 0;
                    0, 0, 1]...
                    * [cos(-i), 0, sin(-i);
                      0, 1, 0;
                      -sin(-i), 0, cos(-i)] * [self.field_strength; 0; 0];


            % Random Noise
            eta_mag_x = randn()*self.sigma_mag_x;
            eta_mag_y = randn()*self.sigma_mag_y;
            eta_mag_z = randn()*self.sigma_mag_z;

            beta_mag = [self.beta_mag_x; self.beta_mag_y; self.beta_mag_z];
            eta_mag = [eta_mag_x; eta_mag_y; eta_mag_z];

            y_mag = Rv2b*(m_i + beta_mag) + eta_mag ;


            y_mag_x = y_mag(1);
            y_mag_y = y_mag(2);
            y_mag_z = y_mag(3);



        end

        function [y_gyro_x, y_gyro_y, y_gyro_z] = gyroscope(self, states)
            % Unpack states
            wx = states(5);
            wy = states(6);
            wz = states(7);
            
            % Calculate gyroscope measurements (with noise)
            eta_gyro_x = randn() * self.sigma_gyro_x;
            eta_gyro_y = randn() * self.sigma_gyro_y;
            eta_gyro_z = randn() * self.sigma_gyro_z;
            
            y_gyro_x = wx + eta_gyro_x;
            y_gyro_y = wy + eta_gyro_y;
            y_gyro_z = wz + eta_gyro_z;
        end
        function [y_quat_w, y_quat_x, y_quat_y, y_quat_z] = star_tracker(self, states)
           % Unpack states (quaternion part)
            qw = states(1);
            qx = states(2);
            qy = states(3);
            qz = states(4);
            
            % Calculate star tracker measurements (with noise)
            % For quaternions, noise can't be simply added as it would break the unit quaternion constraint
            % So we create a small random rotation quaternion and apply it
            
            % Create a small random rotation
            eta_angle = randn() * self.sigma_star_quaternion; % Random small angle
            eta_axis = randn(3,1);
            eta_axis = eta_axis / norm(eta_axis); % Normalize to unit vector
            
            eta_quat = [cos(eta_angle/2);
                        sin(eta_angle/2) * eta_axis];
            
            % Apply the noise as a quaternion multiplication
            quat_noisy = quatmultiply([qw; qx; qy; qz]', eta_quat');
            
            % Ensure it's a unit quaternion
            quat_norm = norm(quat_noisy);
            quat_noisy = quat_noisy / quat_norm;
            
            y_quat_w = quat_noisy(1);
            y_quat_x = quat_noisy(2);
            y_quat_y = quat_noisy(3);
            y_quat_z = quat_noisy(4);
        end
    end
end