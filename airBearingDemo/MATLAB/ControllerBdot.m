classdef ControllerBdot < handle
    properties

        % Magnetorquer saturation limits
        m_min
        m_max

        % BDot Gain
        K_Bdot

        % Previous known B for numeric calculation of BDot
        B_prev

        % Time step
        dt
       
        % B dot itself
        B_dot

        % First order filtering for jittery BDot
        alpha_Bdot

        Bdot_num_filt
        beta_fuse

    end
    
    methods
        function self = ControllerBdot(Param)
            % Instantiate above properties           
            self.B_prev = zeros(3,1); %xyz
            self.Bdot_num_filt = zeros(3,1);
            self.beta_fuse = 0.1;
            self.B_dot = zeros(3,1); %xyz
            self.m_min = -Param.m_max;
            self.m_max = Param.m_max;
            self.K_Bdot = Param.K_Bdot;
            self.dt = Param.Ts;
            self.alpha_Bdot = Param.alpha_Bdot;
        end
    
        function [tau_sat, states_m] = update(self, measurements)
            
            % Outputs tau_sat, which is just m_sat x B at the current time.

            % m_sat: commanded magnetic dipole to run thru MTQ's
            % [mx;my;mz]. Unites A*m^2

            % B: measured magnetic field at current time step            
            
            B_now = measurements(17:19);
            omega_meas = measurements(4:6);

            % Make our first command 0 upon instantiation
            if all(self.B_prev == 0)
                self.B_prev = B_now;
                self.B_dot  = zeros(3,1);
                tau_sat     = [0;0;0]; 
                return;
            end
            Bdot_num = (B_now - self.B_prev) / self.dt;
            self.Bdot_num_filt = (1 - self.alpha_Bdot)*Bdot_num + self.alpha_Bdot*self.Bdot_num_filt;
            Bdot_gyro = -cross(omega_meas, B_now);
            % Simple numeric differentation + simple 1st order filter
            self.B_dot = (1-self.beta_fuse)*Bdot_gyro + self.beta_fuse * self.Bdot_num_filt;

            % m = -K*B_dot | Classic BDot control law
            m_tilde = -self.K_Bdot*self.B_dot;

            % Project m_tilde to push against the magnetic field. 
            if norm(B_now) > 1e-9
                Bh = B_now / norm(B_now);
                P_perp = eye(3) - Bh*Bh.';
                m_tilde = P_perp * m_tilde;
            end

            % Saturation
            m_sat = Saturate(m_tilde,self.m_min,self.m_max);

            % Compute resultant torques
            tau_sat = cross(m_sat, B_now);

            % Reference model doesn't exist here, but need to keep things
            % consistent with the Parent Controller class. 
            states_m = NaN(7,1);

            % This time step is now over; make prev = now
            self.B_prev = B_now;
        end
    end
end