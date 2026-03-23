classdef Dynamics < handle
    properties
        % list class properties here.
        Ts
        mu_E
        r_E
        I 
        G
        states
        reference
        states_dot
        S
        I_wheel
        b_wheel
        b_dot_test
        c_w
        tau_c
        omega_eps
    end
    methods
        function self = Dynamics(Param)

            % simulation time step
            self.Ts = Param.Ts; 

            % mass properties
            self.mu_E = Param.mu_E; % gravitational parameter
            self.r_E = Param.r_E;   % radius of earth

            % initialize states 
            self.states = Param.states_init;            

            % inertia tensor
            self.I = Param.I;

            % gravitational constant
            self.G = Param.G;
            
            % reaction wheels 
            self.S = Param.S;
            self.I_wheel = Param.I_wheel;
            self.c_w = 0;
            self.tau_c = 0;
            self.omega_eps = 0;

            self.states_dot = zeros(size(self.states));
        end

        function update(self, inputs, mode)
            % Propigate the forces, moments, and state
            
            % update state variables one step with numerical integrator
            self.rk4_kinetics(inputs, mode);  

        end

        function rk4_kinetics(self, inputs, mode)
            % Integrate ODE using Runge-Kutta RK4 algorithm

            F1 = self.kinetics(self.states,inputs, mode);
            F2 = self.kinetics(self.states + self.Ts/2*F1,inputs, mode);
            F3 = self.kinetics(self.states + self.Ts/2*F2,inputs, mode);
            F4 = self.kinetics(self.states + self.Ts  *F3,inputs, mode);

            self.states = self.states + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            self.states(7:10) = self.states(7:10) / norm(self.states(7:10));
            % Clamp wheel speeds to physical limits (matches C++ behavior)
            omega_w_max = 2*pi/60 * 13600; % 13600 RPM in rad/s
            self.states(14:17) = max(min(self.states(14:17), omega_w_max), -omega_w_max);
        end

        
        function states_dot = kinetics(self,states,inputs, mode)
            % find time derivative of states (6 DOF equations) equations
            % can be found on page 317 of Small Unmanned Aircraft by Beard
            % & McLain

            % unpack states
            R     = states( 1:3);  % inertial position vector
            V     = states( 4:6);  % inertial velocity vector
			q     = states( 7:10); % quaturnion vector
			omega = states(11:13); % angular velocity vector
            omega_wheel = states(14:17);

            % calculate forces and moments
            [~,tau] = self.forces_and_moments(inputs, mode); % forces and moments acting on sat
            tau_rw = tau(1:4);
            tau_mtq = tau(5:7);

            h_w = self.S * (self.I_wheel * omega_wheel);
            tau_fric = 0;
            tau_ext = tau_mtq - self.S * (tau_rw - tau_fric);


            % Equation 1: Translational Velocity (inertial frame)
            R_dot = V;

            % Equation 2: Two Body Problem (translational acceleration)
            % Orbital Mechanics for Engineering Students (Eqn 2.22)
            V_dot = two_body(self.mu_E,self.r_E,R);

            % Equation 3: Quaternian Angular Velocity (body frame)
            % UAV Book (Eqn B.5)
			q_vec = q(2:4);   % vector part (x,y,z)
            q0    = q(1);     % scalar part
            
            q_dot_vec = 0.5*( q0*omega + cross(q_vec, omega) );
            q_dot0    = -0.5*(q_vec.'*omega);

            q_dot = [q_dot0; q_dot_vec];

            % Equation 4: Eulers Angular Acceleration (body frame)
            % Mechanics of Space Systems (Eqn 4.35)
			omega_dot = self.I\(tau_ext - cross(omega, self.I * omega + h_w));  


            omega_wheel_dot = self.I_wheel \ (tau_rw - tau_fric);
            
            % pack time derivative of states
			states_dot = [R_dot;V_dot;q_dot;omega_dot;omega_wheel_dot];

            % save for external use
            self.states_dot = states_dot;
        end
        
        function [forces, moments] = forces_and_moments(~,inputs, mode)

            % calculate forces and moments acting on satellite
            forces  =    zeros(3,1);
            moments =    inputs;


        end
    end % end of methods section
end