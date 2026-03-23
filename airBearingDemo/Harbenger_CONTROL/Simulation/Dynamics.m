classdef Dynamics < handle
	properties
		I
	end
	methods
		function self = Dynamics(parameters)
            self.I = parameters.I;

		end
		function tau = moments(self,u)
			 tau = u;
		end
		function x_dot = kinetics(self,x,u)
            I = self.I;

			% q1
			% q2
			% q3
			% q4
			% Wx
			% Wy
			% Wz
			q = x(1:4);
			omega = x(5:7);

			q_dot = 0.5 .* quatmultiply([0;omega]',q');

			omega_dot = inv(I) * (u - cross(omega, I * omega));

			x_dot = [q_dot;omega_dot];
		end
		function x = update(self,x,u,time)
			% Saturate Inputs
			
			tau = self.moments(u);
        

			% Propagate Model
			x = Integrator.rk4(@(x) self.kinetics(x,tau),x,time);

			% Normalize quaturnions
			x(1:4) = x(1:4) ./ norm(x(1:4));

			% Return State
		end
	end
end