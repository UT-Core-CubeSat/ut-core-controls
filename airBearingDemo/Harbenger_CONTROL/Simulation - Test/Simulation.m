clear();
clc();
close();
close all;

    
Parameters
observer = Observer(parameters);
dynamics = Dynamics(parameters);
sensors = Sensors(parameters);

time = 0:0.01:30;

% Initialize state as 0
x = zeros(7,1);
x(1:4) = [ 1,0,0,0];

u = [0,0,0]';

r(1:4) = [0.5, -0.5, -0.5, -0.5]';

r(1:4) = r(1:4) ./ norm(r(1:4));

r=r';


for i = 1:length(time)

    measurements = sensors.update(x, 0, time(i), false);                   

 	x_hat = observer.update(u, measurements);
    u = controller(x_hat,r,time(i));

			% Saturate Inputs
			
			tau = u;
        
			% Propagate Model
			x = Integrator.rk4(@(x) kinetics(x,tau),x,0.01);

			% Normalize
			x(1:4) = x(1:4) ./ norm(x(1:4));

	y = x(1:4);
	    
end

function x_dot = kinetics(self,x,u)
            I = self.I;

			
			% Wz
			q = x(1:4);
			omega = x(5:7);

			q_dot = 0.5 .* quatmultiply([0;omega]',q');

			omega_dot = inv(I) * (u - cross(omega, I * omega));

			x_dot = [q_dot;omega_dot];
end
function u = controller(self, x,r,time)

r(1:4) = [0.5, -0.5, -0.5, -0.5]';

r(1:4) = r(1:4) ./ norm(r(1:4));

r=r';
            % State is in Quaterinon   
            % Calculate error Quaternion
                % state Quaternion Matrix inverse multiplied by refrence
            q_e = invquatmultiply(x(1:4),r);
            
            quaternion_error = sqrt(sum((x(1:4) - r(1:4)).^2));
                
                 if abs(quaternion_error ) < 0.001 && ~parameters.controlledPrinted % Use tolerance for floating point comparison
                    fprintf('Controlled - Time: %.3f seconds\n', time);
                    parameters.controlledPrinted = true;
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

             % Extract only the quaternion columns from the table
             q_table = parameters.gains(:, 1:4); 

            Q_e = invquatmultiply(x(1:4), q_table');

             % Find the index of the nearest match
                [~, nearest_idx] = max(abs(Q_e(1,:)));
                x_e = self.gains(nearest_idx,1:4);

             % Retrieve the corresponding gains [Kx, Kr] from the table
                K_x = reshape(self.gains(nearest_idx, 5:22),3,[]); % [Kx]
                K_r = reshape(self.gains(nearest_idx, 23:31),3,[]); % [Kr]

           
            u = -K_x * [MRP-quat2MRP(x_e); omega] + K_r * (MRP_r-quat2MRP(x_e));
            
            RPM = tau2RPM(u);

            RPM(RPM > 2000) = 2000;
            RPM(RPM < -2000) = -2000;

            u = RPM2tau(RPM);
end