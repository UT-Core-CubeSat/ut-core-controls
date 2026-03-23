clear();
clc();
close();
close all;

% initialize instances of objects    
Parameters
animation  = Animation();
observer   = Observer(parameters);  % EKF state estimation
controller = Controller(parameters);% LQR controller
dynamics   = Dynamics(parameters);
sensors    = Sensors(parameters);   % Sensors modeled with noise

% set simulation time
time = 0:0.01:300;

% Initialize state as 0
x = zeros(7,1);      % [quarturnion;omega]
x(1:4) = [ 1,0,0,0]; % initialize quarturnion
%% Initialize Plots

plots.q1_r.color = "r";
plots.q1_r.unit = "";
plots.q1_r.index = 1;
plots.q1_r.value = 0;

plots.q1_hat.color = "g";
plots.q1_hat.unit = "";
plots.q1_hat.index = 1;
plots.q1_hat.value = 0;

plots.q1.color = "b";
plots.q1.unit = "";
plots.q1.index = 1;
plots.q1.value = 0;

plots.q2_hat.color = "g";
plots.q2_hat.unit = "";
plots.q2_hat.index = 2;
plots.q2_hat.value = 0;

plots.q2_r.color = "r";
plots.q2_r.unit = "";
plots.q2_r.index = 2;
plots.q2_r.value = 0;

plots.q2.color = "b";
plots.q2.unit = "";
plots.q2.index = 2;
plots.q2.value = 0;

plots.q3_hat.color = "g";
plots.q3_hat.unit = "";
plots.q3_hat.index = 3;
plots.q3_hat.value = 0;

plots.q3_r.color = "r";
plots.q3_r.unit = "";
plots.q3_r.index = 3;
plots.q3_r.value = 0;

plots.q3.color = "b";
plots.q3.unit = "";
plots.q3.index = 3;
plots.q3.value = 0;

plots.q4_r.color = "r";
plots.q4_r.unit = "";
plots.q4_r.index = 4;
plots.q4_r.value = 0;

plots.q4_hat.color = "g";
plots.q4_hat.unit = "";
plots.q4_hat.index = 4;
plots.q4_hat.value = 0;

plots.q4.color = "b";
plots.q4.unit = "";
plots.q4.index = 4;
plots.q4.value = 0;

plots.Wx_r.color = "r";
plots.Wx_r.unit = "rad/sec";
plots.Wx_r.index = 5;
plots.Wx_r.value = 0;

plots.Wx.color = "b";
plots.Wx.unit = "rad/sec";
plots.Wx.index = 5;
plots.Wx.value = 0;

plots.Wy_r.color = "r";
plots.Wy_r.unit = "rad/sec";
plots.Wy_r.index = 6;
plots.Wy_r.value = 0;

plots.Wy.color = "b";
plots.Wy.unit = "rad/sec";
plots.Wy.index = 6;
plots.Wy.value = 0;

plots.Wz_r.color = "r";
plots.Wz_r.unit = "rad/sec";
plots.Wz_r.index = 7;
plots.Wz_r.value = 0;

plots.Wz.color = "b";
plots.Wz.unit = "rad/sec";
plots.Wz.index = 7;
plots.Wz.value = 0;

plots.Tx.color = "r";
plots.Tx.unit = "torque";
plots.Tx.index = 8;
plots.Tx.value = 0;

plots.Ty.color = "g";
plots.Ty.unit = "torque";
plots.Ty.index = 8;
plots.Ty.value = 0;

plots.Tz.color = "b";
plots.Tz.unit = "Newton-meters";
plots.Tz.index = 8;
plots.Tz.value = 0;

plots.RW1.color = "b";
plots.RW1.unit = "RPM";
plots.RW1.index = 9;
plots.RW1.value = 0;

plots.RW2.color = "g";
plots.RW2.unit = "RPM";
plots.RW2.index = 9;
plots.RW2.value = 0;

plots.RW3.color = "r";
plots.RW3.unit = "RPM";
plots.RW3.index = 9;
plots.RW3.value = 0;

plots.RW4.color = "c";
plots.RW4.unit = "RPM";
plots.RW4.index = 9;
plots.RW4.value = 0;
%% Create Plotter Object
P = Plotter(plots,parameters);
%% Set initial refrence
u = [0,0,0]';
r = zeros(7,1);
%r(1:4) = rand(4,1);
%r(1:4) = [ 0.1, 0.7, 0.1, 0.7];
%r(1:4) = [ .707, 0, 0.707, 0];
%r(1:4) = [  0.9694508, 0.1130111, 0.1130111, 0.1130111 ];
r(1:4) = [0.5,0.5, -0.5, -0.5];
%r(1:4)  = [0.0, sqrt(2)/2, 0.0, sqrt(2)/2];
%r(1:4) = [  -0.8433914, 0, 0, -0.5372996 ];
%r(1:4) = [  -0.0436194, 0, -0.9990482, 0 ];
%r(1:4) = [.8,.1,.1,.1];
%r(1:4) = [ 0.4796982, -0.4758966, 0.51935, 0.5231517 ];

r(1:4) = r(1:4) ./ norm(r(1:4));
%% Main Simulation Loop
for i = 1:length(time)
    if mod(time(i),20) == 0 % if current time is a multiple of 20 seconds reset refrence
        
        r      = zeros(7,1);             % zero out refrence
        r(1:4) = rand(4,1);              % randomly generate quarturnion
        r(1:4) = r(1:4) ./ norm(r(1:4)); % normalize new quarturnion
    end
    noise = .01;
%     measurements = [normrnd(x(1),     noise    );
%                     normrnd(x(2),     noise    );
%                     normrnd(x(3),     noise    );
%                     normrnd(x(4),     noise    );
%                     normrnd(x(5),     noise    );
%                     normrnd(x(6),     noise    );
%                     normrnd(x(7),     noise    )];

    measurements = sensors.update(x, 0, time(i), false);
                    

 	x_hat = observer.update(u, measurements);

	u = controller.update(x_hat, r);

    RPM = tau2RPM(u);

    PWM = tau2PWM(u);

	x = dynamics.update(x,u,0.01);

	y = x(1:4);
if mod(time(i),.1) == 0
	animation.update(y);
end
	drawnow;

	%pause(parameters.sample);
%%
    plots.q1.value = x(1);
    plots.q1_r.value = r(1);
    plots.q1_hat.value = x_hat(1);
    plots.q2.value = x(2);
    plots.q2_hat.value = x_hat(2);
    plots.q2_r.value = r(2);
    plots.q3.value = x(3);
    plots.q3_hat.value = x_hat(3);
    plots.q3_r.value = r(3);
    plots.q4.value = x(4);
    plots.q4_r.value = r(4);
    plots.q4_hat.value = x_hat(4);
    plots.Wx.value = x(5);
    plots.Wx_r.value = r(5);
    plots.Wy.value = x(6);
    plots.Wy_r.value = r(6);
    plots.Wz.value = x(7);
    plots.Wz_r.value = r(7);
    plots.Tx.value = u(1);
    plots.Ty.value = u(2);
    plots.Tz.value = u(3);
    plots.RW1.value = RPM(1);
    plots.RW2.value = RPM(2);
    plots.RW3.value = RPM(3);
    plots.RW4.value = RPM(4);
    parameters.time = parameters.time+parameters.sample;
%%
    P.update(plots,parameters);
    

	    
end