% Time
Param.t_start = 0;
Param.t_end   = 120;
Param.Ts      = 0.025;
Param.t_plot  = 0.1;
Param.speed   = Inf;
%Param.DateTime = datetime("now","TimeZone","UTC");
Param.DateTime = datetime(2025,1,1,0,0,0,"TimeZone","UTC");
% Mission Schedule
Param.schedule = [
    struct('t_start', 0, ...
    't_end',        120, ...
    'mode',     'point', ...
    'face',     [0;0;1], ...
    'target',   'sun', ...
    'slew',     []); ...
    ];

% Earth and Sun
env = getEarthConstants();
sun = getSolarConstants();

% Spacecraft
sc = getSpacecraftProperties();

orbit = getOrbitProperties(env);
% Initial Orbit State
[r_ECI, v_ECI] = generateOrbitICs(orbit);

x0 = getInitialStates(r_ECI, v_ECI);

% Sensors and Actuators
sensors   = getSensorParams();
actuators = getActuatorParams();

% Control and Estimation
ctrl = getControllerParams();
est  = getObserverParams(sensors, Param);

% Plotting
animation = getAnimationSettings(orbit);
plotCfg = getPlotSettings();

% Flatten
Param = mergeStructs(Param, env);
Param = mergeStructs(Param, sun);
Param = mergeStructs(Param, sc);
Param = mergeStructs(Param, orbit);
Param = mergeStructs(Param, sensors);
Param = mergeStructs(Param, actuators);
Param = mergeStructs(Param, ctrl);
Param = mergeStructs(Param, est);
Param = mergeStructs(Param, plotCfg);
Param = mergeStructs(Param, animation);

% Add final state init
Param.states_init = x0;

%% End of Parameters - Functions below

% Environmental constants
function env = getEarthConstants()
env.G = 6.67408e-11;          % [m^3/kg/s^2]
env.m_earth = 5.974e24;       % [kg]
env.mu_E = env.G * env.m_earth;
env.r_E = 6378137;            % [m]
env.J2 = 1.08262668e-3;       % []
env.omega_earth = [0;0;7.2921159e-5];   % [rad/s]
end
function sun = getSolarConstants()
sun.S0 = 1361;                % [W/m^2]
sun.c  = 2.9979e8;            % [m/s]
sun.AU = 1.495978707e11;      % [m]
end

% Spacecraft
function sc = getSpacecraftProperties()
sc.mass = 5;                           % [kg]
I_principle = [0.0523, 0.0520, 0.0083]; % diagonals
Ixy = 0.0032; % first product of inertia 
Ixz = -0.0018; % second product of inertia
Iyz = 0.0025; % third product of inertia
theta = [I_principle(1);I_principle(2);I_principle(3);
                    Ixy;           Ixz;          Iyz];
sc.I = [theta(1), theta(4), theta(5);
        theta(4), theta(2), theta(6);
        theta(5), theta(6), theta(3)];   % [kg·m^2]
sc.dim = [0.1, 0.1, 0.3];             % [m]
end

% Orbit initialization
function orbit = getOrbitProperties(env)
orbit.altitude = 600E3;
orbit.inclination = deg2rad(45);
orbit.RAAN = deg2rad(0);
orbit.argPeriapsis = deg2rad(0);
orbit.trueAnomaly = deg2rad(0);
orbit.mu_E = env.mu_E;
orbit.a = orbit.altitude + 6378137;
orbit.e = 0;
end
function [r_ECI, v_ECI] = generateOrbitICs(orbit)
a = orbit.a;
e = orbit.e;
r_mag = a*(1-e^2)/(1+e*cos(orbit.trueAnomaly));
r_PQW = [r_mag*cos(orbit.trueAnomaly);
    r_mag*sin(orbit.trueAnomaly); 0];
v_PQW = [-sqrt(orbit.mu_E/a)*sin(orbit.trueAnomaly);
    sqrt(orbit.mu_E/a)*(e+cos(orbit.trueAnomaly));
    0];
RzO = [cos(orbit.RAAN) -sin(orbit.RAAN) 0;
    sin(orbit.RAAN)  cos(orbit.RAAN) 0;
    0 0 1];
Rxi = [1 0 0;
    0 cos(orbit.inclination) -sin(orbit.inclination);
    0 sin(orbit.inclination)  cos(orbit.inclination)];
Rzw = [cos(orbit.argPeriapsis) -sin(orbit.argPeriapsis) 0;
    sin(orbit.argPeriapsis)  cos(orbit.argPeriapsis) 0;
    0 0 1];
Q = RzO * Rxi * Rzw;
r_ECI = Q * r_PQW;
v_ECI = Q * v_PQW;
end

% Initial States (11 elements for air bearing: q, omega, omega_wheel)
function x0 = getInitialStates(~, ~)
q0 = [1;0;0;0];
omega = pi/180*[0;0;0];
omega_wheel = 2*pi/60*[0;0;0;0]; % array in RPM
x0 = [q0; omega; omega_wheel];  % 11 elements total
end

% Sensors
function s = getSensorParams()
s.reduction = 'IAU-2000/2006';
% Accelerometers
s.beta_a = 0 * ones(3,1);
s.sigma_a = 3E-4 * ones(3,1); % Accelerometer noise
% Gyros
s.sigma_bias_walk = deg2rad(0.002) * ones(3,1);
s.beta_gyro  = deg2rad(0.02) * ones(3,1);
s.sigma_gyro = deg2rad(0.1) * ones(3,1); % Gyro noise
% Magnetometers
s.beta_mag = 2E-7 * ones(3,1);
s.sigma_mag = 5E-8 * ones(3,1); % Magnetometer noise
% Star Tracker
s.beta_star = [0;0;0]; % [rad] - misalignment bias [x;y;z]

star_tracker_accuracy_arcsec = 100;  % Star tracker accuracy in arcseconds
s.sigma_star_rad = (star_tracker_accuracy_arcsec / 3600) * (pi/180); % Star tracker noise in radians
s.sigma_star = s.sigma_star_rad * ones(3,1);  % [rad]

s.star_update = .95;  % star tracker update probaility, 97% chance a frame is valid
s.q_bias = [1;0;0;0];      % quaternion bias (no bias)
s.small_angle_tol = 0.02;  % [rad] - threshold for small angle approximation
s.T_star = 0.5;            % [1/hz] - star tracker update rate

% Coarse Sun Sensors
s.beta_css = 0.1E-3 * ones(6,1); % [A] - static css bias [x;y;z]
s.sigma_css = 0.2E-3 * ones(6,1); % [A] - css standard deviation [x;y;z]
s.I_max = 10E-3;

% GPS Module
s.T_gps = 1/5;           % [1/hz] - update rate
s.k_gps = 1/1100;        % [1/tau] - pole of random walk (value from textbook)
s.beta_gps = [0;0;0];    % [m] - static gps bias [x;y;z]
s.sigma_gps_cep = 2;     % [m] - gps circular error probable standard deviation
s.K_gps = 1/1100;
s.sigma_cep = 1.699; % GPS std dev in meters
s.sigma_V = 0.1; % GPS velocity std dev in m/s

% Wheels 
s.sigma_wheel = 1e-5; % Reaction wheel speed noise
s.alpha_wheel = 0.01;
end

% Actuators
function a = getActuatorParams()

% Magnetorquers
a.m_max = 0.04; % A*m^2
a.k_desat = 25; %desat gain

% Reaction wheels
a.I_wheel = 1.13E-6;
a.RPM_max = 13600;
a.alpha_max = 90E3;
a.tau_max = 13E-3;
a.lambda_min_model = 0.1;
a.k_null = 2e-7;
% Geometry
wheel_thickness = 0.0112;
wheel_radius = 0.0162;
offset = 0.040; % radially, from center
theta = deg2rad(50); % tilt angle
S = [ cos(theta), -cos(theta),  0,            0;
      0,           0,           cos(theta),  -cos(theta);
      sin(theta),  sin(theta),  sin(theta),   sin(theta) ];
S = S ./ vecnorm(S);
a.S = S;
a.N = eye(size(a.S,2)) - a.S.' * ((a.S * a.S.')\a.S);
z_bias = -0.02;
r_center = offset * S + [0;0;z_bias];
a.r_top    = r_center + 0.5 * wheel_thickness * S;
a.r_bottom = r_center - 0.5 * wheel_thickness * S;

a.S_pseudo = pinv(a.S);
a.r_wheel = wheel_radius;
a.t_wheel = wheel_thickness;
end

% Controller (NDI + Bdot) - Tuned for Air Bearing Demo
function c = getControllerParams()

% NDI - faster settling for ground demo
c.t_s_plant = 5;     % [s] plant tracks model
c.zeta_plant = 0.9;
c.t_s_model = 12;    % [s] model tracks reference
c.zeta_model = 0.85;

% Bdot (not used for air bearing)
c.K_Bdot = 100000;
c.alpha_Bdot = 0.95;
end

% Observer (MEKF)
function e = getObserverParams(sensors, Param)
e.P_0 = diag([deg2rad(5)^2 * ones(1,3), deg2rad(0.5)^2 * ones(1,3)]);
e.G = [-eye(3), zeros(3,3);
    zeros(3,3), eye(3,3)];
gyro_density = sensors.sigma_gyro ./ sqrt(Param.Ts);
e.Q = diag([gyro_density'.^2, sensors.sigma_bias_walk'.^2]);
e.R_star = 4*(sensors.sigma_star_rad).^2 * eye(3);
quest_accuracy = deg2rad(5);
e.R_quest = (quest_accuracy^2) * eye(3);
end

% Plot settings
function p = getAnimationSettings(orbit)
% Satellite animation
p.sat_width = .1; % m
p.sat_length = .1; % m
p.sat_height = .3; % m
p.xLim_orientation = [-0.2, 0.2]; %m
p.yLim_orientation = [-0.2, 0.2]; %m
p.zLim_orientation = [-0.2, 0.2]; %m

% Orbit animation
p.orbit_margin = 10E3;
p.xLim_orbit = [-(orbit.a + p.orbit_margin),(orbit.a + p.orbit_margin)];
p.yLim_orbit = p.xLim_orbit;
p.zLim_orbit = p.xLim_orbit;
p.line_width   = 1.5;
end

function p = getPlotSettings()
p.active_figure = 1;
p.animate_orientation = false;
p.animate_orbit       = false;
p.plot_translation    = false;
p.plot_orientation    = true;
p.plot_input          = true;
p.plot_RPM            = true;
p.plot_reference      = true;
p.plot_estimate       = true;
p.plot_model          = false;
p.plot_adapt          = false;
p.plot_error          = false;
p.plot_measurements   = false;
end

function s = mergeStructs(s, new)
fields = fieldnames(new);
for i = 1:numel(fields)
    s.(fields{i}) = new.(fields{i});
end
end