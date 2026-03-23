clear all, close all, clc

addpath('C:\Users\trace\OneDrive\Documents\Fall 2024\mice\lib');
savepath;

% Load SPICE kernels
kernelFile = 'C:\Users\trace\OneDrive\Documents\Fall 2024\Kernel\de440.bsp';
cspice_furnsh(kernelFile);
cspice_furnsh('C:\Users\trace\OneDrive\Documents\Fall 2024\Kernel\earth_latest_high_prec.bpc');
cspice_furnsh('C:\Users\trace\OneDrive\Documents\Fall 2024\Kernel\pck00010.tpc');
cspice_furnsh('C:\Users\trace\OneDrive\Documents\Fall 2024\Kernel\naif0012.tls');

% Earth ellipsoid parameters
b = 6356.752 * 1000; % polar radius of Earth [m]
a = 3958.8 * 1609;   % equatorial radius of Earth [m]
phi = 40.7608 * pi / 180; % geodetic latitude (radians)
lambda = -111.891 * pi / 180; % longitude (radians)
h = 600 * 1000; % altitude [m]

% Compute the radius of curvature in the prime vertical
N = a^2 / sqrt(a^2 * cos(phi)^2 + b^2 * sin(phi)^2);

% ECEF Coordinates [X, Y, Z] in km
X = ((N + h) * cos(phi) * cos(lambda))/1000;
Y = ((N + h) * cos(phi) * sin(lambda))/1000;
Z = ((b^2 / a^2 * N + h) * sin(phi))/1000;

% Julian Date Calculation
Ye = 2003;
M = 5;
D = 21;
U = 7;
J = 367 * Ye - floor(7 * (Ye + floor((M + 9) / 12)) / 4) + floor(275 * M / 9) + D + 1721013.5 + U / 24;
T = (J - 2451545) / 36525;

% Calculate GMS (Greenwich Mean Sidereal time)
GMS = 280.46061837 + 360.98564736629 * (J - 2451545) + 0.000387933 * T^2 - T^3 / 38710000;
GMS_sub = round(GMS / 360);
GMS = GMS - 360 * GMS_sub;
theta = GMS * pi / 180; % Convert GMS to radians

% ECEF position vector
r_ecef = [X; Y; Z];


% Rotation matrix to convert ECEF to ICRF (Earth rotation)
Rz = [cos(theta), -sin(theta), 0;
      sin(theta), cos(theta), 0;
      0, 0, 1];

% Convert ECEF to ICRF
r_icrf = Rz * r_ecef;

% Display ICRF coordinates
disp('ICRF coordinates (manual calculation):');
disp(r_icrf);
X = r_icrf(1); % First element
Y = r_icrf(2); % Second element
Z = r_icrf(3); % Third element
x_data = [X]; % X coordinates
y_data = [Y]; % Y coordinates
z_data = [Z]; % Z coordinates

%% Spice checker
% Define the time for conversion
et = cspice_str2et('2003-05-21T07:00:00');  % Specify the desired time

% Get the rotation matrix from ECEF to ICRF
[rot_matrix] = cspice_pxform('IAU_EARTH', 'J2000', et);

% Convert ECEF coordinates to ICRF using SPICE
icrf_coords = rot_matrix * r_ecef;  % Transpose for matrix multiplication

% Display the ICRF coordinates
disp('ICRF Coordinates (SPICE calculation):');
disp(icrf_coords);
