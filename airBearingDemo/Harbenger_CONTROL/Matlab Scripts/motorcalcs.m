%% Motor Calculations

clear all, close all, clc

% Inertia

m = 5; % kg
z = 0.32; % m
x = 0.1; % m
y = 0.1; % m
rho_al = 2700; % density of aluminum
rho_ss = 7700; % density of stainless steel

I_xy = 1/12 * m * (y^2 + z^2); % Moment of inertia about x and y axis

I_z = 1/12 * m * (x^2 + y^2); % Inertia about z axis

% Parallel Axis Theorem

d_x = 0.035;
d_y = d_x;
d_z = sqrt(2*d_x^2);
I_x_prime = I_xy + m * d_x^2;
I_y_prime = I_x_prime;
I_z_prime = I_z + m *d_z^2;

ratio_prime = I_x_prime/I_z_prime;

theta = acos(sqrt(ratio_prime^2/(2*ratio_prime^2+1)));

phi = 90 - rad2deg(acos(sqrt((cos(theta))^2/ratio_prime^2))); %angle between xy plane

% Motor specs and reaction wheel specs
r_motor = 35/2000; % m
h_motor = 0.75 * 15/1000; %m
M = 0.042; % estimated mass of motor kg
%

I_motor = 0.75 * M* r_motor^2;

% Calculating speed of reaction wheels in rpm, for all three specs
tracking_speed = 1 / 90;


ws_xy = linspace(0.5 * tracking_speed, 2 * tracking_speed, 10); % Different Satellite speeds in rpm
ws_xy(end + 1) = 0.5;

wrw_motor = abs(I_x_prime * ws_xy/(4*cos(theta)*I_motor));

figure(1)
plot(ws_xy,wrw_motor,'g-*','linewidth',3)
hold on;
plot(tracking_speed,wrw_motor(5),'r');

xlabel("Speed of Satellite [rpm]")
ylabel("Different speeds of motor [rpm]")



