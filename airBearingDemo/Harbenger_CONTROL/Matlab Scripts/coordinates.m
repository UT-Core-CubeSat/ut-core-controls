clear all, close all, clc

addpath('C:\Users\trace\OneDrive\Documents\Fall 2024\mice\src\mice')
savepath;

cspice_furnsh('C:\Users\trace\OneDrive\Documents\Fall 2024\Kernel\de440.bsp');

b = 6356.752 * 1000; % polar radius of earth [m]
a = 3958.8 * 1609; % equatorial radius of earth [m]
phi = 40.7608*pi/180; % geodetic latitude
lambda = -111.891 * pi/180; %longitude
h = 350 * 1000; %altitude
N = a^2/(sqrt(a^2*cos(phi)^2+b^2*sin(phi)^2));

% ECEF Coordinates

X = ((N+h) * cos(phi)*cos(lambda))/1000;
Y = ((N+h) * cos(phi)*sin(lambda))/1000;
Z = ((b^2/a^2*N+h)*sin(phi))/1000;


% Julians Date
Ye = 2024;
M = 9;
D = 27;
U = 20;
J = 367 * Ye - (7*(Ye+(M+9)/12))/4+275*M/9 + D + 1721013.5+U/24;
T = (J-2451545)/36525;
GMS = 280.46061837 + 360.98564736629 * (J-2451545) + 0.000387933*T^2 - T^3/38710000;
GMS_sub = round(GMS/360);
GMS = GMS - 360*GMS_sub;
theta = GMS * pi/180;


r_ecef = [X;Y;Z];
Rz = [cos(theta),-sin(theta),0;
    sin(theta),cos(theta),0;
    0,0,1];

% ICRF
r_icrf = Rz*r_ecef;
