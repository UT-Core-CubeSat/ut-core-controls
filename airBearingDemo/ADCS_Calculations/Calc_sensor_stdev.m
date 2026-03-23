% the purpose of this script is to calculate the standard deviations of the
% sensors used in the ADCS. parameters are taken from the sensor data
% sheets. 

% Parameters
% Rn - rate noise density 
% An - acceleration noise density
% BW - Band width (sample rate)
%-------------------------------------------------------------------------
% Calculate standard deviation for isolated rate gyro.
% A3G4250D MEMS motion sensor:
R_n = 0.15; % deg/(s*sqrt(HZ))
BW  = 100;  % Hz
sigma_gyro = R_n * sqrt(BW)

% calculate linear acceleration standard deviation LSM6DSV
A_n = 100e-6; % g/sqrt(HZ)
BW  = 100; % Hz
sigma_acc = A_n*sqrt(BW)

% Calculate standard deviation for isolated rate gyro LSM6DSV
R_n = 2.8e-3; % deg/(s*sqrt(HZ))
BW  = 100;  % Hz
sigma_gyro_IMU = R_n * sqrt(BW)

%


