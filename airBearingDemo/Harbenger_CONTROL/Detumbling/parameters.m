bDotCntrllrGain_xMgntrqr = 10000; %[unitless] *Range: bDotCntrllrGain_xMgntrqr ≥ 0.0*
bDotCntrllrGain_yMgntrqr = 10000; %[unitless] *Range: bDotCntrllrGain_yMgntrqr ≥ 0.0*
bDotCntrllrGain_zMgntrqr = 10000; %[unitless] *Range: bDotCntrllrGain_zMgntrqr ≥ 0.0*
numCoils_xMgntrqr = 264; %[unitless] *Range: numCoils_xMgntrqr > 0.0*
numCoils_yMgntrqr = 264; %[unitless] *Range: numCoils_yMgntrqr > 0.0*
numCoils_zMgntrqr = 320; %[unitless] *Range: numCoils_zMgntrqr > 0.0*
sctnArea_xMgntrqr = 0.006771*3; %[m^2] *Range: sctnArea_xMgntrqr > 0.0*
sctnArea_yMgntrqr = 0.006771*3; %[m^2] *Range: sctnArea_yMgntrqr > 0.0*
sctnArea_zMgntrqr = 8.7695E-3; %[m^2] *Range: sctnArea_zMgntrqr > 0.0*
maxCrrnt_xMgntrqr = 0.1; %[A] *Range: maxCrrnt_xMgntrqr > 0.0*
maxCrrnt_yMgntrqr = 0.1; %[A] *Range: maxCrrnt_yMgntrqr > 0.0*
maxCrrnt_zMgntrqr = 0.2135; %[A] *Range: maxCrrnt_zMgntrqr > 0.0*

%Magnetometer Parameters
lpFltrGain_mgntmtr = 0.4; %[unitless] *Range: 0.0 ≤ lpFltrGain_mgntmtr ≤ 1.0*
sampRte_mgntmtr = 80; %[Hz] *Range: sampRte_mgntmtr > 0*
cnstntBiasVctr_mgntmtr = [0; 0; 0]; %[T] *Range: cnstntBiasVctr_mgntmtr ≥ [0.0; 0.0; 0.0]*
maxNoiseAmpltde_mgntmtr = 1E-6; %[T] *Range: maxNoiseAmpltde_mgntmtr ≥ 0.0*

%Gyroscope Parameters (IMU)
lpFltrGain_gyro = 0.45; %[unitless] *Range: 0.0 ≤ lpFltrGain_gyro ≤ 1.0*
sampRte_gyro = 500; %[Hz] *Range: sampRte_gyro > 0*
cnstntBiasVctr_gyro = [0; 0; 0]; %[deg/s] *Range: cnstntBiasVctr_gyro ≥ [0.0; 0.0; 0.0]*
maxNoiseAmpltde_gyro = 0.25; %[deg/s] *Range: maxNoiseAmpltde_gyro ≥ 0.0*
%Spacecraft Parameters
mass_spcrft = 3.4; %[kg] *Range: mass_spcrft > 0.0*
dimnsnsX_spcrft = 0.1; %[m] *Range: dimnsnsX_spcrft > 0.0*
dimnsnsY_spcrft = 0.1; %[m] *Range: dimnsnsY_spcrft > 0.0*
dimnsnsZ_spcrft = 0.3; %[m] *Range: dimnsnsZ_spcrft > 0.0*
inrtaTnsr_spcrft = [36857.5, .06,     -7.6;
                             .06,     36771.47, 42.7;
                           -7.6,  42.7,     7705.886].*10^-6; %[kg-m^2] *Range: inrtaTnsr_spcrft > [0.0, 0.0, 0.0;
                                             %                                     0.0, 0.0, 0.0;
                                             %                                     0.0, 0.0, 0.0]* don't update this
initYawAngl_spcrft = 0; %[deg] *Range: 0.0 ≤ initYawAngl_spcrft < 360.0*
initPtchAngl_spcrft = 0; %[deg] *Range: 0.0 ≤ initPtchAngl_spcrft < 360.0*
initRollAngl_spcrft = 0; %[deg] *Range: 0.0 ≤ initRollAngl_spcrft < 360.0*
initBdyAnglrRteX_spcrft = 15; %[deg/s] *Range: initBdyAnglrRteX_spcrft ≥ 0.0*
initBdyAnglrRteY_spcrft = 10; %[deg/s] *Range: initBdyAnglrRteY_spcrft ≥ 0.0*
initBdyAnglrRteZ_spcrft = 20; %[deg/s] *Range: initBdyAnglrRteZ_spcrft ≥ 0.0*
dsirdBdyAnglrRteX_spcrft = 0; %[deg/s] *Range: dsirdBdyAnglrRteX_spcrft ≥ 0.0*
dsirdBdyAnglrRteY_spcrft = 0; %[deg/s] *Range: dsirdBdyAnglrRteY_spcrft ≥ 0.0*
dsirdBdyAnglrRteZ_spcrft = 0; %[deg/s] *Range: dsirdBdyAnglrRteZ_spcrft ≥ 0.0*

%Orbit Parameters
alt_orbt = 6E5; %[m] *Range: 2.0E6 < alt_orb < 8.5E6*
inc_orbt = 97.787; %[deg] *Range: 0.0 ≤ inc_orb ≤ 180.0*
aop_orbt = 144.8873; %[deg] *Range: 0.0 ≤ aop_orb < 360.0*
raan_orbt = 59.4276; %[deg] *Range: 0.0 ≤ raan_orb < 360.0*
numOrbts = 1; %[unitless] *Range: numOrbs > 0.0*