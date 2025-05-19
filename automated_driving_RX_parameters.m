%% initialize
clear
close all


%% Constant
    
d2r = pi/180;
r2d = 180/pi;

%% Ethernet parameters
% run('parameters/LoadEthParameters_1.m');
% run('parameters/LoadEthParameters_2kai.m'); % v2‚Í“Ç‚ÝŽæ‚è?ê—p‚Ì‚½‚ß
% run('parameters/LoadEthParameters_3.m'); % v3‚ÍEtherƒf?[ƒ^??‰Á‚Ì‚½‚ß

%% Parameters
run('parameters/EKF_simulink_param.m');
run('parameters/parameters_autonomous_vehicle.m');
run('parameters/parameters_automated_RX.m');
run('parameters/longitude_latitude.m');
run('parameters/parameters_RX.m');


%% course
% Path
% DP-B
load('Path\Iga_DPB_traj_v2.mat');
% load('Path\path_DPB_kitagawa_offset.mat');
% load('Path\Path_Iga_DPB_complex.mat');
% circuit
% load('Path\rec1_017.mat');
% load('Path\Winding_LaneChange_left_v2.mat');
% load('Path\path_winding_course_center_v1.mat');

%% library parameter
run('parameters\parameters_hod_camry_DP.m');

%% StateSpace
run('parameters\StateSpace_observer.m');

%% Assist map
% run('..\Parameters\parameters_assist_map.m');

%% Sharing control
run('parameters\parameters_SharingControl_camry.m');

%% Parameters divergence
torque_sensor_offset = 0;                  % [Nm] (compensated manually each time)