clear 
close all

%% Map
load('Low_Mu_Map.mat');% Low mu area

%% Cal Map angle
theta(1) = atan(abs(Y(2)-Y(1))/abs(X(2)-X(1)));
theta(2) = atan(abs(Y(6)-Y(5))/abs(X(6)-X(5)));

theta_map = mean(theta);

%% Coordinate
C = [cos(theta_map), -sin(theta_map)
     sin(theta_map), cos(theta_map)];

XY_C = C * [X;Y];
 
%% Check
figure(1)
plot(XY_C(1,:),XY_C(2,:));hold on; 
grid on;
xlabel('X [m]');ylabel('Y [m]');

%% Design Map
% load('Iga_Low_Mu_Oval_R20.mat');
% load('Iga_DPB_Oval_R10.mat');
load('traj_v2.mat');

path_XY_C =  path';

path_XY_C = [path_XY_C(1,:) + 0 ;path_XY_C(2,:) - 0];
% path_XY_C = [path_XY_C(1,:) + 592 ;path_XY_C(2,:) - 154];
% path_XY_C = [path_XY_C(1,:) + 575 ;path_XY_C(2,:) - 154];
% path_XY_C = [path_XY_C(1,:) + 328 ;path_XY_C(2,:) - 170]; % Circle 

%% Check
figure(1)
plot(path_XY_C(1,:),path_XY_C(2,:));
daspect([1 1 1])

%% DeCoordinate
% path_XY =  C'*path_XY_C;
path_XY =  path_XY_C;

%% Check 
figure(2)
plot(X,Y);hold on;
plot(path_XY(1,:),path_XY(2,:));hold on;
grid on;
daspect([1 1 1])
%% Filter
MapFilter; %　実際の軌跡に使用　ガウスフィルタ
[path_s, path_X, path_Y, path_dirX, path_dirY, path_kappa] = computeLookupTables(path_XY');

%% Save
% save('Low_Mu_test.mat','path_s','path_X','path_Y','path_dirX','path_dirY','path_kappa');
save('Iga_DPB_traj_v2.mat','path_s','path_X','path_Y','path_dirX','path_dirY','path_kappa');

