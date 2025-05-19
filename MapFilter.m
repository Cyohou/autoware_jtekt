%%
% load('Iga_Low_Mu_Oval_R20.mat');

% XY = path_XY';
% XY = [x,y];
XY = path_XY_C';

clear path
%%
% XY = [Vehicle_X__m_',Vehicle_Y__m_'];
path = processTrajectory(XY,0.2,0.1);% 座標,ガウスフィルタ係数,間隔
[path_s, path_X, path_Y, path_dirX, path_dirY, path_kappa] = computeLookupTables(path);

%% 
figure
plot(path(:,1),path(:,2),'o');hold on;
plot(XY(:,1),XY(:,2));
grid on;

figure
plot(path_s,path_X);
grid on;

