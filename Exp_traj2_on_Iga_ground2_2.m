 % calculate test trajecory path on JTEKT Iga test ground - type2

% V0.2.7 20220821 xiataokai, new transition end pos and data format

clear;clc

% if you want to watch the position of calculated path on the Iga test ground 
DRAW_GLOBAL_POS = 1;

% data of Iga test ground full path, check directory before running this program
if DRAW_GLOBAL_POS
    load ./presets/Full_path_right.mat
    xl = GPSPosX;
    yl = GPSPosY;
    load ./presets/Full_path_left.mat
    xr = GPSPosX;
    yr = GPSPosY;
end

% global position of target test trajecory path on Iga test ground
x_center = 496;
y_center = -248;
yaw_center = 20/180*pi;

% road point density (distance betwwen), m
STEP = 1;

% radius of the center arc on the taeget curve path, m
R0 = 70;
% length of the center arc on the taeget curve path, m
L0 = 40;
% length of clothoid at the beginning and end part on the taeget curve path, m
L1 = 40;
% length of straight line for acceleration or decelration on the low speed path, m
L2 = 50;
% radius of the arc on the low speed path, m
R1 = 20;

% expected vehicle speed on the taeget curve road£¬km/h
VMAX = 50;
% expected vehicle speed on the low speed road£¬km/h
VMIN = 10;

% transition section for acceleration/deceleration on the taeget curve path£¬m
LTIN = 2;
% transition section for acceleration/deceleration on the low speed path£¬m
LTOUT = 64;

% number of cones aside £¬-
NCONE = 10;
% distance between cones and lane center£¬m
WCONE = 2.2;
% number of lane border points aside, -
NPLAN = 24;
% distance between planning area lane border points and lane center£¬m
WPLAN = WCONE - 2.0/2;
% distance of the first cone (or lane border point) to the end of target
% curve path, m
DCONE = 0;

% distance of trigger box center to the end of target curve path, m
DTRIG = 10;
% length of trigger box, m 
LTRIG = 20;
% width of trigger box, m
WTRIG = 8;
% distance of transition trajectory end to trigger box center, m ( > LTRIG/2)
LTE = 12;


data_x0 = [];
data_y0 = [];
data_v0 = [];
p0 = STEP/2;
data_cone_l_x0 = [];
data_cone_l_y0 = [];
data_cone_r_x0 = [];
data_cone_r_y0 = [];
data_plan_l_x0 = [];
data_plan_l_y0 = [];
data_plan_r_x0 = [];
data_plan_r_y0 = [];
data_trig_out_x = [];
data_trig_out_y = [];
data_init_out_x = [];
data_init_out_y = [];
data_init_out_dx = [];
data_init_out_dy = [];
num_cone0 = floor(NCONE/2);
num_plan0 = floor(NPLAN/2);
count = 1;
count_plan = 1;
d_cone0 = (L0+L1*2-DCONE*2)/(num_cone0*2-1);
d_plan0 = (L0+L1*2-DCONE*2)/(num_plan0*2-1);
while p0 < L0/2
    alpha = p0/R0;
    data_x0(end+1,1) = sin(alpha)*R0; %#ok<*SAGROW>
    data_y0(end+1,1) = (cos(alpha)-1)*R0;
    if(p0 > L0/2+L1-LTIN)
        pos_v = (L0/2+L1-p0+LTOUT)/(LTIN+LTOUT);
        data_v0(end+1,:) = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
    else
        data_v0(end+1,:) = VMAX;
    end
    p0 = p0+STEP;
end
l_cone0 = (count-0.5)*d_cone0;
while count <= num_cone0 && l_cone0 <= L0/2
    alpha1 = l_cone0/R0;
    cone_dx = cos(alpha1);
    cone_dy = -sin(alpha1);
    cone_xc = R0*sin(alpha1);
    cone_yc = R0*(cos(alpha1)-1);
    data_cone_l_x0(end+1,:) = cone_xc-cone_dy*WCONE;
    data_cone_l_y0(end+1,:) = cone_yc+cone_dx*WCONE;
    data_cone_r_x0(end+1,:) = cone_xc+cone_dy*WCONE;
    data_cone_r_y0(end+1,:) = cone_yc-cone_dx*WCONE;
    count = count + 1;
    l_cone0 = (count-0.5)*d_cone0;
end
l_plan0 = (count_plan-0.5)*d_plan0;
while count_plan <= num_plan0 && l_plan0 <= L0/2
    alpha1 = l_plan0/R0;
    plan_dx = cos(alpha1);
    plan_dy = -sin(alpha1);
    plan_xc = R0*sin(alpha1);
    plan_yc = R0*(cos(alpha1)-1);
    data_plan_l_x0(end+1,:) = plan_xc-plan_dy*WPLAN;
    data_plan_l_y0(end+1,:) = plan_yc+plan_dx*WPLAN;
    data_plan_r_x0(end+1,:) = plan_xc+plan_dy*WPLAN;
    data_plan_r_y0(end+1,:) = plan_yc-plan_dx*WPLAN;
    count_plan = count_plan + 1;
    l_plan0 = (count_plan-0.5)*d_plan0;
end
a = 1.0/sqrt(2*R0*L1);
syms s l;
clo_fx = int(cos(s^2)/a, 0,a*l);
clo_fy = int(sin(s^2)/a, 0,a*l);
clo_d_yaw = pi-L0/2/R0-atan2(sin((a*L1)^2), cos((a*L1)^2));
clo_x0 = double(subs(clo_fx, l, L1));
clo_y0 = double(subs(clo_fy, l, L1));
clo_x1 = sin(L0/2/R0)*R0;
clo_y1 = (cos(L0/2/R0)-1)*R0;
p0 = p0-L0/2;
while p0 < L1
    clo_xr = double(subs(clo_fx, l, L1-p0))-clo_x0;
    clo_yr = double(subs(clo_fy, l, L1-p0))-clo_y0;
    data_x0(end+1,1) = clo_xr*cos(clo_d_yaw)-clo_yr*sin(clo_d_yaw)+clo_x1;
    data_y0(end+1,1) = clo_xr*sin(clo_d_yaw)+clo_yr*cos(clo_d_yaw)+clo_y1;
    if(p0 > L1-LTIN)
        pos_v = (L1-p0+LTOUT)/(LTIN+LTOUT);
        data_v0(end+1,:) = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
    else
        data_v0(end+1,:) = VMAX;
    end
    p0 = p0+STEP;
end
l_cone0 = (count-0.5)*d_cone0-L0/2;
while count <= num_cone0 && l_cone0 <= L1
    cone_xr = double(subs(clo_fx, l, L1-l_cone0))-clo_x0;
    cone_yr = double(subs(clo_fy, l, L1-l_cone0))-clo_y0;
    cone_xc = cone_xr*cos(clo_d_yaw)-cone_yr*sin(clo_d_yaw)+clo_x1;
    cone_yc = cone_xr*sin(clo_d_yaw)+cone_yr*cos(clo_d_yaw)+clo_y1;
    cone_dxr = -cos((L1*a-l_cone0*a)^2);
    cone_dyr = -sin((L1*a-l_cone0*a)^2);
    cone_dl = sqrt(cone_dxr^2+cone_dyr^2);
    cone_dx = cone_dxr/cone_dl*cos(clo_d_yaw)-cone_dyr/cone_dl*sin(clo_d_yaw);
    cone_dy = cone_dxr/cone_dl*sin(clo_d_yaw)+cone_dyr/cone_dl*cos(clo_d_yaw);
    cone_dx = 1.0*cone_dx;
    cone_dy = 1.0*cone_dy;
    data_cone_l_x0(end+1,:) = cone_xc-cone_dy*WCONE;
    data_cone_l_y0(end+1,:) = cone_yc+cone_dx*WCONE;
    data_cone_r_x0(end+1,:) = cone_xc+cone_dy*WCONE;
    data_cone_r_y0(end+1,:) = cone_yc-cone_dx*WCONE;
    count = count + 1;
    l_cone0 = (count-0.5)*d_cone0-L0/2;
end
l_plan0 = (count_plan-0.5)*d_plan0-L0/2;
while count_plan <= num_plan0 && l_plan0 <= L1
    plan_xr = double(subs(clo_fx, l, L1-l_plan0))-clo_x0;
    plan_yr = double(subs(clo_fy, l, L1-l_plan0))-clo_y0;
    plan_xc = plan_xr*cos(clo_d_yaw)-plan_yr*sin(clo_d_yaw)+clo_x1;
    plan_yc = plan_xr*sin(clo_d_yaw)+plan_yr*cos(clo_d_yaw)+clo_y1;
    plan_dxr = -cos((L1*a-l_plan0*a)^2);
    plan_dyr = -sin((L1*a-l_plan0*a)^2);
    plan_dl = sqrt(plan_dxr^2+plan_dyr^2);
    plan_dx = plan_dxr/plan_dl*cos(clo_d_yaw)-plan_dyr/plan_dl*sin(clo_d_yaw);
    plan_dy = plan_dxr/plan_dl*sin(clo_d_yaw)+plan_dyr/plan_dl*cos(clo_d_yaw);
    plan_dx = 1.0*plan_dx;
    plan_dy = 1.0*plan_dy;
    data_plan_l_x0(end+1,:) = plan_xc-plan_dy*WPLAN;
    data_plan_l_y0(end+1,:) = plan_yc+plan_dx*WPLAN;
    data_plan_r_x0(end+1,:) = plan_xc+plan_dy*WPLAN;
    data_plan_r_y0(end+1,:) = plan_yc-plan_dx*WPLAN;
    count_plan = count_plan + 1;
    l_plan0 = (count_plan-0.5)*d_plan0-L0/2;
end
data_x0 = [-flipud(data_x0);data_x0];
data_y0 = [flipud(data_y0);data_y0];
data_v0 = [flipud(data_v0);data_v0];
data_cone_l_x0 = [-flipud(data_cone_l_x0);data_cone_l_x0];
data_cone_l_y0 = [flipud(data_cone_l_y0);data_cone_l_y0];
data_cone_r_x0 = [-flipud(data_cone_r_x0);data_cone_r_x0];
data_cone_r_y0 = [flipud(data_cone_r_y0);data_cone_r_y0];
data_plan_l_x0 = [-flipud(data_plan_l_x0);data_plan_l_x0];
data_plan_l_y0 = [flipud(data_plan_l_y0);data_plan_l_y0];
data_plan_r_x0 = [-flipud(data_plan_r_x0);data_plan_r_x0];
data_plan_r_y0 = [flipud(data_plan_r_y0);data_plan_r_y0];

data_x1 = [];
data_y1 = [];
data_v1 = [];
trig_found = 0;
init_found = 0;
trans_yaw0 = clo_d_yaw-pi;
clo_x2 = double(subs(clo_fx, l, 0));
clo_y2 = double(subs(clo_fy, l, 0));
trans_x0 = (clo_x2-clo_x0)*cos(clo_d_yaw)-(clo_y2-clo_y0)*sin(clo_d_yaw)+clo_x1;
trans_y0 = (clo_x2-clo_x0)*sin(clo_d_yaw)+(clo_y2-clo_y0)*cos(clo_d_yaw)+clo_y1;
p0 = p0-L1;
while p0 < L2
    data_x1(end+1,:) = trans_x0+p0*cos(trans_yaw0);
    data_y1(end+1,:) = trans_y0+p0*sin(trans_yaw0);
    if(p0 < LTOUT)
        pos_v = (LTOUT-p0)/(LTIN+LTOUT);
        data_v1(end+1,:) = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
    else
        data_v1(end+1,:) = VMIN;
    end
    if(trig_found==0 && p0 > DTRIG)
        trig_found = 1;
        trig_xc = trans_x0+DTRIG*cos(trans_yaw0);
        trig_yc = trans_y0+DTRIG*sin(trans_yaw0);
        trig_dx = cos(trans_yaw0);
        trig_dy = sin(trans_yaw0);
        data_trig_out_x = trig_xc + LTRIG/2*trig_dx*[1;-1;-1;1] + WTRIG/2*trig_dy*[-1;-1;1;1];
        data_trig_out_y = trig_yc + LTRIG/2*trig_dy*[1;-1;-1;1] + WTRIG/2*trig_dx*[1;1;-1;-1];
    end
    if(init_found==0 && p0 > DTRIG+LTE)
        init_found = 1;
        init_xc = trans_x0+(DTRIG+LTE)*cos(trans_yaw0);
        init_yc = trans_y0+(DTRIG+LTE)*sin(trans_yaw0);
        init_dx = cos(trans_yaw0);
        init_dy = sin(trans_yaw0);
        data_init_out_x = init_xc;
        data_init_out_y = init_yc;
        pos_v = (LTOUT-(DTRIG+LTE))/(LTIN+LTOUT);
        data_init_out_v = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
        data_init_out_dx = init_dx;
        data_init_out_dy = init_dy;
    end
    p0 = p0+STEP;
end

larc_yaw0 = trans_yaw0;
larc_x0 = trans_x0+L2*cos(trans_yaw0)+R1*sin(trans_yaw0);
larc_y0 = trans_y0+L2*sin(trans_yaw0)-R1*cos(trans_yaw0);
larc_L = R1*(larc_yaw0+pi);
p0 = p0-L2;
while p0 < larc_L
    alpha = (pi/2+larc_yaw0)-p0/R1;
    data_x1(end+1,:) = larc_x0 + cos(alpha)*R1;
    data_y1(end+1,:) = larc_y0 + sin(alpha)*R1;
    if(p0 < LTOUT-L2)
        pos_v = (LTOUT-L2-p0)/(LTIN+LTOUT);
        data_v1(end+1,:) = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
    else
        data_v1(end+1,:) = VMIN;
    end
    if(trig_found==0 && p0 > DTRIG-L2)
        trig_found = 1;
        trig_alpha = (pi/2+larc_yaw0)-(DTRIG-L2)/R1;
        trig_xc = larc_x0 + cos(trig_alpha)*R1;
        trig_yc = larc_y0 + sin(trig_alpha)*R1;
        trig_dx = sin(trig_alpha);
        trig_dy = -cos(trig_alpha);
    end
    if(init_found==0 && p0 > (DTRIG+LTE)-L2)
        init_found = 1;
        init_alpha = (pi/2+larc_yaw0)-((DTRIG+LTE)-L2)/R1;
        init_xc = larc_x0 + cos(init_alpha)*R1;
        init_yc = larc_y0 + sin(init_alpha)*R1;
        init_dx = sin(trig_alpha);
        init_dy = -cos(trig_alpha);
        data_init_out_x = init_xc;
        data_init_out_y = init_yc;
        pos_v = (LTOUT-L2-((DTRIG+LTE)-L2))/(LTIN+LTOUT);
        data_init_out_v = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
        data_init_out_dx = init_dx;
        data_init_out_dy = init_dy;
    end
    p0 = p0+STEP;
end
lline_x0 = larc_x0 + cos(-pi/2)*R1;
lline_y0 = larc_y0 + sin(-pi/2)*R1;
lline_L = lline_x0;
p0 = p0-larc_L;
while p0 < lline_L
    data_x1(end+1,:) = lline_x0-p0;
    data_y1(end+1,:) = lline_y0;
    if(p0 < LTOUT-L2-larc_L)
        pos_v = (LTOUT-L2-larc_L-p0)/(LTIN+LTOUT);
        data_v1(end+1,:) = sqrt((1-pos_v)*VMIN^2+pos_v*VMAX^2);
    else
        data_v1(end+1,:) = VMIN;
    end
    if(trig_found==0 && p0 > DTRIG-L2-larc_L)
        trig_found = 1;
        trig_xc = lline_x0-(DTRIG-L2-larc_L);
        trig_yc = lline_y0;
        trig_dx = -1;
        trig_dy = 0;
        data_trig_out_x = trig_xc + LTRIG/2*trig_dx*[1;-1;-1;1] + WTRIG/2*trig_dy*[-1;-1;1;1];
        data_trig_out_y = trig_yc + LTRIG/2*trig_dy*[1;-1;-1;1] + WTRIG/2*trig_dx*[1;1;-1;-1];
    end
    p0 = p0+STEP;
end
data_x1 = [data_x1;-flipud(data_x1)];
data_y1 = [data_y1;flipud(data_y1)];
data_v1 = [data_v1;flipud(data_v1)];
data_trig_in_x = -data_trig_out_x;
data_trig_in_y = data_trig_out_y;
data_init_in_x = -data_init_out_x;
data_init_in_y = data_init_out_y;
data_init_in_dx = data_init_out_dx;
data_init_in_dy = -data_init_out_dy;
data_init_in_v = data_init_out_v;

% the complete test trajecory data in local coordinate
data_x = [data_x1;data_x0];
data_y = [data_y1;data_y0];
data_v = [data_v1;data_v0];

% the complete test trajecory data in global coordinate
rdata_x = x_center + data_x * cos(yaw_center) + data_y * sin(yaw_center);
rdata_y = y_center + data_x * -sin(yaw_center) + data_y * cos(yaw_center);
rdata_v = data_v;

rdata_cone_l_x0 = x_center + data_cone_l_x0 * cos(yaw_center) + data_cone_l_y0 * sin(yaw_center);
rdata_cone_l_y0 = y_center + data_cone_l_x0 * -sin(yaw_center) + data_cone_l_y0 * cos(yaw_center);
rdata_cone_r_x0 = x_center + data_cone_r_x0 * cos(yaw_center) + data_cone_r_y0 * sin(yaw_center);
rdata_cone_r_y0 = y_center + data_cone_r_x0 * -sin(yaw_center) + data_cone_r_y0 * cos(yaw_center);

rdata_plan_l_x0 = x_center + data_plan_l_x0 * cos(yaw_center) + data_plan_l_y0 * sin(yaw_center);
rdata_plan_l_y0 = y_center + data_plan_l_x0 * -sin(yaw_center) + data_plan_l_y0 * cos(yaw_center);
rdata_plan_r_x0 = x_center + data_plan_r_x0 * cos(yaw_center) + data_plan_r_y0 * sin(yaw_center);
rdata_plan_r_y0 = y_center + data_plan_r_x0 * -sin(yaw_center) + data_plan_r_y0 * cos(yaw_center);

rdata_trig_out_x = x_center + data_trig_out_x * cos(yaw_center) + data_trig_out_y * sin(yaw_center);
rdata_trig_out_y = y_center + data_trig_out_x * -sin(yaw_center) + data_trig_out_y * cos(yaw_center);
rdata_trig_in_x = x_center + data_trig_in_x * cos(yaw_center) + data_trig_in_y * sin(yaw_center);
rdata_trig_in_y = y_center + data_trig_in_x * -sin(yaw_center) + data_trig_in_y * cos(yaw_center);
rdata_init_out_x = x_center + data_init_out_x * cos(yaw_center) + data_init_out_y * sin(yaw_center);
rdata_init_out_y = y_center + data_init_out_x * -sin(yaw_center) + data_init_out_y * cos(yaw_center);
rdata_init_in_x = x_center + data_init_in_x * cos(yaw_center) + data_init_in_y * sin(yaw_center);
rdata_init_in_y = y_center + data_init_in_x * -sin(yaw_center) + data_init_in_y * cos(yaw_center);
rdata_init_out_dx = data_init_out_dx * cos(yaw_center) + data_init_out_dy * sin(yaw_center);
rdata_init_out_dy = data_init_out_dx * -sin(yaw_center) + data_init_out_dy * cos(yaw_center);
rdata_init_in_dx = data_init_in_dx * cos(yaw_center) + data_init_in_dy * sin(yaw_center);
rdata_init_in_dy = data_init_in_dx * -sin(yaw_center) + data_init_in_dy * cos(yaw_center);

rdata_init_in_v = data_init_in_v;
rdata_init_out_v = data_init_out_v;

figure(220728002);clf(220728002);
plot(data_x, data_y, 'k.-');
hold on;
scatter(data_x, data_y, 6, data_v);
colormap jet;
colorbar();
% scatter(data_cone_l_x0, data_cone_l_y0, 25, 'r');
% scatter(data_cone_r_x0, data_cone_r_y0, 25, 'r');
init_dist = 10;
plot([data_cone_l_x0,data_cone_r_x0],[data_cone_l_y0,data_cone_r_y0],'.r')
plot([data_plan_l_x0,data_plan_r_x0],[data_plan_l_y0,data_plan_r_y0],'.-k')
plot([data_trig_in_x;data_trig_in_x(1)],[data_trig_in_y;data_trig_in_y(1)],'-g');
plot([data_trig_out_x;data_trig_out_x(1)],[data_trig_out_y;data_trig_out_y(1)],'-g');
plot([data_init_out_x;data_init_in_x],[data_init_out_y;data_init_in_y],'*g');
plot([data_init_out_x+[0;init_dist*data_init_out_dx];NaN;data_init_in_x+[0;init_dist*data_init_in_dx]],...
    [data_init_out_y+[0;init_dist*data_init_out_dy];NaN;data_init_in_y+[0;init_dist*data_init_in_dy]],'-g');
plot(data_x0,data_y0,'-b','LineWidth',2);
axis equal;
title('Trajecory2 in local coordinate');
xlabel('Local x(m)'); ylabel('Local y(m)');

% draw the test trajecory path in global coordinate
if DRAW_GLOBAL_POS
    figure(220728001);clf(220728001);
    plot(xl, yl, '-k');
    hold on;
    plot(xr, yr, '-k');
    axis equal;
    hold on;
    % plot(rdata_x, rdata_y, 'k-');
    scatter(rdata_x, rdata_y, 2, data_v);
    colormap jet;
    % scatter(rdata_cone_l_x0, rdata_cone_l_y0, 25, 'r');
    % scatter(rdata_plan_r_x0, rdata_plan_r_y0, 25, 'r');
    plot([rdata_cone_l_x0,rdata_cone_r_x0],[rdata_cone_l_y0,rdata_cone_r_y0],'.r');
    plot([rdata_plan_l_x0,rdata_plan_r_x0],[rdata_plan_l_y0,rdata_plan_r_y0],'-k');
    % plot([rdata_trig_in_x;rdata_trig_in_x(1)],[rdata_trig_in_y;rdata_trig_in_y(1)],'-g');
    % plot([rdata_trig_out_x;rdata_trig_out_x(1)],[rdata_trig_out_y;rdata_trig_out_y(1)],'-g');
    % plot([rdata_init_out_x;rdata_init_in_x],[rdata_init_out_y;rdata_init_in_y],'*g');
    % plot([rdata_init_out_x+[0;init_dist*rdata_init_out_dx];NaN;rdata_init_in_x+[0;init_dist*rdata_init_in_dx]],...
    % [rdata_init_out_y+[0;init_dist*rdata_init_out_dy];NaN;rdata_init_in_y+[0;init_dist*rdata_init_in_dy]],'-g');
    title('Trajecory in global coordinate');
    xlabel('Global X(m)'); ylabel('Global Y(m)');
end

% save calculated data
traj0 = struct();
traj0.param = struct();
traj0.param.VMAX = VMAX;
traj0.param.VMIN = VMIN;
traj0.param.LTIN = LTIN;
traj0.param.LTOUT = LTOUT ;
traj0.param.NCONE  = NCONE ;
traj0.param.WCONE = WCONE;
traj0.param.NPLAN = NPLAN;
traj0.param.WPLAN = WPLAN;
traj0.param.DCONE = DCONE;
traj0.param.DTRIG = DTRIG;
traj0.param.LTRIG = LTRIG;
traj0.param.WTRIG = WTRIG;
traj0.param.LTE = LTE;
traj0.x = rdata_x;
traj0.y = rdata_y;
traj0.speed = rdata_v;
% traj0.v = rdata_v;
% traj0.plan_l_x0 = rdata_plan_l_x0;
% traj0.plan_l_y0 = rdata_plan_l_y0;
% traj0.plan_r_x0 = rdata_plan_r_x0;
% traj0.plan_r_y0 = rdata_plan_r_y0;
traj0.cone_l_x0 = rdata_cone_l_x0;
traj0.cone_l_y0 = rdata_cone_l_y0;
traj0.cone_r_x0 = rdata_cone_r_x0;
traj0.cone_r_y0 = rdata_cone_r_y0;
traj0.trig_in_x = rdata_trig_in_x;
traj0.trig_in_y = rdata_trig_in_y;
traj0.trig_out_x = rdata_trig_out_x;
traj0.trig_out_y = rdata_trig_out_y;
% traj0.init_in_x = rdata_init_in_x;
% traj0.init_in_y = rdata_init_in_y;
% traj0.init_out_x = rdata_init_out_x;
% traj0.init_out_y = rdata_init_out_y;
% traj0.init_in_dx = rdata_init_in_dx;
% traj0.init_in_dy = rdata_init_in_dy;
% traj0.init_out_dx = rdata_init_out_dx;
% traj0.init_out_dy = rdata_init_out_dy;
% traj0.init_in_v = rdata_init_in_v;
% traj0.init_out_v = rdata_init_out_v;
traj0.Xl = rdata_plan_l_x0;
traj0.Yl = rdata_plan_l_y0;
traj0.Xr = rdata_plan_r_x0;
traj0.Yr = rdata_plan_r_y0;
traj0.p_in = [rdata_init_in_x, rdata_init_in_y];
traj0.v_in = [rdata_init_in_dx, rdata_init_in_dy];
traj0.p_out = [rdata_init_out_x, rdata_init_out_y];
traj0.v_out = [rdata_init_out_dx, rdata_init_out_dy];
traj0.speed0_in = rdata_init_in_v;
traj0.speed0_out = rdata_init_out_v;

% save to file
save('./presets/traj0.mat','traj0');