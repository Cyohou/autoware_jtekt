function [ out ] = generateEightTurn( r, l, ls, dl, angle )
%GENERATEEIGHTTURN Ç±ÇÃä÷êîÇÃäTóvÇÇ±Ç±Ç…ãLèq
%   generateEightTurn( r, l, ls, dl, angle )
%       r : half-circle radius
%       l : lenght of straight lines
%       ls : horizontal length of sinus
%       dl : approximative sampling length
%

%%
% param

g = pi/ls;

dt1 = dl; % çèÇ›ÇÃç◊Ç©Ç≥
dt2 = dt1/r; % çèÇ›ÇÃç◊Ç©Ç≥

%% 
% Compute line

% Sin + íºê¸ïî á@

X1 = 0:dt1:ls+l
Y1 = zeros(length(X1),1)
Y_rv1 = Y1;
L1 = length(X1);

for i = 1:L1
    
    if  X1(i) < ls
        Y1(i) = r*(sin(g*X1(i)-pi/2)+1); 
    elseif X1(i)>=pi/g 
        Y1(i) = 2*r;
    end
    Y_rv1(i) = - (Y1(i)-r)+r;
    
end

% â~å ïîï™
theta = 0:dt2:pi;
Xr_1 = zeros(length(theta),1);
Yr_1 = zeros(length(theta),1);
Xr_2 = zeros(length(theta),1);
Yr_2 = zeros(length(theta),1);

Lr = length(theta);

for i = 1:Lr
    Xr_1(i) = ls+l + r*cos(pi/2-theta(i));
    Yr_1(i) = r + r*sin(pi/2-theta(i));
    Xr_2(i) = -l + r*cos(pi/2+theta(i));
    Yr_2(i) = r + r*sin(pi/2+theta(i));    
end

% íºê¸ïîÅ@áA
X2 = -l:dt1:0;
Y2 = zeros(length(X2),1);
Y_rv2 = Y2;

L2 = length(X2);

for i = 1:L2
 
        Y2(i) = 2*r;     
        Y_rv2(i) = 0;
    
end

% figure
% plot(X1,Y1);  hold on
% plot(Xr_1,Yr_1);  hold on
% plot(X2,Y2);  hold on
% plot(Xr_2,Yr_2);
% grid on

path_X = zeros( L1*2 + Lr*2 + L2*2 ,1);
path_Y = zeros( L1*2 + Lr*2 + L2*2 ,1);

for p = 1:length(path_X)-1
    
    if p < L2                
        path_X(p) = X2( p );
        path_Y(p) = Y_rv2( p );
    elseif p >= L2 && p < L2 + L1
        path_X(p) = X1( p - L2 + 1 );
        path_Y(p) = Y1( p - L2 + 1 );
    elseif p >= L2 + L1  && p < L1 + L2 + Lr
        path_X(p) = Xr_1( p - (L2 + L1) + 1  );
        path_Y(p) = Yr_1( p - (L2 + L1) + 1  );
    elseif p >= L1 + L2 + Lr && p < L1*2 + L2 + Lr
        path_X(p) = X1( L1*2 + L2 + Lr - p );
        path_Y(p) = Y_rv1( L1*2 + L2 + Lr - p );
    elseif p >= L1*2 + L2 + Lr && p < L1*2 + L2*2 + Lr
        path_X(p) = X2( L1*2 + L2*2 + Lr - p );
        path_Y(p) = Y2( L1*2 + L2*2 + Lr - p );
    elseif p >= L1*2 + L2*2 + Lr
        path_X(p) = Xr_2( p - (L1*2 + L2*2 + Lr) + 1  );
        path_Y(p) = Yr_2( p - (L1*2 + L2*2 + Lr) + 1  );
        
    end
end

path_X = path_X(1:length(path_X)-1)+l;
path_Y      = path_Y(1:length(path_Y)-1);


%% Rotate
mat = [cos(angle) -sin(angle); sin(angle) cos(angle)];
path = mat * [path_X' ; path_Y'];
path_X = path(1,:)';
path_Y = path(2,:)';

out = [path_X, path_Y];

out(end,:) = [0 0];


end

