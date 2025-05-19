function [path_s, path_X, path_Y, path_dirX, path_dirY, path_kappa] = computeLookupTables(path)
%COMPUTELOOKUPTABLES Compute lookup tables for looping path
%
%   [path_s, path_X, path_Y, path_dirX, path_dirY, path_kappa] = computeLookupTables(path)
%
%   The path has to be a loop ! : path(1,:) == path(end,:)

% X and Y
path_X = path(:,1);
path_Y = path(:,2);

% Compute curvilinear position
path_s = [0;cumsum(sqrt(diff(path_X).^2+diff(path_Y).^2))];

% Compute curve radius
path_kappa = zeros(length(path_X), 1);
for i=2:(length(path_kappa)-1)
    path_kappa(i) = curvaturePoints([path_X(i-1:i+1) path_Y(i-1:i+1)]);
end
path_kappa(1) = curvaturePoints([path_X(end-1) path_Y(end-1) ; path_X(1) path_Y(1) ; path_X(2) path_Y(2)]);
path_kappa(end) = path_kappa(1);

% Direction (described as an unit vector)
diffX = diff(path_X);
diffY = diff(path_Y);
diffX = [diffX ; diffX(1)];
diffY = [diffY ; diffY(1)];
path_dirX = diffX ./ sqrt(diffX.^2 + diffY.^2);
path_dirY = diffY ./ sqrt(diffX.^2 + diffY.^2);

end

