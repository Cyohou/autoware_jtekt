function [ out ] = processTrajectory( in, Fc ,dl)
%PROCESSTRAJECTORY
%   Detailed explanation goes here

x = in(:,1);
y = in(:,2);

%% Remove doubles
z = x + 1i*y;
[~, ia] = unique(z);
z = z(sort(ia));
x = real(z);
y = imag(z);

% x = [x ; x(1)];
% y = [y ; y(1)];

%% Resample

% Distance
dist = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];

% Resample
dl = dl;
distq = linspace(0, dist(end), 2^nextpow2(round(dist(end)/dl)))';
Vq = interp1(dist, [x y], distq);
dl = mean(diff(distq));

%% Filter
%Fs = length(distq) / distq(end); % [m^-1]
for i = 1:2
    %Vq(:,i) = fourierFilter(Vq(:,i), Fs, Fc);
    Vq(:,i) = gaussFilterLoop(Vq(:,i), 1/Fc/dl);
    %1/Fc/dl
end

%% Join with or without end
%out = [Vq ; Vq(1,:)];
%out = [Vq];
out = [Vq(1:end-1,1) Vq(1:end-1,2)];
end