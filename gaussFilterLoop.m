function [ out ] = gaussFilterLoop( in, sigma )
%GAUSSFILTERLOOP Gaussian filter for a loop path (in(end) == in(1))
%   Detailed explanation goes here

% Window size
sz = ceil(3*sigma)*2+1;

% Construct filter
x = linspace(-(sz-1)/2, (sz-1)/2, sz);
gaussFilter = exp(-x .^ 2 / (2 * sigma ^ 2));
gaussFilter = gaussFilter / sum (gaussFilter); % normalize

% Loop path several times to ensure continuity at start
Nloop = 7;
Np = length(in)-1;
inl = zeros(Nloop*Np+1, 1);
for i=1:Nloop
    inl( ((i-1)*Np+1):(i*Np) )  = in(1:end-1);
end
inl(end) = inl(1);

tmp = filter(gaussFilter, 1, inl);
out = tmp((1:Np+1) + (Np*floor(Nloop/2) + (sz-1)/2));

end
