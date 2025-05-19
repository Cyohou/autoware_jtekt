function [ c ] = curvaturePoints( P )
%CIRCONSCRIT Circonscrite circle to P(1,:), P(2,:), P(3,:)
%   Detailed explanation goes here

x1 = P(1,1);
y1 = P(1,2);
x2 = P(2,1);
y2 = P(2,2);
x3 = P(3,1);
y3 = P(3,2);

c = 2 * (x1*y2 + x2*y3 + x3*y1 - x1*y3 - x2*y1 - x3*y2)/ sqrt( ((x2-x1)^2 + (y2-y1)^2) * ((x3-x2)^2 + (y3-y2)^2) * ((x1-x3)^2 + (y1-y3)^2) );

end

