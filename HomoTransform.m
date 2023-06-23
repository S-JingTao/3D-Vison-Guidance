function T = HomoTransform(alpha, a, d, theta)
%%
% zhonghang
%%
a1 = [cos(theta) -sin(theta) 0 a];
a2 = [sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha)];
a3 = [sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) d*cos(alpha)];
a4 = [0 0 0 1];
%
T = [a1;a2;a3;a4];

