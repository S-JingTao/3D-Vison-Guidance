function UAM_plot(quad, u, xydia, zdia, nextplot)
%% 
% plot tthe aerial manipulator in the specified figure
%%
gcf;
set(gca, 'Position', [0.1300 0.1100 0.7750 0.8150]);
if(nextplot == 0)
    clf;
end
%% figure parameters
xy_dia = xydia;
z_dia = zdia;
%% draw coord
axis([-xy_dia xy_dia -xy_dia xy_dia 0 z_dia]);
view([60 15]);
set(gca, 'NextPlot', 'Add');
% plot the ground boundaries and the big cross
s = xy_dia;
plot3([-s -s],[s -s],[0 0],'-b');
plot3([-s s],[s s],[0 0],'-b');
plot3([s -s],[-s -s],[0 0],'-b');
plot3([s s],[s -s],[0 0],'-b');
plot3([s -s],[-s s],[0 0],'-b');
plot3([-s s],[-s s],[0 0],'-b');
% plot interia coord
plot3([0 s/3], [0 0], [0 0], 'Color', [128,128,128]/255);
plot3([0 0], [0 s/3], [0 0], 'Color', [128,128,128]/255);
plot3([0 0], [0 0], [0 s/3], 'Color', [128,128,128]/255);
text(s/3, 0, 0, '\itx', 'FontSize', 12, 'Color', [128,128,128]/255);
text(0, s/3, 0, '\ity', 'FontSize', 12, 'Color', [128,128,128]/255);
text(0, 0, s/3, '\itz', 'FontSize', 12, 'Color', [128,128,128]/255);
%%
a1s = zeros(1, quad.nrotors);
b1s = zeros(1, quad.nrotors);
% vehicle dimensons
d = quad.d; %Hub displacement from COG
r = quad.r; %Rotor radius
for i = 1:quad.nrotors
    theta = (i-1)/quad.nrotors*2*pi;
    %   Di      Rotor hub displacements (1x3)
    % first rotor is on the x-axis, clockwise order looking down from above
    D(:,i) = [ d*cos(theta); d*sin(theta); 0];
    scal = xy_dia/4;
    %Attitude center displacements
    C(:,i) = [ scal*cos(theta); scal*sin(theta); 0];
end
%% draw quadrotor
%READ STATE
z = [u(1);u(2);u(3)];
n = [u(4);u(5);u(6)];
th = [u(7);u(8);u(9);u(10)];

%PREPROCESS ROTATION MATRIX
phi = n(1);    %Euler angles
the = n(2);
psi = n(3);

R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
    cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
    -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];

%Manual Construction
%Q3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];   %Rotation mappings
%Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];
%Q1 = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
%R = Q3*Q2*Q1;    %Rotation matrix

%CALCULATE FLYER TIP POSITONS USING COORDINATE FRAME ROTATION
F = [1 0 0;0 1 0;0 0 1];

%Draw flyer rotors
t = [0:pi/8:2*pi];
for j = 1:length(t)
    circle(:,j) = [r*sin(t(j));r*cos(t(j));0];
end

for i = 1:quad.nrotors
    hub(:,i) = F*(z + R*D(:,i)); %points in the inertial frame

    q = 1; %Flapping angle scaling for output display - makes it easier to see what flapping is occurring
    Rr = [cos(q*a1s(i))  sin(q*b1s(i))*sin(q*a1s(i)) cos(q*b1s(i))*sin(q*a1s(i));   %Rotor > Plot frame
        0              cos(q*b1s(i))               -sin(q*b1s(i));
        -sin(q*a1s(i)) sin(q*b1s(i))*cos(q*a1s(i)) cos(q*b1s(i))*cos(q*a1s(i))];

    tippath(:,:,i) = F*R*Rr*circle;
    plot3([hub(1,i)+tippath(1,:,i)],[hub(2,i)+tippath(2,:,i)],[hub(3,i)+tippath(3,:,i)],'-', ...
        'Color', [176,196,222]/255)
end

%Draw flyer
hub0 = F*z;  % centre of vehicle
for i = 1:quad.nrotors
    % line from hub to centre plot3([hub(1,N) hub(1,S)],[hub(2,N) hub(2,S)],[hub(3,N) hub(3,S)],'-b')
    plot3([hub(1,i) hub0(1)],[hub(2,i) hub0(2)],[hub(3,i) hub0(3)],'-', ...
        'Color', [175,238,238]/255)

    % plot a circle at the hub itself
    plot3([hub(1,i)],[hub(2,i)],[hub(3,i)],'o')
end
%% Draw manipulator
% transfer matrix
interia_T = [[R; [0 0 0]] [z;1]]; 
Base_T = r2t(rotz(-pi/4))*[1 0 0 0; 0 1 0 0; 0 0 -1 0;0 0 0 1];
p0 = z;
RGB = [255,0,255]/255;
% link one
T1_0 = HomoTransform(quad.alpha0, quad.a0, quad.d1, quad.theta1+th(1));
T1_I = interia_T*Base_T*T1_0;
p1 = transl(T1_I);
plot3([p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], 'LineWidth', 2, ...
    'Color', RGB, 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerSize', 3)
% link two
T2_1 = HomoTransform(quad.alpha1, quad.a1, quad.d2, quad.theta2+th(2));
T2_I = T1_I*T2_1;
p2 = transl(T2_I);
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'LineWidth', 2, ...
    'Color', RGB, 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerSize', 3)
% link three
T3_2 = HomoTransform(quad.alpha2, quad.a2, quad.d3, quad.theta3+th(3));
T3_I = T2_I*T3_2;
p3 = transl(T3_I);
plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], 'LineWidth', 2, ...
    'Color', RGB, 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerSize', 3)
% link four
T4_3 = HomoTransform(quad.alpha3, quad.a3, quad.d4, quad.theta4+th(4));
T4_I = T3_I*T4_3;
p4 = transl(T4_I);
plot3([p3(1) p4(1)], [p3(2) p4(2)], [p3(3) p4(3)], 'LineWidth', 2, ...
    'Color', RGB, 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerSize', 3)
% end effector transform
Te_4 = [1 0 0 0;...
    0 1 0 0;...
    0 0 1 0.03822;...
    0 0 0 1];
Te_I = T4_I*Te_4;
pe = transl(Te_I);
plot3([p4(1) pe(1)], [p4(2) pe(2)], [p4(3) pe(3)], 'LineWidth', 2, ...
    'Color', RGB)
% end effector coordinate system
ratio = 0.1;
ex = Te_I*[ratio*1;0;0;1];
ey = Te_I*[0;ratio*1;0;1];
ez = Te_I*[0;0;ratio*1;1];
plot3([pe(1) ex(1)], [pe(2) ex(2)], [pe(3) ex(3)], 'LineWidth', 0.5, ...
    'Color', [0,255,255]/255)
plot3([pe(1) ey(1)], [pe(2) ey(2)], [pe(3) ey(3)], 'LineWidth', 0.5, ...
    'Color', [60,179,113]/255)
plot3([pe(1) ez(1)], [pe(2) ez(2)], [pe(3) ez(3)], 'LineWidth', 0.5, ...
    'Color', [255,165,0]/255)
text(ex(1), ex(2), ex(3), '\itx', 'FontSize', 6);
text(ey(1), ey(2), ey(3), '\ity', 'FontSize', 6);
text(ez(1), ez(2), ez(3), '\itz', 'FontSize', 6);
axis([-xy_dia xy_dia -xy_dia xy_dia 0 z_dia]);

