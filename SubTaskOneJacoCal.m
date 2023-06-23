%% calculate the Rt_I matrix in symbol
%% arm parameters
% i=1
aerialmanipulator.alpha0 = 0; aerialmanipulator.a0 = 0.01475;
aerialmanipulator.d1 = 0.15665; aerialmanipulator.theta1 = 0;
% i=2
aerialmanipulator.alpha1 = pi/2; aerialmanipulator.a1 = 0;
aerialmanipulator.d2 = 0; aerialmanipulator.theta2 = 0;
% i=3
aerialmanipulator.alpha2 = 0; aerialmanipulator.a2 = 0.083;
aerialmanipulator.d3 = 0; aerialmanipulator.theta3 = pi/2;
% i=4
aerialmanipulator.alpha3 = pi/2; aerialmanipulator.a3 = 0;
aerialmanipulator.d4 = 0.068; aerialmanipulator.theta4 = pi/2;
% define symbol variables
syms roll pitch yaw
syms th1 th2 th3 th4
%% sub rotation matrix
% bse transform
Base_R =[0.7071 0.7071 0; ...
    -0.7071 0.7071 0; ...
    0 0 -1.0000];
% arm transform matrix
R1_0 = [cos(th1) -sin(th1) 0; ...
    cos(aerialmanipulator.alpha0)*sin(th1) cos(aerialmanipulator.alpha0)*cos(th1) -sin(aerialmanipulator.alpha0); ...
    sin(aerialmanipulator.alpha0)*sin(th1) sin(aerialmanipulator.alpha0)*cos(th1) cos(aerialmanipulator.alpha0)];
R2_1 = [cos(th2) -sin(th2) 0; ...
    cos(aerialmanipulator.alpha1)*sin(th2) cos(aerialmanipulator.alpha1)*cos(th2) -sin(aerialmanipulator.alpha1); ...
    sin(aerialmanipulator.alpha1)*sin(th2) sin(aerialmanipulator.alpha1)*cos(th2) cos(aerialmanipulator.alpha1)];
R3_2 = [-sin(th3) -cos(th3) 0; ...
    cos(aerialmanipulator.alpha2)*cos(th3) -cos(aerialmanipulator.alpha2)*sin(th3) -sin(aerialmanipulator.alpha2); ...
    sin(aerialmanipulator.alpha2)*cos(th3) -sin(aerialmanipulator.alpha2)*sin(th3) cos(aerialmanipulator.alpha2)];
R4_3 = [-sin(th4) -cos(th4) 0; ...
    cos(aerialmanipulator.alpha3)*cos(th4) -cos(aerialmanipulator.alpha3)*sin(th4) -sin(aerialmanipulator.alpha3); ...
    sin(aerialmanipulator.alpha3)*cos(th4) -sin(aerialmanipulator.alpha3)*sin(th4) cos(aerialmanipulator.alpha3)];
Rt_0 = R1_0 * R2_1 * R3_2 * R4_3;
% Z-Y-Z fix coordina
RX = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
RY = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
RZ = [1 0 0; 0 cos(yaw) -sin(yaw); 0 sin(yaw) cos(yaw)];
Rb_I = RZ * RY * RX;
%% rotation matrix
Rt_I = Rb_I * Base_R * Rt_0;
%% sub task one
Rs1 = Rt_I * [0 0 1]';
%% diff R
d_Rs1_roll = diff(Rs1, 'roll'); d_Rs1_pitch = diff(Rs1, 'pitch'); d_Rs1_yaw = diff(Rs1, 'yaw');
d_Rs1_th1 = diff(Rs1, 'th1'); d_Rs1_th2 = diff(Rs1, 'th2'); d_Rs1_th3 = diff(Rs1, 'th3'); d_Rs1_th4 = diff(Rs1, 'th4');
d_Rs1 = [zeros([3 6]) d_Rs1_th1 d_Rs1_th2 d_Rs1_th3 d_Rs1_th4];
%% inline function
d_Rs1_inline = inline(d_Rs1, 'roll', 'pitch', 'yaw', 'th1', 'th2', 'th3', 'th4');
%%

