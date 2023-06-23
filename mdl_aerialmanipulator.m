%MDL_aerialmanipulatorCOPTER Dynamic parameters for a aerialmanipulatorrotor.
%
% MDL_aerialmanipulatorCOPTER is a script creates the workspace variable aerialmanipulator which
% describes the dynamic characterstics of a aerialmanipulatorrotor flying robot.
%
% Properties::
%
% This is a structure with the following elements:
%
% nrotors   Number of rotors (1x1)
% J         Flyer rotational inertia matrix (3x3)
% h         Height of rotors above CoG (1x1)
% d         Length of flyer arms (1x1)
% nb        Number of blades per rotor (1x1)
% r         Rotor radius (1x1)
% c         Blade chord (1x1)
% e         Flapping hinge offset (1x1)
% Mb        Rotor blade mass (1x1)
% Mc        Estimated hub clamp mass (1x1)
% ec        Blade root clamp displacement (1x1)
% Ib        Rotor blade rotational inertia (1x1)
% Ic        Estimated root clamp inertia (1x1)
% mb        Static blade moment (1x1)
% Ir         Total rotor inertia(1x1)
% Ct        Non-dim. thrust coefficient (1x1)
% Cq        Non-dim. torque coefficient (1x1)
% sigma     Rotor solidity ratio (1x1)
% thetat    Blade tip angle (1x1)
% theta0    Blade root angle (1x1)
% theta1    Blade twist angle (1x1)
% theta75   3/4 blade angle (1x1)
% thetai    Blade ideal root approximation (1x1)
% a         Lift slope gradient (1x1)
% A         Rotor disc area (1x1)
% gamma       (1x1)
%
%
% Notes::
% - SI units are used.
%
% References::
% - Design, Construction and Control of a Large quadrotor micro air vehicle.
%   P.Pounds, PhD thesis, 
%   Australian National University, 2007.
%   http://www.eng.yale.edu/pep5/P_Pounds_Thesis_2008.pdf
% - This is a heavy lift quadrotor
%
% See also sl_quadrotor.

% MODEL: quadrotor

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

aerialmanipulator.nrotors = 4;                %   4 rotors
aerialmanipulator.g = 9.81;                   %   g       Gravity                             1x1
aerialmanipulator.rho = 1.184;                %   rho     Density of air                      1x1
aerialmanipulator.muv = 1.5e-5;               %   muv     Viscosity of air                    1x1

% Airframe
aerialmanipulator.M = 4;                      %   M       Mass                                1x1
Ixx = 0.082;
Iyy = 0.082;
Izz = 0.149;%0.160;
aerialmanipulator.J = diag([Ixx Iyy Izz]);    %   I       Flyer rotational inertia matrix     3x3

aerialmanipulator.h = -0.007;                 %   h       Height of rotors above CoG          1x1
aerialmanipulator.d = 0.2735;                  %   d       Length of flyer arms                1x1

%Rotor
aerialmanipulator.nb = 2;                      %   b       Number of blades per rotor          1x1
aerialmanipulator.r = 0.165;%0.31/2;                  %   r       Rotor radius                        1x1

aerialmanipulator.c = 0.018;                  %   c       Blade chord                         1x1

aerialmanipulator.e = 0.0;                    %   e       Flapping hinge offset               1x1
aerialmanipulator.Mb = 0.005;                 %   Mb      Rotor blade mass                    1x1
aerialmanipulator.Mc = 0.010;                 %   Mc      Estimated hub clamp mass            1x1
aerialmanipulator.ec = 0.004;                 %   ec      Blade root clamp displacement       1x1
aerialmanipulator.Ib = aerialmanipulator.Mb*(aerialmanipulator.r-aerialmanipulator.ec)^2/4 ;        %   Ib      Rotor blade rotational inertia      1x1
aerialmanipulator.Ic = aerialmanipulator.Mc*(aerialmanipulator.ec)^2/4;           %   Ic      Estimated root clamp inertia        1x1
aerialmanipulator.mb = aerialmanipulator.g*(aerialmanipulator.Mc*aerialmanipulator.ec/2+aerialmanipulator.Mb*aerialmanipulator.r/2);    %   mb      Static blade moment                 1x1
aerialmanipulator.Ir = aerialmanipulator.nb*(aerialmanipulator.Ib+aerialmanipulator.Ic);             %   Ir      Total rotor inertia                 1x1

aerialmanipulator.Ct = 0.0048;                %   Ct      Non-dim. thrust coefficient         1x1
aerialmanipulator.Cq = aerialmanipulator.Ct*sqrt(aerialmanipulator.Ct/2);         %   Cq      Non-dim. torque coefficient         1x1

aerialmanipulator.sigma = aerialmanipulator.c*aerialmanipulator.nb/(pi*aerialmanipulator.r);         %   sigma   Rotor solidity ratio                1x1
aerialmanipulator.thetat = 6.8*(pi/180);      %   thetat  Blade tip angle                     1x1
aerialmanipulator.theta0 = 14.6*(pi/180);     %   theta0  Blade root angle                    1x1
aerialmanipulator.theta1 = aerialmanipulator.thetat - aerialmanipulator.theta0;   %   theta1  Blade twist angle                   1x1
aerialmanipulator.theta75 = aerialmanipulator.theta0 + 0.75*aerialmanipulator.theta1;%   theta76 3/4 blade angle                     1x1
aerialmanipulator.thetai = aerialmanipulator.thetat*(aerialmanipulator.r/aerialmanipulator.e);      %   thetai  Blade ideal root approximation      1x1
aerialmanipulator.a = 5.5;                    %   a       Lift slope gradient                 1x1

% derived constants
aerialmanipulator.A = pi*aerialmanipulator.r^2;                 %   A       Rotor disc area                     1x1
aerialmanipulator.gamma = aerialmanipulator.rho*aerialmanipulator.a*aerialmanipulator.c*aerialmanipulator.r^4/(aerialmanipulator.Ib+aerialmanipulator.Ic);%   gamma   Lock number                         1x1

aerialmanipulator.b = aerialmanipulator.Ct*aerialmanipulator.rho*aerialmanipulator.A*aerialmanipulator.r^2; % T = b w^2
aerialmanipulator.k = aerialmanipulator.Cq*aerialmanipulator.rho*aerialmanipulator.A*aerialmanipulator.r^3; % Q = k w^2

aerialmanipulator.verbose = false;

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

%% arm model
deg = pi/180;
L1 = Link('alpha', 0, 'a', 0.01475, 'd', 0.119+0.0548, 'modified', ...
    'I', diag(1e-7*[3663 6454 4205]), ...
    'm', 0.00907, ...
    'qlim', [-180 180]*deg);
L2 = Link('alpha', pi/2, 'a', 0, 'd', 0, 'modified', ...
    'I', diag(1e-7*[270570 290565 58140]), ...
    'm', 0.27, ...
    'qlim', [-45 225]*deg);
L3 = Link('alpha', 0, 'a', 0.083, 'd', 0, 'modified', ...
     'I', diag(1e-7*[48218 39949 67495]), ...
    'm', 0.15, ...
    'qlim', [-135 135]*deg);
L4 = Link('alpha', pi/2, 'a', 0, 'd', 0.068, 'modified', ...
    'I', diag(1e-7*[141 124 189]), ...
    'm', 0.08, ...
    'qlim', [-180 180]*deg);
baseT = [0 1 0 0;...
    -1 0 0 0;...
    0 0 -1 0;...
    0 0 0 1];
manipulator = SerialLink([L1 L2 L3 L4], 'name', 'Arm');
manipulator.offset = [0 0 pi/2 -pi/2];

%%  cmucam5 parameters
% focal: 0.0025 m ?
% pixel size: 3 um
% resulution: 1280*800
% centre : 640*400
% fov: 75*51 deg
cam = CentralCamera('focal', 0.0025, 'pixel', 3*1e-6, 'resolution', ...
    [1280 800], 'centre', [640 400], 'name', 'cmucam5');
CenterBias = [0.6 0.3 0]';
Marker = mkgrid(2, 0.1) + repmat(CenterBias, 1,4);
[M_m M_n] = size(Marker);
TargetHeight = 0.3;
ps = cam.project(Marker, 'Tcam', transl(CenterBias'+[0 0 TargetHeight])*trotx(-pi));

