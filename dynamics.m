
clear; clc; close all;

syms x y z vx vy vz phi theta psi p q r
% syms xDot yDot zDot vxDot vyDot vzDot phiDot thetaDot psiDot pDot qDot rDot
syms u1 u2 u3 u4

m = 0.5;
g = 9.81;
kt = 0.01691;
l = 0.17;
Ixx = 0.0032;
Iyy = 0.0032;
Izz = 0.0055;

% Define individual rotation matrices
R1_phi = [1, 0, 0;
          0, cos(phi), sin(phi);
          0, -sin(phi), cos(phi)];
      
R2_theta = [cos(theta), 0, -sin(theta);
            0, 1, 0;
            sin(theta), 0, cos(theta)];
        
R3_psi = [cos(psi), sin(psi), 0;
          -sin(psi), cos(psi), 0;
          0, 0, 1];
      
% Combine rotation matrices
R = R1_phi * R2_theta * R3_psi;

% Euler angle representation
P =		 [1, tan(theta)*sin(phi), tan(theta)*cos(phi);
		  0, cos(phi), -sin(phi);
		  0, sin(phi)/cos(theta), cos(phi)/cos(theta)];


%{
Fpos = [vx; vy; vz] - [xDot; yDot; zDot];
FVel = (1/m) * R * [0; 0; u1 + u2 + u3 + u4] + [0; 0; -g] - [vxDot; vyDot; vzDot];
FAngle = P * [p; q; r] - [phiDot; thetaDot; psiDot];

FBody = [sqrt(2)/(2 * Ixx) * (u1 + u3 - u2 - u4) * l - (Izz - Iyy) * q * r / Ixx;
	 sqrt(2)/(2 * Ixx) * (u3 + u4 - u1 - u2) * l + (Izz - Ixx) * p * r / Iyy;
	 kt * (u1 + u4 - u2 - u3) / Izz] - [pDot; qDot; rDot];
%}

Fpos = [vx; vy; vz];
FVel = (1/m) * R * [0; 0; u1 + u2 + u3 + u4] + [0; 0; -g];
FAngle = P * [p; q; r];
FBody = [sqrt(2)/(2 * Ixx) * (u1 + u3 - u2 - u4) * l - (Izz - Iyy) * q * r / Ixx;
	 sqrt(2)/(2 * Ixx) * (u3 + u4 - u1 - u2) * l + (Izz - Ixx) * p * r / Iyy;
	 kt * (u1 + u4 - u2 - u3) / Izz];

F = [Fpos; FVel; FAngle; FBody];

xState = [x y z vx vy vz phi theta psi p q r];
u = [u1 u2 u3 u4];

A = jacobian(F, xState);
B = jacobian(F, u);


computeA_fn = matlabFunction(A);
computeB_fn = matlabFunction(B);

computeA = @(phi, theta, p, q, r, u1, u2, u3, u4) computeA_fn(p, phi, q, r, theta, u1, u2, u3, u4);
computeB = @(phi, theta) computeB_fn(phi, theta);

