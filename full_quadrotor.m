function [dyn] = full_quadrotor(dt)
%   Detailed explanation goes here

% Get parameters
m = 0.5;
g = 9.81;
kt = 0.01691;
l = 0.17;
Ixx = 0.0032;
Iyy = 0.0032;
Izz = 0.0055;

syms x y z phi theta ps p q r vx vy vz u1 u2 u3 u4

coord = [x; y; z; phi; theta; ps];
vel = [vx; vy; vz; p; q; r];
state = [coord; vel];
controls = [u1; u2; u3; u4];
vars = [state; controls];

%% Get equations of motion
% Starting with R
% Define individual rotation matrices
R1_phi = [1, 0, 0;
          0, cos(phi), sin(phi);
          0, -sin(phi), cos(phi)];
      
R2_theta = [cos(theta), 0, -sin(theta);
            0, 1, 0;
            sin(theta), 0, cos(theta)];
        
R3_psi = [cos(ps), sin(ps), 0;
          -sin(ps), cos(ps), 0;
          0, 0, 1];
      
% Combine rotation matrices
R = R1_phi * R2_theta * R3_psi;

% Linear acceleration
lin_acc = (1/m)*R*[0;0; u1 + u2 + u3 + u4] + [0;0;-m*g];

% Rotational acceleration by Euler's eqn
rot_acc = [(1/Ixx)*(sqrt(2)/2)*(u1+u3-u2-u4)*l - (Izz - Iyy)*q*r;
    (1/Iyy)*(sqrt(2)/2)*(u3+u4-u1-u2)*l - (Izz - Ixx)*p*r;
    (1/Izz)*kt*(u1+u4-u2-u3)];

% Set eom
eom = [lin_acc; rot_acc];

f = [coord + dt * vel;
     vel + dt * eom];

fx = jacobian(f, state);
fu = jacobian(f, controls);

numeric_f = matlabFunction(f, Vars=vars);
numeric_fx = matlabFunction(fx, Vars=vars);
numeric_fu = matlabFunction(fu, Vars=vars);

numeric_fxx = {};
numeric_fxu = {};
numeric_fuu = {};

for i = 1:12
    numeric_fxx{i} = matlabFunction(hessian(f(i), state), Vars=vars);
    numeric_fxu{i} = matlabFunction(jacobian(jacobian(f(i), state), controls), Vars=vars);
    numeric_fuu{i} = matlabFunction(hessian(f(i), controls), Vars=vars);
end


dyn = @(x, u) dynamics_wrapper(x, u, numeric_f, numeric_fx, numeric_fu, numeric_fxx, numeric_fxu, numeric_fuu);

end


function [wrapper_f, wrapper_fx, wrapper_fu, wrapper_fxx, wrapper_fxu, wrapper_fuu] = dynamics_wrapper(state, control, num_f, num_fx, num_fu, num_fxx, num_fxu, num_fuu)
    % Deconstruct state vector
    x = state(1);  % translational pos
    y = state(2);
    z = state(3);
    vx = state(4);  % translational vel
    vy = state(5);
    vz = state(6);
    phi = state(7);  % euler pitch roll yaw
    theta = state(8);
    psi = state(9);
    p = state(10);  % body roll pitch yaw
    q = state(11);
    r = state(12);

    % Deconstruct control vector
    u1 = control(1);
    u2 = control(2);
    u3 = control(3);
    u4 = control(4);

    wrapper_f = num_f(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);
    wrapper_fx = num_fx(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);
    wrapper_fu = num_fu(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);

    wrapper_fxx = zeros(12, 12, 12);
    wrapper_fxu = zeros(12, 12, 4);
    wrapper_fuu = zeros(12, 4, 4);

    for i = 1:12
        wrapper_fxx(i, :, :) = num_fxx{i}(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);
        wrapper_fxu(i, :, :) = num_fxu{i}(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);
        wrapper_fuu(i, :, :) = num_fuu{i}(x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,u1,u2,u3,u4);
    end
end

