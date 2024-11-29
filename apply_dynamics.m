function [next_state, fx, fu, fxx, fxu, fuu] = apply_dynamics(state, control, computeA, computeB, dt)
% actually apply the dynamical model given state and control
% also given fn handles for A and B matrices which vary with time
% this fn is run every time step and propagates out the state
% state and control are ROW vectors
% dt: timestep scalar (0.01)

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
u1 = control(1);  % rotor thrust
u2 = control(2);
u3 = control(3);
u4 = control(4);

A = computeA(p, phi, q, r, theta, u1, u2, u3, u4);
B = computeB(phi, theta);

% TODO: get the outputs here
% need to double check that we can say next_state = Ax + Bu
xdot = A * state.' + B * control.';  % col vec
next_state = state.' + xdot * dt;

% TODO

end