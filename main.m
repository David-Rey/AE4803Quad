clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % final state
tf = 8;  % final time
dt = 0.01;  % time step

% yield dynamical systems
dynamics();

% TODO: implement functions below
dyn = @(state, control);
% form: [next_state, fx, fu, fxx, fxu, fuu] = dyn(state, control)

% Tune here!
Q = eye(length(x0));
R = eye(4);
Qf = 500*Q;

iters = 10;
regularizer = 0.0;
mode = "ilqr";
initial_controls = zeros(tf / dt, 4);  % 800-by-4
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);
% form: [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control)
% form: [cost, cx, cxx] = term_costfn(state)

% run controller
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode);

