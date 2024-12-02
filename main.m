clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
tf = 6;  % final time
dt = 0.01;  % time step

% set up dynamics
dyn = full_quadrotor(dt);

% Tune here!
Q = eye(length(x0));
R = eye(4);
Qf = 100*Q;

iters = 1000;
regularizer = 0;
mode = "ilqr";
initial_controls = zeros(tf / dt, 4);  % 800-by-4
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);
% form: [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control)
% form: [cost, cx, cxx] = term_costfn(state)

% run controller
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode);

% Plot result
xs = controller.states(:,1);
ys = controller.states(:,2);
zs = controller.states(:,3);

figure(1)
plot3(xs,ys,zs)
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])