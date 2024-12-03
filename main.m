clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
tf = 8;  % final time
dt = 0.01;  % time step

% set up dynamics
dyn = full_quadrotor(dt);

% Tune here!
Q = 1*eye(length(x0));
R = 0.01*eye(4);
Qf = 6*eye(length(x0));

iters = 10;
regularizer = 1;
mode = "ddp";
initial_controls = 0.612*ones(tf / dt, 4);  % initialize to neutral thrust
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

figure(2)
plot(controller.controls(:,1))
hold on
plot(controller.controls(:,2))
plot(controller.controls(:,3))
plot(controller.controls(:,4))
legend(["u1","u2","u3","u4"])