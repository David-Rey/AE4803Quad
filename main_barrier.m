clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state. Added w
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state. Added w
tf = 8;  % final time
dt = 0.01;  % time step

% set up dynamics
dyn = full_quadrotor_barrier(dt, xf);  % now needs xf too 

% Tune here!
pos_gain = 1;
vel_gain = 1;
ang_gain = 1;
ang_vel_gain = 1;
w_gain = 10;
Q = diag([pos_gain, pos_gain, pos_gain, vel_gain, vel_gain, vel_gain, ang_gain, ang_gain, ang_gain, ang_vel_gain, ang_vel_gain, ang_vel_gain, w_gain]);
R = 0.8*eye(4);
Qf = 180*Q;

iters = 13;
regularizer = 1;  % initial value. Will increment automatically unless this is 0
line_search_iters = 2;  % 1 for no line search
mode = "ilqr";
initial_controls = 1.225*ones(tf / dt, 4);  % initialize to neutral thrust
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);
% form: [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control)
% form: [cost, cx, cxx] = term_costfn(state)

% run controller
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters, xf);

total_costs(end)
final_cost = norm(controller.states(end,:) - xf)


%% Plot result
xs = controller.states(:,1);
ys = controller.states(:,2);
zs = controller.states(:,3);

figure(1)
title("Position")
plot3(xs,ys,zs)
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])
axis("equal")
grid("on")

figure(2)
title("Controls")
plot(controller.controls(:,1))
hold on
plot(controller.controls(:,2))
plot(controller.controls(:,3))
plot(controller.controls(:,4))
legend(["u1","u2","u3","u4"])

figure(3)
title("Attitude")
plot(controller.states(:,4))
hold on
plot(controller.states(:,5))
plot(controller.states(:,6))
legend(["\phi","\theta","\psi"])
