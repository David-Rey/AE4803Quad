clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
tf = 8;  % final time
dt = 0.01;  % time step
t_arr = 0:dt:tf;

% set up dynamics
dyn = full_quadrotor(dt);

% Tune here!
% Q = 100.244*eye(length(x0));
% R = 12.1*eye(4);
% Qf = 37*eye(length(x0));

pos_gain = 1;
vel_gain = 1;
ang_gain = 1;
ang_vel_gain = 1;
Q = diag([pos_gain, pos_gain, pos_gain, vel_gain, vel_gain, vel_gain, ang_gain, ang_gain, ang_gain, ang_vel_gain, ang_vel_gain, ang_vel_gain]);
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
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters);

total_costs(end)
final_cost = norm(controller.states(end,:) - xf)


%% Plot result
xs = controller.states(:,1);
ys = controller.states(:,2);
zs = controller.states(:,3);

figure(1)
plot3(xs,ys,zs)
xlabel("X")
ylabel("Y")
zlabel("Z")
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])
axis("equal")
grid("on")

figure(2)
plot(t_arr(1:end-1), controller.controls(:,1))
title("Controls")
xlabel("Time (s)")
ylabel("Contol")
grid on
hold on
plot(t_arr(1:end-1), controller.controls(:,2))
plot(t_arr(1:end-1), controller.controls(:,3))
plot(t_arr(1:end-1), controller.controls(:,4))
legend(["u1","u2","u3","u4"])

figure(3)
plot(t_arr, controller.states(:,7))
title("Attitude")
xlabel("Time (s)")
ylabel("Rad")
grid on
hold on
plot(t_arr, controller.states(:,8))
plot(t_arr, controller.states(:,9))
legend(["\phi","\theta","\psi"])

figure(4)
plot(t_arr, controller.states(:, 10))
title("Body Rate")
xlabel("Time (s)")
ylabel("Rad/sec")
grid on
hold on
plot(t_arr, controller.states(:, 11))
plot(t_arr, controller.states(:, 12))
legend(["p","q","r"])
