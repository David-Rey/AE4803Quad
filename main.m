clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
tf = 8;  % final time
dt = 0.01;  % time step

% set up dynamics
dyn = full_quadrotor(dt);

% Tune here!
% Q = 0.244*eye(length(x0));
% R = 112.1*eye(4);
% Qf = 37*eye(length(x0));
Q = 0.244*eye(length(x0));
R = 112.1*eye(4);
Qf = 37*eye(length(x0));

iters = 15;
%regularizer = 1;
% iters = 5;
regularizer = 0.1;
mode = "ddp";
initial_controls = 0.612*ones(tf / dt, 4);  % initialize to neutral thrust
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);
% form: [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control)
% form: [cost, cx, cxx] = term_costfn(state)

% run controller
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode);

total_costs(end)
final_cost = norm(controller.states(end,:) - xf)


% Plot result
xs = controller.states(:,1);
ys = controller.states(:,2);
zs = controller.states(:,3);

figure(1)
title("Position")
plot3(xs,ys,zs)
xlabel("X")
ylabel("Y")
zlabel("Z")
hold on

plot3(-3,-2,-1,"ro") %Initial
plot3(5,3,2,"rx") %Final
legend(["Flight path","Start Point","Goal"])

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
