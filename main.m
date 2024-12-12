clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
tf = 8;  % final time
dt = 0.01;  % time step

% set up dynamics
dyn = full_quadrotor_barrier(dt, xf);

pos_gain = 1;
vel_gain = 1;
ang_gain = 1;
ang_vel_gain = 1;
w_gain = 0;  % no barrier state
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
[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters);

total_costs(end)
final_cost = norm(controller.states(end,:) - xf)


%% Plot result
xs = controller.states(:,1);
ys = controller.states(:,2);
zs = controller.states(:,3);

figure(1)

plot3(xs,ys,zs)
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])
axis("equal")
grid("on")
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
title("Position")
saveas(gcf, './normal/3d.png')

%% 2D plots

figure(2)

plot(controller.controls(:,1))
hold on
grid on
plot(controller.controls(:,2))
plot(controller.controls(:,3))
plot(controller.controls(:,4))
legend(["u1","u2","u3","u4"])
xlabel('Time (s)')
ylabel('Control Input (N)')
title("Controls")
saveas(gcf, './normal/controls.png')

figure(3)

plot(controller.states(:,4))
hold on
grid on
plot(controller.states(:,5))
plot(controller.states(:,6))
legend(["\phi","\theta","\psi"])
xlabel('Time (s)')
ylabel('Angle (rad)')
title("Attitude")
saveas(gcf, './normal/attitude.png')

figure(4)

plot(controller.states(:, 10))
grid on
hold on
plot(controller.states(:, 11))
plot(controller.states(:, 12))
legend(["p","q","r"])
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title("Body Rate")
saveas(gcf, './normal/ang_vel.png')

figure(5)

plot(controller.states(:,1))
grid on
hold on
plot(controller.states(:,2))
plot(controller.states(:,3))
legend(["x","y","z"])
xlabel('Time (s)')
ylabel('Position (m)')
title("Position")
saveas(gcf, './normal/position.png')

figure(6)

plot(controller.states(:,4))
grid on
hold on
plot(controller.states(:,5))
plot(controller.states(:,6))
legend(["vx","vy","vz"])
xlabel('Time (s)')
title("Velocity")
ylabel('Velocity (m/s)')
saveas(gcf, './normal/velocity.png')
