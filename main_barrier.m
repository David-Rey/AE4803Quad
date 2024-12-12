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
w_gain = 100;
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

% Define the grid size and the range
grid_size = 50;
x_range = linspace(-5, 5, grid_size);
y_range = linspace(-5, 5, grid_size);
z_range = linspace(-5, 5, grid_size);

% Create the 3D grid of points
[X, Y, Z] = meshgrid(x_range, y_range, z_range);

% Define the implicit functions for the spheres
h1 = @(x, y, z) (x - 2.2).^2 + (z - 1).^2 - 1;
h2 = @(x, y, z) x.^2 + (y + 0.2).^2 + z.^2 - 1;
h3 = @(x, y, z) (x - 3).^2 + y.^2 + (z - 0.5).^2 - 1;

% Evaluate the functions on the grid
H1 = h1(X, Y, Z);
H2 = h2(X, Y, Z);
H3 = h3(X, Y, Z);

%% plot trajectory
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
saveas(gcf, './barrier/3d_nobarriers.png')

% now plot with barriers
figure(2)
plot3(xs,ys,zs)
hold on
plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")

% plot barriers

h1_surf = isosurface(X, Y, Z, H1, 0);
h2_surf = isosurface(X, Y, Z, H2, 0);
h3_surf = isosurface(X, Y, Z, H3, 0);
p1 = patch(h1_surf, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
p2 = patch(h2_surf, 'FaceColor', 'green', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
p3 = patch(h3_surf, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

legend(["Flight path","Start Point","Goal"])
axis("equal")
grid("on")
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
title("Position")
saveas(gcf, './barrier/3d.png')

%% 2D plots

figure(3)

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
saveas(gcf, './barrier/controls.png')

figure(4)

plot(controller.states(:,4))
hold on
grid on
plot(controller.states(:,5))
plot(controller.states(:,6))
legend(["\phi","\theta","\psi"])
xlabel('Time (s)')
ylabel('Angle (rad)')
title("Attitude")
saveas(gcf, './barrier/attitude.png')

figure(5)

plot(controller.states(:, 10))
grid on
hold on
plot(controller.states(:, 11))
plot(controller.states(:, 12))
legend(["p","q","r"])
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title("Body Rate")
saveas(gcf, './barrier/ang_vel.png')

figure(6)

plot(controller.states(:,1))
grid on
hold on
plot(controller.states(:,2))
plot(controller.states(:,3))
legend(["x","y","z"])
xlabel('Time (s)')
title("Position")
ylabel('Position (m)')
saveas(gcf, './barrier/position.png')

figure(7)

plot(controller.states(:,4))
grid on
hold on
plot(controller.states(:,5))
plot(controller.states(:,6))
legend(["vx","vy","vz"])
xlabel('Time (s)')
title("Velocity")
ylabel('Velocity (m/s)')
saveas(gcf, './barrier/velocity.png')

% barrier state plot
figure(8)
plot(controller.states(:,13))
grid on
xlabel('Time (s)')
ylabel('Barrier State')
title("Barrier State")
saveas(gcf, './barrier/barrier_state.png')
