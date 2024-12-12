clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
dt = 0.01;  % time step
sim_horizon = 600;   % total sim time (iterations, i.e 50 => 0.5 seconds)
planning_horizon = 3;  % "look ahead" amount (seconds)
n = length(x0);
m = 4;

t_arr = (0:sim_horizon) * dt;


% set up dynamics
dyn = full_quadrotor_barrier(dt, xf);

% Tune here!
pos_gain = 1;
vel_gain = 1;
ang_gain = 1;
ang_vel_gain = 1;
w_gain = 0;  % no barrier state
Q = diag([pos_gain, pos_gain, pos_gain, vel_gain, vel_gain, vel_gain, ang_gain, ang_gain, ang_gain, ang_vel_gain, ang_vel_gain, ang_vel_gain, w_gain]);
R = 1*eye(4);
Qf = 10*Q;

iters = 1;
regularizer = 10;  % initial value. Will increment automatically
line_search_iters = 3;
mode = "ilqr";
initial_controls = 1.225*ones(planning_horizon / dt, 4);  % initialize to neutral thrust
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);

% run controller receding horizon
controls = initial_controls;

state_hist = zeros(n, sim_horizon);
contol_hist = zeros(m, sim_horizon);

current_state = ic;
for t = 1:sim_horizon
    disp(t)
    % get controller
    %if mod(t, 4)
    [controller, total_costs] = ddp(current_state, controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters);
    %end
    % use difference in current state and desired to get controls
    K = squeeze(controller.K(1,:,:));
    k = controller.k(1,:);
    ubar = controller.controls(1,:);
    xbar = controller.states(1,:);
    controls = ubar + (K * (current_state - xbar.')).' + k;

    % increment state
    [current_state, ~, ~, ~, ~, ~] = dyn(current_state, controls(1, :));

    % warm start
    controls = controller.controls; %; controller.controls(end,:)];

    % save vars
    % TODO: save here for plotting purposes
    states(t,:,:) = controller.states;

    state_hist(:, t) = current_state;
    contol_hist(:, t) = controls(1, :).';
end

total_costs(end)
final_cost = norm(controller.states(end,:) - xf)


%% Plot result

xs = state_hist(1, :);
ys = state_hist(2, :);
zs = state_hist(3, :);

figure(1)
plot3(xs,ys,zs)
xlabel("X")
ylabel("Y")
zlabel("Z")
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])
grid("on")
axis("equal")
title("Position")
saveas(gcf, './receding/3d.png')


%% 2D plots

figure(2)
plot(t_arr(1:end-1), contol_hist(1, :))
hold on
grid on
plot(t_arr(1:end-1), contol_hist(2, :))
plot(t_arr(1:end-1), contol_hist(3, :))
plot(t_arr(1:end-1), contol_hist(4, :))
legend(["u1","u2","u3","u4"])
xlabel('Time (s)')
ylabel('Control Input (N)')
title("Controls")
saveas(gcf, './receding/controls.png')

figure(3)
plot(t_arr(1:end-1), state_hist(7, :))
hold on
grid on
plot(t_arr(1:end-1), state_hist(8, :))
plot(t_arr(1:end-1), state_hist(9, :))
legend(["\phi","\theta","\psi"])
xlabel('Time (s)')
ylabel('Angle (rad)')
title("Attitude")
saveas(gcf, './receding/attitude.png')

figure(4)

plot(t_arr(1:end-1), state_hist(10, :))
grid on
hold on
plot(t_arr(1:end-1), state_hist(11, :))
plot(t_arr(1:end-1), state_hist(12, :))
legend(["p","q","r"])
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title("Body Rate")
saveas(gcf, './receding/ang_vel.png')

figure(5)

plot(t_arr(1:end-1), state_hist(1, :))
grid on
hold on
plot(t_arr(1:end-1), state_hist(2, :))
plot(t_arr(1:end-1), state_hist(3, :))
legend(["x","y","z"])
xlabel('Time (s)')
ylabel('Position (m)')
title("Position")
saveas(gcf, './receding/position.png')

figure(6)
plot(t_arr(1:end-1), state_hist(4, :))
grid on
hold on
plot(t_arr(1:end-1), state_hist(5, :))
plot(t_arr(1:end-1), state_hist(6, :))
legend(["vx","vy","vz"])
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title("Velocity")
saveas(gcf, './receding/velocity.png')
