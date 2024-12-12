clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
dt = 0.01;  % time step
sim_horizon = 300;   % total sim time (iterations, i.e 50 => 0.5 seconds)
planning_horizon = 3;  % "look ahead" amount (seconds)
n = length(x0);
m = 4;

% set up dynamics
dyn = full_quadrotor_barrier(dt, xf);

% Tune here!
pos_gain = 1;
vel_gain = 1;
ang_gain = 1;
ang_vel_gain = 1;
w_gain = 0;  % no barrier state
Q = diag([pos_gain, pos_gain, pos_gain, vel_gain, vel_gain, vel_gain, ang_gain, ang_gain, ang_gain, ang_vel_gain, ang_vel_gain, ang_vel_gain, w_gain]);
R = 2*eye(4);
Qf = 10*Q;

iters = 1;
regularizer = 1;  % initial value. Will increment automatically
line_search_iters = 3;
mode = "ilqr";
initial_controls = 1.225*ones(planning_horizon / dt, 4);  % initialize to neutral thrust
ic = x0;

% get cost functions
[costfn, term_costfn] = quad_cost(Q, R, Qf, xf);



%% MPC BELOW
% PASTED FROM HW2 CODE:

% n = size(ic, 1);
% m = size(initial_controls, 2);
% planning_horizon = size(initial_controls, 1) + 1; % h + 1.
% 
% states = zeros(sim_horizon, planning_horizon, n);
% controls = initial_controls;
% 
% current_state = ic;
% controls_history = zeros(sim_horizon, m);


% pseudocode to help developing:
% for t = 1:sim_horizon
% run ddp for planning_horizon timesteps and ddp_iters iters
% apply controller to get next state
% current_state = next_state

% run controller receding horizon
controls = initial_controls;

%[states, controls] = simmpc(ic, sim_horizon, initial_controls, iters, regularizer, dyn, costfn, term_costfn, 'ilqr', line_search_iters);
%function [states, controls] = simmpc(ic, sim_horizon, initial_controls, ddp_iters, regularizer, dyn, costfn, term_costfn, mode)

%plot_states = squeeze(states(:, 1, :));

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

%disp(states(1, :, 1))

%xs = controller.states(:,1);
%ys = controller.states(:,2);
%zs = controller.states(:,3);

xs = state_hist(1, :);
ys = state_hist(2, :);
zs = state_hist(3, :);

figure(1)
title("Position")
plot3(xs,ys,zs)
hold on

plot3(-3,-2,-1,"ro")
plot3(5,3,2,"rx")
legend(["Flight path","Start Point","Goal"])
grid("on")
axis("equal")

figure(2)
title("Controls")
plot(contol_hist(1, :))
grid on
hold on

plot(contol_hist(2, :))
plot(contol_hist(3, :))
plot(contol_hist(4, :))
legend(["u1","u2","u3","u4"])

figure(3)
title("Attitude")
plot(state_hist(7, :))
grid on
hold on
plot(state_hist(8, :))
plot(state_hist(9, :))
legend(["\phi","\theta","\psi"])

figure(4)
title("Body Rate")
plot(state_hist(10, :))
grid on
hold on
plot(state_hist(11, :))
plot(state_hist(12, :))
legend(["p","q","r"])