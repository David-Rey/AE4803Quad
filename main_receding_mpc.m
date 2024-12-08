clear; clc; close all;

% Given parameters
x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
dt = 0.01;  % time step
sim_horizon = 8;   % total sim time
planning_horizon = 3;  % "look ahead" amount

% set up dynamics
dyn = full_quadrotor(dt);

% Tune here!
pos_gain = 100;
vel_gain = 100;
ang_gain = 10;
ang_vel_gain = 10;
Q = diag([pos_gain, pos_gain, pos_gain, vel_gain, vel_gain, vel_gain, ang_gain, ang_gain, ang_gain, ang_vel_gain, ang_vel_gain, ang_vel_gain]);
R = 15*eye(4);
Qf = 100*Q;

iters = 10;
regularizer = 1;  % initial value. Will increment automatically
line_search_iters = 10;
mode = "ddp";
initial_controls = 0.612*ones(planning_horizon / dt, 4);  % initialize to neutral thrust
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
current_state = ic;
for t = 1:sim_horizon
    % get controller
    [controller, total_costs] = ddp(current_state, controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters);

    % use difference in current state and desired to get controls
    K = squeeze(controller.K(1,:,:));
    k = controller.k(1,:);
    ubar = controller.controls(1,:);
    xbar = controller.states(1,:);
    controls = ubar + (K * (current_state - xbar.')).' + k;

    % increment state
    [current_state, ~, ~, ~, ~, ~] = dyn(current_state, controls);

    % warm start
    controls = controller.controls; %; controller.controls(end,:)];

    % save vars
    % TODO: save here for plotting purposes
    states(t,:,:) = controller.states;
end

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
