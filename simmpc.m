function [states, controls] = simmpc(ic, sim_horizon, initial_controls, ddp_iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters)
%SIMMPC Simulation of receding-horizon model-predictive control.
%
%   This function simulates the execution of iLQR / DDP in a
%   receding-horizon manner. That is, at each time step, the selected
%   control algorithm is run for some number of iterations using a planning
%   horizon of a selected length. The feedback controller returned by the
%   algorithm for the first time step of the planning horizon is then used
%   to compute a control input. This control is then used to advance the
%   system state. This process is repeated using the new system state as
%   the initial condition for the optimal control algorithm.
%
%   The first execution of iLQR / DDP will use a guessed control sequence
%   to initialize the optimizer. Subsequently, you should warm-start the
%   process, i.e., use the nominal control sequence (u_bar) returned by the
%   control algorithm at the previous time step as the initial guess for
%   the current time step.
%
%   ic: An n-by-1 state serving as the initial condition for the control
%   problem.
%
%   sim_horizon: An integer representing the number of time steps for which
%   the receding-horizon algorithm is implemented. That is, we will run
%   iLQR / DDP and advance the system state based on the resulting control
%   law sim_horizon number of times.
%
%   initial_controls: The initial guess control sequence for the
%   simulation. This is fed to iLQR / DDP at the first time. It is an
%   h-by-m matrix, where (h + 1) is the planning horizon used by iLQR / DDP.
%
%   ddp_iters: The number of iterations of iLQR / DDP to run each time
%   step.
%
%   regularizer: The value of the regularizer passed to iLQR / DDP.
%
%   dyn: A function defining the robot dynamics and pertinent derivatives.
%   It has the form:
%
%       [next_state, fx, fu, fxx, fxu, fuu] = dynamics_wrapper(state, control)
%
%   All derivatives are evaluated at (state, control).
%
%   costfn: A function that computes the cost at each time step before the
%   terminal step. It also evaluates the pertinent derivatives at the
%   current state and control. It has the form:
%
%       [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control).
%
%   term_costfn: A function that computes the terminal cost as well as the
%   derivatives of the terminal cost function at a given state. It has the
%   form:
%
%       [cost, cx, cxx] = term_costfn(state)
%
%   mode: A string indicating whether to use iLQR or DDP. To use ddp, set
%
%       mode = 'ddp'
%
%   The default is to use iLQR.
%
%
%   Returns:
%
%   states: A sim_horizon-by-(h + 1)-by-n array. At simulation time step t,
%   the array states(t, :, :) contains the nominal states planned by iLQR /
%   DDP (i.e., controller.states).
%
%   controls: A sim_horizon-by-h-by-m array. At simulation time step
%   t, the array controls(t, :, :) contains the nominal controls planned by
%   iLQR / DDP (i.e., controller.controls).

% Setup Variables
n = size(ic, 1);
m = size(initial_controls, 2);
planning_horizon = size(initial_controls, 1) + 1; % h + 1.

states = zeros(sim_horizon, planning_horizon, n);
controls = initial_controls;

current_state = ic;
controls_history = zeros(sim_horizon, m);

% pseudocode to help developing
% for t = 1:sim_horizon
% run ddp for planning_horizon timesteps and ddp_iters iters
% apply controller to get next state
% current_state = next_state


for t = 1:sim_horizon
    % get controller
    disp(t)
    [controller, ~] = ddp(current_state, controls, ddp_iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters);
    
    % add states and controls to the final output
    % states(t,:,:) = controller.states;
    % controls = controller.controls(1,:);  % first one
    
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
    states(t,:,:) = controller.states;

end

end

