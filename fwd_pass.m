function [states, controls, costs] = fwd_pass(ic, controller, dynamics_wrapper, costfn, term_costfn, alpha, xf)
%FWD_PASS Forward Pass for iLQR / DDP.
%   This function computes the forward pass for the current control law.
%
%   ic: An n-by-1 state serving as the initial condition for the control
%   problem.
%
%   controller: A struct with 4 entries defining the controller
%
%       controller.K: A (tf - 1)-by-m-by-n matrix defining the linear
%       feedback portion of the controller.
%
%       controller.k: A (tf - 1)-by-m matrix defining the affine term of
%       the controller.
%
%       controller.states: A tf-by-n matrix containing the states used
%       during the backward pass for Taylor expansions.
%
%       controller.controls: A (tf - 1)-by-m matrix containing the controls
%       used during the backward pass for Taylor expansions.
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
%   Returns:
%
%   states: A tf-by-n matrix where the row t contains the state of the
%   system at time step t.
%
%   controls: A tf-by-m matrix where the row t contains the control applied
%   at time step t.
%
%   costs: A tf-by-1 matrix where costs(t) contains the cost incurred at
%   time step t. Note that costs(end) (i.e., costs(tf)) should contain the
%   value of the terminal cost function.

% alpha: step size float

% Setup Variables
tf = size(controller.k, 1);
n = size(ic, 1);
m = size(controller.k, 2);

states = zeros(tf + 1, n);
controls = zeros(tf, m);
costs = zeros(tf + 1, 1);

states(1, :) = ic.';
% update w for initial state
barrier_func = get_barrier_func();
states(1,13) = barrier_func(ic) - barrier_func(xf);

%% Your Code Below


for t = 1:tf
    % get control input from control law
    state = states(t,:); % current x
    xbar = controller.states(t,:);  % current xbar
    ubar = controller.controls(t,:);  % current ubar
    K = squeeze(controller.K(t,:,:));  % current K
    k = alpha * (controller.k(t,:));  % current k given step size
    control = ubar + (K * (state.' - xbar.')).' + k;
    controls(t,:) = control;  % append
    
    % calculate cost this step
    [cost, cx, cu, cxx, cxu, cuu] = costfn(state.', control.');
    costs(t) = cost;
    
    % get next state from dynamics
    [next_state, fx, fu, fxx, fxu, fuu] = dynamics_wrapper(state.', control.');
    states(t+1,:) = next_state.';
end
[tcost, tcx, tcxx] = term_costfn(state.');
costs(end) = tcost;

end

