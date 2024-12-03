function [controller, V] = back_pass(states, controls, dyn, costfn, term_costfn, regularizer, mode)
%BACK_PASS The backward pass of iLQR / DDP.
%
%   states: A tf-by-n matrix where the row t contains the state of the
%   system at time step t. These states are to be used for Taylor
%   approximations.
%
%   controls: A tf-by-m matrix where the row t contains the control applied
%   at time step t. These controls are to be used for Taylor
%   approximations.
%
%   dyn: A function defining the robot dynamics and pertinent derivatives.
%   It has the form:
%
%       [next_state, fx, fu, fxx, fxu, fuu] = dynamics_wrapper(state, control)
%
%   Note that, if the dynamics are f(x, u), the second-order derivatives of
%   the i-th entry of f(x, u) are fxx(i, :, :), fxu(i, :, :), fuu(i, :, :).
%
%   regularizer: The value to use for the regularization parameter
%   (defaults to 0).
%
%   mode: A string indicating whether to use iLQR or DDP. To use ddp, set
%
%       mode = 'ddp'
%
%   The default is to use iLQR.
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
%   controller: A struct with 4 entries defining the controller
%
%       controller.K: A (tf - 1)-by-m-by-n matrix defining the linear
%       feedback portion of the controller.
%
%       controller.k: A (tf - 1)-by-m matrix defining the affine term of
%       the controller.
%
%       controller.states: A tf-by-n matrix containing the states used
%       during the backward pass for Taylor expansions (i.e., x_bar).
%
%       controller.controls: A (tf - 1)-by-m matrix containing the controls
%       used during the backward pass for Taylor expansions (i.e., u_bar). 
%
%
%   V (Optional): The value of the value function at t = 0 evaluated at the
%   initial condition x_0, i.e. V_0(x_0). This is not used elsewhere in the
%   homework. You do not need to compute it if you do not want to. However,
%   it may be useful during debugging. When iLQR / DDP is applied to a
%   linear system, this value should match the sum of the costs returned by
%   the forward pass. 


% Defaults to iLQR.
if nargin < 7
    mode = 'ilqr';
end

% Defaults to no regularization.
if nargin < 6
    regularizer = 0;
end


% Some basic setup code for you
horizon = size(controls, 1);
n = size(states, 2);
m = size(controls, 2);

controller = struct;
controller.K = zeros(horizon, m, n);
controller.k = zeros(horizon, m);
controller.states = states;
controller.controls = controls;

%% Fill in your code below

% Initialize with final values
[~, cx, ~, Q, ~, ~] = costfn(states(horizon,:).', controls(horizon,:).');
next_P = Q;
next_p = Q*(states(horizon,:).') - states(horizon,:)*cx;

% Initialize final Vx
[~, cx, cxx] = term_costfn(states(horizon,:).');
Vx_next = cx;
Vxx_next = cxx;

for t = horizon:-1:1
    % Evaluate cost and function
    [~, cx, cu, cxx, cxu, cuu] = costfn(states(t,:).', controls(t,:).');
    [f, fx, fu, fxx, fxu, fuu] = dyn(states(t,:).', controls(t,:).');

    % Get first-order derivatives of Q
    Qx = cx + (fx.')*Vx_next - regularizer*(fx.')*(states(t+1,:).' - f);
    Qu = cu + (fu.')*Vx_next - regularizer*(fu.')*(states(t+1,:).' - f);

    if strcmp(mode, 'ddp')
        % Get second-order derivatives of Q
        Qxx = cxx + (fx.')*Vxx_next*fx + fake_tensor_prod(Vx_next,fxx) + regularizer*(fx.')*fx;
        Qxu = cxu + (fx.')*Vxx_next*fu + fake_tensor_prod(Vx_next,fxu) + regularizer*(fx.')*(fu);
        Qux = Qxu.';
        Quu = cuu + (fu.')*Vxx_next*fu + fake_tensor_prod(Vx_next,fuu) + regularizer*(fu.')*(fu);
        
    elseif strcmp(mode,'ilqr')
        % Get second-order derivatives of Q without tensor prod
        Qxx = cxx + (fx.')*Vxx_next*fx;
        Qxu = cxu + (fx.')*Vxx_next*fu;
        Qux = Qxu.';
        Quu = cuu + (fu.')*Vxx_next*fu;
    end

    % Get new controls
    K = -inv(Quu)*Qux;
    k = -inv(Quu)*Qu;

    % Get new value function
    Vx_next = Qx - (K.')*Quu*k;
    Vxx_next = Qxx - (K.')*Quu*K;

    % Assign
    controller.K(t,:,:) = K;
    controller.k(t,:,:) = k;
end


V = 0; % don't bother defining V
end

function out = fake_tensor_prod(A,B)
    out = 0;
    for i = 1:6
        out = out + squeeze(A(i)*B(i,:,:));
    end
    out = out/3;
end

