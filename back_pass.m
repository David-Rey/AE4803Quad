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

    % Initialize value function gradients at the final time step
    [~, Vxnext, Vxxnext] = term_costfn(states(horizon, :).');

    for t = horizon-1:-1:1
        % Get terms for this iteration
        state = controller.states(t, :);  % xbar
        control = controller.controls(t, :);  % ubar
        next_state = controller.states(t+1,:);
        % next_control = controller.controls(t+1, :);

        % Evaluate dynamics and cost function
        [dynamics_next_state, fx, fu, fxx, fxu, fuu] = dyn(state.', control.');
        [~, cx, cu, cxx, cxu, cuu] = costfn(state.', control.');

        % Q-function derivatives
        Qx = cx + fx.' * Vxnext;
        Qu = cu + fu.' * Vxnext;
        Qxx = cxx + fx.' * Vxxnext * fx;
        Quu = cuu + fu.' * Vxxnext * fu;
        Qxu = cxu + fx.' * Vxxnext * fu;
        Qux = Qxu.';

        % DDP add tensor products
        if strcmp(mode, 'ddp')
            Qxx = Qxx + tprod(Vxnext, fxx);
            Quu = Quu + tprod(Vxnext, fuu);
            Qxu = Qxu + tprod(Vxnext, fxu);
        end

        % Regularization
        
        Quu_reg = Quu + regularizer * (fu).' * fu;

        % Qxx_reg = Qxx - regularizer * fx.' * fx;
        % Qxu_reg = Qxu + regularizer * fx.' * fu;
        % Qux_reg = Qxu_reg.';
        % Qx_reg = Qx - regularizer * fx.' * (next_state.' - dynamics_next_state);
        % Qu_reg = Qu - regularizer * fu.' * (next_state.' - dynamics_next_state);

        % Feedback and feedforward gains
        K_t = -Quu_reg \ Qux;
        k_t = -Quu_reg \  Qu;

        % Update value function gradients

        Vxnext = Qx - K_t.' * Quu * k_t;
        Vxxnext = Qxx - K_t.' * Quu * K_t;

        % Store gains
        controller.K(t, :, :) = K_t;
        controller.k(t, :) = k_t.';
    end

    % Optional: Calculate the value function at the initial state
    % V = Vx * states(1, :)' + 0.5 * states(1, :) * Vxx * states(1, :)';
    V = 0;

    function out = tprod(A, B)
        out = 0;
        for i = 1:6
            val = A(i) * squeeze(B(i,:,:));
            out = out + val;
        end
    end

end
