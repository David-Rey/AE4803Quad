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

initial_mu = 1e-2;
delta_0 = 2;

mu = initial_mu;
mu_min = 1e-6;
delta = delta_0;



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
    reg = regularizer;

	delta = delta_0;
    
    % automatically increment regularizer if Quu not invertible
    for i = 1:50
        % Get first-order derivatives of Q
        Qx = cx + (fx.')*Vx_next - reg*(fx.')*(states(t+1,:).' - f);
        Qu = cu + (fu.')*Vx_next - reg*(fu.')*(states(t+1,:).' - f);
    
        if strcmp(mode, 'ddp')
            % Get second-order derivatives of Q
            Qxx = cxx + (fx.')*Vxx_next*fx + fake_tensor_prod(Vx_next,fxx);% + reg*(fx.')*fx;
            Qxu = cxu + (fx.')*(Vxx_next + reg * eye(length(Qxx)))*fu + fake_tensor_prod(Vx_next,fxu);
            Qux = Qxu.';
            Quu = cuu + (fu.')*(Vxx_next + reg * eye(length(Qxx)))*fu + fake_tensor_prod(Vx_next,fuu);
    
        elseif strcmp(mode,'ilqr')
            % Get second-order derivatives of Q without tensor prod
            Qxx = cxx + (fx.')*Vxx_next*fx;
            Qxu = cxu + (fx.')*Vxx_next*fu;
            Qux = Qxu.';
            Quu = cuu + (fu.')*Vxx_next*fu;
		end
		
		Quu = (Quu + Quu.')/2;
		%Quu = Quu + mu * eye(size(Quu));

        % check if Quu is positive definite
        if all(eig(Quu) > 1e-6) && rcond(Quu) > 1e-8
            break
		end

        % Increase mu if Quu is not positive definite
        delta = max(delta_0, delta * delta_0);
        mu = max(mu_min, mu * delta);

        if reg == 0 || isinf(reg)
            disp('Reg 0 or inf');
            break
        end
        if isnan(rcond(Quu))
            disp('NaN rcond(Quu)');
            break
        end
        % otherwise, increment regularizer
        %reg = reg * 2;

    end % regularizer update loop

    % Get new controls
	
    K = -Quu \ Qux;
    k = -Quu \ Qu;

    % Get new value function
    %Vx_next = Qx - (K.')*Quu*k
	Vx_next = Qx + K.'*Quu*k + K.'*Qu + Qux.'*k;
	Vxx_next = Qxx + K.'*Quu*K + K.'*Qux + Qux.'*K;
    %Vxx_next = Qxx - (K.')*Quu*K;

    % Decrease mu for next iteration if successful
    delta = min(1 / delta_0, delta / delta_0);
    if mu * delta > mu_min
        mu = mu * delta;
    else
        mu = 0;
    end

	%disp(sort(eig(Quu)))

    % Assign
    controller.K(t,:,:) = K;
    controller.k(t,:,:) = k;
end

V = -0.5 * (Qu.' * pinv(Quu) * Qu);  % cost to go
end

function out = fake_tensor_prod(A,B)
    out = 0;
	n = length(A);
    for i = 1:n
        out = out + squeeze(A(i)*B(i,:,:));
	end
	out = out / 3;
end

%% Fill in your code below
%{
%ILQR
%Determine final cost at last time t
%[cost, cx, cxx] = term_costfn(states(end,:)');
[~, cx, ~, Q, ~, ~] = costfn(states(horizon,:).', controls(horizon,:).');


%Initial value is the final state (work backwards)
P_plus_1 = Q; %Initial Value = Final State Q
p_plus_1 = Q*(states(horizon,:).') - states(horizon,:)*cx; %Initial Value = Final State q

%DDP
%Initial value is the final state (work backwards)
[cost, cx, cxx] = term_costfn(states(horizon, :).');
V = cost;
Vx = cx;
Vxx = cxx;


% [~, cx, ~, Q, ~, ~] = costfn(states(horizon,:).', controls(horizon,:).');
% next_P = Q;
% next_p = Q*(states(horizon,:).') - states(horizon,:)*cx;
% 
% 
% % Initialize final Vx
% [~, cx, cxx] = term_costfn(states(horizon,:).');
% Vx_next = cx;
% Vxx_next = cxx;

for t = horizon:-1:1

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %DDP Without Tensor Products = ILQR
    %Determine cost derivatives at time t
    [cost, cx, cu, cxx, cxu, cuu] = costfn(states(t,:)', controls(t,:)');

    %Determine state derivatives at time t
    [f, fx, fu, fxx, fxu, fuu] = dyn(states(t,:)', controls(t,:)');
    
    %Solve for Q functions
    Qx = cx + fx' * Vx - regularizer*(fx.')*(states(t+1,:).' - f);
    Qu = cu + fu' * Vx - regularizer*(fu.')*(states(t+1,:).' - f);

    Qxx = cxx + fx'*Vxx*fx;
    Qxu = cxu + fx'*Vxx*fu;
    Quu = cuu + fu'*Vxx*fu;

    %Add tensor product terms for DDP
    if strcmp(mode, 'ddp')
        for i = 1:n
            Qxx = Qxx + Vx(i) * squeeze(fxx(i,:,:));
            Qxu = Qxu + Vx(i) * squeeze(fxu(i,:,:));
            Quu = Quu + Vx(i) * squeeze(fuu(i,:,:));
        end
        % Add regularization after tensor terms
        Qxx = Qxx + regularizer * (fx.') * fx;
        Qxu = Qxu + regularizer * (fx.') * fu;
        Quu = Quu + regularizer * (fu.') * fu;
    end

    Qux = Qxu'; %Basic transpose property

    %Regularizer
    %Quu = Quu + regularizer * eye(m);

    %Solve for K and k
    K = -Quu \ Qux;
    k = -Quu \ Qu;

    %Find new Vx and Vxx for next iteration
    Vx = Qx - K'*Quu*k;
    Vxx = Qxx - K'*Quu*K;

    %Save K and k in struc
    controller.K(t,:, :) = K;
    controller.k(t, :) = k;

    %Value function adds all costs at each step to find value at t = 0
    V = V + cost;
    

end
end
%}