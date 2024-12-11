function [controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode, line_search_iters)
%DDP Solves for a locally optimal controller using iLQR / DDP.
%
%   ic: An n-by-1 state serving as the initial condition for the control
%   problem.
%
%   initial_controls: The initial guess control sequence fo the algorithm.
%   It is an (tf - 1)-by-m matrix, where tf is the planning horizon used by
%   iLQR / DDP. 
%
%   iters: The number of iterations to use.
%
%   regularizer: The value to use for the regularizer parameter (rho).
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
% line_search_iters (int): max num of line search iterations. Min 1.

% Setup variables
tf = size(initial_controls, 1);
n = size(ic, 1);
m = size(initial_controls, 2);

controller = struct;
controller.K = zeros(tf, m, n);
controller.k = zeros(tf, m);
controller.states = zeros(tf + 1, n);
controller.controls = initial_controls;

total_costs = zeros(iters, 1);

% alphas = 1:-1/line_search_iters:0;  % decreasing step size sequence
alphas = exp(-(0:line_search_iters));

%% Your code below

[states, controls, best_costs] = fwd_pass(ic, controller, dyn, costfn, term_costfn, alphas(1));

for i = 1:iters
    [controller, ~] = back_pass(states, controls, dyn, costfn, term_costfn, regularizer, mode);
    
    for j = 1:line_search_iters-1
        % line search logic
        [states, controls, new_costs] = fwd_pass(ic, controller, dyn, costfn, term_costfn, alphas(j+1));
        if sum(new_costs) < sum(best_costs)
            best_costs = new_costs;
        end
    end

    total_costs(i,1) = sum(best_costs); 
end

end

