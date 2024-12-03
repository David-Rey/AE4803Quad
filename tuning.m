%% For tuning parameters
clear; close all; clc;

p0 = [0.244;112.1;37;1];
p0 = [0.1234;246.39;23.0943;1];
jiggle = 0.1;


p = p0;
old_cost = 100;
new_cost = 90;
costs = [];
while new_cost < old_cost
    directions = zeros(1,8);
    perturbs = [jiggle*p.*eye(4);-jiggle*p.*eye(4)];
    for j = 1:8
        perturb = perturbs(j,:).';
        directions(:,j) = one_run(p + perturb);
    end
    [m,n] = min(directions,[],"all");
    perturb = perturbs(n,:).';
    p = p + perturb;
    
    old_cost = new_cost;
    new_cost = one_run(p)
    costs(end+1) = new_cost;
end


function cost = one_run(parameters)% Given parameters
    x0 = [-3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % initial state
    xf = [5, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0].';  % final state
    tf = 8;  % final time
    dt = 0.01;  % time step
    
    % set up dynamics
    dyn = full_quadrotor(dt);
    
    % Tune here!
    Q = parameters(1)*eye(length(x0));
    R = parameters(2)*eye(4);
    Qf = parameters(3)*eye(length(x0));
    
    iters = 10;
    regularizer = parameters(4);
    mode = "ddp";
    initial_controls = 0.612*ones(tf / dt, 4);  % initialize to neutral thrust
    ic = x0;
    
    % get cost functions
    [costfn, term_costfn] = quad_cost(Q, R, Qf, xf);
    % form: [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control)
    % form: [cost, cx, cxx] = term_costfn(state)
    
    % run controller
    [controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode);
    
    cost = norm(controller.states(end,:) - xf);
end