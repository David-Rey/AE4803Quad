close all;
clear;
clc;

%% Dynamics Model

mass = 1;
inertia = 2.5e-4;
radius = 1;
gravity = 9.8;
dt = 0.05;
tf = 100;

ic = [0; 0; 0; 0; 0; 0];
goal = [4; 2; 0; 0; 0; 0];

dyn = planar_quadrotor(mass, inertia, radius, gravity, dt);

%% Tunable Parameters

% Tune here!

Q = eye(6);
R = eye(2);
Qf = 500*Q;

regularizer = 20;  % 0 has no regularizing
initial_controls = zeros(tf, 2);
mode = 'ilqr';  % ilqr, ddp



%% Enable / Disable Animation

animate = true;

%%

[costfn, term_costfn] = quad_cost(Q, R, Qf, goal);

iters = 10;
initial_controls = (mass * gravity / 2) * ones(tf, 2);

[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn, mode);


%%

num_quads = 15;

rng(1);
cov = diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]);
sampled_ics = mvnrnd(ic, cov, num_quads);

trajs = {};

for i = 1:num_quads
    [states, ~, ~] = fwd_pass(sampled_ics(i, :)', controller, dyn, costfn, term_costfn);
    trajs{i} = states;
end

[nom_states, nom_controls, costs] = fwd_pass(ic, controller, dyn, costfn, term_costfn);

disp(['Nominal Cost: ' num2str(sum(costs))]);



%%

figure(Position=[10 10 1200 600])

layout = tiledlayout(2, 5);
nexttile(layout, [2, 3]);
hold on;

lines = {};
bodies = {};
wings = {};

for i = 1:num_quads
    lines{i} = plot([0 0], [0 0], LineWidth=1.5, Color=[1 0 0 0.25]);
    body{i} = scatter(0, 0, 'filled', 'o', MarkerFaceColor='k');
    wings{i} = plot([0 0], [0 0], 'k', LineWidth=2);
end

plot(states(:, 1), states(:, 2), '--', LineWidth=2, Color='k');
%scatter(goal(1), goal(2), 1000, 'k', 'o', LineWidth=3);
viscircles(goal(1:2)', 0.5, Color='b');

xlim([-5, 4.5])
ylim([-5, 3.5])
axis equal;

title('Planar Quadrotor Trajectories', FontSize=16);

nexttile(layout, [1 2]);

hold on;
max_nom_controls = max(nom_controls(:, 1), nom_controls(:, 2));
plot(max_nom_controls, LineWidth=2);
title('Largest Control Value', FontSize=16);

nexttile(layout, [1 2]);

hold on;
plot(total_costs, LineWidth=2);
title('Objective Cost', FontSize=16);

if animate
    for t = 1:tf
        for i = 1:num_quads
            lines{i}.XData = trajs{i}(1:t, 1);
            lines{i}.YData = trajs{i}(1:t, 2);
            bodies{i}.XData = trajs{i}(t, 1);
            bodies{i}.YData = trajs{i}(t, 2);
        
            wings{i}.XData = [trajs{i}(t, 1) - 0.1 * cos(trajs{i}(t, 3)); trajs{i}(t, 1) + 0.1 * cos(trajs{i}(t, 3))];
            wings{i}.YData = [trajs{i}(t, 2) + 0.1 * sin(trajs{i}(t, 3)); trajs{i}(t, 2) - 0.1 * sin(trajs{i}(t, 3))];
        end

        drawnow;
        pause(0.02)
    end
else
    for i = 1:num_quads
        lines{i}.XData = trajs{i}(1:end, 1);
        lines{i}.YData = trajs{i}(1:end, 2);
        bodies{i}.XData = trajs{i}(end, 1);
        bodies{i}.YData = trajs{i}(end, 2);
    
        wings{i}.XData = [trajs{i}(end, 1) - 0.1 * cos(trajs{i}(end, 3)); trajs{i}(end, 1) + 0.1 * cos(trajs{i}(end, 3))];
        wings{i}.YData = [trajs{i}(end, 2) + 0.1 * sin(trajs{i}(end, 3)); trajs{i}(end, 2) - 0.1 * sin(trajs{i}(end, 3))];
    end

    drawnow;
end