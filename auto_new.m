% Script to perform open-loop simulation for path tracking in autonomous vehicles across multiple runs with different initial conditions.
% 
% Author: Peter Schleede
clear

% get parameter struct
run('params');

% Define initial conditions for each run
initial_conditions = [
    0.05, 0.1, 0.02, 0, 0.05;    % Mild side slip and yaw rate with small heading deviation
    0.01, -0.05, 0.03, 10, 0.1;   % Moderate yaw rate with slight slip and heading deviation further along the path
    -0.05, 0.15, -0.05, 20, 0.15; % Higher yaw rate and sideslip, opposite direction, increased lateral deviation
    0.03, -0.1, 0.04, 30, 0;      % Strong negative yaw rate and slip, starting from a central path position
    -0.02, 0.08, -0.03, 40, 0.2;  % Strong sideslip in one direction with moderate yaw rate, further down the path
    0.04, 0.12, 0.05, 50, -0.1;   % Significant sideslip and yaw rate in a forward position, small negative deviation
];

L = size(initial_conditions, 1); % Number of runs

% Preallocate matrices for states and control inputs
all_x_hist = zeros(5, P.prob.num_steps+1, L);
all_F_hist = zeros(1, P.prob.num_steps, L);

% Main Experiment Loop
for run_idx = 1:L
    fprintf('Starting run %d/%d\n', run_idx, L);

    % Set initial condition for the current run
    x0 = initial_conditions(run_idx, :)';

    % Generate random control inputs within the defined force constraints
    F_hist = P.con.Fmin + (P.con.Fmax - P.con.Fmin) .* rand(1, P.prob.num_steps);

    % Initialize variables
    prev_alpha_r    = zeros(P.prob.T_long, 1);
    alpha_r_peak    = atan2(3*P.veh.mass*9.81*P.veh.mu*P.veh.a, P.veh.Ca*P.veh.L);
    r_max           = 9.81 * P.veh.mu / P.veh.Ux;
    r_min           = -r_max;
    K               = -0.005*ones(P.prob.T_long+1, 1);  % Fixed curvature
    f_inv           = create_f_tire_inv(P);  % Lookup table for tire forces

    x_hist = zeros(P.prob.num_states, P.prob.num_steps+1);
    x_hist(:,1) = x0;
    x_t = x0;

    % Simulation loop
    tic
    for k = 1:P.prob.num_steps
        % Simulate one time step
        alpha_r = alpha_r_peak;  % Assuming alpha_r is constant for simplification
        x_t = simulate(x_t, F_hist(k), alpha_r, K(1), P.prob.dt, f_inv, P);
        
        % Save the observed results
        x_hist(:, k+1) = x_t;
    end
    toc

    % Store results in the preallocated matrices
    all_x_hist(:, :, run_idx) = x_hist;
    all_F_hist(:, :, run_idx) = F_hist;

    fprintf('Completed run %d/%d\n', run_idx, L);
end

% Optionally, save the results to a file or workspace
save('simulation_results.mat', 'all_x_hist', 'all_F_hist');

% Sample visualization (optional)
% This block can be enhanced to generate more sophisticated visuals as needed
figure, hold on
for i = 1:L
    plot3(squeeze(all_x_hist(4,:,i)), squeeze(all_x_hist(5,:,i)), i*ones(size(all_x_hist(4,:,i))), 'DisplayName', sprintf('Run %d', i));
end
xlabel('s (m)'), ylabel('e (m)'), zlabel('Run Index');
title('Path Tracking Results across Runs');
legend show
