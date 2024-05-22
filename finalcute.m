
n = 5;
m = 1;
L = m+n;
N = 250;

inputdata = load('input_cute.mat');
statedata = load('state_cute.mat');

X = statedata.all_x_hist;
U = inputdata.all_F_hist;

%% check rank condition
for k = 1:N
    % Extract the state and input data for time k from all experiments
    X_k = permute(X(:, k, :), [1, 3, 2]);  % State data at time k
    U_k = permute(U(:, k, :), [1, 3, 2]);  % Input data at time k
    % Combine state and input data into a single matrix
    combined_data = [X_k; U_k];
    % Check if the combined data matrix has full rank (should be n + m)
    if rank(combined_data) ~= n + m
        error('Rank condition failed at step %d', k);
    end
end
%% LQR Parameters
Q_f = 50 * eye(n);
for k = 1:N
    Qk = Q(k, n);
    Rk = R(k, m);
end
%% Optimization Variables
S = sdpvar(n, n, N+1, 'symmetric');
H = sdpvar(L, n, N, 'full');
O = sdpvar(m, m, N, 'symmetric');
%% Optimization Constraints
Constraints = [S(:,:,1) >= eye(n)]; % Initial constraint on S(0)
% Adding SDP constraints for each time step
for k = 1:N
    X_k = permute(X(:, k, :), [1, 3, 2]);   % State data at time k, reshaped correctly
    X_k_1 = permute(X(:, k+1, :), [1, 3, 2]);
    U_k = permute(U(:, k, :), [1, 3, 2]);   % Input data at time k, reshaped correctly
    % Construct block matrix constraints
    Constraints = [Constraints, ...
        [S(:,:,k+1) - eye(n), X_k_1 * H(:,:,k); ...
         H(:,:,k)' * X_k_1', S(:,:,k)] >= 0, ...
        [O(:,:,k), sqrtm(R(k,m)) * U_k * H(:,:,k); ...
         H(:,:,k)' * U_k' * sqrtm(R(k,m)), S(:,:,k)] >= 0, ...
        S(:,:,k) == X_k * H(:,:,k)];
end
%% Optimization Objective
% Initialize the objective
objective = 0;
% Add terms for each time step
for k = 1:N
    % Assuming Q(k) returns a matrix for the cost at time step k
    objective = objective + trace(Q(k,n) * S(:,:,k)) + trace(O(:,:,k));
end
% Add the final term involving Q_f and S(N)
objective = objective + trace(Q_f * S(:,:,N+1));  % Index N+1 for S(N) due to MATLAB indexing
% Setup the optimization problem
% Note: You need to define the constraints array before this step
options = sdpsettings('solver', 'mosek');  % Using MOSEK as the solver
sol = optimize(Constraints, objective, options);
% Check the result
if sol.problem == 0
    disp('Optimization solved successfully');
else
    disp(['Problem not solved, status: ' yalmiperror(sol.problem)]);
end
%% Calculate State Feedback Gain (K)
K_star = cell(1, N);  % Initialize cell array to store feedback gains
% Calculate the optimal feedback gains K*(k)
for k = 1:N
    % Ensure to correctly extract and use matrix dimensions
    % Assume U_k comes from some defined operational matrix, for example
    U_k = permute(U(:, k, :), [1, 3, 2]);
    % Extract values and compute gain
    Hk_value = value(H(:,:,k));
    Sk_value = value(S(:,:,k));
    K_star{k} = U_k * Hk_value * inv(Sk_value);  % Compute feedback gain using matrix inverse
end
% Print the optimal state feedback gains
disp('Optimal state feedback control gains K*(k):');
for k = 1:N
    fprintf('K*(%d) =\n', k-1);  % Adjusted index for display to match 0-based indexing
    disp(K_star{k});
end
%% Closed Loop Simulation for DD
X0 = [0.4411; 0.2711];
X_closed_loop = simulate_closed_loop(K_star, X0, N);
figure;
plot(0:N, X_closed_loop);
xlabel('Time step');
ylabel('State values');
title('Closed-Loop State Trajectories');
legend('State 1', 'State 2');
grid on;


5:13
%% Optimization using CVX
cvx_begin sdp
    % Define optimization variables
    variable S(n, n, N+1) symmetric
    variable H(L, n, N)
    variable O(m, m, N) symmetric
    % Define constraints
    Constraints = [S(:,:,1) >= eye(n)]; % Initial constraint on S(0)
    for k = 1:N
        X_k = permute(X(:, k, :), [1, 3, 2]);   % State data at time k, reshaped correctly
        X_k_1 = permute(X(:, k+1, :), [1, 3, 2]);
        U_k = permute(U(:, k, :), [1, 3, 2]);   % Input data at time k, reshaped correctly
        % Construct block matrix constraints
        Constraints = [Constraints, ...
            [S(:,:,k+1) - eye(n), X_k_1 * H(:,:,k); ...
             H(:,:,k)' * X_k_1', S(:,:,k)] >= 0, ...
            [O(:,:,k), sqrtm(R(k,m)) * U_k * H(:,:,k); ...
             H(:,:,k)' * U_k' * sqrtm(R(k,m)), S(:,:,k)] >= 0, ...
            S(:,:,k) == X_k * H(:,:,k)];
    end
    % Add constraints to CVX
    subject to
        Constraints;
    % Define the objective function
    objective = 0;
    for k = 1:N
        objective = objective + trace(Q(k,n) * S(:,:,k)) + trace(O(:,:,k));
    end
    objective = objective + trace(Q_f * S(:,:,N+1));
    % Minimize the objective
    minimize(objective)
cvx_end
% Check the result
if strcmp(cvx_status, 'Solved')
    disp('Optimization solved successfully');
else
    disp(['Problem not solved, status: ' cvx_status]);
end
%% Calculate State Feedback Gain (K)
K_star = cell(1, N);  % Initialize cell array to store feedback gains
% Calculate the optimal feedback gains K*(k)
for k = 1:N
    % Ensure to correctly extract and use matrix dimensions
    % Assume U_k comes from some defined operational matrix, for example
    U_k = permute(U(:, k, :), [1, 3, 2]);
    % Extract values and compute gain
    Hk_value = H(:,:,k);
    Sk_value = S(:,:,k);
    K_star{k} = U_k * Hk_value * inv(Sk_value);  % Compute feedback gain using matrix inverse
end











