function [X, U] = generate_data(L, N, n, m)
    % Initialize the state and input matrices
    X = zeros(n, N+1, L);  % States over time for all experiments
    U = zeros(m, N, L);    % Inputs over time for all experiments
    % Loop over each experiment
    for j = 1:L
        X(:, 1, j) = randn(n, 1); % Random initial state for each experiment
        
        % Loop over each time step
        for k = 1:N
            U(:, k, j) = rand(m, 1); % Random input for each experiment
            X(:, k+1, j) = A(k-1) * X(:, k, j) + B(k-1) * U(:, k, j);  % State update
        end
    end
end
