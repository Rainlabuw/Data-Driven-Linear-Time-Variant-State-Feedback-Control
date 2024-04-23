function X = simulate_closed_loop(K, X0, N)
    n = size(A(0), 1);  % Assuming n is the number of states
    X = zeros(n, N+1);
    X(:, 1) = X0;  % Set initial condition
    
    for k = 1:N
        if k < N
            X(:, k+1) = A(k-1) * X(:, k) + B(k-1) * (K{k} * X(:, k));
        % else
        %     % No control applied at the last step (if needed)
        %     X(:, k+1) = A(k-1) * X(:, k);
        end
        disp(['State at step ', num2str(k), ': '])
        disp(X(:, k+1))
    end
end
