function controllable = check_controllability(A_func, B_func, N)
    n = size(A_func(0), 1);  % State dimension
    controllable = true;  % Assume controllability unless proven otherwise
    for k = 0:N-1
        A_k = A_func(k);
        B_k = B_func(k);
        % Use MATLAB's built-in function to build the controllability matrix
        C_k = ctrb(A_k, B_k);
        % Check if the matrix is full rank
        if rank(C_k) < n
            controllable = false;
            break;
        end
    end
end
