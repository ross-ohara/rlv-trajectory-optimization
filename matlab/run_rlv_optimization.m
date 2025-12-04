function run_rlv_optimization
%RUN_RLV_OPTIMIZATION
%   Minimum-energy flip-and-land trajectory optimization for a reusable
%   launch vehicle using a planar rigid-body model and thrust vectoring.
%
%   This script:
%     1) Defines model and optimization parameters
%     2) Builds an initial guess for the controls and final time
%     3) Solves the nonlinear program using fmincon
%     4) Simulates the optimal trajectory
%     5) Plots states and controls

    clear; clc; close all;

    % ------------------------------
    % Model & optimization parameters
    % ------------------------------
    params = rlv_params();

    % Initial state [x; z; theta; vx; vz; omega]
    x0     = -300;     % m
    z0     =  1000;    % m
    th0    =  pi/2;    % rad
    vx0    =   25;     % m/s
    vz0    =  -90;     % m/s
    w0     =    0;     % rad/s
    y0     = [x0; z0; th0; vx0; vz0; w0];

    % ------------------------------
    % Initial guess (no .mat file)
    % ------------------------------
    U0 = build_initial_guess(params);

    % Decision variable bounds
    lb = zeros(size(U0));
    ub = ones(size(U0));
    lb(end) =  1;      % tf_min
    ub(end) = 20;      % tf_max

    % ------------------------------
    % fmincon options
    % ------------------------------
    options = optimoptions('fmincon', ...
        'Display',              'iter', ...
        'Algorithm',            'sqp', ...    % or 'interior-point'
        'StepTolerance',        1e-17, ...
        'ConstraintTolerance',  5e-3, ...
        'MaxFunctionEvaluations', 2e4, ...
        'UseParallel',          true);

    % ------------------------------
    % Solve trajectory optimization
    % ------------------------------
    cost_handle = @(Uvect) rlv_cost(Uvect, params, y0);
    con_handle  = @(Uvect) rlv_constraints(Uvect, params, y0);

    [U_opt, fval, exitflag, output] = fmincon( ...
        cost_handle, U0, [], [], [], [], lb, ub, con_handle, options);

    fprintf('\nOptimization complete. Exit flag: %d, Cost: %.4f\n', exitflag, fval);

    % ------------------------------
    % Simulate and plot results
    % ------------------------------
    rlv_simulate_and_plot(U_opt, params, y0);

    % Optional: animation
    rlv_make_animation(U_opt, params, y0);

end

% ------------------------------
% Helper: initial guess
% ------------------------------
function U0 = build_initial_guess(params)
%BUILD_INITIAL_GUESS Construct a simple initial guess for [thrust; gimbal; tf]

    res = params.res;
    tf0 = 10;  % initial guess for final time (s)

    % Thrust guess: 
    thrust0 = 0.05*zeros(res,1);     % normalized [0,1]
    thrust0(1) = 0.05;

    % Gimbal guess: 0.5 -> maps to 0 rad (no deflection)
    deflect0 = 0.5*zeros(res,1);           % normalized [0,1]
    deflect0(1) = 0.01;

    U0 = [thrust0; deflect0; tf0];

end
