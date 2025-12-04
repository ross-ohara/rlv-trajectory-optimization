function params = rlv_params()
%RLV_PARAMS Define model and optimization parameters for the RLV problem.

    params = struct();

    % Physical parameters
    params.m  = 100000;           % kg
    params.g  = 9.81;             % m/s^2
    params.L  = 50;               % m
    params.r  = params.L / 2;     % m, thrust moment arm
    params.I  = (1/12) * params.m * params.L^2;  % kg m^2 (slender rod approx)

    % Control discretization
    params.res = 15;              % number of control nodes

    % Actuator limits
    params.F_max      = 15e6;     % N, 15 MN
    params.gimbal_max = pi/8;     % rad, +/- 22.5 deg

end
