function [c, ceq] = rlv_constraints(Uvect, params, y0)
%RLV_CONSTRAINTS Nonlinear constraints for RLV landing.
%
%   Equality constraints (ceq):
%     - final state reaches target (x = 0, z = 0, theta = 0, vx = 0, vz = 0, omega = 0)
%     - final accelerations approximately zero (ax = 0, az = 0, alpha = 0)
%
%   Inequality constraints (c <= 0):
%     - max |theta| <= pi/2  (no more than 90 deg from vertical)
%     - z(t) >= 0 for all t  (no underground penetration)

    res   = params.res;
    tf    = Uvect(end);
    tspan = [0 tf];

    dyn = @(t,x) rlv_dynamics(t, x, params, Uvect);
    [t, X] = ode45(dyn, tspan, y0);

    % Final state and final accelerations
    x_final   = X(end, :).';
    xdot_final = rlv_dynamics(t(end), X(end,:).', params, Uvect);

    % Desired terminal state [0;0;0;0;0;0]
    ceq_state = x_final;          % shoot for zero across all states
    ceq_accel = xdot_final(4:6);  % ax, az, alpha -> ~0

    ceq = [ceq_state;
           ceq_accel];

    % Path constraints
    theta_traj = X(:,3);
    z_traj     = X(:,2);

    c_theta = max(abs(theta_traj)) - pi/2;  % <= 0
    c_z     = -min(z_traj);                 % <= 0  (enforce z >= 0)

    c = [c_theta;
         c_z];

end
