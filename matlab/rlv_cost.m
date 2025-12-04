function J = rlv_cost(Uvect, params, y0)
%RLV_COST Integral of squared normalized controls over time.

    res = params.res;

    tf    = Uvect(end);
    tspan = [0 tf];

    % Simulate trajectory
    dyn = @(t,x) rlv_dynamics(t, x, params, Uvect);
    [t, ~] = ode45(dyn, tspan, y0);

    thrustCurve     = Uvect(1:res);
    deflectionAngle = Uvect(res+1:2*res);
    tvect           = linspace(0, tf, res);

    % Interpolate normalized controls over the continuous time grid
    Tc = interp1(tvect, thrustCurve,     t, 'pchip');
    Dc = interp1(tvect, deflectionAngle, t, 'pchip');

    % Simple L2 cost on normalized controls
    J = trapz(t, Tc.^2 + Dc.^2);

end
