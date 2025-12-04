function xdot = rlv_dynamics(t, x, params, Uvect)
%RLV_DYNAMICS Nonlinear planar RLV dynamics with thrust vectoring.
%
%   States:
%       x(1) = x    (horizontal position)
%       x(2) = z    (vertical position)
%       x(3) = theta (pitch angle)
%       x(4) = vx   (horizontal velocity)
%       x(5) = vz   (vertical velocity)
%       x(6) = omega (angular rate)
%
%   Controls (encoded in Uvect over params.res nodes):
%       thrustCurve     \in [0,1] -> scaled by F_max
%       deflectionAngle \in [0,1] -> mapped to [-gimbal_max, +gimbal_max]

    m   = params.m;
    g   = params.g;
    r   = params.r;
    I   = params.I;
    res = params.res;

    thrustCurve     = Uvect(1:res);
    deflectionAngle = Uvect(res+1:2*res);
    tf              = Uvect(end);

    % Time grid for control nodes
    tvect = linspace(0, tf, res);

    % Interpolate controls at current time
    Ut  = interp1(tvect, thrustCurve,     t, 'pchip');
    Uda = interp1(tvect, deflectionAngle, t, 'pchip');

    % Map normalized controls to physical values
    Ft      = Ut  * params.F_max;                          % N
    theta_t = (Uda - 0.5) * 2 * params.gimbal_max;         % rad

    % State aliases
    theta = x(3);
    vx    = x(4);
    vz    = x(5);
    omega = x(6);

    % Forces in inertial frame
    Fx = Ft * sin(theta_t) * cos(theta) + Ft * cos(theta_t) * sin(theta);
    Fz = -Ft * sin(theta_t) * sin(theta) + Ft * cos(theta_t) * cos(theta) - m*g;

    % Moment about center of mass
    M = Ft * sin(theta_t) * r;

    % Accelerations
    ax     = Fx / m;
    az     = Fz / m;
    alpha  = M  / I;

    % State derivatives
    xdot = [vx;
            vz;
            omega;
            ax;
            az;
            alpha];

end
