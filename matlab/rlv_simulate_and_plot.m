function rlv_simulate_and_plot(Uvect, params, y0)
%RLV_SIMULATE_AND_PLOT Simulate optimal trajectory and plot states/controls.

    res = params.res;
    tf  = Uvect(end);
    tspan = [0 tf];

    dyn = @(t,x) rlv_dynamics(t, x, params, Uvect);
    [t, X] = ode45(dyn, tspan, y0);

    thrustCurve     = Uvect(1:res);
    deflectionAngle = Uvect(res+1:2*res);
    tvect           = linspace(0, tf, res);
    Tc = interp1(tvect, thrustCurve,     t, 'pchip');
    Dc = interp1(tvect, deflectionAngle, t, 'pchip');

    Ft      = Tc * params.F_max / 1e6;             % MN
    theta_t = (Dc - 0.5) * 2 * params.gimbal_max; % rad

    figure('Name','RLV States and Controls','NumberTitle','off');
    subplot(2,3,1);
    plot(t, X(:,1:2));
    legend('x_E','z_E');
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on; grid minor;

    subplot(2,3,2);
    plot(t, X(:,4:5));
    legend('v_x','v_z');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    grid on; grid minor;

    subplot(2,3,3);
    plot(t, X(:,3));
    legend('\theta');
    xlabel('Time (s)');
    ylabel('Orientation (rad)');
    grid on; grid minor;

    subplot(2,3,4);
    plot(X(:,1), X(:,2), 'k');
    legend('Flight Path');
    xlabel('X (m)');
    ylabel('Z (m)');
    axis equal;
    grid on; grid minor;

    subplot(2,3,5);
    plot(t, Ft);
    legend('Thrust');
    xlabel('Time (s)');
    ylabel('Force (MN)');
    grid on; grid minor;

    subplot(2,3,6);
    plot(t, theta_t);
    legend('\delta');
    xlabel('Time (s)');
    ylabel('Gimbal Angle (rad)');
    grid on; grid minor;

end
