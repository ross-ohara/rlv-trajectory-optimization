function rlv_make_animation(Uvect, params, y0, filename, fps)
%RLV_MAKE_ANIMATION Generate an MP4 animation of the RLV flip-and-land maneuver.
%
%   rlv_make_animation(Uvect, params, y0)
%   rlv_make_animation(Uvect, params, y0, filename)
%   rlv_make_animation(Uvect, params, y0, filename, fps)

    if nargin < 4 || isempty(filename)
        filename = 'rlv_landing.mp4';
    end
    if nargin < 5 || isempty(fps)
        fps = 30;
    end

    res = params.res;
    L   = params.L;
    tf  = Uvect(end);

    % ---- Simulate trajectory on a fixed time grid ----
    nFrames = max(150, round(fps * tf));
    tspan   = linspace(0, tf, nFrames);

    dyn = @(t,x) rlv_dynamics(t, x, params, Uvect);
    [t, X] = ode45(dyn, tspan, y0);

    % ---- Resample controls ----
    thrustCurve     = Uvect(1:res);
    deflectionAngle = Uvect(res+1:2*res);
    tvect           = linspace(0, tf, res);

    Tc = interp1(tvect, thrustCurve,     t, 'pchip');   % [0,1]
    Dc = interp1(tvect, deflectionAngle, t, 'pchip');   % [0,1]

    % Map to gimbal angle (same mapping as dynamics/plots)
    gimbal = (Dc - 0.5) * 2 * params.gimbal_max;

    % ---- Video writer ----
    vid = VideoWriter(filename, 'MPEG-4');
    vid.FrameRate = fps;
    open(vid);

    figure('Name','RLV Animation','NumberTitle','off');

    for k = 1:length(t)
        clf;

        % Current state
        x_pos  = X(k,1);
        z_pos  = X(k,2);
        theta  = X(k,3);
        delta  = gimbal(k);

        % NOTE: match your original convention:
        % Rot = [cos(theta), sin(theta); -sin(theta), cos(theta)];
        Rot = [cos(theta),  sin(theta);
              -sin(theta),  cos(theta)];

        % In your original, state z is at the bottom, and COM is L/2 above
        rCOM = [x_pos; z_pos + L/2];

        % Body vertices (copied from original, just cleaned)
        p1 = rCOM + Rot*[ 0;   65/2];
        p2 = rCOM + Rot*[ 9/2; 50/2];
        p3 = rCOM + Rot*[ 9/2;-50/2];
        p4 = rCOM + Rot*[-9/2;-50/2];
        p5 = rCOM + Rot*[-9/2; 50/2];

        % Gimbal rotation (same as original RotT)
        RotT = [cos(delta), -sin(delta);
                sin(delta),  cos(delta)];

        % Engine location at bottom center in body frame, then rotated
        t1 = rCOM + Rot*[0; -50/2];

        % Thrust vector length scaled by normalized thrust
        thrust_len = 50 * max(Tc(k), 0);   % arbitrary scaling

        % Point thrust down along body, then apply RotT for gimbal
        t2 = t1 + Rot * (RotT * [0; -thrust_len]);

        % ---- Draw scene ----
        hold on;
        % Landing pad / ground
        yline(0,'k','LineWidth',1.0);
        plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor','k');

        % Body outline
        plot([p1(1),p2(1)],[p1(2),p2(2)],'k','LineWidth',2);
        plot([p2(1),p3(1)],[p2(2),p3(2)],'k','LineWidth',2);
        plot([p3(1),p4(1)],[p3(2),p4(2)],'k','LineWidth',2);
        plot([p4(1),p5(1)],[p4(2),p5(2)],'k','LineWidth',2);
        plot([p5(1),p1(1)],[p5(2),p1(2)],'k','LineWidth',2);

        % COM + thrust vector
        plot(rCOM(1), rCOM(2), 'og', 'MarkerFaceColor','g');
        plot([t1(1),t2(1)], [t1(2),t2(2)], 'r-', 'LineWidth',2);

        % Axes
        pad = 100;
        xlim([rCOM(1)-pad, rCOM(1)+pad]);
        ylim([rCOM(2)-pad, rCOM(2)+pad]);
        grid on; grid minor;
        axis equal;

        xlabel('X (m)');
        ylabel('Z (m)');
        title(sprintf('t = %.2f s', t(k)));

        drawnow;
        frame = getframe(gcf);
        writeVideo(vid, frame);
    end

    % Hold final frame for 1 second
    for i = 1:round(fps * 1.0)
        frame = getframe(gcf);
        writeVideo(vid, frame);
    end

    close(vid);
    fprintf('Animation saved to: %s\n', filename);

end
