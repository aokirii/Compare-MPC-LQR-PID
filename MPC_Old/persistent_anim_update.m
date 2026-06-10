function persistent_anim_update(x, y, th, delta_cmd, ey_in, eh_in)
% CANLI ANIMASYON + VERI KAYDEDICI (MPC) - tek kaynak.
% Yorungeyi cizer VE t,x,y,theta,delta,ey,eh,s sinyallerini base
% workspace'e 'sim_log_MPC' adiyla yazar.
%
% delta, ey, eh : KONTROLCUNUN KENDI sinyalleri (4/5/6. giris) - burada
%   YENIDEN HESAPLANMAZ, oldugu gibi loglanir.
% s : yol uzerinde ilerleme (yay uzunlugu, turlar dahil) (x,y)'den hesaplanir.

persistent initialized fig_h ax_h hRef hStart hGoal hTrail hSprite ...
           carImg carAlpha hasAlpha halfW halfH theta_offset ...
           trail_x trail_y idx_log step_count anim_every Ts_local ...
           xref yref thref nRef idxPrev ...
           s_arr sEnd lap prev_bi vlog ...
           t_log x_log y_log th_log delta_log ey_log eh_log s_log Nmax

    if nargin < 4, delta_cmd = NaN; end
    if nargin < 5, ey_in     = NaN; end
    if nargin < 6, eh_in     = NaN; end

    %% --- Ilk cagri: figure + sprite + log tamponlari ---
    if isempty(initialized)
        xref   = evalin('base','x_ref');
        yref   = evalin('base','y_ref');
        thref  = evalin('base','theta_ref');
        nRef   = numel(xref);
        idxPrev = 1;

        % --- Yol uzerinde ilerleme (s) icin yay uzunlugu ---
        s_arr  = [0; cumsum(hypot(diff(xref(:)), diff(yref(:))))];
        sEnd   = s_arr(end);
        lap    = 0;
        prev_bi = 1;
        try, vlog = evalin('base','v_const'); catch, vlog = 8; end

        startPt  = evalin('base','sim_params.startPt');
        goal     = evalin('base','sim_params.goal');
        Ts_local = evalin('base','Ts');
        try, T_end = evalin('base','T_end'); catch, T_end = 250; end

        step_count   = 0;
        anim_every   = 4;
        theta_offset = pi/2;

        fig_h = figure('Color','w','Name','MPC Trajectory Tracking', ...
                       'Position',[100 100 900 700]);
        ax_h  = axes('Parent', fig_h);
        hold(ax_h,'on'); grid(ax_h,'on'); axis(ax_h,'equal');

        hRef   = plot(ax_h, xref, yref, 'b--', 'LineWidth',1.5);
        hStart = plot(ax_h, startPt(1), startPt(2), 'go','MarkerSize',10, ...
                      'LineWidth',2, 'MarkerFaceColor','g');
        hGoal  = plot(ax_h, goal(1), goal(2), 'rx','MarkerSize',12,'LineWidth',2);
        hTrail = plot(ax_h, nan, nan, 'm-','LineWidth',1.8);

        title(ax_h, 'MPC Live Tracking');
        xlabel(ax_h, 'x [m]'); ylabel(ax_h, 'y [m]');
        legend(ax_h, [hRef hStart hGoal hTrail], ...
               {'Referans','Start','Goal','Arac'}, 'Location','best');

        pad = 30;
        xlim(ax_h, [min(xref)-pad, max(xref)+pad]);
        ylim(ax_h, [min(yref)-pad, max(yref)+pad]);

        iconPath = '/Users/kasimesen/Desktop/Tasarım/car.png';
        hasAlpha = false;
        try
            [carImg, ~, carAlpha] = imread(iconPath);
            carImg = im2double(carImg);
            hasAlpha = ~isempty(carAlpha);
            if hasAlpha, carAlpha = im2double(carAlpha); end

            mapScale = min(max(xref)-min(xref), max(yref)-min(yref));
            carSize  = 0.04 * mapScale;
            halfW = carSize/2; halfH = carSize/2;

            carRot = imrotate(carImg, -rad2deg(th + theta_offset), 'bilinear','crop');
            hSprite = image(ax_h, [x-halfW, x+halfW], [y-halfH, y+halfH], carRot);
            if hasAlpha
                set(hSprite, 'AlphaData', imrotate(carAlpha, ...
                    -rad2deg(th + theta_offset),'bilinear','crop'));
            end
            uistack(hSprite,'top');
        catch
            hSprite = plot(ax_h, x, y, 'ko','MarkerSize',12, ...
                           'MarkerFaceColor','y','LineWidth',2);
            halfW = 0; halfH = 0;
        end

        Nmax = round(T_end / Ts_local) + 2000;   % bol marj (gercek adim sayisi tahmini asabilir)
        t_log     = nan(Nmax,1); x_log = nan(Nmax,1); y_log = nan(Nmax,1);
        th_log    = nan(Nmax,1); delta_log = nan(Nmax,1);
        ey_log    = nan(Nmax,1); eh_log = nan(Nmax,1); s_log = nan(Nmax,1);
        trail_x   = nan(Nmax,1); trail_y = nan(Nmax,1);
        idx_log   = 0;
        initialized = true;
    end

    %% --- En yakin referans nokta (sadece s ilerlemesi icin) ---
    win = 120;
    i2  = min(nRef, idxPrev + win);
    bestd = inf; bi = idxPrev;
    for ii = idxPrev:i2
        d2 = (xref(ii)-x)^2 + (yref(ii)-y)^2;
        if d2 < bestd, bestd = d2; bi = ii; end
    end
    idxPrev = max(idxPrev, bi);
    if idxPrev >= nRef - 5, idxPrev = 1; end

    % delta/ey/eh = kontrolcunun gercek sinyalleri (passed-in)
    ey = ey_in;
    eh = eh_in;

    %% --- Yol uzerinde ilerleme (turlar dahil, monoton) ---
    if bi < prev_bi - 0.5*nRef     % index basa sardi -> yeni tur
        lap = lap + 1;
    end
    prev_bi = bi;
    s_prog = lap*sEnd + s_arr(bi);

    %% --- Sayac + log ---
    step_count = step_count + 1;
    t_now      = (step_count - 1) * Ts_local;
    idx_log    = idx_log + 1;
    if idx_log <= Nmax
        t_log(idx_log)     = t_now;
        x_log(idx_log)     = x;
        y_log(idx_log)     = y;
        th_log(idx_log)    = th;
        delta_log(idx_log) = delta_cmd;
        ey_log(idx_log)    = ey;
        eh_log(idx_log)    = eh;
        s_log(idx_log)     = s_prog;
        trail_x(idx_log)   = x;
        trail_y(idx_log)   = y;
    end

    %% --- base'e periyodik yaz ---
    if mod(step_count, 25) == 0
        K = min(idx_log, Nmax);
        S.t = t_log(1:K); S.x = x_log(1:K); S.y = y_log(1:K); S.theta = th_log(1:K);
        S.ey = ey_log(1:K); S.eh = eh_log(1:K); S.delta = delta_log(1:K); S.s = s_log(1:K);
        S.meta = struct('ctrl','MPC','Ts',Ts_local,'nRef',nRef,'sEnd',sEnd,'v',vlog);
        assignin('base', 'sim_log_MPC', S);
    end

    if ~ishandle(fig_h)
        return;
    end

    %% --- Animasyon (throttled) ---
    if mod(step_count, anim_every) == 0
        if ishandle(hTrail)
            set(hTrail,'XData', trail_x(1:idx_log), 'YData', trail_y(1:idx_log));
        end
        if ishandle(hSprite)
            try
                if hasAlpha
                    set(hSprite, 'XData', [x-halfW x+halfW], ...
                                 'YData', [y-halfH y+halfH], ...
                                 'CData', imrotate(carImg, ...
                                          -rad2deg(th + theta_offset), ...
                                          'bilinear','crop'));
                    set(hSprite, 'AlphaData', imrotate(carAlpha, ...
                                          -rad2deg(th + theta_offset), ...
                                          'bilinear','crop'));
                else
                    set(hSprite, 'XData', x, 'YData', y);
                end
            catch
            end
        end
        drawnow limitrate;
    end
end
