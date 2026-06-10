function persistent_anim_update(x, y, theta, delta, ey, eh)
% CANLI ANIMASYON + VERI KAYDEDICI (PID) - tek kaynak.
% Yorungeyi cizer VE t,x,y,theta,delta,ey,eh,s sinyallerini base'e
% 'sim_log_PID' adiyla yazar.
%
% delta, ey, eh : KONTROLCUNUN KENDI sinyalleri (Scope'a giden ayni
%   sinyaller) animasyon blokuna girdi olarak baglanir; burada YENIDEN
%   HESAPLANMAZ, oldugu gibi loglanir.
% s : yol uzerinde ilerleme (yay uzunlugu, turlar dahil) (x,y)'den hesaplanir.
%
% ÖNEMLI: animasyon blokunun giris portlari bu imza ile ESLESMELI:
%   1:x  2:y  3:theta  4:delta  5:ey  6:eh
% (Eski 3 portlu blokta delta/ey/eh NaN loglanir; portlari ekle.)

if nargin < 4, delta = NaN; end
if nargin < 5, ey    = NaN; end
if nargin < 6, eh    = NaN; end

persistent hFig hTrail hSprite trailX trailY idx
persistent carImg carAlpha hasAlpha halfW halfH theta_offset
persistent initialized updateCounter
persistent xref yref nRef idxPrev
persistent s_arr sEnd lap prev_bi vlog
persistent Ts k t_log x_log y_log th_log ey_log eh_log d_log s_log Nmax

if isempty(initialized)
    initialized   = true;
    updateCounter = 0;
    idx           = 0;
    trailX        = [];
    trailY        = [];
    theta_offset  = -pi/2;

    % --- Referans diziler (cizim + s hesabi icin) ---
    xref  = evalin('base', 'x_ref_arr');
    yref  = evalin('base', 'y_ref_arr');
    nRef  = numel(xref);
    idxPrev = 1;

    % --- Yol uzerinde ilerleme (s) icin yay uzunlugu ---
    s_arr  = [0; cumsum(hypot(diff(xref(:)), diff(yref(:))))];
    sEnd   = s_arr(end);
    lap    = 0;
    prev_bi = 1;
    try, vlog = evalin('base','v_ref'); catch, vlog = 8; end

    % --- Log tamponlari ---
    try, Ts    = evalin('base','Ts_anim'); catch, Ts    = 0.02; end
    try, T_sim = evalin('base','T_sim');   catch, T_sim = 250;  end
    Nmax  = round(T_sim/Ts) + 2000;
    k     = 0;
    t_log = nan(Nmax,1); x_log = nan(Nmax,1); y_log = nan(Nmax,1);
    th_log= nan(Nmax,1); ey_log= nan(Nmax,1); eh_log= nan(Nmax,1);
    d_log = nan(Nmax,1); s_log = nan(Nmax,1);

    % --- Sprite yukle ---
    iconPath = '/Users/kasimesen/Desktop/Tasarım/car.png';
    try
        [carImg, ~, carAlpha] = imread(iconPath);
        carImg = im2double(carImg);
        hasAlpha = ~isempty(carAlpha);
        if hasAlpha, carAlpha = im2double(carAlpha); end
    catch
        carImg = ones(20,20,3);
        hasAlpha = false;
        carAlpha = [];
    end
    halfW = 3; halfH = 3;

    % --- Figur ---
    hFig = figure('Name','PID Live Tracking','Color','w','NumberTitle','off');
    hold on; grid on; axis equal;
    title('PID Live Tracking'); xlabel('x'); ylabel('y');
    plot(xref, yref, 'b--', 'LineWidth', 2);
    pad = 15;
    xlim([min(xref)-pad max(xref)+pad]);
    ylim([min(yref)-pad max(yref)+pad]);

    hTrail = plot(nan, nan, 'm-', 'LineWidth', 1.5);
    rotImg = imrotate(carImg, -rad2deg(theta + theta_offset), 'bilinear', 'crop');
    hSprite = image([x-halfW x+halfW], [y-halfH y+halfH], rotImg);
    if hasAlpha
        rotAlpha = imrotate(carAlpha, -rad2deg(theta + theta_offset), 'bilinear', 'crop');
        set(hSprite, 'AlphaData', rotAlpha);
    end
    uistack(hSprite, 'top');
end

% --- En yakin referans nokta (sadece s ilerlemesi icin) ---
win = 120;
i2  = min(nRef, idxPrev + win);
bestd = inf; bi = idxPrev;
for ii = idxPrev:i2
    d2 = (xref(ii)-x)^2 + (yref(ii)-y)^2;
    if d2 < bestd, bestd = d2; bi = ii; end
end
idxPrev = max(idxPrev, bi);
if idxPrev >= nRef - 5, idxPrev = 1; end

% --- Yol uzerinde ilerleme (turlar dahil, monoton) ---
if bi < prev_bi - 0.5*nRef     % index basa sardi -> yeni tur
    lap = lap + 1;
end
prev_bi = bi;
s_prog = lap*sEnd + s_arr(bi);

% --- Logla (delta/ey/eh = kontrolcunun gercek sinyalleri) ---
k = k + 1;
if k <= Nmax
    t_log(k) = (k-1)*Ts; x_log(k) = x; y_log(k) = y; th_log(k) = theta;
    d_log(k) = delta; ey_log(k) = ey; eh_log(k) = eh;
    s_log(k) = s_prog;
end

% --- base'e periyodik yaz ---
if mod(k, 25) == 0
    kk = min(k, Nmax);
    S.t = t_log(1:kk); S.x = x_log(1:kk); S.y = y_log(1:kk); S.theta = th_log(1:kk);
    S.ey = ey_log(1:kk); S.eh = eh_log(1:kk); S.delta = d_log(1:kk); S.s = s_log(1:kk);
    S.meta = struct('ctrl','PID','Ts',Ts,'nRef',nRef,'sEnd',sEnd,'v',vlog);
    assignin('base', 'sim_log_PID', S);
end

if ~isvalid(hFig)
    return;
end

% --- Animasyon (throttled) ---
idx = idx + 1;
trailX(idx) = x;
trailY(idx) = y;
updateCounter = updateCounter + 1;

if mod(updateCounter, 10) == 0
    set(hTrail, 'XData', trailX(1:idx), 'YData', trailY(1:idx));
    set(hSprite, 'XData', [x-halfW x+halfW], 'YData', [y-halfH y+halfH]);
    if mod(updateCounter, 50) == 0
        rotImg = imrotate(carImg, -rad2deg(theta + theta_offset), 'bilinear', 'crop');
        set(hSprite, 'CData', rotImg);
        if hasAlpha
            rotAlpha = imrotate(carAlpha, -rad2deg(theta + theta_offset), 'bilinear', 'crop');
            set(hSprite, 'AlphaData', rotAlpha);
        end
    end
    drawnow limitrate;
end
end
