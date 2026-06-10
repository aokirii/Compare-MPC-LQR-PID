function compare_controllers()
% COMPARE_CONTROLLERS  Uc kontrolcunun (PID, LQR, MPC) sonucunu birlestirir.
%
% VERI KAYNAKLARI (scope + animasyon birlesimi):
%   * Trajectory (x,y) ve ilerleme s(t)  -> animasyon logu  sim_log_<ctrl>
%   * ey, eh, delta                       -> Scope logu      scope_<ctrl>
%     (Scope > Log data to workspace; Variable name = scope_PID/LQR/MPC;
%      Format = Structure With Time; sinyal sirasi: 1) ey  2) eh  3) delta)
%   * scope_<ctrl> yoksa, sim_log_<ctrl> icindeki ey/eh/delta kullanilir.
%
% Cikti:  1) Yorunge overlay   2) Ilerleme s(t)
%         3) ey/eh/delta (kontrolcu basina)   4) Metrik tablosu

names  = {'PID','LQR','MPC'};
colors = {[0.85 0.10 0.10], [0.10 0.45 0.85], [0.10 0.65 0.20]};

logs = struct();
have = false(1,3);

%% --- 1) Animasyon logu: trajectory + s ---
for i = 1:numel(names)
    S = [];
    try, S = evalin('base', ['sim_log_' names{i}]); catch, S = []; end
    if isempty(S) && exist(['results_' names{i} '.mat'],'file')
        S = load(['results_' names{i} '.mat']);
    end
    if ~isempty(S) && isfield(S,'t')
        logs.(names{i}) = S;
        have(i) = true;
    else
        warning('%s: sim_log_%s bulunamadi (trajectory/s gosterilemez).', ...
                names{i}, names{i});
    end
end
if ~any(have)
    error('Hicbir kontrolcu logu yok. Once simulasyonlari calistir.');
end

%% --- 2) Scope logu: ey/eh/delta (varsa sim_log uzerine yazar) ---
for i = 1:numel(names)
    if ~have(i), continue; end
    raw = [];
    try, raw = evalin('base', ['scope_' names{i}]); catch, raw = []; end
    if ~isempty(raw)
        [ts, V] = parse_logged(raw);
        if ~isempty(ts) && ~isempty(V)
            logs.(names{i}).ts    = ts(:);
            logs.(names{i}).ey    = V(:,1);
            if size(V,2) >= 2, logs.(names{i}).eh    = V(:,2); else, logs.(names{i}).eh    = nan(numel(ts),1); end
            if size(V,2) >= 3, logs.(names{i}).delta = V(:,3); else, logs.(names{i}).delta = nan(numel(ts),1); end
            fprintf('%s: scope_%s okundu (%d ornek, %d sinyal).\n', ...
                    names{i}, names{i}, numel(ts), size(V,2));
        end
    end
    % ts yoksa sinyal zamani = trajectory zamani
    if ~isfield(logs.(names{i}),'ts') || isempty(logs.(names{i}).ts)
        logs.(names{i}).ts = logs.(names{i}).t;
    end
end

%% --- Ortak zaman penceresi ---
tmax = inf;
for i = 1:numel(names)
    if have(i)
        tt = logs.(names{i}).t;
        te = tt(find(isfinite(tt),1,'last'));
        if ~isempty(te), tmax = min(tmax, te); end
    end
end
if ~isfinite(tmax) || isempty(tmax), tmax = 0; end

% --- Referans yol (varsa) ---
xref = []; yref = [];
try, xref = evalin('base','x_ref_arr'); yref = evalin('base','y_ref_arr');
catch
    try, xref = evalin('base','x_ref'); yref = evalin('base','y_ref'); catch, end
end

%% 1) XY yorunge overlay
figure('Color','w','Name','Trajectory overlay'); hold on; grid on; axis equal;
if ~isempty(xref)
    plot(xref, yref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Referans');
end
sty = {':', '-.', '-'};   % PID = noktali, LQR = nokta-tire, MPC = duz
lw  = [1.2, 2.2, 3.2];    % PID ince(on) -> LQR orta -> MPC kalin(arka): ucu de gorunur
for i = [3 2 1]           % MPC (arka) -> LQR -> PID (en onde, noktali)
    if have(i)
        S = logs.(names{i});
        plot(S.x, S.y, sty{i}, 'Color', colors{i}, 'LineWidth', lw(i), ...
             'DisplayName', names{i});
    end
end
legend('Location','best');
title('Yorunge Karsilastirmasi  (KareMap, v=8 m/s, 3 tur)');
xlabel('x [m]'); ylabel('y [m]');


%% 2) Sinyaller - her kontrolcu kendi satirinda (kendi y-olceginde)
rows = nnz(have);
figure('Color','w','Name','Signals per controller', ...
       'Position',[80 80 1100 max(230*rows,300)]);
r = 0;
for i = 1:numel(names)
    if ~have(i), continue; end
    r = r + 1;
    S  = logs.(names{i});
    c  = colors{i};
    ts = S.ts;
    ey = getf(S,'ey'); eh = getf(S,'eh'); dl = getf(S,'delta');

    % e_y
    subplot(rows,3,(r-1)*3+1);
    if anyfinite(ey), plot(ts, ey, 'Color', c, 'LineWidth', 1.2); grid on;
        if tmax>0, xlim([0 tmax]); end
    else, axis off; text(0.5,0.5,'e_y yok','Units','normalized','HorizontalAlignment','center'); end
    ylabel([names{i} '   e_y [m]']);
    if r==1,    title('Yanal hata e_y'); end
    if r==rows, xlabel('t [s]');         end

    % e_h
    subplot(rows,3,(r-1)*3+2);
    if anyfinite(eh), plot(ts, rad2deg(eh), 'Color', c, 'LineWidth', 1.2); grid on;
        if tmax>0, xlim([0 tmax]); end
    else, axis off; text(0.5,0.5,'e_h yok','Units','normalized','HorizontalAlignment','center'); end
    ylabel('e_h [deg]');
    if r==1,    title('Heading hata e_h'); end
    if r==rows, xlabel('t [s]');           end

    % delta
    subplot(rows,3,(r-1)*3+3);
    if anyfinite(dl), plot(ts, rad2deg(dl), 'Color', c, 'LineWidth', 1.2); grid on;
        if tmax>0, xlim([0 tmax]); end
    else, axis off; text(0.5,0.5,'\delta yok (scope log yok)','Units','normalized','HorizontalAlignment','center'); end
    ylabel('\delta [deg]');
    if r==1,    title('Direksiyon \delta'); end
    if r==rows && anyfinite(dl), xlabel('t [s]'); end
end

%% 2b) Sinyaller - UC KONTROLCU
% Ayni eksende, kendi renk + stilleriyle (PID noktali, LQR nokta-tire, MPC duz)
figure('Color','w','Name','Signals overlay (ic ice)','Position',[120 120 1000 760]);

subplot(3,1,1); hold on; grid on; ylabel('e_y [m]'); title('Yanal hata e_y');
for i = 1:numel(names)
    if have(i)
        ey = getf(logs.(names{i}),'ey');
        if anyfinite(ey)
            plot(logs.(names{i}).ts, ey, sty{i}, 'Color', colors{i}, ...
                 'LineWidth', 1.4, 'DisplayName', names{i});
        end
    end
end
if tmax>0, xlim([0 tmax]); end
legend('Location','best');

subplot(3,1,2); hold on; grid on; ylabel('e_h [deg]'); title('Heading hata e_h');
for i = 1:numel(names)
    if have(i)
        eh = getf(logs.(names{i}),'eh');
        if anyfinite(eh)
            plot(logs.(names{i}).ts, rad2deg(eh), sty{i}, 'Color', colors{i}, ...
                 'LineWidth', 1.4, 'DisplayName', names{i});
        end
    end
end
if tmax>0, xlim([0 tmax]); end
legend('Location','best');

subplot(3,1,3); hold on; grid on; ylabel('\delta [deg]'); xlabel('t [s]');
title('Direksiyon \delta');
for i = 1:numel(names)
    if have(i)
        dl = getf(logs.(names{i}),'delta');
        if anyfinite(dl)
            plot(logs.(names{i}).ts, rad2deg(dl), sty{i}, 'Color', colors{i}, ...
                 'LineWidth', 1.4, 'DisplayName', names{i});
        end
    end
end
if tmax>0, xlim([0 tmax]); end
legend('Location','best');

%% 3) Metrik tablosu (ortak pencerede)
fprintf('\n=================== KARSILASTIRMA METRIKLERI ===================\n');
fprintf('%-5s %10s %10s %10s %10s %12s %8s\n', ...
        'Ctrl','RMS|ey|','max|ey|','RMS eh','max eh','effort','t_son');
fprintf('%-5s %10s %10s %10s %10s %12s %8s\n', ...
        '',     '[m]',   '[m]',   '[deg]', '[deg]', '[rad^2 s]','[s]');
fprintf('---------------------------------------------------------------\n');
for i = 1:numel(names)
    if ~have(i), continue; end
    S  = logs.(names{i});
    ts = S.ts;
    ey = getf(S,'ey'); eh = getf(S,'eh'); dl = getf(S,'delta');
    inwin = isfinite(ts) & (ts <= tmax + 1e-9);

    ok = inwin & isfinite(ey);
    if any(ok), rms_ey = sqrt(mean(ey(ok).^2)); max_ey = max(abs(ey(ok)));
    else, rms_ey = NaN; max_ey = NaN; end
    okh = inwin & isfinite(eh);
    if any(okh), rms_eh = rad2deg(sqrt(mean(eh(okh).^2))); max_eh = rad2deg(max(abs(eh(okh))));
    else, rms_eh = NaN; max_eh = NaN; end
    okd = inwin & isfinite(dl);
    if any(okd)
        dd = dl(okd); Tss = median(diff(ts));
        if isempty(Tss)||~isfinite(Tss)||Tss<=0, Tss = 0.02; end
        effort = sum(dd.^2)*Tss;
    else, effort = NaN; end
    tt = S.t; t_end = tt(find(isfinite(tt),1,'last'));

    fprintf('%-5s %10.4f %10.4f %10.3f %10.3f %12.4g %8.1f\n', ...
            names{i}, rms_ey, max_ey, rms_eh, max_eh, effort, t_end);
end
fprintf('===============================================================\n');
fprintf('Metrikler ortak %.1f s penceresine kirpildi (t_son = gercek bitis).\n', tmax);
fprintf('ey/eh/delta kaynagi: scope_<ctrl> (yoksa sim_log_<ctrl>).\n');
fprintf('NaN => o sinyal icin scope log yok.\n');
end

% =======================================================================
function tf = anyfinite(v)
    tf = ~isempty(v) && any(isfinite(v));
end

function v = getf(S, f)
    if isfield(S, f), v = S.(f); else, v = []; end
end

function [t, V] = parse_logged(raw)
% Scope/To-Workspace logunu [t, V] (V = Nx(sinyal)) olarak cozer.
% Destek: Structure With Time, Array (matris), timeseries-benzeri.
    t = []; V = [];
    if isnumeric(raw)
        if size(raw,2) >= 2, t = raw(:,1); V = raw(:,2:end); end
    elseif isstruct(raw)
        if isfield(raw,'time') && isfield(raw,'signals')
            t = raw.time(:);
            sig = raw.signals;
            if numel(sig) == 1 && isfield(sig,'values')
                V = sig.values;
            else
                V = [];
                for kk = 1:numel(sig)
                    vv = sig(kk).values; vv = vv(:);
                    V = [V, vv]; %#ok<AGROW>
                end
            end
        elseif isfield(raw,'Time') && isfield(raw,'Data')
            t = raw.Time(:); V = raw.Data;
        end
    end
    if ~isempty(V) && size(V,1) ~= numel(t) && size(V,2) == numel(t)
        V = V.';
    end
end
