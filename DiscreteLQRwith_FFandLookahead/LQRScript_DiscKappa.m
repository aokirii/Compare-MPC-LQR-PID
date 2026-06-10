clc;
clear nearest_ref_point persistent_anim_update
baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);

%% 1. HARİTA SEÇİMİ
map_type = 2;   % 1=Çember, 2=Dikdörtgen, 3=Hand Figure

switch map_type
    case 1
        run(fullfile(baseDir, 'YuvarlakMap.m'));
    case 2
        run(fullfile(baseDir, 'KareMap.m'));
    case 3
        run(fullfile(baseDir, 'HandFigureMap.m'));
    otherwise
        error('Geçersiz harita seçimi.');
end

if ~exist('waypoints','var') || size(waypoints,2)~=2 || size(waypoints,1)<2
    error('waypoints bulunamadı veya format yanlış. Nx2 olmalı.');
end

%% 2. PARAMETRE

v_ref = 8;   % ortak hiz (PID/MPC ile ayni)

%% 3. REFERANS YOL
wp = waypoints;
wp = wp([true; sqrt(sum(diff(wp).^2,2)) > 1e-9], :);
s  = [0; cumsum(sqrt(sum(diff(wp).^2,2)))];
[sU, ia] = unique(s, 'stable'); wpU = wp(ia,:);

sEnd = sU(end);
T_1lap = sEnd / v_ref;
N_loops = 3;   % ortak tur sayisi (v=8, KareMap -> ~3 tur ~ 120 s)
T_sim = T_1lap * N_loops;

% Dense referans dizileri
s_dense   = linspace(0, sEnd, 1500)';
x_ref_arr = interp1(sU, wpU(:,1), s_dense, 'pchip');   % PID/MPC ile ayni interp
y_ref_arr = interp1(sU, wpU(:,2), s_dense, 'pchip');

dx = gradient(x_ref_arr);
dy = gradient(y_ref_arr);
ddx = gradient(dx);
ddy = gradient(dy);
theta_ref_arr = atan2(dy, dx);

%% 4. START / GOAL
if exist('startPt','var') && numel(startPt)==2
    sp = startPt(:).';
else
    sp = [wp(1,1), wp(1,2)];
end

%% 5. BAŞLANGIÇ KOŞULLARI
x_init = sp(1);
y_init = sp(2);

% En yakın referans noktasının heading'i
dists = (x_ref_arr - x_init).^2 + (y_ref_arr - y_init).^2;
[~, closest_idx] = min(dists);
theta_init = theta_ref_arr(closest_idx);

%% 5b. DISCRETE LQR
L         = 2.0;
Ts        = 0.02;

A_c = [0, v_ref; 0, 0];
B_c = [0; v_ref / L];

% Zero-Order Hold ile ayrıklaştırma
sys_c = ss(A_c, B_c, eye(2), zeros(2,1));
sys_d = c2d(sys_c, Ts, 'zoh');
A_d   = sys_d.A;
B_d   = sys_d.B;

% Discrete LQR
Q_lqr = [70, 0; 0, 50];
R_lqr = 15;
K_lqr = dlqr(A_d, B_d, Q_lqr, R_lqr);

fprintf('Discrete LQR K : [%.4f, %.4f]\n', K_lqr(1), K_lqr(2));

%% 5c. CURVATURE (EĞRİLİK) HESAPLAMA

% Curvature hesaplama
kappa = (dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);


%% 6. WORKSPACE'E YÜKLEME
assignin('base', 'v_ref', v_ref);
assignin('base', 'Ts_anim', Ts);   % animasyon log adimi
assignin('base', 'T_sim', T_sim);
assignin('base', 'x_init', x_init);
assignin('base', 'y_init', y_init);
assignin('base', 'theta_init', theta_init);
assignin('base', 'x_ref_arr', x_ref_arr);
assignin('base', 'y_ref_arr', y_ref_arr);
assignin('base', 'theta_ref_arr', theta_ref_arr);
assignin('base', 'K_lqr', K_lqr);
assignin('base', 'kappa_arr', kappa);
assignin('base', 's_dense',   s_dense);

%% 7. BİLGİ ÇIKTISI
fprintf('\n========================================\n');
fprintf('Script\n');
fprintf('========================================\n');
fprintf('Harita tipi      : %d\n', map_type);
fprintf('Referans nokta   : %d\n', numel(x_ref_arr));
fprintf('Toplam yol       : %.1f m\n', sEnd);
fprintf('Tek tur süresi   : %.1f s\n', T_1lap);
fprintf('Tur sayısı       : %d\n', N_loops);
fprintf('Toplam süre      : %.1f s\n', T_sim);
fprintf('Başlangıç        : [%.2f, %.2f]\n', x_init, y_init);
fprintf('theta_init       : %.1f deg\n', rad2deg(theta_init));
fprintf('========================================\n');

%% 8. REFERANS YOL GÖRSELLEŞTİRME
figure('Color','w'); hold on; grid on; axis equal;
plot(x_ref_arr, y_ref_arr, 'b--', 'LineWidth', 2);
plot(x_init, y_init, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
title('Referans Yol'); xlabel('x'); ylabel('y');
legend('Referans', 'Start', 'Location', 'best');