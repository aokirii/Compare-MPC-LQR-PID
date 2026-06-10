function [xr, yr, thr, kappa_r] = nearest_ref_point(x, y)
% NEAREST POINT FINDER
% Workspace'teki x_ref_arr, y_ref_arr, theta_ref_arr dizilerinden
% dar pencerede en yakın noktayı bulur.
% Yol sonunda başa sarar.

persistent idx_prev x_ref y_ref theta_ref kappa_ref nRef

if isempty(idx_prev)
    x_ref     = evalin('base', 'x_ref_arr');
    y_ref     = evalin('base', 'y_ref_arr');
    theta_ref = evalin('base', 'theta_ref_arr');
    kappa_ref = evalin('base', 'kappa_arr');
    nRef      = numel(x_ref);
    idx_prev  = 1;
end

win_fwd = 50;

lookahead_steps = evalin('base', 'lookahead_steps');

% Dar pencerede ara
i1 = idx_prev;
i2 = min(nRef, idx_prev + win_fwd);

best_d2 = inf;
best_idx = i1;
for ii = i1:i2
    d2 = (x_ref(ii) - x)^2 + (y_ref(ii) - y)^2;
    if d2 < best_d2
        best_d2 = d2;
        best_idx = ii;
    end
end

% Monoton ilerleme
idx_prev = max(idx_prev, best_idx);

% Lookahead
ref_idx = best_idx + lookahead_steps;

if ref_idx > nRef
    ref_idx = ref_idx - nRef;
end

% Yol sonuna gelince başa sar
if idx_prev >= nRef - 5
    idx_prev = 1;
end

xr      = x_ref(ref_idx);
yr      = y_ref(ref_idx);
thr     = theta_ref(ref_idx);
kappa_r = kappa_ref(ref_idx);
end