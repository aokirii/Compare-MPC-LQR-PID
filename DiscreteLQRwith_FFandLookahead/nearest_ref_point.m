function [xr, yr, thr, kappa_r] = nearest_ref_point(x, y)
persistent x_ref y_ref theta_ref nRef kappa_ref idx_prev s_ref initialized

if isempty(idx_prev)
    x_ref     = evalin('base', 'x_ref_arr');
    y_ref     = evalin('base', 'y_ref_arr');
    theta_ref = evalin('base', 'theta_ref_arr');
    kappa_ref = evalin('base', 'kappa_arr');
    s_ref     = evalin('base', 's_dense');
    nRef      = numel(x_ref);
    idx_prev  = 1;
    initialized = false;
end

% en yakın nokta
if ~initialized
    best_d2 = inf;
    for ii = 1:nRef
        d2 = (x_ref(ii)-x)^2 + (y_ref(ii)-y)^2;
        if d2 < best_d2
            best_d2 = d2; idx_prev = ii;
        end
    end
    initialized = true;
end

% monoton pencere
win_fwd = 120;
i1 = idx_prev;
i2 = min(nRef, idx_prev + win_fwd);
best_d2 = inf; best_idx = i1;
for ii = i1:i2
    d2 = (x_ref(ii)-x)^2 + (y_ref(ii)-y)^2;
    if d2 < best_d2
        best_d2 = d2; best_idx = ii;
    end
end
idx_prev = max(idx_prev, best_idx);
if idx_prev >= nRef - 5; idx_prev = 1; end

% Position lookahead (3m)
lookahead_m = 3.0;
look_idx = best_idx;
for ii = best_idx:nRef
    d = s_ref(ii) - s_ref(best_idx);
    if d >= lookahead_m
        look_idx = ii;
        break;
    end
end

xr      = x_ref(look_idx);
yr      = y_ref(look_idx);
thr     = theta_ref(look_idx);
kappa_r = kappa_ref(look_idx);
end