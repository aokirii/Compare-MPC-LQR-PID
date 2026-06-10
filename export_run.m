function export_run(ctrl)
% EXPORT_RUN  base'deki sim_log_<ctrl> yapisini results_<ctrl>.mat'e kaydeder.
%
% Bir simulasyon bittikten sonra (ayni oturumda) cagir:
%   export_run('PID')   ->  results_PID.mat
%   export_run('LQR')   ->  results_LQR.mat
%   export_run('MPC')   ->  results_MPC.mat
% Argumansiz cagrilirsa base'de bulunan tum sim_log_* lari kaydeder.
%
% Boylece MPC_Setup gibi base'i temizleyen adimlar veya farkli oturumlar
% loglari silse bile compare_controllers diskten okuyabilir.

if nargin < 1 || isempty(ctrl)
    list = {'PID','LQR','MPC'};
else
    list = {ctrl};
end

thisDir = fileparts(mfilename('fullpath'));
saved = 0;

for i = 1:numel(list)
    vname = ['sim_log_' list{i}];
    try
        S = evalin('base', vname);
    catch
        continue;
    end
    if isstruct(S) && isfield(S,'t')
        out = fullfile(thisDir, ['results_' list{i} '.mat']);
        save(out, '-struct', 'S');
        fprintf('Kaydedildi: %s  (%d ornek)\n', out, numel(S.t));
        saved = saved + 1;
    end
end

if saved == 0
    warning('Kaydedilecek sim_log_* bulunamadi.');
end
end
