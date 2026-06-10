function mpc_to_compare()
% MPC_TO_COMPARE  Yeni MPC (7.Week tasarimi) ciktisini karsilastirma duzenine baglar.
%
% Yeni MPC, animasyon logunu base workspace'e GENEL 'sim_log' adiyla yazar.
% compare_controllers ise kontrolcu-bazli 'sim_log_MPC' (veya results_MPC.mat) bekler.
% Bu kopru ikisini birlestirir -- MPC dosyalarinin ICERIGINE DOKUNMADAN.
%
% KULLANIM (ayni MATLAB oturuminda):
%   1) MPC/MPC_Setup.m calistir, menuden haritayi sec
%   2) TrajectoryTracking_MPC.slx simulasyonunu calistir (bitene kadar)
%   3) >> mpc_to_compare      % sim_log -> sim_log_MPC + results_MPC.mat
%   4) PID ve LQR simulasyonlarini da calistirip export_run ile kaydet
%   5) >> compare_controllers

try
    S = evalin('base', 'sim_log');
catch
    error(['base workspace''de ''sim_log'' yok. ' ...
           'Once MPC simulasyonunu calistir (sim bitince persistent_anim_update onu yazar).']);
end

if ~isstruct(S) || ~isfield(S, 't')
    error('''sim_log'' beklenen formatta degil (struct ve .t alani gerekli).');
end

assignin('base', 'sim_log_MPC', S);
fprintf('Kopru: sim_log -> sim_log_MPC  (%d ornek, alanlar: %s)\n', ...
        numel(S.t), strjoin(fieldnames(S)', ', '));

% Diske de kaydet (oturum kapansa/temizlense bile compare okuyabilsin)
if exist('export_run', 'file')
    export_run('MPC');
else
    warning('export_run bulunamadi; sadece base''e yazildi (results_MPC.mat kaydedilmedi).');
end
end
