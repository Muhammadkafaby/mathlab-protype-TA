function build_boost_simulink()
%% Pembangun Model Simulink Boost Converter 3V -> 24V (otomatis)
% ----------------------------------------------------------------------
% Skrip ini membangun model Simulink memakai blok dasar (tanpa Simscape),
% sehingga bisa dijalankan di MATLAB dengan Simulink standar saja.
%
% Cara pakai:
%   >> build_boost_simulink
%   - Model 'boost_3v_24v' otomatis terbuka
%   - Tekan Run di Simulink, atau:
%   >> sim('boost_3v_24v')
%
% Topologi disimulasikan dengan persamaan diferensial averaged-switch
% (model state-space CCM) di dalam blok MATLAB Function. Pendekatan ini
% jauh lebih cepat daripada switching model dan cukup akurat untuk
% memverifikasi loop kontrol.
% ----------------------------------------------------------------------

modelName = 'boost_3v_24v';

% Tutup model lama jika ada
if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end

new_system(modelName);
open_system(modelName);

%% Blok-blok utama
% Step input untuk Vref
add_block('simulink/Sources/Step', [modelName '/Vref'], ...
    'Time','1e-3','Before','0','After','24','Position',[30 60 70 90]);

% Konstanta Vin
add_block('simulink/Sources/Constant',[modelName '/Vin'], ...
    'Value','3','Position',[30 150 70 180]);

% Konstanta Rload
add_block('simulink/Sources/Constant',[modelName '/Rload'], ...
    'Value','120','Position',[30 220 70 250]);

% Sum: error = Vref - Vout
add_block('simulink/Math Operations/Sum',[modelName '/Err'], ...
    'Inputs','+-','Position',[140 60 170 90]);

% PI controller (subsystem Discrete-Time Integrator + gain)
add_block('simulink/Continuous/PID Controller',[modelName '/PI'], ...
    'Controller','PI','P','0.0008','I','6.0', ...
    'Position',[210 50 280 100]);

% Saturation duty 0.05..0.93
add_block('simulink/Discontinuities/Saturation',[modelName '/SatD'], ...
    'UpperLimit','0.93','LowerLimit','0.05', ...
    'Position',[310 55 350 95]);

% Mux: [d ; Vin ; Rload]
add_block('simulink/Signal Routing/Mux',[modelName '/MuxIn'], ...
    'Inputs','3','Position',[400 60 420 220]);

% MATLAB Function block: averaged boost plant
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/BoostPlant'],'Position',[460 80 600 200]);

% Demux output: [Vout ; iL]
add_block('simulink/Signal Routing/Demux',[modelName '/DemuxOut'], ...
    'Outputs','2','Position',[640 100 660 180]);

% Scope Vout
add_block('simulink/Sinks/Scope',[modelName '/Scope_Vout'], ...
    'Position',[720 80 770 130]);

% Scope iL
add_block('simulink/Sinks/Scope',[modelName '/Scope_IL'], ...
    'Position',[720 160 770 210]);

% To Workspace
add_block('simulink/Sinks/To Workspace',[modelName '/Vout_log'], ...
    'VariableName','vout_log','SaveFormat','Timeseries', ...
    'Position',[720 50 770 75]);

%% Sambungan
add_line(modelName,'Vref/1','Err/1','autorouting','on');
add_line(modelName,'DemuxOut/1','Err/2','autorouting','on');
add_line(modelName,'Err/1','PI/1','autorouting','on');
add_line(modelName,'PI/1','SatD/1','autorouting','on');
add_line(modelName,'SatD/1','MuxIn/1','autorouting','on');
add_line(modelName,'Vin/1','MuxIn/2','autorouting','on');
add_line(modelName,'Rload/1','MuxIn/3','autorouting','on');
add_line(modelName,'MuxIn/1','BoostPlant/1','autorouting','on');
add_line(modelName,'BoostPlant/1','DemuxOut/1','autorouting','on');
add_line(modelName,'DemuxOut/1','Scope_Vout/1','autorouting','on');
add_line(modelName,'DemuxOut/1','Vout_log/1','autorouting','on');
add_line(modelName,'DemuxOut/2','Scope_IL/1','autorouting','on');

%% Isi MATLAB Function (model averaged CCM)
fcnCode = sprintf([ ...
'function y = boost_avg(u)\n' ...
'%% u = [d ; Vin ; Rload]\n' ...
'%% State: x = [iL ; vC]\n' ...
'persistent x dt L C RL RC Vf\n' ...
'if isempty(x)\n' ...
'    x = [0;0]; dt = 1e-6;\n' ...
'    L = 10e-6; C = 47e-6;\n' ...
'    RL = 0.030; RC = 0.020; Vf = 0.40;\n' ...
'end\n' ...
'd = u(1); Vin = u(2); R = u(3);\n' ...
'dp = 1 - d;\n' ...
'iL = x(1); vC = x(2);\n' ...
'vout = vC; %% abaikan ESR untuk model averaged\n' ...
'%% Persamaan averaged CCM:\n' ...
'diL = (Vin - iL*RL - dp*(vout + Vf))/L;\n' ...
'dvC = (dp*iL - vout/R)/C;\n' ...
'x   = x + [diL; dvC]*dt;\n' ...
'if x(1) < 0, x(1) = 0; end\n' ...
'y = [x(2); x(1)];\n' ...
'end\n']);

% Tulis fungsi ke dalam blok MATLAB Function
sf = sfroot;
chart = sf.find('-isa','Stateflow.EMChart','Path',[modelName '/BoostPlant']);
chart.Script = fcnCode;

%% Pengaturan solver
set_param(modelName,'Solver','ode4','FixedStep','1e-6', ...
    'StopTime','5e-3','SolverType','Fixed-step');

%% Simpan dan tampilkan
save_system(modelName);
fprintf('\nModel "%s.slx" berhasil dibuat.\n', modelName);
fprintf('Jalankan dengan: sim(''%s'')  atau klik Run di Simulink.\n', modelName);
end
