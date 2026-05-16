%% Simulasi TEG -> Boost Converter + MPPT IT2FLC -> Baterai 24V
% ----------------------------------------------------------------------
% Skenario aplikasi (proyek Ayub):
%   - Sumber: 8 modul Thermoelectric Generator (TEG) terhubung SERI
%     (bukan paralel — paralel hanya menambah arus, tegangan tetap 0.4V)
%   - Output: baterai 24 V (model sumber tegangan + resistansi internal)
%   - Algoritma MPPT: Interval Type-2 Fuzzy Logic Controller (IT2FLC)
%     -> diimplementasi di file it2flc_mppt.m
%
% Model yang dipakai: AVERAGED MODEL boost (CCM). Dipilih karena:
%   - Dinamika MPPT (orde milidetik) jauh lebih lambat dari switching
%     (mikrodetik), sehingga averaged model akurat untuk evaluasi MPPT.
%   - Simulasi 0.6 detik selesai dalam beberapa detik, bukan menit.
%
% Catatan penting:
%   Pada TEG, Maximum Power Point (MPP) berada di V = Voc/2.
%   Jika Voc total = 3.2 V, maka V_mpp ~ 1.6 V (BUKAN 3.2 V).
%   Tegangan 3.2 V hanyalah Voc (tanpa beban). MPPT akan menarik
%   titik kerja ke ~1.6 V supaya daya maksimum.
%   Konsekuensi: rasio konversi naik dari 8x menjadi ~15x (D ~ 0.93),
%   yang sudah di batas praktis boost satu tingkat. Lihat README.
% ----------------------------------------------------------------------

clear; clc; close all;

%% --- Parameter TEG ---
N_module    = 8;            % jumlah modul seri
Voc_module  = 0.4;          % tegangan open-circuit per modul [V]
Rs_module   = 0.10;         % resistansi internal per modul [ohm]
Voc_total   = N_module * Voc_module;     % 3.2 V
Rs_total    = N_module * Rs_module;      % 0.8 ohm
% MPP analitis (model linier TEG): V_mpp = Voc/2
V_mpp_ana   = Voc_total/2;               % 1.6 V
P_mpp_ana   = (Voc_total^2)/(4*Rs_total);% 3.2 W

%% --- Parameter boost converter ---
L     = 22e-6;        % naikkan L karena duty tinggi: 22uH
Cin   = 220e-6;       % Cin lebih besar (smoothing TEG): 220uF (kombinasi)
Cout  = 100e-6;       % output kapasitor 100uF
RL    = 0.030;
RC    = 0.020;
fsw   = 100e3;        % turunkan fsw agar duty tinggi lebih praktis (lebih lama on-time)

%% --- Parameter baterai (model Thevenin sederhana) ---
% Baterai 24V: misal Li-ion 7S (Vnominal ~ 25.9V) atau 6S (~22.2V).
% Di sini pakai Vbatt_oc = 24V, Rbatt = 0.1 ohm.
Vbatt_oc = 24.0;
Rbatt    = 0.10;

%% --- Parameter MPPT IT2FLC ---
Tmppt = 1e-3;           % perioda update MPPT 1 ms (1 kHz)
deltaD_max = 0.005;     % step duty maksimum per update (0.5%)

%% --- Pengaturan simulasi (averaged model) ---
Tend = 0.6;             % 600 ms (cukup untuk lihat MPPT konvergen + perubahan)
dt   = 5e-6;            % step 5 us (jauh > Tsw, tidak resolusi switching)
N    = round(Tend/dt);
t    = (0:N-1)*dt;

% State variables
iL    = zeros(1,N);     % arus induktor [A]
vC    = zeros(1,N);     % tegangan Cout [V]
vCin  = zeros(1,N);     % tegangan Cin [V] = tegangan TEG terminal
vCin(1) = 1.0;          % start dari tegangan rendah
duty  = zeros(1,N);
Pin   = zeros(1,N);
Vmpp_log = zeros(1,N);

% Inisialisasi
duty(1) = 0.5;
d_now   = 0.5;
P_prev  = 0; V_prev = 0;
mppt_timer = 0;

% Profil ΔT (perubahan kondisi sumber): simulasi step kondisi
% 0 - 200 ms: kondisi nominal (Voc=3.2 V)
% 200 - 400 ms: ΔT turun -> Voc 70% = 2.24 V
% 400 - 600 ms: ΔT naik kembali -> Voc 110% = 3.52 V (panel surya menambah)
Voc_profile = @(tt) Voc_total*(1.0)*(tt<0.2) + ...
                    Voc_total*(0.7)*(tt>=0.2 & tt<0.4) + ...
                    Voc_total*(1.1)*(tt>=0.4);

%% --- Loop simulasi (averaged model boost CCM) ---
for k = 1:N-1
    Voc_k = Voc_profile(t(k));

    % --- TEG sebagai sumber: V_teg = Voc - I_teg * Rs ---
    % I_teg = arus yang ditarik dari TEG = arus charging Cin + iL
    % Cin tegangan = vCin, jadi I_teg = (Voc - vCin)/Rs
    iTEG = (Voc_k - vCin(k)) / Rs_total;
    % Arus masuk Cin = iTEG - iL
    iCin = iTEG - iL(k);
    dvCin = iCin / Cin;

    % --- Tegangan output (asumsi ESR kecil): vout ~ vC ---
    vout = vC(k);

    % --- Arus ke baterai ---
    % Output boost terhubung ke baterai via Rbatt
    % Saat dioda konduksi, arus iL(1-D) mengisi Cout dan baterai
    iout = (vout - Vbatt_oc) / Rbatt;     % bisa negatif kalau vout < Vbatt
    if vout < Vbatt_oc, iout = 0; end     % cegah arus balik (ada dioda)

    % --- Persamaan averaged boost CCM ---
    % diL/dt = (Vin - iL*RL - (1-D)*vout) / L
    % dvC/dt = ((1-D)*iL - iout) / Cout
    Dp = 1 - d_now;
    diL = (vCin(k) - iL(k)*RL - Dp*vout) / L;
    dvC = (Dp*iL(k) - iout) / Cout;

    % --- Integrasi Euler ---
    iL(k+1)  = max(0, iL(k) + diL*dt);
    vC(k+1)  = vC(k) + dvC*dt;
    vCin(k+1)= max(0, vCin(k) + dvCin*dt);

    % --- Update IT2FLC MPPT setiap Tmppt ---
    mppt_timer = mppt_timer + dt;
    P_now = vCin(k) * iTEG;       % daya output TEG
    Pin(k) = P_now;
    Vmpp_log(k) = vCin(k);

    if mppt_timer >= Tmppt
        mppt_timer = 0;
        dP = P_now - P_prev;
        dV = vCin(k) - V_prev;
        % Panggil IT2FLC
        dD = it2flc_mppt(dP, dV, deltaD_max);
        % Pada boost: untuk MENAIKKAN V_in (TEG), turunkan D
        % Konvensi IT2FLC kita: output dD positif = naikkan V -> kurangi D
        d_now = d_now - dD;
        d_now = max(0.30, min(0.95, d_now));
        P_prev = P_now;
        V_prev = vCin(k);
    end
    duty(k+1) = d_now;
end
Pin(end) = Pin(end-1);
Vmpp_log(end) = Vmpp_log(end-1);

%% --- Hasil & metrik ---
fprintf('\n=== Parameter MPP TEG (analitis) ===\n');
fprintf(' Voc total      : %.2f V\n', Voc_total);
fprintf(' Rs total       : %.2f ohm\n', Rs_total);
fprintf(' V_mpp analitis : %.2f V\n', V_mpp_ana);
fprintf(' P_mpp analitis : %.2f W\n', P_mpp_ana);

% Window steady-state pada masing-masing fase
phase1 = t > 0.15 & t < 0.20;
phase2 = t > 0.35 & t < 0.40;
phase3 = t > 0.55 & t < 0.60;

fprintf('\n=== Hasil MPPT IT2FLC ===\n');
fprintf(' Fase 1 (Voc=3.20 V) : V_in=%.2f V  P_in=%.2f W  D=%.3f\n', ...
    mean(vCin(phase1)), mean(Pin(phase1)), mean(duty(phase1)));
fprintf(' Fase 2 (Voc=2.24 V) : V_in=%.2f V  P_in=%.2f W  D=%.3f\n', ...
    mean(vCin(phase2)), mean(Pin(phase2)), mean(duty(phase2)));
fprintf(' Fase 3 (Voc=3.52 V) : V_in=%.2f V  P_in=%.2f W  D=%.3f\n', ...
    mean(vCin(phase3)), mean(Pin(phase3)), mean(duty(phase3)));
fprintf('\n V_out rata-rata    : %.2f V\n', mean(vC(phase1)));
fprintf(' I baterai rata-rata: %.3f A\n', mean( max(0,(vC(phase1)-Vbatt_oc)/Rbatt) ));

%% --- Plot ---
figure('Name','TEG-Boost-IT2FLC-Battery','Color','w','Position',[60 60 1200 850]);

subplot(4,1,1);
plot(t, arrayfun(Voc_profile,t),'k--','LineWidth',1); hold on;
plot(t, vCin,'b','LineWidth',1.2);
yline(V_mpp_ana,':r','V_{mpp} nominal (1.6 V)');
grid on; ylabel('V_{TEG} [V]');
title('Tegangan terminal TEG (input boost) vs target MPPT');
legend('V_{oc}(t)','V_{TEG} aktual','V_{mpp}','Location','best');

subplot(4,1,2);
plot(t, Pin,'Color',[0.85 0.33 0.10],'LineWidth',1.2);
yline(P_mpp_ana,':r','P_{mpp} nominal (3.2 W)');
grid on; ylabel('P_{TEG} [W]');
title('Daya yang ditarik dari TEG (target: maksimum)');

subplot(4,1,3);
plot(t, vC,'g','LineWidth',1.2); hold on;
yline(Vbatt_oc,'--k','V_{batt} = 24 V');
grid on; ylabel('V_{out} [V]');
title('Tegangan output boost (sisi baterai)');

subplot(4,1,4);
plot(t, duty,'m','LineWidth',1.2);
grid on; ylabel('Duty'); xlabel('Waktu [s]');
title('Duty cycle (output IT2FLC MPPT)');
ylim([0 1]);

%% --- Plot kurva P-V TEG dan trajectory MPPT ---
figure('Name','Kurva P-V TEG + Trajectory MPPT','Color','w','Position',[100 100 800 500]);
Vsweep = linspace(0, Voc_total*1.2, 200);
Psweep = Vsweep .* (Voc_total - Vsweep)/Rs_total;
Psweep(Psweep<0) = 0;
plot(Vsweep, Psweep,'b-','LineWidth',1.5); hold on;
plot(V_mpp_ana, P_mpp_ana,'r*','MarkerSize',12,'LineWidth',2);
% Trajektori IT2FLC (downsample untuk visualisasi)
idx = 1:200:length(t);
plot(vCin(idx), Pin(idx),'.','Color',[0.85 0.33 0.10],'MarkerSize',6);
grid on; xlabel('V_{TEG} [V]'); ylabel('P_{TEG} [W]');
title('Kurva P-V TEG + jejak operating point MPPT IT2FLC');
legend('P-V curve (Voc=3.2 V)','MPP analitis','Trajectory IT2FLC',...
       'Location','best');
