%% Boost Converter 3V -> 24V : Switching Model Simulation (MATLAB)
% ----------------------------------------------------------------------
% Simulasi model switching (transient) menggunakan integrasi numerik
% pada persamaan diferensial induktor & kapasitor. Tidak butuh Simulink.
%
% Topology: Non-synchronous boost (NMOS + Schottky)
% Vin  = 3 V
% Vout = 24 V (target)
% Iout = 200 mA (Rload = 120 ohm)
% fsw  = 500 kHz
% L    = 10 uH, Cout = 47 uF
%
% Author: Kilo (untuk proyek Ayub)
% ----------------------------------------------------------------------

clear; clc; close all;

%% --- Parameter rangkaian ---
Vin   = 3.0;          % Tegangan input  [V]
Vout_ref = 24.0;      % Target tegangan output [V]
Rload = 120;          % Beban [ohm]  -> Iout = 0.2 A
L     = 10e-6;        % Induktansi [H]
RL    = 0.030;        % DCR induktor [ohm]
Cout  = 47e-6;        % Kapasitor output [F]
RC    = 0.020;        % ESR Cout [ohm]
Rds   = 0.030;        % Rds(on) MOSFET [ohm]
Vf    = 0.40;         % Vf Schottky [V]
Rd    = 0.050;        % Resistansi seri dioda saat ON [ohm]

fsw   = 500e3;        % Frekuensi switching [Hz]
Tsw   = 1/fsw;

% --- Duty cycle ideal (steady state) untuk inisialisasi ---
D_ideal = 1 - Vin/Vout_ref;   % ~ 0.875

%% --- Kontroler PI sederhana untuk regulasi closed-loop ---
% (Type-II compensator versi diskrit yang sederhana)
Kp = 0.0008;
Ki = 6.0;
Dmin = 0.05;
Dmax = 0.93;

%% --- Pengaturan simulasi ---
Tend = 4e-3;          % Durasi total 4 ms (cukup melihat startup + steady state)
dt   = 20e-9;         % Step 20 ns -> 100 sample per periode switching
N    = round(Tend/dt);

t   = (0:N-1)*dt;
iL  = zeros(1,N);
vC  = zeros(1,N);     % Tegangan kapasitor (vout = vC + iC*RC)
vout= zeros(1,N);
d   = zeros(1,N);
sw  = zeros(1,N);     % State MOSFET (1=ON, 0=OFF)

% State integrator PI
err_int = 0;

% Soft-start: ramp referensi 0 -> 24 V dalam 1 ms
t_ss = 1e-3;

%% --- Loop simulasi ---
phase = 0;            % posisi dalam periode switching [0..Tsw)
duty  = D_ideal;      % nilai duty awal

for k = 1:N-1
    % --- Soft-start reference ---
    if t(k) < t_ss
        Vref_k = Vout_ref * (t(k)/t_ss);
    else
        Vref_k = Vout_ref;
    end

    % --- Update kontroler tiap awal periode switching ---
    if phase < dt
        err = Vref_k - vout(k);
        err_int = err_int + err*Tsw;
        duty = Kp*err + Ki*err_int;
        duty = max(Dmin, min(Dmax, duty));
    end
    d(k) = duty;

    % --- Tentukan state switch (PWM) ---
    if phase < duty*Tsw
        sw(k) = 1;    % MOSFET ON
    else
        sw(k) = 0;    % MOSFET OFF (dioda konduksi bila iL>0)
    end

    % --- Hitung turunan (Euler) ---
    if sw(k) == 1
        % Mode ON: induktor terhubung Vin - GND lewat Rds
        diL = (Vin - iL(k)*(RL + Rds)) / L;
        % Kapasitor hanya melepas ke beban
        iout = vout(k)/Rload;
        dvC  = -iout / Cout;
    else
        % Mode OFF: induktor mengalir lewat dioda ke Cout||Rload
        if iL(k) > 0
            % Vout = vC + (iL - iout)*RC ; tapi simplifikasi: anggap iC = iL - iout
            iout = vout(k)/Rload;
            iC   = iL(k) - iout;
            diL  = (Vin - iL(k)*(RL + Rd) - Vf - vout(k)) / L;
            dvC  = iC / Cout;
        else
            % DCM: induktor kosong
            iL(k) = 0;
            diL   = 0;
            iout  = vout(k)/Rload;
            dvC   = -iout / Cout;
        end
    end

    % --- Integrasi Euler ---
    iL(k+1) = max(0, iL(k) + diL*dt);
    vC(k+1) = vC(k) + dvC*dt;

    % --- Hitung tegangan output (termasuk drop ESR) ---
    if sw(k) == 0 && iL(k) > 0
        iC_now = iL(k) - vout(k)/Rload;
    else
        iC_now = -vout(k)/Rload;
    end
    vout(k+1) = vC(k+1) + iC_now*RC;

    % --- Update fase PWM ---
    phase = phase + dt;
    if phase >= Tsw
        phase = phase - Tsw;
    end
end
d(end) = d(end-1); sw(end) = sw(end-1);

%% --- Hasil & metrik ---
idx_ss = t > 3e-3;                         % window steady state
Vout_avg = mean(vout(idx_ss));
Vripple  = max(vout(idx_ss)) - min(vout(idx_ss));
IL_avg   = mean(iL(idx_ss));
IL_pp    = max(iL(idx_ss)) - min(iL(idx_ss));
Pout     = Vout_avg^2 / Rload;
Pin      = Vin * mean(iL(idx_ss));
eta      = 100 * Pout / Pin;

fprintf('\n=== Hasil Simulasi Boost 3V -> 24V ===\n');
fprintf(' Vout rata-rata   : %.3f V\n', Vout_avg);
fprintf(' Ripple Vout (pp) : %.1f mV\n', Vripple*1e3);
fprintf(' IL rata-rata     : %.3f A\n', IL_avg);
fprintf(' IL ripple (pp)   : %.3f A\n', IL_pp);
fprintf(' Duty steady-state: %.3f\n', mean(d(idx_ss)));
fprintf(' Pout             : %.3f W\n', Pout);
fprintf(' Pin              : %.3f W\n', Pin);
fprintf(' Estimasi efisiensi: %.1f %%\n', eta);

%% --- Plot hasil ---
figure('Name','Boost 3V->24V Switching Sim','Color','w','Position',[100 100 1100 750]);

subplot(3,1,1);
plot(t*1e3, vout, 'b','LineWidth',1); hold on;
yline(24,'--r','Vref = 24 V','LabelHorizontalAlignment','left');
grid on; ylabel('V_{out} [V]'); title('Tegangan Output (startup + steady)');
xlim([0 Tend*1e3]);

subplot(3,1,2);
plot(t*1e3, iL,'Color',[0.85 0.33 0.10],'LineWidth',1);
grid on; ylabel('I_L [A]'); title('Arus Induktor');
xlim([0 Tend*1e3]);

subplot(3,1,3);
plot(t*1e3, d,'k','LineWidth',1);
grid on; ylabel('Duty'); xlabel('Waktu [ms]');
title('Duty cycle (output kontroler PI)');
ylim([0 1]); xlim([0 Tend*1e3]);

% --- Zoom-in 5 periode switching di steady state ---
figure('Name','Zoom Steady-State Switching','Color','w','Position',[150 150 1000 600]);
twin = t > 3.5e-3 & t < (3.5e-3 + 5*Tsw);
subplot(2,1,1);
plot(t(twin)*1e6, vout(twin),'b'); grid on;
ylabel('V_{out} [V]'); title('Ripple V_{out} steady state');
subplot(2,1,2);
plot(t(twin)*1e6, iL(twin),'Color',[0.85 0.33 0.10]); grid on;
xlabel('Waktu [\mus]'); ylabel('I_L [A]'); title('Ripple Arus Induktor');
