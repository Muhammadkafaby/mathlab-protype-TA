%% Boost Converter 3V -> 24V : Averaged Small-Signal Model & Stability
% ----------------------------------------------------------------------
% Analisis frekuensi (Bode), pole/zero, RHPZ, dan margin stabilitas
% berdasarkan model state-space averaged (CCM).
%
% Plant control-to-output:  Gvd(s) = vhat(s)/dhat(s)
% Dengan kompensator Type-II untuk current-mode controller.
% ----------------------------------------------------------------------

clear; clc; close all;

%% --- Parameter ---
Vin   = 3.0;
Vout  = 24.0;
Iout  = 0.20;
R     = Vout/Iout;     % 120 ohm
L     = 10e-6;
C     = 47e-6;
RC    = 0.020;         % ESR Cout
RL    = 0.030;         % DCR
fsw   = 500e3;

D     = 1 - Vin/Vout;  % 0.875
Dp    = 1 - D;         % 0.125
M     = 1/Dp;          % 8

%% --- Model averaged kecil-sinyal (lossless approx, CCM) ---
% Gvd(s) = (Vout/Dp) * (1 - s*L/(Dp^2*R)) / (1 + s*L/(Dp^2*R) + s^2*L*C/Dp^2)
%
% Rumus klasik Erickson (Fundamentals of Power Electronics, ch. 8).

s = tf('s');

num_RHPZ = [-L/(Dp^2*R) 1];           % zero kanan (RHPZ)
den_LC   = [L*C/Dp^2  L/(Dp^2*R)  1]; % polinomial LC orde-2

Gvd = (Vout/Dp) * tf(num_RHPZ, den_LC);

% Tambahkan zero ESR (LHP zero dari kapasitor): 1 + s*RC*C
Gvd = Gvd * (1 + s*RC*C);

% --- Frekuensi karakteristik ---
f0   = Dp/(2*pi*sqrt(L*C));     % LC corner frequency
fz_RHP = (Dp^2*R)/(2*pi*L);     % RHPZ frequency
fz_ESR = 1/(2*pi*RC*C);         % ESR zero
fc_target = fz_RHP/5;           % Aturan: crossover <= fRHPZ/5

fprintf('=== Karakteristik Plant Boost 3V->24V ===\n');
fprintf(' D                 : %.3f\n', D);
fprintf(' f_LC (resonansi)  : %.2f kHz\n', f0/1e3);
fprintf(' f_RHPZ            : %.2f kHz\n', fz_RHP/1e3);
fprintf(' f_ESR zero        : %.2f kHz\n', fz_ESR/1e3);
fprintf(' Target crossover  : %.2f kHz (= f_RHPZ/5)\n', fc_target/1e3);

%% --- Kompensator Type-II ---
% Struktur: Gc(s) = K * (1 + s/wz) / [s * (1 + s/wp)]
% wz ditempatkan di f_LC supaya membatalkan fase plant
% wp ditempatkan di f_ESR atau ~fsw/2 untuk meredam HF noise

wz = 2*pi*f0;            % zero di LC corner
wp = 2*pi*min(fz_ESR, fsw/4);   % pole di ESR zero atau fsw/4
% Skala gain agar |T(jwc)| = 1 pada fc_target
wc = 2*pi*fc_target;
Gc_unity = (1 + s/wz)/(s*(1 + s/wp));
mag_at_wc = abs(evalfr(Gvd*Gc_unity, 1j*wc));
K = 1/mag_at_wc;
Gc = K * Gc_unity;

% --- Komponen Type-II nyata (referensi nilai) ---
% Asumsikan Gm error amp + Ramp pembagi 1/29
% Tinggal informasi orientasi nilai komponen:
Rfb_top = 287e3;
Rfb_bot = 10e3;
% Cc1 = 1/(K*wz),  Rc = K*Rfb_top  (tergantung topologi error amp)
% Hanya sebagai panduan; nilai presisi dihitung dari datasheet IC.

%% --- Loop transfer function ---
T = Gvd * Gc;

%% --- Plot Bode plant, kompensator, loop ---
fmin = 10; fmax = fsw*2;
w = logspace(log10(2*pi*fmin), log10(2*pi*fmax), 2000);

figure('Name','Bode Boost 3V->24V','Color','w','Position',[80 80 1100 700]);
[mP,pP] = bode(Gvd,w);  mP = squeeze(mP); pP = squeeze(pP);
[mC,pC] = bode(Gc,w);   mC = squeeze(mC); pC = squeeze(pC);
[mT,pT] = bode(T,w);    mT = squeeze(mT); pT = squeeze(pT);

subplot(2,1,1);
semilogx(w/(2*pi), 20*log10(mP),'b','LineWidth',1.3); hold on;
semilogx(w/(2*pi), 20*log10(mC),'g','LineWidth',1.3);
semilogx(w/(2*pi), 20*log10(mT),'r','LineWidth',1.6);
yline(0,'--k');
grid on; ylabel('Magnitude [dB]');
legend('Plant G_{vd}','Kompensator G_c','Loop T = G_{vd} G_c','Location','southwest');
title('Bode plot');

subplot(2,1,2);
semilogx(w/(2*pi), pP,'b','LineWidth',1.3); hold on;
semilogx(w/(2*pi), pC,'g','LineWidth',1.3);
semilogx(w/(2*pi), pT,'r','LineWidth',1.6);
yline(-180,'--k'); yline(-360,'--k');
grid on; ylabel('Phase [deg]'); xlabel('Frekuensi [Hz]');

%% --- Margin stabilitas ---
[Gm,Pm,Wcg,Wcp] = margin(T);
fprintf('\n=== Margin Stabilitas Loop ===\n');
fprintf(' Crossover gain    : %.2f kHz\n', Wcp/(2*pi)/1e3);
fprintf(' Phase Margin (PM) : %.1f deg  (target >= 45 deg)\n', Pm);
if isfinite(Gm)
    fprintf(' Gain Margin  (GM) : %.1f dB   (target >= 6 dB)\n', 20*log10(Gm));
else
    fprintf(' Gain Margin  (GM) : Inf (tidak ada crossing -180)\n');
end

%% --- Closed-loop step response (load step kecil-sinyal) ---
Tcl = feedback(Gvd*Gc, 1);
figure('Name','Step Response Closed Loop','Color','w');
step(Tcl, 5e-3); grid on;
title('Respons step closed-loop (model averaged)');

%% --- Pole-zero plant dan loop ---
figure('Name','Pole-Zero Map','Color','w');
subplot(1,2,1); pzmap(Gvd); grid on; title('Plant G_{vd}(s)');
subplot(1,2,2); pzmap(T);   grid on; title('Loop T(s)');
