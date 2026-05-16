function dD = it2flc_mppt(dP, dV, scaleOut)
%% Interval Type-2 Fuzzy Logic Controller untuk MPPT
% ----------------------------------------------------------------------
% Input  : dP        = perubahan daya  (P_now - P_prev)   [W]
%          dV        = perubahan tegangan (V_now - V_prev) [V]
%          scaleOut  = skala output (delta-duty maksimum, mis. 0.005)
%
% Output : dD        = perubahan duty cycle (akan ditambah ke duty saat ini)
%
% Aturan dasar IT2FLC mengikuti hill-climbing dP/dV pada karakteristik P-V:
%   - Jika dP/dV > 0 (kiri MPP)  -> naikkan V (turunkan duty pada boost)
%   - Jika dP/dV < 0 (kanan MPP) -> turunkan V (naikkan duty pada boost)
%
% Type-2 fuzzy memakai Footprint of Uncertainty (FOU): tiap MF punya
% Upper MF (UMF) dan Lower MF (LMF). Defuzzifikasi pakai metode Nie-Tan
% (rata-rata centroid UMF & LMF) -> sederhana tapi efektif.
% ----------------------------------------------------------------------

if nargin < 3, scaleOut = 0.005; end

%% --- Normalisasi input ke rentang [-1, 1] ---
% Skala disesuaikan dengan operating point boost 3V->24V
dP_n = max(-1, min(1, dP / 0.5));    % asumsikan |dP| max ~0.5 W
dV_n = max(-1, min(1, dV / 0.2));    % asumsikan |dV| max ~0.2 V

%% --- Definisi 5 himpunan fuzzy: NB, NS, ZE, PS, PB ---
% UMF: lebih lebar (uncertain), LMF: lebih sempit (certain)
% Pusat: NB=-1, NS=-0.5, ZE=0, PS=0.5, PB=1
centers = [-1 -0.5 0 0.5 1];
sigmaU  = 0.45;     % lebar UMF
sigmaL  = 0.30;     % lebar LMF

mfU = @(x,c) exp(-((x-c).^2)/(2*sigmaU^2));   % Gaussian UMF
mfL = @(x,c) exp(-((x-c).^2)/(2*sigmaL^2));   % Gaussian LMF

% Hitung derajat keanggotaan untuk dP_n dan dV_n
muU_dP = arrayfun(@(c) mfU(dP_n,c), centers);
muL_dP = arrayfun(@(c) mfL(dP_n,c), centers);
muU_dV = arrayfun(@(c) mfU(dV_n,c), centers);
muL_dV = arrayfun(@(c) mfL(dV_n,c), centers);

%% --- Aturan fuzzy 5x5 (output: NB,NS,ZE,PS,PB sesuai indeks 1..5) ---
% Baris   = dP (NB,NS,ZE,PS,PB)
% Kolom   = dV (NB,NS,ZE,PS,PB)
% Konvensi output: nilai delta-duty (negatif = kurangi duty)
%
% Logika: kalau dP > 0 dan dV > 0 -> arah benar (kanan MPP dari kiri),
%         lanjutkan naikkan V (PB dV) -> kurangi duty -> output NB
% Tabel ini standar referensi MPPT fuzzy:
ruleTable = [ ...
    3 4 5 4 3 ;   % dP = NB
    2 3 4 3 2 ;   % dP = NS
    1 2 3 4 5 ;   % dP = ZE  -> dorong sesuai arah dV
    2 3 2 3 4 ;   % dP = PS
    3 4 1 2 3 ];  % dP = PB

% Output crisp untuk tiap label NB..PB (delta-duty ternormalisasi)
outCenters = [-1 -0.5 0 0.5 1];

%% --- Inferensi: untuk tiap rule, firing strength = min(mu(dP), mu(dV)) ---
sumU_num = 0; sumU_den = 0;
sumL_num = 0; sumL_den = 0;

for i = 1:5
    for j = 1:5
        outIdx = ruleTable(i,j);
        yi = outCenters(outIdx);

        fU = min(muU_dP(i), muU_dV(j));   % firing strength UMF
        fL = min(muL_dP(i), muL_dV(j));   % firing strength LMF

        sumU_num = sumU_num + fU * yi;
        sumU_den = sumU_den + fU;
        sumL_num = sumL_num + fL * yi;
        sumL_den = sumL_den + fL;
    end
end

% Defuzzifikasi Nie-Tan (rata-rata centroid UMF dan LMF)
if sumU_den < 1e-9, yU = 0; else, yU = sumU_num/sumU_den; end
if sumL_den < 1e-9, yL = 0; else, yL = sumL_num/sumL_den; end
y = 0.5*(yU + yL);

%% --- Skala balik ke delta-duty riil ---
dD = y * scaleOut;

end
