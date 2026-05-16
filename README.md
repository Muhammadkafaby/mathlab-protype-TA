# Simulasi Boost Converter 3V → 24V

Paket simulasi untuk verifikasi desain boost converter 3 V → 24 V (200 mA, 500 kHz).
Tersedia tiga jalur simulasi: **MATLAB script murni**, **Simulink**, dan **SPICE/Proteus**.

## Struktur File

| File | Fungsi |
|---|---|
| `boost_switching_sim.m` | Simulasi switching model (transient) di MATLAB tanpa Simulink. Termasuk PWM, kontroler PI, soft-start, dan plot lengkap. |
| `boost_averaged_model.m` | Model averaged kecil-sinyal, Bode plot plant + kompensator + loop, analisis margin stabilitas, RHPZ, step response closed-loop. |
| `build_boost_simulink.m` | Skrip otomatis yang membangun model Simulink (`boost_3v_24v.slx`) memakai blok dasar. Tidak perlu Simscape. |
| `boost_3v_24v.cir` | Netlist SPICE untuk LTspice / ngspice. Open-loop PWM 87.5% duty, plus rangka closed-loop sebagai komentar. |
| `boost_proteus_schematic.txt` | Daftar komponen + net list manual untuk digambar di Proteus ISIS. |

## Parameter Desain (dipakai di semua file)

| Besaran | Nilai |
|---|---|
| Vin | 3.0 V |
| Vout | 24.0 V |
| Iout | 200 mA (Rload = 120 Ω) |
| L | 10 µH (DCR 30 mΩ) |
| Cout | 47 µF (ESR 20 mΩ) |
| fsw | 500 kHz |
| D ideal | 0.875 |

## Cara Menjalankan

### 1. MATLAB — Switching Model
```matlab
>> boost_switching_sim
```
Hasil di Command Window:
- Vout rata-rata, ripple, IL rata-rata & ripple, duty cycle, efisiensi.

Plot:
- Vout, IL, duty selama 4 ms (startup + steady state).
- Zoom 5 periode switching di steady state untuk melihat ripple.

### 2. MATLAB — Averaged Model & Stabilitas
```matlab
>> boost_averaged_model
```
Output:
- Frekuensi karakteristik: f_LC, f_RHPZ, f_ESR, target crossover.
- Bode plot plant + kompensator + loop.
- Phase margin & gain margin.
- Step response closed-loop dan pole-zero map.

### 3. Simulink (otomatis)
```matlab
>> build_boost_simulink
>> sim('boost_3v_24v')
```
Atau klik Run di Simulink setelah model terbuka.

### 4. LTspice
- Buka `boost_3v_24v.cir` lewat **File → Open**.
- Klik **Run**.
- Lihat `.meas` di SPICE Error Log (`Ctrl+L`) untuk Vout_avg, ripple, IL, dll.

### 5. Proteus
- Ikuti petunjuk di `boost_proteus_schematic.txt`.
- Gambar komponen sesuai tabel, sambungkan sesuai NET LIST.
- Atur Transient Analysis: stop 4 ms, max step 50 ns.

## Hasil yang Diharapkan (referensi)

| Metrik | Nilai target |
|---|---|
| Vout rata-rata | 23.8 – 24.1 V |
| Ripple Vout (pp) | < 100 mV |
| IL rata-rata | ~ 1.7 – 1.9 A |
| IL ripple (pp) | ~ 0.5 – 0.6 A |
| Duty steady-state | ~ 0.87 – 0.90 |
| Efisiensi (model) | 78 – 85 % |
| Phase margin | ≥ 45° |
| Gain margin | ≥ 6 dB |
| Crossover frequency | ~ 6 kHz (≤ f_RHPZ/5) |

## Urutan Verifikasi yang Disarankan

1. Jalankan `boost_averaged_model.m` dulu untuk menentukan crossover dan margin yang aman.
2. Pakai `boost_switching_sim.m` untuk konfirmasi bentuk gelombang & ripple.
3. Validasi di SPICE (`boost_3v_24v.cir`) dengan model device yang lebih realistis.
4. Pindahkan ke Proteus untuk verifikasi visual sebelum bikin PCB.

## Catatan Penting

- Duty cycle 0.875 ada di batas praktis boost satu tingkat. Bila beban naik > 500 mA, evaluasi topologi *coupled-inductor boost* atau dua tingkat (3 V → 9 V → 24 V).
- Untuk hasil presisi di SPICE, ganti `.model NMOS_SW` dan `.model DSCH` dengan model SPICE pabrikan (mis. SiR826DP, PMEG4030ER).
- Snubber 2.2 Ω + 470 pF di SPICE sudah disiapkan; tuning halus dilakukan setelah lihat ringing Vds.
- Closed-loop di SPICE tinggal mengaktifkan blok dalam komentar (lihat bagian `COMMENT` di `.cir`).
