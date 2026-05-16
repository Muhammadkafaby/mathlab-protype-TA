# Simulasi Boost Converter + MPPT IT2FLC untuk TEG → Baterai 24 V

Paket simulasi untuk proyek konversi daya **Thermoelectric Generator (TEG)** → **Boost Converter** → **Baterai 24 V** dengan algoritma MPPT **Interval Type-2 Fuzzy Logic Controller (IT2FLC)**.

## Skenario Aplikasi

- 8 modul TEG dirangkai **seri** (bukan paralel) → V_oc total = 8 × 0.4 = 3.2 V
- TEG ditempel pada panel surya (memanfaatkan ΔT dari panas matahari)
- Boost converter menaikkan tegangan TEG ke 24 V untuk pengisian baterai
- MPPT IT2FLC memastikan TEG selalu beroperasi di titik daya maksimum

> **Penting:** untuk menambah tegangan, modul harus diserikan. Memparalelkan 8 modul tetap menghasilkan ~0.4 V (yang bertambah hanya arus). Maximum Power Point TEG ada di V ≈ Voc/2 ≈ **1.6 V**, bukan 3.2 V — jadi rasio konversi sebenarnya ~15× (D ≈ 0.93).

## Struktur File

| File | Fungsi |
|---|---|
| `it2flc_mppt.m` | Fungsi Interval Type-2 Fuzzy Logic Controller (5 MF Gaussian, 25 rules, defuzzifikasi Nie-Tan). Input: dP, dV. Output: Δduty. |
| `boost_teg_mppt_battery_sim.m` | **Simulasi utama:** TEG (model Voc + Rs) → Boost averaged CCM → Baterai 24 V, dengan IT2FLC sampling 1 kHz. Mensimulasikan perubahan ΔT (Voc berubah-ubah) untuk uji tracking MPPT. |
| `boost_switching_sim.m` | Simulasi switching model boost (sumber ideal 3 V, kontroler PI). Untuk verifikasi waveform PCB. |
| `boost_averaged_model.m` | Model averaged kecil-sinyal, Bode plot, margin stabilitas, RHPZ — untuk desain kompensator. |
| `build_boost_simulink.m` | Pembangun model Simulink otomatis. |
| `boost_3v_24v.cir` | Netlist SPICE (LTspice/ngspice). |
| `boost_proteus_schematic.txt` | Panduan skema Proteus ISIS. |

## Parameter Sistem

### TEG
| Besaran | Nilai |
|---|---|
| Jumlah modul (seri) | 8 |
| V_oc per modul | 0.4 V |
| R_s per modul | 0.10 Ω |
| **V_oc total** | **3.2 V** |
| **R_s total** | **0.8 Ω** |
| **V_mpp** (= Voc/2) | **1.6 V** |
| **P_mpp** (= Voc²/4Rs) | **3.2 W** |

### Boost Converter (disesuaikan untuk duty tinggi)
| Besaran | Nilai | Alasan |
|---|---|---|
| L | 22 µH | Ditingkatkan karena duty ~0.93 → ripple lebih besar |
| Cin | 220 µF | Smoothing kuat untuk impedansi sumber TEG yang tinggi |
| Cout | 100 µF | Ripple output lebih kecil saat charging baterai |
| f_sw | 100 kHz | Diturunkan agar on-time absolut realistis di duty tinggi |

### MPPT IT2FLC
| Besaran | Nilai |
|---|---|
| Sampling rate | 1 kHz (Tmppt = 1 ms) |
| Δduty maksimum/step | 0.005 (0.5%) |
| MF | 5 Gaussian (NB, NS, ZE, PS, PB) |
| FOU σ_U / σ_L | 0.45 / 0.30 |
| Defuzzifikasi | Nie-Tan |

### Baterai
| Besaran | Nilai |
|---|---|
| V_batt OC | 24.0 V |
| R_batt internal | 0.10 Ω |

## Cara Menjalankan

### Simulasi Utama (TEG + MPPT + Baterai)
```matlab
>> boost_teg_mppt_battery_sim
```
Simulasi 600 ms dengan 3 fase ΔT berbeda:
- 0–200 ms: V_oc = 3.2 V (nominal)
- 200–400 ms: V_oc = 2.24 V (ΔT turun 30%)
- 400–600 ms: V_oc = 3.52 V (ΔT naik, panel surya menambah panas)

Output:
- V_TEG, P_TEG, V_out, duty cycle terhadap waktu
- Kurva P-V TEG dengan trajectory operating point dari IT2FLC

### Simulasi Pendukung
```matlab
>> boost_averaged_model       % desain kompensator & cek stabilitas
>> boost_switching_sim        % verifikasi waveform switching
>> build_boost_simulink       % bikin model Simulink
```

## Hasil yang Diharapkan

| Metrik | Target |
|---|---|
| V_TEG steady-state | mendekati 1.6 V (V_mpp) |
| P_TEG steady-state | mendekati 3.2 W (P_mpp) |
| Tracking efficiency | > 95% |
| V_out (sisi baterai) | ~24.0–24.5 V |
| Settling time MPPT | < 50 ms per perubahan ΔT |
| Duty cycle | ~0.92–0.94 |

## Catatan Penting Aplikasi

1. **Rasio konversi ekstrem.** V_mpp 1.6 V → V_out 24 V berarti rasio 15× (D ≈ 0.93). Single-stage boost di sini bekerja di batas praktis. Pertimbangkan:
   - **Two-stage boost** (1.6 V → 6 V → 24 V): efisiensi total bisa lebih baik daripada satu tingkat dengan D > 0.9.
   - **Coupled-inductor (tapped) boost**: rasio konversi tinggi tanpa duty ekstrem.
   - **Charge pump + boost**: efisien untuk daya rendah (di bawah 5 W).

2. **Pemilihan MOSFET kritis.** Vgs harus diaktifkan dari rail terpisah (tidak bisa dari Vin = 1.6 V). Pakai IC dengan internal charge pump (LT8330, LT8362) atau gate-drive boot supply.

3. **Output capacitance baterai.** Saat Vout < V_batt, dioda boost mencegah backflow, jadi inrush perlu di-handle dengan soft-start atau hot-swap FET di output. Pasang juga **fuse** + **TVS 27 V** di sisi baterai.

4. **Validasi MPPT IT2FLC.** Bandingkan dengan MPPT konvensional (P&O, IncCond) dengan menjalankan ulang simulasi mengganti pemanggilan `it2flc_mppt` dengan algoritma lain. IT2FLC unggul saat sumber sangat noisy (TEG memang noisy karena fluktuasi ΔT).

5. **Implementasi hardware.** IT2FLC bisa dijalankan di MCU low-cost (STM32F1, ESP32) pada 1 kHz tanpa masalah. Tabel rule + Gaussian MF cukup ringan secara komputasi.

## Roadmap Lanjutan

- [ ] Tambah model TEG non-linier (Seebeck coefficient sebagai fungsi suhu)
- [ ] Bandingkan IT2FLC vs Type-1 FLC vs P&O dalam satu plot
- [ ] Tambah model baterai Li-ion lengkap (SoC, kurva charging CC/CV)
- [ ] Verifikasi di Simulink Simscape Electrical untuk validasi presisi
- [ ] Implementasi closed-loop di SPICE dengan IC controller realistis
