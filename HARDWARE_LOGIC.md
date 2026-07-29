# Dokumentasi Hardware & Logika Rotarod (5-Lane System)

Dokumen ini berisi panduan teknis mengenai penyambungan (wiring) pin dan logika operasional dari alat Rotarod 5-Lane. Informasi ini sangat penting bagi perancang PCB (Hardware Engineer) dan referensi di masa mendatang.

## 1. Pemetaan 30 Input ke 4x IC 74HC165 (Shift Register)
Alat ini menggunakan **4 buah IC 74HC165** yang di-cascade, memberikan total 32 pin input digital. Kita menggunakan 30 pin pertama (Pin 0 s.d 29) yang dibagi merata menjadi **6 pin untuk masing-masing Lane**.

### Tabel Koneksi Pin per Lane (Sesuai Skematik Faktual):
Setiap IC 74HC165 membaca 8 bit. Pin dialokasikan secara kategorial (bukan sekuensial) berdasarkan skematik:

| Fungsi Sensor / Tombol | Lane 1 | Lane 2 | Lane 3 | Lane 4 | Lane 5 |
|-------------------------|--------|--------|--------|--------|--------|
| **1. Tombol Start Onboard** | Pin 0  | Pin 3  | Pin 6  | Pin 9  | Pin 12 |
| **2. Tombol Stop Onboard**  | Pin 1  | Pin 4  | Pin 7  | Pin 10 | Pin 13 |
| **3. Tombol Reset Onboard** | Pin 2  | Pin 5  | Pin 8  | Pin 11 | Pin 14 |
| **4. Sensor Magnet (Jatuh)**| Pin 15 | Pin 16 | Pin 17 | Pin 18 | Pin 19 |
| **5. Kabel Eksternal Detect**| Pin 20 | Pin 21 | Pin 22 | Pin 23 | Pin 24 |
| **6. Tombol Eksternal**     | Pin 25 | Pin 26 | Pin 27 | Pin 28 | Pin 29 |

*(Catatan: Pin 30 dan Pin 31 tidak digunakan).*

---

## 2. Pemetaan 7 Layar TM1637 (Display)
Sistem memiliki 7 buah modul layar 4-digit TM1637 yang dihubungkan ke mikrokontroler. Pastikan pemasangan kabel CLK dan DIO pada STM32 disesuaikan dengan indeks berikut:

- **Display 0 (Index 0):** Timer Lane 1 (Format MM:SS)
- **Display 1 (Index 1):** Timer Lane 2 (Format MM:SS)
- **Display 2 (Index 2):** Timer Lane 3 (Format MM:SS)
- **Display 3 (Index 3):** Timer Lane 4 (Format MM:SS)
- **Display 4 (Index 4):** Timer Lane 5 (Format MM:SS)
- **Display 5 (Index 5):** Actual RPM (Kecepatan putar motor aktual, rata-rata setiap 2 detik)
- **Display 6 (Index 6):** Set RPM (Target kecepatan motor dari potensiometer/sistem)

---

## 3. State Machine (Logika Cara Kerja)

Setiap Lane (1 sampai 5) berjalan secara **sepenuhnya independen** dengan aturan logika berikut:

### A. Fitur Pendeteksi Kabel Eksternal (*Override*)
- Pin **Kabel Eksternal Detect** bertipe *Active Low*. Artinya, jika kabel ditancapkan ke konektor PCB, pin ini akan terhubung ke `GND`.
- **Jika Kabel Tercolok (GND):** Sistem akan langsung mematikan/mengabaikan fungsi tombol *Start* dan *Stop* Onboard. Sistem hanya akan merespons penekanan dari **Tombol Eksternal**. Tombol eksternal ini berfungsi ganda sebagai saklar tekan (*Toggle*); tekan pertama untuk *Start*, tekan kedua untuk *Stop*.
- **Jika Kabel Dicabut (HIGH):** Tombol *Start* dan *Stop* Onboard akan berfungsi normal kembali, dan pin Tombol Eksternal diabaikan.
- Tombol **Reset Onboard** tidak terpengaruh oleh kabel eksternal (selalu aktif untuk mereset angka ke 0 saat timer sedang berhenti).

### B. Fitur Pencatatan CSV Otomatis (Flashdisk USB)
- Saat timer sedang berjalan (*Running*), mikrokontroler akan terus mendengarkan **Sensor Magnet** di masing-masing Lane.
- Jika ada tikus yang jatuh (Sensor Magnet terpicu / *Edge detection*), Timer di layar TM1637 untuk Lane tersebut akan **langsung membeku/berhenti**.
- Waktu lari (dalam satuan ms, menggunakan memori 32-bit yang tahan hingga 49 hari), RPM, dan nomor Lane akan otomatis diformat dan dikirim ke sistem antrean.
- Saat Flashdisk / PC mendeteksi penyimpanan, baris data baru (misal: `29/07/26,13:10:48,2421,15,3`) akan ditambahkan secara permanen ke bagian bawah file `ROTAROD.CSV`.

---
*Dokumen ini dibuat otomatis sebagai panduan manufaktur dan troubleshooting.*
