#include "motor_control.h"
#include <stdio.h>
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define BLDC_PULSES_PER_MOTOR_REV  16
// Kalibrasi Fisik: Pada rasio 25, 10 RPM membutuhkan 55.96 detik (seharusnya 60 detik).
// Rasio Aktual = 25.0 * (55.96 / 60.0) = 23.3166...
#define GEARBOX_RATIO              23.3167f
#define OUTPUT_PULSES_PER_REV      (BLDC_PULSES_PER_MOTOR_REV * GEARBOX_RATIO)
// 8 sample: window ~8ms @ 1000 MRPM — cukup untuk respon cepat saat deselerasi
#define PULSE_HISTORY_LEN          8
#define PWM_MAX_DUTY               3788

// ================================================================
// Variabel yang di-share antara ISR dan Motor_Process
// ISR hanya MENULIS, Motor_Process MEMBACA dengan critical section
// Semua volatile agar compiler tidak men-cache nilai lama
// ================================================================
static uint32_t isr_interval_buf[PULSE_HISTORY_LEN]; // 64 × 4 = 256 bytes
static volatile uint16_t isr_interval_idx    = 0;
static volatile uint16_t isr_valid_intervals = 0;
static volatile uint64_t isr_interval_sum    = 0;    // Running sum interval

static volatile uint32_t rpm = 0;
static volatile float precise_rpm = 0.0f;
static volatile uint32_t bldc_pulse_count = 0;
static uint32_t adc_raw = 0;
static volatile uint32_t pwm_duty = 0; // volatile: dimodifikasi dari EXTI ISR
static volatile uint32_t set_value = 0;

// Default PID akan digunakan jika EEPROM kosong
static float pid_kp = 0.50f;
static float pid_ki = 0.60f;
static float pid_kd = 0.00f;
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;


static uint32_t last_capture_time = 0;
static volatile uint32_t last_valid_pulse_time = 0;
static volatile uint32_t timer_overflow = 0;
static uint32_t overflow_count = 0;

// ================================================================
// Deferred UART print — aman dipanggil dari EXTI ISR
// ISR menulis pesan & set flag → Motor_Process() mencetak di main loop
// ================================================================
#define DEFERRED_MSG_LEN  80
static volatile uint8_t deferred_msg_pending = 0;
static char deferred_msg_buf[DEFERRED_MSG_LEN];

uint32_t Motor_GetRPM(void) {
    // Timeout motor-berhenti di-handle oleh Motor_Process() — getter ini murni read-only
    return rpm;
}

uint32_t Motor_GetADC(void) { return adc_raw; }
uint32_t Motor_GetPWMDuty(void) { return pwm_duty; }
uint32_t Motor_GetSetValue(void) { return set_value; }
uint32_t Motor_GetPulseCount(void) { return bldc_pulse_count; }
uint32_t Motor_GetOutputRevolutions(void) { return bldc_pulse_count / OUTPUT_PULSES_PER_REV; }
float    Motor_GetPreciseMotorRPM(void) { return precise_rpm; } // Motor RPM (sebelum gearbox)
float    Motor_GetPreciseOutputRPM(void) { return precise_rpm / (float)GEARBOX_RATIO; } // Output RPM float presisi

void Motor_ForceRPMReset(void) {
    rpm = 0;
    precise_rpm = 0.0f;
}

void Motor_SetCLITarget(uint32_t rpm_target) {
    if (rpm_target > 150) rpm_target = 150;
    set_value = rpm_target;
}

void Motor_SetPID(float kp, float ki, float kd) {
    pid_kp = kp;
    pid_ki = ki;
    pid_kd = kd;
    // JANGAN reset pid_integral di sini!
    // Mereset integral saat motor sedang berputar di kecepatan tinggi akan 
    // menyebabkan PWM drop instan ke 0, yang memicu back-EMF / tegangan kejut besar
    // dari motor dan membuat mikrokontroler restart (brownout).
}

void Motor_GetPID(float *kp, float *ki, float *kd) {
    if (kp) *kp = pid_kp;
    if (ki) *ki = pid_ki;
    if (kd) *kd = pid_kd;
}

void Motor_GetPIDState(float *error, float *integral) {
    if (error) *error = pid_prev_error;
    if (integral) *integral = pid_integral;
}

static MotorControlMode control_mode = MOTOR_MODE_PID;

void Motor_SetMode(MotorControlMode mode) {
    control_mode = mode;
    // Reset PID state on mode switch to avoid integral windup from previous mode
    pid_integral   = 0.0f;
    pid_prev_error = 0.0f;
}

MotorControlMode Motor_GetMode(void) {
    return control_mode;
}

void Motor_Process(void) {
    static uint32_t last_adc_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_adc_tick >= 2) {
        float dt = (now - last_adc_tick) / 1000.0f;
        if (dt <= 0.0f) dt = 0.002f;

        // ============================================================
        // Hitung RPM dari ring buffer ISR (critical section)
        // Ini adalah satu-satunya tempat float/division dijalankan!
        // ============================================================
        if (HAL_GetTick() - last_capture_time > 500) {
            // Motor berhenti
            rpm         = 0;
            precise_rpm = 0.0f;
            __disable_irq();
            isr_valid_intervals = 0;
            isr_interval_sum    = 0;
            isr_interval_idx    = 0;
            __enable_irq();
        } else {
            // Baca state ISR dengan critical section (minimal waktu disable IRQ)
            __disable_irq();
            uint16_t vi  = isr_valid_intervals;
            uint64_t isum = isr_interval_sum;
            __enable_irq();

            if (vi >= 4) {
                // Rata-rata interval → Motor RPM → Output RPM
                float mean_interval = (float)isum / (float)vi;

                // Koreksi Matematis Wajib untuk Tachometer Interval:
                // Jika waktu menunggu pulsa berikutnya sudah melampaui interval rata-rata sebelumnya,
                // maka secara matematis motor SUDAH melambat. Kita gunakan waktu tunggu saat ini
                // sebagai interval (dengan margin 2ms untuk menoleransi jitter timer 1ms).
                // Ini mencegah RPM 'membeku' saat motor direm mendadak (PWM=0).
                uint32_t current_tim4 = __HAL_TIM_GET_COUNTER(&htim4);
                uint32_t extended_now = (timer_overflow * 65536UL) + current_tim4;
                // Cegah race condition overflow
                if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET && current_tim4 < 32768) {
                    extended_now += 65536UL;
                }
                
                uint32_t current_wait_us = extended_now - last_valid_pulse_time;
                if ((float)current_wait_us > mean_interval + 2000.0f) {
                    mean_interval = (float)current_wait_us;
                }

                precise_rpm = 60000000.0f / (mean_interval * BLDC_PULSES_PER_MOTOR_REV);
                rpm = (uint32_t)(precise_rpm / GEARBOX_RATIO + 0.5f);
            }
        }

        // ADC dibuang (diganti Rotary Encoder)
        // (last_adc_tick di-update satu kali di akhir fungsi — baris duplikat dihapus)

        if (control_mode == MOTOR_MODE_DIRECT) {
            // =====================================================
            // MODE DIRECT: Rotary Encoder langsung ke PWM (open loop)
            // =====================================================
            // pwm_duty diubah langsung oleh interrupt rotary encoder
            set_value = Motor_GetRPM(); // SET menampilkan RPM aktual saat ini
            pid_integral = 0.0f;
            pid_prev_error = 0.0f;

        } else {
            // =====================================================
            // MODE PID & CLI: Rotary Encoder / CLI -> Target RPM -> PID -> PWM
            // =====================================================
            if (control_mode == MOTOR_MODE_PID) {
                // Mode PID: set_value diubah oleh Rotary Encoder via EXTI callback
                // (Tidak perlu polling ADC lagi)
            }
            // Mode CLI: set_value sudah di-set dari luar via Motor_SetCLITarget()

            // Snapshot set_value satu kali — cegah ISR (rotary encoder) mengubah nilai
            // di tengah kalkulasi PID sehingga target_motor_rpm dan sdiff selalu konsisten
            uint32_t sv = set_value;

            // PID operates in Motor RPM domain (25x higher resolution)
            float target_motor_rpm = (float)(sv * GEARBOX_RATIO);
            float current_motor_rpm = precise_rpm;

            if (target_motor_rpm < 1.0f) {
                pwm_duty = 0;
                pid_integral   = 0.0f;
                pid_prev_error = 0.0f;
            } else {
                float error = target_motor_rpm - current_motor_rpm;

                float raw_derivative = (error - pid_prev_error) / dt;
                pid_prev_error = error;
                
                // Low-pass filter (IIR) untuk Derivative
                // Sangat penting pada update rate tinggi (500Hz) agar Kd tidak bereaksi brutal 
                // terhadap jitter mekanis sekecil apapun dari sensor.
                static float filtered_derivative = 0.0f;
                filtered_derivative = 0.1f * raw_derivative + 0.9f * filtered_derivative;

                float p_term = pid_kp * error;
                float d_term = pid_kd * filtered_derivative;
                float i_term_tentative = pid_ki * (pid_integral + error * dt);

                float output = p_term + i_term_tentative + d_term;

                // Anti-windup Standar Industri (Conditional Integration)
                // Wajib ada untuk sistem fisik (motor) agar I-term tidak menumpuk saat PWM mentok (0 atau MAX).
                // Tanpa ini, motor akan tersendat (berhenti lama sebelum jalan lagi) karena 
                // nilai integral harus "merangkak" naik dari minus yang sangat dalam.
                if (output >= PWM_MAX_DUTY) {
                    output = PWM_MAX_DUTY;
                    if (error < 0) pid_integral += error * dt;
                } else if (output <= 0) {
                    output = 0;
                    if (error > 0) pid_integral += error * dt;
                } else {
                    pid_integral += error * dt;
                }

                pwm_duty = (uint32_t)output;
            }
        }

        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm_duty);
        last_adc_tick = now;

        // Flush pesan UART yang di-defer dari ISR rotary encoder
        if (deferred_msg_pending) {
            char local_buf[DEFERRED_MSG_LEN];
            __disable_irq();
            memcpy(local_buf, (const void*)deferred_msg_buf, DEFERRED_MSG_LEN);
            deferred_msg_pending = 0;
            __enable_irq();
            UART_Print(local_buf);
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        uint32_t current = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t ovf = overflow_count;

        // Race condition: overflow flag pending tapi belum diservice
        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET && current < 32768) {
            ovf++;
        }

        uint32_t extended = (ovf * 65536UL) + current;
        
        // NOISE FILTER (Anti-Aliasing Debounce)
        // Kita butuh 2 variabel statik: waktu edge apapun (termasuk noise), dan waktu pulse valid terakhir.
        static uint32_t last_raw_edge = 0;

        uint32_t delta_from_raw = extended - last_raw_edge;
        last_raw_edge = extended; // Selalu update ke edge paling baru

        // Threshold 600us (setara maks ~6250 Motor RPM). 
        // Jika jarak antar edge < 600us, anggap sebagai getaran/noise.
        // Karena last_raw_edge selalu diupdate, rentetan noise 100us akan TERUS ditolak
        // dan tidak akan pernah terakumulasi menjadi pulse palsu (mencegah sub-harmonic aliasing).
        if (delta_from_raw < 600) {
            return; // Buang noise
        }

        // Pulse valid! Hitung delta sesungguhnya dari pulse valid sebelumnya
        uint32_t delta = extended - last_valid_pulse_time;
        last_valid_pulse_time = extended;

        bldc_pulse_count++;

        // Outlier rejection (berdasarkan threshold) dihapus karena menghalangi 
        // pembacaan deselerasi ekstrem (saat motor di-rem, delta menjadi sangat besar 
        // dan salah dianggap sebagai noise, menyebabkan RPM 'membeku' di nilai tinggi).

        // Update ring buffer: hanya integer add/subtract (O(1))
        if (isr_valid_intervals == PULSE_HISTORY_LEN) {
            isr_interval_sum -= isr_interval_buf[isr_interval_idx];
        } else {
            isr_valid_intervals++;
        }
        isr_interval_buf[isr_interval_idx] = delta;
        isr_interval_sum += delta;
        isr_interval_idx = (isr_interval_idx + 1) % PULSE_HISTORY_LEN;

        last_capture_time = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        timer_overflow++;
        overflow_count++;
    }
}
// ====================================================
// ROTARY ENCODER API
// ====================================================
void Motor_RotaryIncrement(void) {
    // PENTING: fungsi ini dipanggil dari EXTI ISR!
    // Dilarang memanggil UART_Print langsung (blocking HAL).
    // Pesan ditulis ke buffer, Motor_Process() yang mencetak dari main loop.
    if (control_mode == MOTOR_MODE_PID || control_mode == MOTOR_MODE_CLI) {
        if (set_value < 150) set_value++;
        if (!deferred_msg_pending) {
            snprintf(deferred_msg_buf, DEFERRED_MSG_LEN,
                     "[ROTARY CW] Mode: PID/CLI, Target RPM: %lu\r\n", (unsigned long)set_value);
            deferred_msg_pending = 1;
        }
    } else if (control_mode == MOTOR_MODE_DIRECT) {
        if (pwm_duty + 10 <= PWM_MAX_DUTY) pwm_duty += 10;
        else pwm_duty = PWM_MAX_DUTY;
        if (!deferred_msg_pending) {
            snprintf(deferred_msg_buf, DEFERRED_MSG_LEN,
                     "[ROTARY CW] Mode: DIRECT, PWM: %lu\r\n", (unsigned long)pwm_duty);
            deferred_msg_pending = 1;
        }
    }
}

void Motor_RotaryDecrement(void) {
    // PENTING: fungsi ini dipanggil dari EXTI ISR!
    // Dilarang memanggil UART_Print langsung (blocking HAL).
    // Pesan ditulis ke buffer, Motor_Process() yang mencetak dari main loop.
    if (control_mode == MOTOR_MODE_PID || control_mode == MOTOR_MODE_CLI) {
        if (set_value > 0) set_value--;
        if (!deferred_msg_pending) {
            snprintf(deferred_msg_buf, DEFERRED_MSG_LEN,
                     "[ROTARY CCW] Mode: PID/CLI, Target RPM: %lu\r\n", (unsigned long)set_value);
            deferred_msg_pending = 1;
        }
    } else if (control_mode == MOTOR_MODE_DIRECT) {
        if (pwm_duty >= 10) pwm_duty -= 10;
        else pwm_duty = 0;
        if (!deferred_msg_pending) {
            snprintf(deferred_msg_buf, DEFERRED_MSG_LEN,
                     "[ROTARY CCW] Mode: DIRECT, PWM: %lu\r\n", (unsigned long)pwm_duty);
            deferred_msg_pending = 1;
        }
    }
}
