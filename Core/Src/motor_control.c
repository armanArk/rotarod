#include "motor_control.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define BLDC_PULSES_PER_MOTOR_REV  16
#define GEARBOX_RATIO              25
#define OUTPUT_PULSES_PER_REV      (BLDC_PULSES_PER_MOTOR_REV * GEARBOX_RATIO) // 400
#define PULSE_HISTORY_LEN          OUTPUT_PULSES_PER_REV
#define PWM_MAX_DUTY               3788

// ================================================================
// Variabel yang di-share antara ISR dan Motor_Process
// ISR hanya MENULIS, Motor_Process MEMBACA dengan critical section
// Semua volatile agar compiler tidak men-cache nilai lama
// ================================================================
static uint32_t isr_interval_buf[PULSE_HISTORY_LEN]; // 400 × 4 = 1600 bytes
static volatile uint16_t isr_interval_idx    = 0;
static volatile uint16_t isr_valid_intervals = 0;
static volatile uint64_t isr_interval_sum    = 0;    // Running sum interval
static volatile uint32_t isr_outlier_threshold = 0xFFFFFFFFUL; // Diset Motor_Process

static volatile uint32_t rpm = 0;
static volatile float precise_rpm = 0.0f;
static volatile uint32_t bldc_pulse_count = 0;
static uint32_t adc_raw = 0;
static uint32_t pwm_duty = 0;
static uint32_t set_value = 0;

// Default PID akan digunakan jika EEPROM kosong
static float pid_kp = 0.50f;
static float pid_ki = 0.60f;
static float pid_kd = 0.00f;
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;

static uint32_t last_capture = 0;
static uint32_t last_capture_time = 0;
static volatile uint32_t timer_overflow = 0;
static uint32_t overflow_count = 0;
static uint32_t last_extended = 0;

uint32_t Motor_GetRPM(void) {
    if (HAL_GetTick() - last_capture_time > 500) {
        rpm = 0;
        precise_rpm = 0.0f;
    }
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
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
}

MotorControlMode Motor_GetMode(void) {
    return control_mode;
}

void Motor_Process(void) {
    static uint32_t last_adc_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_adc_tick >= 10) {
        float dt = (now - last_adc_tick) / 1000.0f;
        if (dt <= 0.0f) dt = 0.01f;

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
                precise_rpm = 60000000.0f / (mean_interval * BLDC_PULSES_PER_MOTOR_REV);
                rpm = (uint32_t)(precise_rpm / GEARBOX_RATIO + 0.5f);

                // Update outlier threshold untuk ISR (dijalankan setiap 50ms, bukan di ISR)
                isr_outlier_threshold = (uint32_t)(mean_interval * 3.0f);
            }
        }

        // Baca ADC (polling, 1× per 50ms - sesuai untuk potensiometer)
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            adc_raw = HAL_ADC_GetValue(&hadc1);
        }

        last_adc_tick = now;

        if (control_mode == MOTOR_MODE_DIRECT) {
            // =====================================================
            // MODE DIRECT: Potensiometer langsung ke PWM (open loop)
            // ADC 0-4095 dipetakan langsung ke PWM 0-PWM_MAX_DUTY
            // =====================================================
            pwm_duty = (adc_raw * PWM_MAX_DUTY) / 4095;
            set_value = Motor_GetRPM(); // SET menampilkan RPM aktual saat ini
            pid_integral = 0.0f;
            pid_prev_error = 0.0f;

        } else {
            // =====================================================
            // MODE PID & CLI: Potensiometer / CLI -> Target RPM -> PID -> PWM
            // =====================================================
            if (control_mode == MOTOR_MODE_PID) {
                // Mode PID: ADC menentukan set_value (RPM target)
                // Hysteresis: hanya update set_value jika berubah >= 2 RPM
                // Mencegah SET terus loncat antara 2 nilai saat potensiometer di perbatasan
                uint32_t new_set = (adc_raw * 150) / 4095;
                if (new_set > 150) new_set = 150;
                int32_t diff = (int32_t)new_set - (int32_t)set_value;
                if (diff >= 2 || diff <= -2) {
                    set_value = new_set;
                }
            }
            // Mode CLI: set_value sudah di-set dari luar via Motor_SetCLITarget()

            // PID operates in Motor RPM domain (25x higher resolution)
            float target_motor_rpm = (float)(set_value * GEARBOX_RATIO);
            float current_motor_rpm = precise_rpm;

            // Integral Pre-load: saat target berubah signifikan, isi integral
            // langsung ke perkiraan nilai yang tepat agar motor tidak merangkak dari 0.
            // Berdasarkan data motor: PWM ≈ 0.60 × MRPM
            // Sehingga: integral = (target_motor_rpm × 0.60) / Ki
            static uint32_t prev_set_value_pid = 0;
            if (set_value != prev_set_value_pid) {
                int32_t sdiff = (int32_t)set_value - (int32_t)prev_set_value_pid;
                if (sdiff > 5 || sdiff < -5) {
                    // Pre-load integral ke estimasi steady-state
                    if (pid_ki > 0.001f) {
                        pid_integral = (target_motor_rpm * 0.60f) / pid_ki;
                    }
                    pid_prev_error = 0.0f;
                }
                prev_set_value_pid = set_value;
            }

            if (target_motor_rpm < 1.0f) {
                pwm_duty = 0;
                pid_integral = 0.0f;
                pid_prev_error = 0.0f;
            } else {
                float error = target_motor_rpm - current_motor_rpm;

                float derivative = (error - pid_prev_error) / dt;
                pid_prev_error = error;

                float p_term = pid_kp * error;
                float d_term = pid_kd * derivative;

                float i_term_tentative = pid_ki * (pid_integral + error * dt);

                float output = p_term + i_term_tentative + d_term;

                // Anti-windup (Conditional Integration)
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
    }
}

void Motor_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        uint32_t current = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t ovf = overflow_count;

        // Race condition: overflow flag pending tapi belum diservice
        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET && current < 32768) {
            ovf++;
        }

        uint32_t extended = (ovf * 65536UL) + current;
        uint32_t delta = extended - last_extended;

        // ============================================================
        // ATURAN ISR: TIDAK BOLEH ada float, division, atau operasi lambat!
        // ISR hanya boleh: integer add/subtract/compare, array write.
        // Semua komputasi berat (RPM, float) dilakukan di Motor_Process().
        // ============================================================

        // Noise filter: buang pulsa yang tidak mungkin secara fisik
        // Timer 1MHz, max 150 RPM output = 1000 motor pulse/sec = 1000µs/pulse
        // Threshold 800µs << 1000µs → aman, tidak membuang pulsa valid
        if (delta < 800) {
            return;
        }

        last_extended = extended;
        bldc_pulse_count++;

        // Outlier rejection: bandingkan delta dengan threshold
        if (isr_valid_intervals >= 10 && delta > isr_outlier_threshold) {
            last_capture_time = HAL_GetTick();
            return;
        }

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

void Motor_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        timer_overflow++;
        overflow_count++;
    }
}
