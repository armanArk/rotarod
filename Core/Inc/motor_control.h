#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "main.h"

// Control Mode
typedef enum {
    MOTOR_MODE_PID    = 0,  // Potentiometer -> Target RPM -> PID -> PWM
    MOTOR_MODE_DIRECT = 1,  // Potentiometer -> PWM directly (open loop)
    MOTOR_MODE_CLI    = 2,  // CLI sets RPM target directly, PID still active
} MotorControlMode;

void Motor_SetMode(MotorControlMode mode);
MotorControlMode Motor_GetMode(void);
// Motor & Sensor getters
uint32_t Motor_GetRPM(void);
uint32_t Motor_GetADC(void);
uint32_t Motor_GetPWMDuty(void);
uint32_t Motor_GetSetValue(void);
uint32_t Motor_GetPulseCount(void);
uint32_t Motor_GetOutputRevolutions(void);
float    Motor_GetPreciseMotorRPM(void);  // Motor RPM float (sebelum gearbox)
float    Motor_GetPreciseOutputRPM(void); // Output RPM float presisi dengan 1 desimal

// PID Control
void Motor_SetPID(float kp, float ki, float kd);
void Motor_GetPID(float *kp, float *ki, float *kd);
void Motor_GetPIDState(float *error, float *integral);

// Call periodically in main loop (e.g. every tick, it handles its own 50ms interval)
void Motor_Process(void);

// TIM Callbacks are now standard HAL callbacks

// Add an event from the UI button (simulate fall)
void Motor_ForceRPMReset(void);

// CLI Mode: set target RPM directly (only active in MOTOR_MODE_CLI)
void Motor_SetCLITarget(uint32_t rpm_target);

#endif // MOTOR_CONTROL_H
