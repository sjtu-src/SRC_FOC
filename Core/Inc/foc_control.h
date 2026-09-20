#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H
#include "main.h"
#include "adc.h"
#include "tim.h"
#include <stdint.h>
#define FOC_PI 3.14159265358979323846f
#define FOC_2PI 6.2831853071795864769f
#define FOC_ENCODER_NOT_READY    (1UL << 0)
#define FOC_ENCODER_SIGNAL_ERROR (1UL << 1)
#define FOC_MOTOR_POLE_PAIRS     8.0f
#define FOC_STATE_IDLE          0U
#define FOC_STATE_CALIBRATING   1U
#define FOC_STATE_ALIGNING      2U
#define FOC_STATE_RUNNING       3U
#define FOC_STATE_FAULT         4U
#define FOC_FAULT_DRIVER        (1UL << 0)
#define FOC_FAULT_STARTUP       (1UL << 1)
#define FOC_FAULT_ENCODER       (1UL << 2)
#define FOC_FAULT_ALIGNMENT     (1UL << 3)
#define FOC_FAULT_OVERCURRENT   (1UL << 4)
#define FOC_FAULT_BUS_VOLTAGE   (1UL << 5)
#define LED_5V      1
#define LED_3V3     2

typedef enum
{
    FOC_MODE_TORQUE = 0U,
    FOC_MODE_SPEED = 1U
} FOC_ControlMode;

/* Unified command payload:
   - FOC_MODE_TORQUE: signed value in 1 mN*m/count.
   - FOC_MODE_SPEED:  signed value in 1 rpm/count. */
typedef struct
{
    FOC_ControlMode mode;
    int16_t value;
} FOC_CommandFrame;

void FOC_Init(void);
/* Apply a mode-dependent command payload. Returns 1 when MODE is valid. */
uint32_t FOC_ApplyCommandFrame(const FOC_CommandFrame *command);
void FOC_SetTorqueMilliNewtonMeter(float torque_mnm);
void FOC_SetSpeedRPM(float speed_rpm);
void FOC_PollDriverFault(void);
void Blink_LED(int led);
float FOC_GetAngle(void);
float FOC_GetSpeed(void);
float FOC_GetIq(void);
float FOC_GetId(void);
float FOC_GetBusVoltage(void);
/* 0..1 software I^2t usage and the corresponding present q-current ceiling. */
float FOC_GetCurrentThermalUtilization(void);
float FOC_GetDynamicIqLimit(void);
FOC_ControlMode FOC_GetControlMode(void);
/* Single-turn mechanical angle scaled to the legacy 0..16383 range. */
uint16_t FOC_GetEncoderRawAngle(void);
/* Zero means TIM3 A/B counting is active and no implausible edge burst exists. */
uint32_t FOC_GetEncoderStatus(void);
uint32_t FOC_GetState(void);
uint32_t FOC_GetFault(void);
#endif

