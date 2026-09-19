#include "foc_control.h"
#include "adc.h"
#include "gpio.h"
#include "DRV8353.h"
#include <math.h>

#define ADC_FS 4095.0f
#define SHUNT_OHM 0.001f
/* Uses DRV8353 SOA/SOB only. U15/U16 OUT pins must be disconnected because
   the schematic ties those INA240 outputs to the same nets. */
#define AMP_GAIN 40.0f
#define VREF 3.3f
#define DC_BUS_NOMINAL_V 16.0f
#define DC_BUS_DIVIDER_TOP_OHM 100000.0f
#define DC_BUS_DIVIDER_BOTTOM_OHM 7500.0f
#define DC_BUS_MIN_VALID_V 5.0f
#define DC_BUS_MAX_VALID_V 40.0f
#define PWM_PERIOD 2125U
#define MAX_MOD 0.90f
#define MAX_IQ_CURRENT 10.0f
#define PHASE_OVERCURRENT_LIMIT 10.0f
#define CURRENT_OUTPUT_LIMIT 0.8f
#define CURRENT_LOOP_HZ 20000.0f
#define IQ_SLEW_RATE_A_PER_S 30.0f
#define CURRENT_SENSE_BLANK_SAMPLES 20U
#define MOTOR_TORQUE_CONSTANT_MNM_PER_A 25.1f
#define MAX_SPEED_COMMAND_RPM 9000.0f
#define RPM_TO_RAD_PER_SEC (FOC_2PI / 60.0f)
#define SPEED_LOOP_HZ 1000.0f
#define SPEED_REFERENCE_SLEW_RPM_PER_S 3000.0f
#define SPEED_LOOP_CURRENT_LIMIT 2.0f
#define ALIGN_VOLTAGE 0.04f
/* MT6816 AB pulse count per mechanical revolution. TIM3 encoder mode counts
   four edges per pulse. Change this to 1000 for MT6816xx-AKD, etc. */
#ifndef MT6816_AB_PULSES_PER_REV
#define MT6816_AB_PULSES_PER_REV 1024U
#endif
#define ENCODER_COUNTS_PER_REV (4U * MT6816_AB_PULSES_PER_REV)
#define ENCODER_SPEED_SAMPLES 20U
/* More than 256 counts in 50 us would exceed 75 krpm at 4096 count/rev and
   is treated as an electrical glitch rather than real shaft motion. */
#define ENCODER_MAX_DELTA_PER_SAMPLE 256
#define ENCODER_MAX_CONSECUTIVE_GLITCHES 8U

typedef struct { float kp, ki, integ, out; } pi_t;
static volatile float angle, angle_multi, speed, iq, id, iq_ref_amp, iq_ref_target;
static volatile float speed_target_rad_s, speed_ref_rad_s;
static volatile float dc_bus_voltage = DC_BUS_NOMINAL_V;
static volatile float current_pi_bus_scale = 1.0f;
static volatile uint32_t dc_bus_voltage_valid;
static volatile FOC_ControlMode control_mode = FOC_MODE_TORQUE;
static volatile uint8_t enc_initialized;
static volatile uint16_t enc_raw;
static volatile uint32_t enc_status = FOC_ENCODER_NOT_READY;
static uint16_t enc_last_counter;
static volatile int32_t enc_position_counts;
static volatile int32_t enc_speed_counts;
static int32_t enc_speed_accumulator;
static uint32_t enc_speed_sample_count;
static uint32_t enc_consecutive_glitches;
volatile uint32_t encoder_bad_edge_count;
volatile uint32_t encoder_fault_status_snapshot, encoder_fault_age_ms;
static volatile uint32_t foc_state = FOC_STATE_IDLE, foc_fault;
static volatile uint32_t overcurrent_detail;
static volatile uint32_t current_sense_blank_samples;
/* Retained fault snapshot for the debugger. At 40 V/V and 1 milliohm,
   one ADC count is about 20.15 mA. */
volatile uint16_t foc_fault_adc_u, foc_fault_adc_v;
volatile float foc_fault_iu, foc_fault_iv, foc_fault_iw;
volatile float foc_current_offset_u, foc_current_offset_v;
volatile uint16_t drv_fault_status1_snapshot, drv_fault_status2_snapshot;
static volatile uint32_t offset_samples;
static float offset_u, offset_v;
static volatile float align_theta;
static float encoder_direction = 1.0f, electrical_offset;
/* 330285: Rll=0.464 ohm, Lll=0.322 mH. Conservative current-loop
   tuning for a 20 kHz update rate and approximately 16 V DC bus. */
static pi_t pi_q = {0.1f, 0.003f, 0, 0};
static pi_t pi_d = {0.1f, 0.003f, 0, 0};
/* 1-kHz mechanical speed loop. Output is q-axis current in amperes.
   kp unit: A/(rad/s); ki is the per-sample integral coefficient. */
static pi_t pi_speed = {0.02f, 0.00015f, 0, 0};

static float clamp(float x, float lo, float hi);

static uint32_t update_bus_voltage(void)
{
    uint32_t voltage_raw;
    float measured;

    if (HAL_ADC_Start(&hadc2) != HAL_OK) { return 0U; }

    /* ADC2 rank 1 is the temperature input; rank 2 is ADC_V. */
    if (HAL_ADC_PollForConversion(&hadc2, 2U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc2);
        return 0U;
    }
    (void)HAL_ADC_GetValue(&hadc2);

    if (HAL_ADC_PollForConversion(&hadc2, 2U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc2);
        return 0U;
    }
    voltage_raw = HAL_ADC_GetValue(&hadc2);
    (void)HAL_ADC_Stop(&hadc2);

    measured = (float)voltage_raw * VREF / ADC_FS *
               (DC_BUS_DIVIDER_TOP_OHM + DC_BUS_DIVIDER_BOTTOM_OHM) /
               DC_BUS_DIVIDER_BOTTOM_OHM;
    if (!isfinite(measured) || measured < DC_BUS_MIN_VALID_V ||
        measured > DC_BUS_MAX_VALID_V)
    {
        return 0U;
    }

    /* The first sample must take effect before PWM starts. Later samples are
       lightly filtered because this value only compensates slow bus changes. */
    if (!dc_bus_voltage_valid)
    {
        dc_bus_voltage = measured;
        dc_bus_voltage_valid = 1U;
    }
    else
    {
        dc_bus_voltage = 0.2f * measured + 0.8f * dc_bus_voltage;
    }
    current_pi_bus_scale = clamp(DC_BUS_NOMINAL_V / dc_bus_voltage, 0.4f, 1.5f);
    return 1U;
}

static void led_set(int led, uint32_t on)
{
    GPIO_TypeDef *port = led == LED_5V ? LED1_GPIO_Port : LED2_GPIO_Port;
    uint16_t pin = led == LED_5V ? LED1_Pin : LED2_Pin;
    /* Both LEDs are fed from their rail through a resistor; the MCU sinks
       current, so GPIO low turns the LED on. */
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Blink_LED(int led)
{
    if (led == LED_5V)
    {
        for(int i=0; i<5; i++)
        {
            led_set(LED_5V, 1U);
            HAL_Delay(200);
            led_set(LED_5V, 0U);
            HAL_Delay(200);
        }
    }
    else if (led == LED_3V3)
    {
        for(int i=0; i<5; i++)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(200);
            led_set(LED_3V3, 0U);
            HAL_Delay(200);
        }    
    }
}

static void Blink_DRV_Error(void)
{
    uint32_t reason = drv835x_debug_error;
    uint32_t stage = drv835x_debug_stage;
    if (reason < 1U || reason > 2U) { reason = 3U; }
    if (stage < 1U || stage > 13U) { stage = 14U; }

    /* Repeating code on the 3V3 LED:
       long flashes = error reason, then short flashes = failing stage. */
    while (1)
    {
        for (uint32_t i = 0U; i < reason; ++i)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(500U);
            led_set(LED_3V3, 0U);
            HAL_Delay(300U);
        }
        HAL_Delay(700U);
        for (uint32_t i = 0U; i < stage; ++i)
        {
            led_set(LED_3V3, 1U);
            HAL_Delay(120U);
            led_set(LED_3V3, 0U);
            HAL_Delay(180U);
        }
        HAL_Delay(1500U);
    }
}

static uint32_t Blink_GDF_Detail(void)
{
    uint32_t detail = 0U;
    DRV835X_read_FaultStatusReg1();
    DRV835X_read_FaultStatusReg2();

    if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 8))
    {
        uint16_t vgs = stru_DRV8353Obj.faultStatusReg2_obj.data & 0x003fU;
        if (vgs != 0U && (vgs & (vgs - 1U)) == 0U)
        {
            while ((vgs & 1U) == 0U) { ++detail; vgs >>= 1; }
            ++detail;
        }
        else if (vgs != 0U) { detail = 7U; }
        else { detail = 8U; }
    }

    for (uint32_t i = 0U; i < detail; ++i)
    {
        led_set(LED_3V3, 1U);
        HAL_Delay(180U);
        led_set(LED_3V3, 0U);
        HAL_Delay(220U);
    }
    return detail;
}

static void Blink_Alignment_Detail(float delta, float expected)
{
    uint32_t detail;
    float travel = fabsf(delta);

    /* Long 3V3 flashes after the five 5V flashes:
       1 = essentially no net movement, 2 = movement too small,
       3 = movement too large. A GDF code takes priority if present. */
    if (travel < 0.10f * expected) { detail = 1U; }
    else if (travel < 0.70f * expected) { detail = 2U; }
    else { detail = 3U; }

    for (uint32_t i = 0U; i < detail; ++i)
    {
        led_set(LED_3V3, 1U);
        HAL_Delay(500U);
        led_set(LED_3V3, 0U);
        HAL_Delay(350U);
    }
}

static void stop_fault(uint32_t fault)
{
    foc_fault |= fault;
    foc_state = FOC_STATE_FAULT;
    iq_ref_target = 0.0f;
    iq_ref_amp = 0.0f;
    speed_target_rad_s = 0.0f;
    speed_ref_rad_s = 0.0f;
    pi_speed.integ = 0.0f;
    pi_speed.out = 0.0f;
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
}

static uint32_t encoder_healthy(void)
{
    return enc_initialized && enc_consecutive_glitches < ENCODER_MAX_CONSECUTIVE_GLITCHES;
}

static void encoder_reset_position(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    enc_last_counter = 0U;
    enc_position_counts = 0;
    enc_speed_counts = 0;
    enc_speed_accumulator = 0;
    enc_speed_sample_count = 0U;
    enc_consecutive_glitches = 0U;
    enc_raw = 0U;
    angle = 0.0f;
    angle_multi = 0.0f;
    enc_status = 0U;
    enc_initialized = 1U;
}

/* Called synchronously with the 20-kHz current loop, like the TIGERs encoder
   capture path. Signed 16-bit subtraction handles TIM3 counter wrap. */
static void encoder_sample(void)
{
    uint16_t counter = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta = (int16_t)(counter - enc_last_counter);
    enc_last_counter = counter;

    if (delta > ENCODER_MAX_DELTA_PER_SAMPLE ||
        delta < -ENCODER_MAX_DELTA_PER_SAMPLE)
    {
        ++encoder_bad_edge_count;
        ++enc_consecutive_glitches;
        enc_status = FOC_ENCODER_SIGNAL_ERROR;
        return;
    }
    enc_consecutive_glitches = 0U;
    enc_status = 0U;
    enc_position_counts += delta;
    enc_speed_accumulator += delta;
    if (++enc_speed_sample_count >= ENCODER_SPEED_SAMPLES)
    {
        enc_speed_counts = enc_speed_accumulator;
        enc_speed_accumulator = 0;
        enc_speed_sample_count = 0U;
    }

    int32_t single_turn = enc_position_counts % (int32_t)ENCODER_COUNTS_PER_REV;
    if (single_turn < 0) { single_turn += (int32_t)ENCODER_COUNTS_PER_REV; }
    angle = FOC_2PI * (float)single_turn / (float)ENCODER_COUNTS_PER_REV;
    angle_multi = FOC_2PI * (float)enc_position_counts /
                  (float)ENCODER_COUNTS_PER_REV;
    enc_raw = (uint16_t)(((uint32_t)single_turn * 16384U) /
                         ENCODER_COUNTS_PER_REV);
}

static float clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static float wrap(float x){ while(x>FOC_PI)x-=FOC_2PI; while(x<-FOC_PI)x+=FOC_2PI; return x; }

static void pwm(float a,float b,float c){
 a=clamp(a,0,MAX_MOD); b=clamp(b,0,MAX_MOD); c=clamp(c,0,MAX_MOD);
 /* PWM2: high-side duty = 1 - CCR/ARR. All low sides conduct at CNT=0. */
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint32_t)((1.0f-a)*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint32_t)((1.0f-b)*PWM_PERIOD));
 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint32_t)((1.0f-c)*PWM_PERIOD));
}

static void svpwm(float theta,float vd,float vq)
{
    float al=vd*cosf(theta)-vq*sinf(theta),
            be=vd*sinf(theta)+vq*cosf(theta);
    float u_phase=al,
          v_phase=-0.5f*al+0.8660254f*be,
          w_phase=-0.5f*al-0.8660254f*be;

    /* Continuous, symmetric SVPWM. Injecting the same zero-sequence voltage
       into all three phases is equivalent to centering T0 in each PWM cycle. */
    float phase_max=fmaxf(u_phase,fmaxf(v_phase,w_phase));
    float phase_min=fminf(u_phase,fminf(v_phase,w_phase));
    float zero_sequence=-0.5f*(phase_max+phase_min);

    float u=0.5f+0.5f*(u_phase+zero_sequence),
          v=0.5f+0.5f*(v_phase+zero_sequence),
          w=0.5f+0.5f*(w_phase+zero_sequence);
    pwm(u,v,w);
}
void FOC_SetTorqueMilliNewtonMeter(float torque_mnm)
{
    float iq_command = isfinite(torque_mnm) ?
        torque_mnm / MOTOR_TORQUE_CONSTANT_MNM_PER_A : 0.0f;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (control_mode != FOC_MODE_TORQUE)
    {
        pi_speed.integ = 0.0f;
        pi_speed.out = 0.0f;
    }
    control_mode = FOC_MODE_TORQUE;
    speed_target_rad_s = 0.0f;
    speed_ref_rad_s = 0.0f;
    iq_ref_target = clamp(iq_command, -MAX_IQ_CURRENT, MAX_IQ_CURRENT);
    __set_PRIMASK(primask);
}

void FOC_SetSpeedRPM(float speed_rpm)
{
    float limited_rpm = isfinite(speed_rpm) ?
        clamp(speed_rpm, -MAX_SPEED_COMMAND_RPM, MAX_SPEED_COMMAND_RPM) : 0.0f;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (control_mode != FOC_MODE_SPEED)
    {
        pi_speed.integ = 0.0f;
        pi_speed.out = 0.0f;
        iq_ref_target = 0.0f;
        /* Start the command ramp from the actual mechanical speed so that a
           mode change while rotating does not first command an abrupt brake. */
        speed_ref_rad_s = encoder_direction * speed;
    }
    speed_target_rad_s = limited_rpm * RPM_TO_RAD_PER_SEC;
    control_mode = FOC_MODE_SPEED;
    __set_PRIMASK(primask);
}

uint32_t FOC_ApplyCommandFrame(const FOC_CommandFrame *command)
{
    if (command == NULL) { return 0U; }
    if (command->mode == FOC_MODE_TORQUE)
    {
        FOC_SetTorqueMilliNewtonMeter((float)command->value);
        return 1U;
    }
    if (command->mode == FOC_MODE_SPEED)
    {
        FOC_SetSpeedRPM((float)command->value);
        return 1U;
    }
    return 0U;
}

void FOC_PollDriverFault(void)
{
    static uint32_t fault_reported;
    uint32_t detail = 0U;
    (void)update_bus_voltage();
    if (fault_reported) { return; }

    if (foc_state == FOC_STATE_FAULT)
    {
        fault_reported = 1U;
        /* Four quick flashes identify a controller-state fault, followed by:
           1=startup, 2=encoder, 3=alignment,
           4=IU positive, 5=IU negative, 6=IV positive, 7=IV negative,
           8=IW positive, 9=IW negative overcurrent, 10=unknown overcurrent,
           11=driver initialization/other driver fault. */
        if (foc_fault & FOC_FAULT_STARTUP) { detail = 1U; }
        else if (foc_fault & FOC_FAULT_ENCODER) { detail = 2U; }
        else if (foc_fault & FOC_FAULT_ALIGNMENT) { detail = 3U; }
        else if (foc_fault & FOC_FAULT_OVERCURRENT)
        {
            detail = overcurrent_detail ? overcurrent_detail : 10U;
        }
        else { detail = 11U; }

        while (1)
        {
            led_set(LED_3V3, 0U);
            HAL_Delay(1200U);
            for (uint32_t preamble = 0U; preamble < 4U; ++preamble)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(100U);
                led_set(LED_3V3, 0U);
                HAL_Delay(150U);
            }
            HAL_Delay(700U);
            for (uint32_t i = 0U; i < detail; ++i)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(400U);
                led_set(LED_3V3, 0U);
                HAL_Delay(300U);
            }
            HAL_Delay(1800U);
        }
    }

    if (foc_state != FOC_STATE_RUNNING) { return; }

    DRV835X_read_FaultStatusReg1();
    DRV835X_read_FaultStatusReg2();
    if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 10))
    {
        fault_reported = 1U;
        drv_fault_status1_snapshot = stru_DRV8353Obj.faultStatusReg1_obj.data;
        drv_fault_status2_snapshot = stru_DRV8353Obj.faultStatusReg2_obj.data;
        stop_fault(FOC_FAULT_DRIVER);
        /* Solid 5V LED means a runtime DRV8353 fault latched the bridge off. */
        led_set(LED_5V, 1U);

        /* For VDS overcurrent, slow 3V3 flashes identify the MOSFET:
           1=LC, 2=HC, 3=LB, 4=HB, 5=LA, 6=HA, 7=multiple/aggregate.
           Other fault classes use 8=GDF, 9=UVLO, 10=OTSD,
           11=shunt-sense OCP, 12=gate-drive UV, 13=other. */
        if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 9))
        {
            uint16_t vds = stru_DRV8353Obj.faultStatusReg1_obj.data & 0x003fU;
            if (vds != 0U && (vds & (vds - 1U)) == 0U)
            {
                while ((vds & 1U) == 0U) { ++detail; vds >>= 1; }
                ++detail;
            }
            else { detail = 7U; }
        }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 8)) { detail = 8U; }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 7)) { detail = 9U; }
        else if (stru_DRV8353Obj.faultStatusReg1_obj.data & (1U << 6)) { detail = 10U; }
        else if (stru_DRV8353Obj.faultStatusReg2_obj.data & 0x0700U) { detail = 11U; }
        else if (stru_DRV8353Obj.faultStatusReg2_obj.data & (1U << 6)) { detail = 12U; }
        else { detail = 13U; }

        /* Repeat forever so a power-on LED transient cannot be mistaken for
           the diagnostic. Three quick flashes mark the start of every code. */
        while (1)
        {
            led_set(LED_3V3, 0U);
            HAL_Delay(1200U);
            for (uint32_t preamble = 0U; preamble < 3U; ++preamble)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(100U);
                led_set(LED_3V3, 0U);
                HAL_Delay(150U);
            }
            HAL_Delay(700U);
            for (uint32_t i = 0U; i < detail; ++i)
            {
                led_set(LED_3V3, 1U);
                HAL_Delay(400U);
                led_set(LED_3V3, 0U);
                HAL_Delay(300U);
            }
            HAL_Delay(1800U);
        }
    }
}

/* Initialization runs once in main; interrupts continue during these waits. */
static uint32_t align_wait(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms)
    {
        if (foc_state == FOC_STATE_FAULT) { return 0U; }
        if (!encoder_healthy())
        {
            stop_fault(FOC_FAULT_ENCODER);
            return 0U;
        }
        HAL_Delay(1U);
    }
    return 1U;
}

void FOC_Init(void)
{
    /* ADC_V keeps the current-loop dynamics independent of supply voltage.
       Sample it before PWM can energize the motor. */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
        !update_bus_voltage())
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    if (DRV835X_Init() != HAL_OK)
    { 
        stop_fault(FOC_FAULT_DRIVER);
        Blink_DRV_Error();
    }
    pwm(0.5f,0.5f,0.5f);
    /* Load all CCR preloads and RCR=3 before starting from CNT=0.
       With 40 kHz center-aligned PWM, update/TRGO runs at 20 kHz. */
    htim1.Instance->EGR = TIM_EGR_UG;
    foc_state = FOC_STATE_CALIBRATING;
    if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    encoder_reset_position();
    /* Gather real CSA offsets with INL held low, not an assumed mid-scale. */
    uint32_t start = HAL_GetTick();
    while (offset_samples < 128U)
    {
        if (HAL_GetTick() - start >= 500U)
        {
            stop_fault(FOC_FAULT_STARTUP);
            return;
        }
        HAL_Delay(1U);
    }
    /* A/B alone has no absolute power-up position. Align on every boot, then
       use the measured count sign and final position to establish Park angle. */
    align_theta = 0.0f;
    current_sense_blank_samples = CURRENT_SENSE_BLANK_SAMPLES;
    foc_state = FOC_STATE_ALIGNING;
    if (!align_wait(800U)) { return; }
    int32_t initial_counts = enc_position_counts;
    for (uint32_t step = 1U; step <= 1500U; ++step)
    {
        align_theta = FOC_PI * (float)step / 1500.0f;
        if (!align_wait(1U)) { return; }
    }
    if (!align_wait(500U)) { return; }
    int32_t delta_counts = enc_position_counts - initial_counts;
    float delta = FOC_2PI * (float)delta_counts /
                  (float)ENCODER_COUNTS_PER_REV;
    float expected = FOC_PI / FOC_MOTOR_POLE_PAIRS;
    if (fabsf(delta) < 0.25f*expected)
    {
        stop_fault(FOC_FAULT_ALIGNMENT);
        Blink_LED(LED_5V);
        HAL_Delay(700U);
        if (Blink_GDF_Detail() == 0U)
        {
            HAL_Delay(700U);
            Blink_Alignment_Detail(delta, expected);
        }
        return;
    }
    encoder_direction = delta_counts > 0 ? 1.0f : -1.0f;
    electrical_offset = wrap(encoder_direction * angle * FOC_MOTOR_POLE_PAIRS - FOC_PI);
    /* Drop INL while publishing the aligned angle and resetting every PI. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_RESET);
    iq_ref_amp = 0.0f;
    iq_ref_target = 0.0f;
    speed_target_rad_s = 0.0f;
    speed_ref_rad_s = 0.0f;
    current_sense_blank_samples = CURRENT_SENSE_BLANK_SAMPLES;
    pi_q.integ = 0.0f;
    pi_d.integ = 0.0f;
    pi_speed.integ = 0.0f;
    pi_speed.out = 0.0f;
    if (foc_state != FOC_STATE_FAULT) { foc_state = FOC_STATE_RUNNING; }
    __set_PRIMASK(primask);
}
float FOC_GetAngle(void){return angle_multi;}
float FOC_GetSpeed(void){return encoder_direction*speed;}
float FOC_GetIq(void){return iq;}
float FOC_GetId(void){return id;}
float FOC_GetBusVoltage(void){return dc_bus_voltage;}
FOC_ControlMode FOC_GetControlMode(void){return control_mode;}
uint16_t FOC_GetEncoderRawAngle(void){return enc_raw;}
uint32_t FOC_GetEncoderStatus(void){return enc_status;}
uint32_t FOC_GetState(void){return foc_state;}
uint32_t FOC_GetFault(void){return foc_fault;}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if(h->Instance==TIM5)
        {
            if (!encoder_healthy()) { speed=0.0f; return; }
            float instant_speed = (float)enc_speed_counts * FOC_2PI *
                                  SPEED_LOOP_HZ /
                                  (float)ENCODER_COUNTS_PER_REV;
            speed=0.15f*instant_speed+0.85f*speed;

            if (foc_state == FOC_STATE_RUNNING && control_mode == FOC_MODE_SPEED)
            {
                const float speed_step = SPEED_REFERENCE_SLEW_RPM_PER_S *
                                         RPM_TO_RAD_PER_SEC / SPEED_LOOP_HZ;
                if (speed_ref_rad_s < speed_target_rad_s)
                {
                    speed_ref_rad_s = fminf(speed_ref_rad_s + speed_step,
                                            speed_target_rad_s);
                }
                else if (speed_ref_rad_s > speed_target_rad_s)
                {
                    speed_ref_rad_s = fmaxf(speed_ref_rad_s - speed_step,
                                            speed_target_rad_s);
                }

                float speed_error = speed_ref_rad_s - encoder_direction * speed;
                float proportional = pi_speed.kp * speed_error;
                float integral_candidate = clamp(pi_speed.integ +
                                                   pi_speed.ki * speed_error,
                                                   -SPEED_LOOP_CURRENT_LIMIT,
                                                   SPEED_LOOP_CURRENT_LIMIT);
                float output_candidate = proportional + integral_candidate;

                /* Do not integrate farther into current saturation. Integration
                   remains active when the error helps the output leave it. */
                if (!((output_candidate > SPEED_LOOP_CURRENT_LIMIT && speed_error > 0.0f) ||
                      (output_candidate < -SPEED_LOOP_CURRENT_LIMIT && speed_error < 0.0f)))
                {
                    pi_speed.integ = integral_candidate;
                }
                pi_speed.out = clamp(proportional + pi_speed.integ,
                                     -SPEED_LOOP_CURRENT_LIMIT,
                                     SPEED_LOOP_CURRENT_LIMIT);
                iq_ref_target = pi_speed.out;
            }
        }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h)
{
    static uint32_t control_divider;
    static uint32_t overcurrent_count;
    if(h->Instance!=ADC1)  return;

    // calibrate ADC
    if (foc_state == FOC_STATE_CALIBRATING)
    {
        if (offset_samples < 128U)
        {
            offset_u += (float)h->Instance->JDR1;
            offset_v += (float)h->Instance->JDR2;
            if (offset_samples == 127U)
            {
                offset_u /= 128.0f;
                offset_v /= 128.0f;
                foc_current_offset_u = offset_u;
                foc_current_offset_v = offset_v;
            }
            ++offset_samples;
        }
        return;
    }

    /* Run Clarke/Park and the PI controllers at 20 kHz. */
    if (++control_divider < 2U) { return; }
    control_divider = 0U;

    encoder_sample();

    if (foc_state != FOC_STATE_ALIGNING && foc_state != FOC_STATE_RUNNING) return;

    if (!encoder_healthy()) 
    { 
        encoder_fault_status_snapshot = enc_status;
        encoder_fault_age_ms = enc_consecutive_glitches;
        stop_fault(FOC_FAULT_ENCODER); 
        return; 
    }

    /* With U15/U16 removed, PA0/PA1 are driven only by the DRV8353 internal
       CSAs. The measured response under sufficient voltage shows that the
       direct SPA-SNA polarity closes the current loop as negative feedback. */

    float iu=VREF*((float)h->Instance->JDR1-offset_u)/(ADC_FS*SHUNT_OHM*AMP_GAIN);
    float iv=VREF*((float)h->Instance->JDR2-offset_v)/(ADC_FS*SHUNT_OHM*AMP_GAIN);

    float iw=-iu-iv;
    if (current_sense_blank_samples > 0U)
    {
        --current_sense_blank_samples;
        overcurrent_count = 0U;
        pwm(0.5f,0.5f,0.5f);
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
        return;
    }

    if (fabsf(iu) > PHASE_OVERCURRENT_LIMIT ||
        fabsf(iv) > PHASE_OVERCURRENT_LIMIT ||
        fabsf(iw) > PHASE_OVERCURRENT_LIMIT)
    {
        if (iu > PHASE_OVERCURRENT_LIMIT) { overcurrent_detail = 4U; }
        else if (iu < -PHASE_OVERCURRENT_LIMIT) { overcurrent_detail = 5U; }
        else if (iv > PHASE_OVERCURRENT_LIMIT) { overcurrent_detail = 6U; }
        else if (iv < -PHASE_OVERCURRENT_LIMIT) { overcurrent_detail = 7U; }
        else if (iw > PHASE_OVERCURRENT_LIMIT) { overcurrent_detail = 8U; }
        else { overcurrent_detail = 9U; }
        /* Reject an isolated PWM-edge sample; three consecutive samples are
           only 150 us at the 20 kHz loop rate. Hardware OCP remains immediate. */
        if (++overcurrent_count >= 30U)
        {
            foc_fault_adc_u = (uint16_t)h->Instance->JDR1;
            foc_fault_adc_v = (uint16_t)h->Instance->JDR2;
            foc_fault_iu = iu;
            foc_fault_iv = iv;
            foc_fault_iw = iw;
            stop_fault(FOC_FAULT_OVERCURRENT);
            led_set(LED_3V3, 1U);
            return;
        }
    }
    else { overcurrent_count = 0U; }

    /* Amplitude-invariant Clarke transform for two-shunt sampling.
       With iw=-iu-iv, this is the reduced form of the full 2/3 transform. */
    float i_alpha=iu;
    float i_beta=0.5773502692f*(iu+2.0f*iv);

    uint32_t aligning = foc_state == FOC_STATE_ALIGNING;
    float theta = aligning ? align_theta :
        wrap(encoder_direction*angle*FOC_MOTOR_POLE_PAIRS-electrical_offset);
    float c=cosf(theta), s=sinf(theta);

    /* Park transform. This sign convention is the inverse of svpwm() above. */
    id=i_alpha*c+i_beta*s;
    iq=-i_alpha*s+i_beta*c;

    if (aligning)
    {
        svpwm(theta, ALIGN_VOLTAGE * current_pi_bus_scale, 0.0f);
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
        return;
    }

    float iq_step=IQ_SLEW_RATE_A_PER_S/CURRENT_LOOP_HZ;
    if (iq_ref_amp < iq_ref_target)
    {
        iq_ref_amp=fminf(iq_ref_amp+iq_step,iq_ref_target);
    }
    else if (iq_ref_amp > iq_ref_target)
    {
        iq_ref_amp=fmaxf(iq_ref_amp-iq_step,iq_ref_target);
    }

    float eq=iq_ref_amp-iq;
    float ed=-id;

    // PI control
    /* PI gains were tuned at DC_BUS_NOMINAL_V. Since PI output is normalized
       PWM modulation, scale it inversely with the measured DC bus voltage. */
    float bus_scale = current_pi_bus_scale;
    float q_proportional=pi_q.kp*eq*bus_scale;
    float d_proportional=pi_d.kp*ed*bus_scale;
    pi_q.integ=clamp(pi_q.integ+pi_q.ki*eq*bus_scale,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_q.out=clamp(q_proportional+pi_q.integ,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    pi_d.integ=clamp(pi_d.integ+pi_d.ki*ed*bus_scale,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_d.out=clamp(d_proportional+pi_d.integ,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    /* Limit the combined voltage vector, not each axis independently. This
       keeps the minimum low-side conduction window available for ADC ranks. */
    float voltage_sq=pi_d.out*pi_d.out+pi_q.out*pi_q.out;
    if (voltage_sq > CURRENT_OUTPUT_LIMIT*CURRENT_OUTPUT_LIMIT)
    {
        float scale=CURRENT_OUTPUT_LIMIT/sqrtf(voltage_sq);
        pi_d.out*=scale;
        pi_q.out*=scale;
    }

    /* Back-calculate both integrators from the voltage actually applied.
       Without this, vector limiting leaves a hidden saturated integrator and
       can drive the phase current past the target when it unwinds. */
    pi_q.integ=clamp(pi_q.out-q_proportional,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_d.integ=clamp(pi_d.out-d_proportional,-CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    svpwm(theta,pi_d.out,pi_q.out);
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
}
