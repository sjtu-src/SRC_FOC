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
#define DC_BUS_RUNTIME_FAULT_SAMPLES 100U
#define DC_BUS_STARTUP_MIN_V 9.0f
#define DC_BUS_STARTUP_TIMEOUT_MS 10000U
#define DC_BUS_STARTUP_STABLE_SAMPLES 20U
#define DC_BUS_STARTUP_STABLE_RANGE_V 0.15f
#define PWM_PERIOD 2125U
#define MAX_MOD 0.90f
#define MAX_IQ_CURRENT 9.0f
/* Attached 18-V winding data: allow TIGERs-like short torque bursts while
   returning to the motor's approximately 3.5-A continuous rating thermally. */
#define CONTINUOUS_IQ_CURRENT 3.5f
#define CURRENT_THERMAL_TIME_CONSTANT_S 17.7f
#define CURRENT_THERMAL_TAPER_START 0.80f
/* The hard path behaves like the TIGERs comparator/OCREF path: an observed
   peak first suppresses the next voltage vector and is allowed to recover.
   Only current which remains excessive while zero voltage is being applied
   is treated as a latched fault. The lower threshold catches a real sustained
   control/sampling failure without reducing the requested 9-A torque current. */
#define PHASE_SUSTAINED_CURRENT_LIMIT 11.0f
#define PHASE_HARD_OVERCURRENT_LIMIT 15.0f
#define PHASE_HARD_OVERCURRENT_SAMPLES 40U
#define PHASE_SUSTAINED_OVERCURRENT_SAMPLES 400U
#define PHASE_SUSTAINED_RECOVERY_PER_SAMPLE 4U
#define CURRENT_OUTPUT_LIMIT 0.88f
#define CURRENT_LOOP_HZ 20000.0f
#define IQ_SLEW_RATE_A_PER_S 300.0f
#define SPEED_IQ_SLEW_RATE_A_PER_S 2000.0f
#define CURRENT_SENSE_BLANK_SAMPLES 20U
#define MOTOR_TORQUE_CONSTANT_MNM_PER_A 25.1f
#define MAX_SPEED_COMMAND_RPM 9000.0f
#define RPM_TO_RAD_PER_SEC (FOC_2PI / 60.0f)
#define SPEED_LOOP_HZ 1000.0f
#define SPEED_LOOP_CURRENT_LIMIT 9.0f
#define ALIGN_VOLTAGE 0.04f
/* MT6816 AB pulse count per mechanical revolution. TIM3 encoder mode counts
   four edges per pulse. Change this to 1000 for MT6816xx-AKD, etc. */
#ifndef MT6816_AB_PULSES_PER_REV
#define MT6816_AB_PULSES_PER_REV 1024U
#endif
#define ENCODER_COUNTS_PER_REV (4U * MT6816_AB_PULSES_PER_REV)
#define ENCODER_SPEED_SAMPLES 20U
/* At 4096 count/rev, 10000 rpm is only 34 counts per 50 us. Keep generous
   overspeed margin while rejecting angle jumps large enough to upset Park. */
#define ENCODER_MAX_DELTA_PER_SAMPLE 64
#define ENCODER_MAX_CONSECUTIVE_GLITCHES 8U

typedef struct { float kp, ki, integ, out; } pi_t;
static volatile float angle, angle_multi, speed, iq, id, iq_ref_amp, iq_ref_target;
static volatile float speed_target_rad_s;
static volatile float dc_bus_voltage = DC_BUS_NOMINAL_V;
static volatile float current_pi_bus_scale = 1.0f;
volatile float foc_dc_bus_voltage_raw;
volatile uint32_t foc_bus_voltage_read_failures;
volatile uint32_t foc_bus_voltage_consecutive_failures;
/* Latched most-recent bus-voltage failure:
   0=no failure since boot, 1=ADC start failed, 2=ADC_V timeout,
   4=non-finite conversion, 5=below 5 V, 6=above 40 V,
   7=bus did not become stable before the startup timeout. */
volatile uint32_t foc_bus_voltage_last_error;
volatile uint32_t foc_bus_voltage_adc_raw;
volatile float foc_current_thermal_utilization;
volatile float foc_dynamic_iq_limit = MAX_IQ_CURRENT;
static volatile uint32_t dc_bus_voltage_valid;
static volatile uint32_t adc2_injected_started;
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
static volatile uint32_t overcurrent_trigger_type;
static volatile uint32_t current_sense_blank_samples;
/* Retained fault snapshot for the debugger. At 40 V/V and 1 milliohm,
   one ADC count is about 20.15 mA. */
volatile uint16_t foc_fault_adc_u, foc_fault_adc_v;
volatile float foc_fault_iu, foc_fault_iv, foc_fault_iw;
volatile uint32_t foc_fault_hard_overcurrent_samples;
volatile uint32_t foc_fault_sustained_overcurrent_samples;
volatile float foc_fault_dynamic_iq_limit, foc_fault_thermal_utilization;
volatile float foc_fault_bus_voltage;
volatile float foc_fault_bus_voltage_raw;
volatile float foc_fault_iq_ref, foc_fault_id, foc_fault_iq;
volatile float foc_fault_vd, foc_fault_vq;
volatile float foc_current_offset_u, foc_current_offset_v;
volatile uint16_t drv_fault_status1_snapshot, drv_fault_status2_snapshot;
volatile uint32_t drv_vds_ocp_event_count;
static volatile uint32_t offset_samples;
static float offset_u, offset_v;
static volatile float align_theta;
static float encoder_direction = 1.0f, electrical_offset;
/* Current PI output is normalized SVPWM modulation. Gains are tuned at the
   16-V nominal bus and compensated by current_pi_bus_scale at run time. */
static pi_t pi_q = {0.12f, 0.005f, 0, 0};
static pi_t pi_d = {0.12f, 0.005f, 0, 0};
/* 1-kHz mechanical speed loop. Output is q-axis current in amperes.
   kp unit: A/(rad/s); ki is the per-sample integral coefficient. */
static pi_t pi_speed = {0.15f, 0.008f, 0, 0};

static float clamp(float x, float lo, float hi);

static uint32_t accept_bus_voltage_raw(uint32_t voltage_raw)
{
    float measured;

    foc_bus_voltage_adc_raw = voltage_raw;
    measured = (float)voltage_raw * VREF / ADC_FS *
               (DC_BUS_DIVIDER_TOP_OHM + DC_BUS_DIVIDER_BOTTOM_OHM) /
               DC_BUS_DIVIDER_BOTTOM_OHM;
    foc_dc_bus_voltage_raw = measured;
    if (!isfinite(measured))
    {
        foc_bus_voltage_last_error = 4U;
        return 0U;
    }
    if (measured < DC_BUS_MIN_VALID_V)
    {
        foc_bus_voltage_last_error = 5U;
        return 0U;
    }
    if (measured > DC_BUS_MAX_VALID_V)
    {
        foc_bus_voltage_last_error = 6U;
        return 0U;
    }

    if (!dc_bus_voltage_valid)
    {
        dc_bus_voltage = measured;
        dc_bus_voltage_valid = 1U;
    }
    else
    {
        dc_bus_voltage = 0.2f * measured + 0.8f * dc_bus_voltage;
    }
    current_pi_bus_scale = clamp(DC_BUS_NOMINAL_V / dc_bus_voltage,
                                 0.4f, 1.5f);
    return 1U;
}

static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

static void update_current_thermal_limit(float current_magnitude_sq)
{
    const float continuous_sq = CONTINUOUS_IQ_CURRENT * CONTINUOUS_IQ_CURRENT;
    const float dt = 1.0f / CURRENT_LOOP_HZ;
    float normalized_loss = current_magnitude_sq / continuous_sq;

    /* First-order winding temperature estimate. A value of 1 corresponds to
       the steady-state temperature rise at the continuous current rating. */
    foc_current_thermal_utilization +=
        (normalized_loss - foc_current_thermal_utilization) * dt /
        CURRENT_THERMAL_TIME_CONSTANT_S;
    foc_current_thermal_utilization = clamp(foc_current_thermal_utilization,
                                            0.0f, 1.0f);

    if (foc_current_thermal_utilization <= CURRENT_THERMAL_TAPER_START)
    {
        foc_dynamic_iq_limit = MAX_IQ_CURRENT;
    }
    else
    {
        float taper = (1.0f - foc_current_thermal_utilization) /
                      (1.0f - CURRENT_THERMAL_TAPER_START);
        foc_dynamic_iq_limit = CONTINUOUS_IQ_CURRENT +
                               taper * (MAX_IQ_CURRENT - CONTINUOUS_IQ_CURRENT);
    }
}

static uint32_t update_bus_voltage(void)
{
    uint32_t voltage_raw;

    /* ADC2 becomes the simultaneous V-current slave once PWM starts. Its bus
       voltage was already measured over a stable 200-ms startup window, so
       retain that value for this run. A different supply is measured on the
       next power-up/reset without disturbing current sampling. */
    if (adc2_injected_started)
    {
        return dc_bus_voltage_valid;
    }

    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
        foc_bus_voltage_last_error = 1U;
        return 0U;
    }

    /* ADC2 has one regular channel only: PA4/ADC2_IN17 (ADC_V). Keeping
       temperature out of this sequence prevents scan-rank misalignment. */
    if (HAL_ADC_PollForConversion(&hadc2, 2U) != HAL_OK)
    {
        if (!adc2_injected_started) { (void)HAL_ADC_Stop(&hadc2); }
        foc_bus_voltage_last_error = 2U;
        return 0U;
    }
    voltage_raw = HAL_ADC_GetValue(&hadc2);
    (void)HAL_ADC_Stop(&hadc2);
    return accept_bus_voltage_raw(voltage_raw);
}

static uint32_t wait_for_stable_bus_voltage(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t block_samples = 0U;
    float block_min = DC_BUS_MAX_VALID_V;
    float block_max = 0.0f;

    while ((HAL_GetTick() - start_ms) < DC_BUS_STARTUP_TIMEOUT_MS)
    {
        if (update_bus_voltage() &&
            foc_dc_bus_voltage_raw >= DC_BUS_STARTUP_MIN_V)
        {
            block_min = fminf(block_min, foc_dc_bus_voltage_raw);
            block_max = fmaxf(block_max, foc_dc_bus_voltage_raw);
            if (++block_samples >= DC_BUS_STARTUP_STABLE_SAMPLES)
            {
                /* Require a 200-ms block whose total variation is small.
                   This lets a capacitor bank charge before enabling PWM. */
                if ((block_max - block_min) <=
                    DC_BUS_STARTUP_STABLE_RANGE_V)
                {
                    return 1U;
                }
                block_samples = 0U;
                block_min = DC_BUS_MAX_VALID_V;
                block_max = 0.0f;
            }
        }
        else
        {
            block_samples = 0U;
            block_min = DC_BUS_MAX_VALID_V;
            block_max = 0.0f;
        }
        HAL_Delay(10U);
    }

    foc_bus_voltage_last_error = 7U;
    return 0U;
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
    static uint32_t consecutive_vds_fault_polls;
    uint32_t detail = 0U;
    uint32_t preamble_flashes = 4U;
    if (update_bus_voltage())
    {
        foc_bus_voltage_consecutive_failures = 0U;
    }
    else
    {
        ++foc_bus_voltage_read_failures;
        ++foc_bus_voltage_consecutive_failures;
    }

    /* A single conversion failure is harmless because the last valid bus
       value remains available. Stop only after about one second of continuous
       failures, which indicates a real ADC/range problem rather than noise. */
    if (foc_bus_voltage_consecutive_failures >=
            DC_BUS_RUNTIME_FAULT_SAMPLES &&
        foc_state == FOC_STATE_RUNNING)
    {
        foc_fault_bus_voltage = foc_dc_bus_voltage_raw;
        stop_fault(FOC_FAULT_BUS_VOLTAGE);
    }
    if (fault_reported) { return; }

    if (foc_state == FOC_STATE_FAULT)
    {
        fault_reported = 1U;
        /* Four quick flashes identify a non-current controller fault.
           Software overcurrent uses five quick flashes for a hard peak or
           six quick flashes for a sustained overload. A DC-bus fault uses
           seven quick flashes, then 1..7 slow flashes for its ADC/range
           error code. Other slow-flash details identify:
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
            /* Five quick flashes mean >15 A persisted for about 2 ms despite
               cycle-by-cycle zero-vector limiting.
               Six quick flashes mean >11 A accumulated for about 20 ms. */
            preamble_flashes = overcurrent_trigger_type == 1U ? 5U : 6U;
        }
        else if (foc_fault & FOC_FAULT_BUS_VOLTAGE)
        {
            preamble_flashes = 7U;
            detail = (foc_bus_voltage_last_error >= 1U &&
                      foc_bus_voltage_last_error <= 7U) ?
                     foc_bus_voltage_last_error : 7U;
        }
        else { detail = 11U; }

        while (1)
        {
            led_set(LED_3V3, 0U);
            HAL_Delay(1200U);
            for (uint32_t preamble = 0U;
                 preamble < preamble_flashes;
                 ++preamble)
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
        uint16_t status1 = stru_DRV8353Obj.faultStatusReg1_obj.data;
        uint16_t status2 = stru_DRV8353Obj.faultStatusReg2_obj.data;
        uint32_t vds_only =
            (status1 & (1U << 9)) != 0U &&
            (status1 & ((1U << 6) | (1U << 7) | (1U << 8))) == 0U &&
            (status2 & 0x07ffU) == 0U;

        drv_fault_status1_snapshot = status1;
        drv_fault_status2_snapshot = status2;

        /* OCP_RETRY removes the bridge drive for 8 ms and then tries again.
           A single obstruction/ringing event must not permanently stop the
           speed loop. Three consecutive 10-ms polls still identify a real
           persistent power-stage fault and take the normal latched path. */
        if (vds_only && ++consecutive_vds_fault_polls < 3U)
        {
            ++drv_vds_ocp_event_count;
            return;
        }

        consecutive_vds_fault_polls = 0U;
        fault_reported = 1U;
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
    else
    {
        consecutive_vds_fault_polls = 0U;
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
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    if (!wait_for_stable_bus_voltage())
    {
        foc_fault_bus_voltage = foc_dc_bus_voltage_raw;
        stop_fault(FOC_FAULT_BUS_VOLTAGE);
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
        HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
    {
        stop_fault(FOC_FAULT_STARTUP);
        return;
    }
    /* Only ADC1/master raises the 20-kHz control interrupt. ADC2/slave still
       converts simultaneously and its JDR1 is read from the ADC1 callback. */
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC | ADC_IT_JEOS);
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
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
    adc2_injected_started = 1U;
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
float FOC_GetCurrentThermalUtilization(void){return foc_current_thermal_utilization;}
float FOC_GetDynamicIqLimit(void){return foc_dynamic_iq_limit;}
FOC_ControlMode FOC_GetControlMode(void){return control_mode;}
uint16_t FOC_GetEncoderRawAngle(void){return enc_raw;}
uint32_t FOC_GetEncoderStatus(void){return enc_status;}
uint32_t FOC_GetState(void){return foc_state;}
uint32_t FOC_GetFault(void){return foc_fault;}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if(h->Instance==TIM5)
        {
            static float speed_history_1, speed_history_2;
            static uint32_t speed_history_valid;
            if (!encoder_healthy()) { speed=0.0f; return; }
            float instant_speed = (float)enc_speed_counts * FOC_2PI *
                                  SPEED_LOOP_HZ /
                                  (float)ENCODER_COUNTS_PER_REV;
            speed=0.15f*instant_speed+0.85f*speed;

            float control_speed = instant_speed;
            if (speed_history_valid >= 2U)
            {
                /* Reject one isolated encoder-speed spike. A real sustained
                   speed change reaches the controller one millisecond later. */
                control_speed = median3(instant_speed,
                                        speed_history_1,
                                        speed_history_2);
            }
            else { ++speed_history_valid; }
            speed_history_2 = speed_history_1;
            speed_history_1 = instant_speed;

            if (foc_state == FOC_STATE_RUNNING && control_mode == FOC_MODE_SPEED)
            {
                /* Use the unfiltered 1-ms encoder delta in the controller.
                   The filtered speed above is telemetry only; feeding it back
                   added several milliseconds of delay and weakened braking. */
                float speed_feedback = encoder_direction * control_speed;
                float speed_error = speed_target_rad_s - speed_feedback;
                float proportional = pi_speed.kp * speed_error;
                float speed_current_limit = fminf(SPEED_LOOP_CURRENT_LIMIT,
                                                  foc_dynamic_iq_limit);
                float integral_candidate = clamp(pi_speed.integ +
                                                   pi_speed.ki * speed_error,
                                                   -speed_current_limit,
                                                   speed_current_limit);
                float output_candidate = proportional + integral_candidate;

                /* Do not integrate farther into current saturation. Integration
                   remains active when the error helps the output leave it. */
                if (!((output_candidate > speed_current_limit && speed_error > 0.0f) ||
                      (output_candidate < -speed_current_limit && speed_error < 0.0f)))
                {
                    pi_speed.integ = integral_candidate;
                }
                pi_speed.out = clamp(proportional + pi_speed.integ,
                                     -speed_current_limit,
                                     speed_current_limit);
                iq_ref_target = pi_speed.out;
            }
        }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h)
{
    static uint32_t control_divider;
    static uint32_t hard_overcurrent_count;
    static uint32_t sustained_overcurrent_score;
    static float current_adc_sum_u;
    static float current_adc_sum_v;
    if(h->Instance!=ADC1)  return;

    // calibrate ADC
    if (foc_state == FOC_STATE_CALIBRATING)
    {
        if (offset_samples < 128U)
        {
            offset_u += (float)h->Instance->JDR1;
            offset_v += (float)hadc2.Instance->JDR1;
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

    /* ADC1/ADC2 sample U/V simultaneously at 40 kHz. Following the TIGERs
       principle of paired current samples, average two PWM-cycle samples for
       one 20-kHz control update instead of discarding the first sample. */
    current_adc_sum_u += (float)h->Instance->JDR1;
    current_adc_sum_v += (float)hadc2.Instance->JDR1;
    if (++control_divider < 2U) { return; }
    control_divider = 0U;
    float current_adc_u = 0.5f * current_adc_sum_u;
    float current_adc_v = 0.5f * current_adc_sum_v;
    current_adc_sum_u = 0.0f;
    current_adc_sum_v = 0.0f;

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

    float iu=VREF*(current_adc_u-offset_u)/(ADC_FS*SHUNT_OHM*AMP_GAIN);
    float iv=VREF*(current_adc_v-offset_v)/(ADC_FS*SHUNT_OHM*AMP_GAIN);

    float iw=-iu-iv;
    if (current_sense_blank_samples > 0U)
    {
        --current_sense_blank_samples;
        hard_overcurrent_count = 0U;
        sustained_overcurrent_score = 0U;
        pwm(0.5f,0.5f,0.5f);
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
        return;
    }

    float abs_iu = fabsf(iu);
    float abs_iv = fabsf(iv);
    float abs_iw = fabsf(iw);
    float max_phase_current = fmaxf(abs_iu, fmaxf(abs_iv, abs_iw));

    if (abs_iu >= abs_iv && abs_iu >= abs_iw)
    {
        overcurrent_detail = iu >= 0.0f ? 4U : 5U;
    }
    else if (abs_iv >= abs_iw)
    {
        overcurrent_detail = iv >= 0.0f ? 6U : 7U;
    }
    else
    {
        overcurrent_detail = iw >= 0.0f ? 8U : 9U;
    }

    if (max_phase_current > PHASE_HARD_OVERCURRENT_LIMIT)
    {
        ++hard_overcurrent_count;
    }
    else { hard_overcurrent_count = 0U; }

    if (max_phase_current > PHASE_SUSTAINED_CURRENT_LIMIT)
    {
        if (sustained_overcurrent_score < PHASE_SUSTAINED_OVERCURRENT_SAMPLES)
        {
            ++sustained_overcurrent_score;
        }
    }
    else if (sustained_overcurrent_score > PHASE_SUSTAINED_RECOVERY_PER_SAMPLE)
    {
        sustained_overcurrent_score -= PHASE_SUSTAINED_RECOVERY_PER_SAMPLE;
    }
    else { sustained_overcurrent_score = 0U; }

    /* TIGERs clears the active PWM pulse from a hardware current comparator
       and resumes automatically. We cannot reproduce that asynchronous path
       with two ADC shunts, so suppress the next complete voltage vector when
       a hard peak is observed. A transient therefore costs torque for one
       control interval instead of latching the whole motor off. */
    if (hard_overcurrent_count >= PHASE_HARD_OVERCURRENT_SAMPLES ||
        sustained_overcurrent_score >= PHASE_SUSTAINED_OVERCURRENT_SAMPLES)
    {
        overcurrent_trigger_type =
            hard_overcurrent_count >= PHASE_HARD_OVERCURRENT_SAMPLES ? 1U : 2U;
        foc_fault_adc_u = (uint16_t)current_adc_u;
        foc_fault_adc_v = (uint16_t)current_adc_v;
        foc_fault_iu = iu;
        foc_fault_iv = iv;
        foc_fault_iw = iw;
        foc_fault_hard_overcurrent_samples = hard_overcurrent_count;
        foc_fault_sustained_overcurrent_samples = sustained_overcurrent_score;
        foc_fault_dynamic_iq_limit = foc_dynamic_iq_limit;
        foc_fault_thermal_utilization = foc_current_thermal_utilization;
        foc_fault_bus_voltage = dc_bus_voltage;
        foc_fault_bus_voltage_raw = foc_dc_bus_voltage_raw;
        foc_fault_iq_ref = iq_ref_amp;
        foc_fault_id = id;
        foc_fault_iq = iq;
        foc_fault_vd = pi_d.out;
        foc_fault_vq = pi_q.out;
        stop_fault(FOC_FAULT_OVERCURRENT);
        led_set(LED_3V3, 1U);
        return;
    }

    if (max_phase_current > PHASE_HARD_OVERCURRENT_LIMIT)
    {
        pwm(0.5f, 0.5f, 0.5f);
        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
        return;
    }

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

    /* I^2t limiter: full 9-A q current is available for acceleration/impact,
       then the limit tapers smoothly toward the continuous winding rating. */
    update_current_thermal_limit(id*id + iq*iq);
    float limited_iq_target = clamp(iq_ref_target,
                                    -foc_dynamic_iq_limit,
                                    foc_dynamic_iq_limit);
    /* This limits the electrical current step, not the speed command. Speed
       mode still reaches 9 A in 4.5 ms, but avoids a discontinuous current
       reference that can excite the current loop and sampling transients. */
    float iq_slew_rate = control_mode == FOC_MODE_SPEED ?
                         SPEED_IQ_SLEW_RATE_A_PER_S : IQ_SLEW_RATE_A_PER_S;
    float iq_step=iq_slew_rate/CURRENT_LOOP_HZ;
    if (iq_ref_amp < limited_iq_target)
    {
        iq_ref_amp=fminf(iq_ref_amp+iq_step,limited_iq_target);
    }
    else if (iq_ref_amp > limited_iq_target)
    {
        iq_ref_amp=fmaxf(iq_ref_amp-iq_step,limited_iq_target);
    }

    float eq=iq_ref_amp-iq;
    float ed=-id;

    /* Restore the previously proven normalized current PI. Scaling its gains
       and integral increment by 16/Vbus preserves the response across the
       intended supply range without changing the PI state representation. */
    float bus_scale = current_pi_bus_scale;
    float q_proportional=pi_q.kp*eq*bus_scale;
    float d_proportional=pi_d.kp*ed*bus_scale;
    pi_q.integ=clamp(pi_q.integ+pi_q.ki*eq*bus_scale,
                     -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_q.out=clamp(q_proportional+pi_q.integ,
                   -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    pi_d.integ=clamp(pi_d.integ+pi_d.ki*ed*bus_scale,
                     -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_d.out=clamp(d_proportional+pi_d.integ,
                   -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

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
    pi_q.integ=clamp(pi_q.out-q_proportional,
                     -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);
    pi_d.integ=clamp(pi_d.out-d_proportional,
                     -CURRENT_OUTPUT_LIMIT,CURRENT_OUTPUT_LIMIT);

    svpwm(theta,pi_d.out,pi_q.out);
    HAL_GPIO_WritePin(DRV_cotr_GPIO_Port, DRV_cotr_Pin, GPIO_PIN_SET);
}
