/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.c
 * Description        :  DRV835X driver 
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2025
* CREATED BY:   ming fei.tang
* DATE:         January 04th, 2025
 ******************************************************************************
 */
/* USER CODE END Header */
#include "DRV8353.h"
#include "gpio.h"
 
/* Private macro -------------------------------------------------------------*/
extern SPI_HandleTypeDef        hspi3;
 
/* Private define ------------------------------------------------------------*/
#define TIME_OUT                100
#define DEFAULT_GAIN            10
#define WRITE_RETRIES           3U
#define DRV835X_SPI_Handle      hspi3
 
// DRV8353 SPI CS PIN 
#define DRV835X_CS_EN           HAL_GPIO_WritePin(DRV_CS_GPIO_Port,DRV_CS_Pin,GPIO_PIN_RESET)
#define DRV835X_CS_DIS          HAL_GPIO_WritePin(DRV_CS_GPIO_Port,DRV_CS_Pin,GPIO_PIN_SET)
 
// DRV8353 ENABLE PIN 
/*
    Gate driver enable. When this pin is logic low the device goes to a low power sleep mode. 
    An 8 to 40-µs low pulse can be used to reset fault conditions.
*/
#define DRV835X_ENABLE_LOW      HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port,DRV_ENBLE_Pin,GPIO_PIN_RESET)
#define DRV835X_ENABLE_HIGH     HAL_GPIO_WritePin(DRV_ENBLE_GPIO_Port,DRV_ENBLE_Pin,GPIO_PIN_SET)
 
// DRV8353 PWML PIN: INLA INLB INLC 
/*
   Low-side gate driver control input. This pin controls the output of the low-side gate driver.
*/
#define DRV835X_PWML_LOW        HAL_GPIO_WritePin(DRV_cotr_GPIO_Port,DRV_cotr_Pin,GPIO_PIN_RESET)
#define DRV835X_PWML_HIGH       HAL_GPIO_WritePin(DRV_cotr_GPIO_Port,DRV_cotr_Pin,GPIO_PIN_SET)
 
/* Private variables ---------------------------------------------------------*/
Stru_DRV835X_Status stru_DRV835X_Status;
Stru_DRV835X stru_DRV8353Obj;
volatile uint32_t drv835x_debug_stage;
volatile uint32_t drv835x_debug_error;
volatile uint32_t drv835x_debug_hal_status;
volatile uint16_t drv835x_debug_tx;
volatile uint16_t drv835x_debug_rx;
volatile uint16_t drv835x_debug_expected;
volatile uint16_t drv835x_debug_csacr;
 
StruDRV835XCfgPara stru_config = 
{
    // Driver Control Register (address = 0x02h)
   .PWM_MODE = PWM_MODE_3X,
    
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
   .SEN_LVL = SEN_LVL_0_25,   //  00b = Sense OCP 0.25 V
   .CSA_GAIN = CSA_GAIN_40,   //  11b = 40-V/V for reliable sub-amp control
   .VREF_DIV = VREF_DIV_2,    //  1b = Sense amplifier reference voltage is VREF divided by 2
    
    // OCP Control Register (address = 0x05h)
   /* VDS protection observes switched MOSFET drain-source voltage, so its
      threshold must include hot RDS(on), switching ringing and PCB parasitics.
      80 mV produced false VDS_HC trips at the 9-A operating point. 200 mV is
      still a much stronger hardware backstop than the former 0.9-V setting;
      the calibrated ADC current path provides the lower 11/15-A limits. */
   .VDS_LVL =  VDS_LVL_0_2,
   .OCP_DEG =  OCP_DEG_4US,
   /* Retry after a transient obstruction instead of requiring a reset. A
      persistent VDS fault is still latched by the controller fault monitor. */
   .OCP_MODE = OCP_RETRY,
   .DEAD_TIME = DEADTIME_400NS,
   .TRETRY = TRETRY_8MS,
    
   //Gate Drive HS Register (address = 0x03h)
   /* BSC028N06NS has about 37 nC total gate charge. Start with controlled
      edges; the former 1 A/2 A maximum settings can create VDS ringing. */
   .IDRIVEP_HS = IDRIVEP_HS_150MA,
   .IDRIVEN_HS = IDRIVEN_HS_300MA,
   .LOCK = LOCK_OFF,
    
   // Gate Drive LS Register (address = 0x04h) 
   .IDRIVEN_LS = IDRIVEN_LS_300MA,
   .IDRIVEP_LS = IDRIVEP_LS_150MA,
   .TDRIVE = TDRIVE_4000NS,
   .CBC = PWM_GIVER_ENABLE,  // 1b = For VDS_OCP and SEN_OCP, the fault is cleared when
                             // a new PWM input is given or after tRETRY
};
 
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef transfer_word(uint16_t command, uint16_t *response);
static HAL_StatusTypeDef read_reg(uint16_t address, uint16_t *data);
static HAL_StatusTypeDef write_reg(uint16_t address, uint16_t data);
 
 
HAL_StatusTypeDef DRV835X_updateCfgPara(void)
{
    uint16_t data;

#define READ_STAGE(stage_, reg_, destination_) do { \
    drv835x_debug_stage = (stage_); \
    if (read_reg((reg_), &(destination_)) != HAL_OK) { return HAL_ERROR; } \
} while (0)
#define WRITE_STAGE(stage_, reg_, value_) do { \
    drv835x_debug_stage = (stage_); \
    if (write_reg((reg_), (value_)) != HAL_OK) { return HAL_ERROR; } \
} while (0)

    READ_STAGE(DRV835X_STAGE_READ_DCR, DCR, stru_DRV8353Obj.drvCtrl_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_CSACR, CSACR, stru_DRV8353Obj.drvCsa_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_DFGCR, DFGCR, stru_DRV8353Obj.drvCfg_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_HSR, HSR, stru_DRV8353Obj.drvGateHS_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_LSR, LSR, stru_DRV8353Obj.drvGateLS_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_OCPCR, OCPCR, stru_DRV8353Obj.drvOcp_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_FSR1, FSR1, stru_DRV8353Obj.faultStatusReg1_obj.data);
    READ_STAGE(DRV835X_STAGE_READ_FSR2, FSR2, stru_DRV8353Obj.faultStatusReg2_obj.data);
 
    // Driver Control Register (address = 0x02h)
    stru_DRV8353Obj.drvCtrl_obj.ctrlRegObj.PWM_MODE  = stru_config.PWM_MODE;
    data = stru_DRV8353Obj.drvCtrl_obj.data;
    WRITE_STAGE(DRV835X_STAGE_WRITE_DCR, DCR, data);
 
    //Gate Drive HS Register (address = 0x03h)
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEP_HS = stru_config.IDRIVEP_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.IDRIVEN_HS = stru_config.IDRIVEN_HS;
    stru_DRV8353Obj.drvGateHS_obj.gateHSRegObj.LOCK = stru_config.LOCK;
    data = stru_DRV8353Obj.drvGateHS_obj.data;
    WRITE_STAGE(DRV835X_STAGE_WRITE_HSR, HSR, data);
 
    // Gate Drive LS Register (address = 0x04h) 
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEN_LS = stru_config.IDRIVEN_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.IDRIVEP_LS = stru_config.IDRIVEP_LS;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.TDRIVE = stru_config.TDRIVE;
    stru_DRV8353Obj.drvGateLS_obj.gateLSRegObj.CBC = stru_config.CBC;
    data = stru_DRV8353Obj.drvGateLS_obj.data;
    WRITE_STAGE(DRV835X_STAGE_WRITE_LSR, LSR, data);
 
    // OCP Control Register (address = 0x05h)
    stru_DRV8353Obj.drvOcp_obj.ocpObj.VDS_LVL =  stru_config.VDS_LVL;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_DEG =  stru_config.OCP_DEG;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.OCP_MODE = stru_config.OCP_MODE;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.DEAD_TIME = stru_config.DEAD_TIME;
    stru_DRV8353Obj.drvOcp_obj.ocpObj.TRETRY = stru_config.TRETRY;
    data = stru_DRV8353Obj.drvOcp_obj.data;
    WRITE_STAGE(DRV835X_STAGE_WRITE_OCPCR, OCPCR, data);
 
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
    stru_DRV8353Obj.drvCsa_obj.csaObj.SEN_LVL  = stru_config.SEN_LVL;
    stru_DRV8353Obj.drvCsa_obj.csaObj.CSA_GAIN = stru_config.CSA_GAIN;
    stru_DRV8353Obj.drvCsa_obj.csaObj.VREF_DIV = stru_config.VREF_DIV;
    data = stru_DRV8353Obj.drvCsa_obj.data;
    WRITE_STAGE(DRV835X_STAGE_WRITE_CSACR, CSACR, data);

    drv835x_debug_stage = DRV835X_STAGE_READY;
#undef READ_STAGE
#undef WRITE_STAGE
    return HAL_OK;
}
 
 
HAL_StatusTypeDef DRV835X_Init(void)
{
    drv835x_debug_stage = DRV835X_STAGE_IDLE;
    drv835x_debug_error = DRV835X_ERROR_NONE;
    drv835x_debug_hal_status = HAL_OK;
    drv835x_debug_tx = 0U;
    drv835x_debug_rx = 0U;
    drv835x_debug_expected = 0U;
    DRV835X_CS_DIS;
    DRV835X_PWML_LOW;
    DRV835X_ENABLE_LOW;
    HAL_Delay(100);
    DRV835X_ENABLE_HIGH;
    HAL_Delay(100);
    
    // SET PWML to low
    DRV835X_PWML_LOW;
    HAL_Delay(200);
    
    if (DRV835X_updateCfgPara() != HAL_OK)
    {
        DRV835X_PWML_LOW;
        DRV835X_ENABLE_LOW;
        return HAL_ERROR;
    }
    return HAL_OK;
}
 
 
void DRV835X_read_FaultStatusReg1(void)
{
    (void)read_reg(FSR1, &stru_DRV8353Obj.faultStatusReg1_obj.data);
}
 
void DRV835X_read_FaultStatusReg2(void)
{
    (void)read_reg(FSR2, &stru_DRV8353Obj.faultStatusReg2_obj.data);
}
 
 
static HAL_StatusTypeDef transfer_word(uint16_t command, uint16_t *response)
{
    uint16_t received = 0U;
    for (uint32_t i = 0U; i < 128U; ++i) { __NOP(); }
    DRV835X_CS_EN;
    for (uint32_t i = 0U; i < 32U; ++i) { __NOP(); }
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&DRV835X_SPI_Handle,
        (uint8_t *)&command, (uint8_t *)&received, 1U, TIME_OUT);
    for (uint32_t i = 0U; i < 32U; ++i) { __NOP(); }
    DRV835X_CS_DIS;
    drv835x_debug_tx = command;
    drv835x_debug_rx = received & 0x07ffU;
    drv835x_debug_hal_status = status;
    if (status != HAL_OK) { drv835x_debug_error = DRV835X_ERROR_SPI; }
    if (response != NULL) { *response = received & 0x07ffU; }
    return status;
}

static HAL_StatusTypeDef read_reg(uint16_t address, uint16_t *data)
{
    Input_WrReg stru_Input_WrRegObj = {0};
    stru_Input_WrRegObj.inputRegObj.WR = R_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    return transfer_word(stru_Input_WrRegObj.data, data);
}

static HAL_StatusTypeDef write_reg(uint16_t address, uint16_t data)
{
    Input_WrReg stru_Input_WrRegObj = {0};
    stru_Input_WrRegObj.inputRegObj.WR =  W_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    stru_Input_WrRegObj.inputRegObj.DATA = data & 0x07ffU;
    drv835x_debug_expected = data & 0x07ffU;

    for (uint32_t attempt = 0U; attempt < WRITE_RETRIES; ++attempt)
    {
        uint16_t verify;
        if (transfer_word(stru_Input_WrRegObj.data, NULL) == HAL_OK &&
            read_reg(address, &verify) == HAL_OK &&
            verify == (data & 0x07ffU))
        {
            if (address == CSACR) { drv835x_debug_csacr = verify; }
            return HAL_OK;
        }
    }
    if (drv835x_debug_error == DRV835X_ERROR_NONE)
    {
        drv835x_debug_error = DRV835X_ERROR_VERIFY;
    }
    return HAL_ERROR;
}
 
 
/* End of this file */
