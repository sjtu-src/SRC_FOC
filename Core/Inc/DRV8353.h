/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.h
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
 
/* Includes ------------------------------------------------------------------*/
#ifndef DRV835X_H
#define DRV835X_H
 
#ifdef __cplusplus
extern "C"
{
#endif

#if defined(__CC_ARM)
#pragma anon_unions
#endif
 
#include "main.h"
#include "DRV8353_reg.h"
 
#define  W_MODE              0
#define  R_MODE              1
 
typedef struct
{
    uint16_t status_FSR1;
    uint16_t status_FSR2;
}Stru_DRV835X_Status;
 
extern Stru_DRV835X_Status stru_DRV835X_Status;
 
/*
    INPUT data structure  
*/
typedef struct
{
    uint16_t DATA      : 11;    // 11 data bits, D (bits B11 through B0)
    uint16_t ADDRESS   : 4;     // 4 address bits, A (bits B14 through B11)
    
    uint16_t WR        : 1;     // 1 read or write bit, W (bit B15)
}Input_WrReg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Input_WrReg_bit inputRegObj;
   };
} Input_WrReg ;
 
/*
    Fault Status Register 1 (address = 0x00h)
*/
typedef struct
{
    uint16_t VDS_LC    : 1;
    uint16_t VDS_HC    : 1;
    uint16_t VDS_LB    : 1;
    uint16_t VDS_HB    : 1;
    
    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;
    uint16_t OTSD      : 1;
    uint16_t UVLO      : 1;
    
    uint16_t GDF       : 1;
    uint16_t VDS_OCP   : 1;
    uint16_t FAULT     : 1;
    
    uint16_t res       : 5; 
    
}Fault_StatusReg1_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Fault_StatusReg1_bit fault1RegObj;
   };
} Fault_StatusReg1 ;
 
/*
    Fault Status Register 2 (address = 0x01h)
*/
typedef struct
{
    uint16_t VGS_LC    : 1;
    uint16_t VGS_HC    : 1;
    
    uint16_t VGS_LB    : 1;
    uint16_t VGS_HB    : 1;
    
    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;
    
    uint16_t GDUV      : 1;
    uint16_t OTW       : 1;
    
    uint16_t SC_OC     : 1;
    uint16_t SB_OC     : 1;
    uint16_t SA_OC     : 1;
    
    uint16_t res       : 5; 
    
}Fault_StatusReg2_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Fault_StatusReg2_bit fault2RegObj;
   };
} Fault_StatusReg2;
 
/*
    Driver Control Register (address = 0x02h)
*/
typedef struct
{
    uint16_t CLR_FLT  : 1;
    uint16_t BRAKE    : 1;
    uint16_t COAST    : 1;
    uint16_t PWM1_DIR : 1;
    
    uint16_t PWM1_COM : 1;
    uint16_t PWM_MODE : 2;
    uint16_t OTW_REP  : 1;
    
    uint16_t DIS_GDF  : 1;
    uint16_t DIS_GDUV : 1;
    uint16_t OCP_ACT  : 1;
    
    uint16_t res      : 5; 
    
}Drv_CtrlReg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_CtrlReg_bit ctrlRegObj;
   };
} Drv_CtrlReg ;
 
/*
    Gate Drive HS Register (address = 0x03h)
*/
typedef struct
{
    uint16_t IDRIVEN_HS  : 4;
    uint16_t IDRIVEP_HS  : 4;
    
    uint16_t LOCK        : 3;
    uint16_t res         : 5; 
    
}Drv_GateHS_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_GateHS_bit gateHSRegObj;
   };
} Drv_GateHS;
 
 
/*
    Gate Drive LS Register (address = 0x04h)
*/
typedef struct
{
    uint16_t IDRIVEN_LS  : 4;
    uint16_t IDRIVEP_LS  : 4;
    
    uint16_t TDRIVE      : 2;
    uint16_t CBC         : 1;
    
    uint16_t res         : 5; 
    
}Drv_GateLS_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_GateLS_bit gateLSRegObj;
   };
} Drv_GateLS;
 
/*
    OCP Control Register (address = 0x05h)
*/
typedef struct
{
    uint16_t VDS_LVL     : 4;
    uint16_t OCP_DEG     : 2;
    
    uint16_t OCP_MODE    : 2;
    uint16_t DEAD_TIME   : 2;
    
    uint16_t TRETRY      : 1;
    
    uint16_t res         : 5; 
    
}Drv_OCP_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_OCP_bit ocpObj;
   };
} Drv_OCP;
 
/*
    CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
*/
typedef struct
{
    uint16_t SEN_LVL     : 2;
    
    uint16_t CSA_CAL_C   : 1;
    uint16_t CSA_CAL_B   : 1;
    uint16_t CSA_CAL_A   : 1;
    uint16_t DIS_SEN     : 1;
    
    uint16_t CSA_GAIN    : 2;
    uint16_t LS_REF      : 1;
    uint16_t VREF_DIV    : 1;
    
    uint16_t CSA_FET     : 1;
    uint16_t res         : 5; 
    
}Drv_CSA_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_CSA_bit csaObj;
   };
} Drv_CSA;
 
/*
    Driver Configuration Register (DRV8353 and DRV8353R Only) (address = 0x07h)
*/
typedef struct
{
    uint16_t SEN_LVL     : 1;
    uint16_t RES         : 10; 
    uint16_t res         : 5; 
    
}Drv_Cfg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_Cfg_bit cfgObj;
   };
} Drv_Cfg;
 
 
typedef struct
{
    Drv_Cfg drvCfg_obj;
    Drv_CSA drvCsa_obj;
    Drv_OCP drvOcp_obj;
    
    Drv_GateLS drvGateLS_obj;
    Drv_GateHS drvGateHS_obj;
    
    Fault_StatusReg1  faultStatusReg1_obj;
    Fault_StatusReg2  faultStatusReg2_obj; 
    
    Drv_CtrlReg drvCtrl_obj;
}Stru_DRV835X;
 
                   
extern Stru_DRV835X stru_DRV8353Obj;

/* Debug values remain at the failing operation when initialization fails. */
#define DRV835X_STAGE_IDLE          0U
#define DRV835X_STAGE_READ_DCR      1U
#define DRV835X_STAGE_READ_CSACR    2U
#define DRV835X_STAGE_READ_DFGCR    3U
#define DRV835X_STAGE_READ_HSR      4U
#define DRV835X_STAGE_READ_LSR      5U
#define DRV835X_STAGE_READ_OCPCR    6U
#define DRV835X_STAGE_READ_FSR1     7U
#define DRV835X_STAGE_READ_FSR2     8U
#define DRV835X_STAGE_WRITE_DCR     9U
#define DRV835X_STAGE_WRITE_HSR    10U
#define DRV835X_STAGE_WRITE_LSR    11U
#define DRV835X_STAGE_WRITE_OCPCR  12U
#define DRV835X_STAGE_WRITE_CSACR  13U
#define DRV835X_STAGE_READY        14U

#define DRV835X_ERROR_NONE          0U
#define DRV835X_ERROR_SPI           1U
#define DRV835X_ERROR_VERIFY        2U

extern volatile uint32_t drv835x_debug_stage;
extern volatile uint32_t drv835x_debug_error;
extern volatile uint32_t drv835x_debug_hal_status;
extern volatile uint16_t drv835x_debug_tx;
extern volatile uint16_t drv835x_debug_rx;
extern volatile uint16_t drv835x_debug_expected;
 
 
typedef struct
{
   // Driver Control Register (address = 0x02h)
   uint8_t PWM_MODE;
    
   // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
   uint8_t SEN_LVL;
   uint8_t CSA_GAIN;
   uint8_t VREF_DIV;
    
   // OCP Control Register (address = 0x05h)
   uint8_t VDS_LVL;
   uint8_t OCP_DEG;
   uint8_t OCP_MODE;
   uint8_t DEAD_TIME;
   uint8_t TRETRY;
    
   //Gate Drive HS Register (address = 0x03h)
   uint8_t IDRIVEP_HS;
   uint8_t IDRIVEN_HS;
   uint8_t LOCK;
    
   // Gate Drive LS Register (address = 0x04h) 
   uint8_t IDRIVEN_LS;
   uint8_t IDRIVEP_LS;
   uint8_t TDRIVE;
   uint8_t CBC;
   
} StruDRV835XCfgPara;
 
 
HAL_StatusTypeDef DRV835X_Init(void);
HAL_StatusTypeDef DRV835X_updateCfgPara(void);
 
void DRV835X_read_FaultStatusReg1(void);
void DRV835X_read_FaultStatusReg2(void);
 
 
 
#ifdef __cplusplus
}
#endif
 
 
#endif  /* DRV835X_H */
