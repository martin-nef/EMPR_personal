/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_PINSEL_H_
#define LPC17XX_PINSEL_H_

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
/*********************************************************************/
#define PINSEL_PORT_0   ((0))   
#define PINSEL_PORT_1   ((1))   
#define PINSEL_PORT_2   ((2))   
#define PINSEL_PORT_3   ((3))   
#define PINSEL_PORT_4   ((4))   
/***********************************************************************
 * Macros define for Pin Function selection
 **********************************************************************/
#define PINSEL_FUNC_0   ((0))   
#define PINSEL_FUNC_1   ((1))   
#define PINSEL_FUNC_2   ((2))   
#define PINSEL_FUNC_3   ((3))   
/***********************************************************************
 * Macros define for Pin Number of Port
 **********************************************************************/
#define PINSEL_PIN_0    ((0))   
#define PINSEL_PIN_1    ((1))   
#define PINSEL_PIN_2    ((2))   
#define PINSEL_PIN_3    ((3))   
#define PINSEL_PIN_4    ((4))   
#define PINSEL_PIN_5    ((5))   
#define PINSEL_PIN_6    ((6))   
#define PINSEL_PIN_7    ((7))   
#define PINSEL_PIN_8    ((8))   
#define PINSEL_PIN_9    ((9))   
#define PINSEL_PIN_10   ((10))  
#define PINSEL_PIN_11   ((11))  
#define PINSEL_PIN_12   ((12))  
#define PINSEL_PIN_13   ((13))  
#define PINSEL_PIN_14   ((14))  
#define PINSEL_PIN_15   ((15))  
#define PINSEL_PIN_16   ((16))  
#define PINSEL_PIN_17   ((17))  
#define PINSEL_PIN_18   ((18))  
#define PINSEL_PIN_19   ((19))  
#define PINSEL_PIN_20   ((20))  
#define PINSEL_PIN_21   ((21))  
#define PINSEL_PIN_22   ((22))  
#define PINSEL_PIN_23   ((23))  
#define PINSEL_PIN_24   ((24))  
#define PINSEL_PIN_25   ((25))  
#define PINSEL_PIN_26   ((26))  
#define PINSEL_PIN_27   ((27))  
#define PINSEL_PIN_28   ((28))  
#define PINSEL_PIN_29   ((29))  
#define PINSEL_PIN_30   ((30))  
#define PINSEL_PIN_31   ((31))  
/***********************************************************************
 * Macros define for Pin mode
 **********************************************************************/
#define PINSEL_PINMODE_PULLUP           ((0))   
#define PINSEL_PINMODE_TRISTATE         ((2))   
#define PINSEL_PINMODE_PULLDOWN         ((3))   
/***********************************************************************
 * Macros define for Pin mode (normal/open drain)
 **********************************************************************/
#define PINSEL_PINMODE_NORMAL           ((0))   
#define PINSEL_PINMODE_OPENDRAIN        ((1))   
/***********************************************************************
 * Macros define for I2C mode
 ***********************************************************************/
#define PINSEL_I2C_Normal_Mode          ((0))   
#define PINSEL_I2C_Fast_Mode            ((1))   
/* Private Macros ------------------------------------------------------------- */

/* Pin selection define */
/* I2C Pin Configuration register bit description */
#define PINSEL_I2CPADCFG_SDADRV0        _BIT(0) 
#define PINSEL_I2CPADCFG_SDAI2C0        _BIT(1) 
#define PINSEL_I2CPADCFG_SCLDRV0        _BIT(2) 
#define PINSEL_I2CPADCFG_SCLI2C0        _BIT(3) 
/* Public Types --------------------------------------------------------------- */

typedef struct
{
        uint8_t Portnum;        
        uint8_t Pinnum;         
        uint8_t Funcnum;        
        uint8_t Pinmode;        
        uint8_t OpenDrain;      
} PINSEL_CFG_Type;

/* Public Functions ----------------------------------------------------------- */
void PINSEL_ConfigPin(PINSEL_CFG_Type *PinCfg);
void PINSEL_ConfigTraceFunc (FunctionalState NewState);
void PINSEL_SetI2C0Pins(uint8_t i2cPinMode, FunctionalState filterSlewRateEnable);


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_PINSEL_H_ */

/* --------------------------------- End Of File ------------------------------ */