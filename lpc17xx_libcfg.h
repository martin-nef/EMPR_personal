/***********************************************************************/
#ifndef LPC17XX_LIBCFG_H_
#define LPC17XX_LIBCFG_H_

#include "lpc_types.h"


/************************** DEBUG MODE DEFINITIONS *********************************/
/* Un-comment the line below to compile the library in DEBUG mode, this will expanse
   the "CHECK_PARAM" macro in the FW library code */

#define DEBUG


/******************* PERIPHERAL FW LIBRARY CONFIGURATION DEFINITIONS ***********************/

/* Comment the line below to disable the specific peripheral inclusion */

/* DEBUG_FRAMWORK ------------------------------ */
//#define _DBGFWK

/* GPIO ------------------------------- */
#define _GPIO

/* EXTI ------------------------------- */
//#define _EXTI

/* UART ------------------------------- */
//#define _UART
//#define _UART0
//#define _UART1
//#define _UART2
//#define _UART3

/* SPI ------------------------------- */
//#define _SPI

/* SSP ------------------------------- */
//#define _SSP
//#define _SSP0
//#define _SSP1

/* SYSTICK --------------------------- */
//#define _SYSTICK

/* I2C ------------------------------- */
//#define _I2C
//#define _I2C0
//#define _I2C1
//#define _I2C2

/* TIMER ------------------------------- */
//#define _TIM

/* WDT ------------------------------- */
//#define _WDT


/* GPDMA ------------------------------- */
//#define _GPDMA


/* DAC ------------------------------- */
//#define _DAC

/* DAC ------------------------------- */
//#define _ADC


/* PWM ------------------------------- */
//#define _PWM
//#define _PWM1

/* RTC ------------------------------- */
//#define _RTC

/* I2S ------------------------------- */
//#define _I2S

/* USB device ------------------------------- */
//#define _USBDEV
//#define _USB_DMA

/* QEI ------------------------------- */
//#define _QEI

/* MCPWM ------------------------------- */
//#define _MCPWM

/* CAN--------------------------------*/
//#define _CAN

/* RIT ------------------------------- */
//#define _RIT

/* EMAC ------------------------------ */
//#define _EMAC

/************************** GLOBAL/PUBLIC MACRO DEFINITIONS *********************************/

#ifdef  DEBUG
/*******************************************************************************
* @brief                The CHECK_PARAM macro is used for function's parameters check.
*                               It is used only if the library is compiled in DEBUG mode.
* @param[in]    expr - If expr is false, it calls check_failed() function
*                       which reports the name of the source file and the source
*                       line number of the call that failed.
*                    - If expr is true, it returns no value.
* @return               None
*******************************************************************************/
#define CHECK_PARAM(expr) ((expr) ? (void)0 : check_failed((uint8_t *)__FILE__, __LINE__))
#else
#define CHECK_PARAM(expr)
#endif /* DEBUG */



/************************** GLOBAL/PUBLIC FUNCTION DECLARATION *********************************/

#ifdef  DEBUG
void check_failed(uint8_t *file, uint32_t line);
#endif


#endif /* LPC17XX_LIBCFG_H_ */