/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_CLKPWR_H_
#define LPC17XX_CLKPWR_H_

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
/**********************************************************************
 * Peripheral Clock Selection Definitions
 **********************************************************************/
#define CLKPWR_PCLKSEL_WDT              ((uint32_t)(0))

#define CLKPWR_PCLKSEL_TIMER0           ((uint32_t)(2))

#define CLKPWR_PCLKSEL_TIMER1           ((uint32_t)(4))

#define CLKPWR_PCLKSEL_UART0            ((uint32_t)(6))

#define CLKPWR_PCLKSEL_UART1            ((uint32_t)(8))

#define CLKPWR_PCLKSEL_PWM1             ((uint32_t)(12))

#define CLKPWR_PCLKSEL_I2C0             ((uint32_t)(14))

#define CLKPWR_PCLKSEL_SPI              ((uint32_t)(16))

#define CLKPWR_PCLKSEL_SSP1             ((uint32_t)(20))

#define CLKPWR_PCLKSEL_DAC              ((uint32_t)(22))

#define CLKPWR_PCLKSEL_ADC              ((uint32_t)(24))

#define CLKPWR_PCLKSEL_CAN1             ((uint32_t)(26))

#define CLKPWR_PCLKSEL_CAN2             ((uint32_t)(28))

#define CLKPWR_PCLKSEL_ACF              ((uint32_t)(30))

#define CLKPWR_PCLKSEL_QEI              ((uint32_t)(32))

#define CLKPWR_PCLKSEL_PCB              ((uint32_t)(36))

#define CLKPWR_PCLKSEL_I2C1             ((uint32_t)(38))

#define CLKPWR_PCLKSEL_SSP0             ((uint32_t)(42))

#define CLKPWR_PCLKSEL_TIMER2           ((uint32_t)(44))

#define CLKPWR_PCLKSEL_TIMER3           ((uint32_t)(46))

#define CLKPWR_PCLKSEL_UART2            ((uint32_t)(48))

#define CLKPWR_PCLKSEL_UART3            ((uint32_t)(50))

#define CLKPWR_PCLKSEL_I2C2             ((uint32_t)(52))

#define CLKPWR_PCLKSEL_I2S              ((uint32_t)(54))

#define CLKPWR_PCLKSEL_RIT              ((uint32_t)(58))

#define CLKPWR_PCLKSEL_SYSCON           ((uint32_t)(60))

#define CLKPWR_PCLKSEL_MC                       ((uint32_t)(62))

/* Peripheral clock divider is set to 4 from CCLK */
#define CLKPWR_PCLKSEL_CCLK_DIV_4  ((uint32_t)(0))

#define CLKPWR_PCLKSEL_CCLK_DIV_1  ((uint32_t)(1))

#define CLKPWR_PCLKSEL_CCLK_DIV_2  ((uint32_t)(2))


/********************************************************************
* Power Control for Peripherals Definitions
**********************************************************************/
#define  CLKPWR_PCONP_PCTIM0    ((uint32_t)(1<<1))
/* Timer/Counter 1 power/clock control bit */
#define  CLKPWR_PCONP_PCTIM1    ((uint32_t)(1<<2))

#define  CLKPWR_PCONP_PCUART0   ((uint32_t)(1<<3))

#define  CLKPWR_PCONP_PCUART1   ((uint32_t)(1<<4))

#define  CLKPWR_PCONP_PCPWM1    ((uint32_t)(1<<6))

#define  CLKPWR_PCONP_PCI2C0    ((uint32_t)(1<<7))

#define  CLKPWR_PCONP_PCSPI     ((uint32_t)(1<<8))

#define  CLKPWR_PCONP_PCRTC     ((uint32_t)(1<<9))

#define  CLKPWR_PCONP_PCSSP1    ((uint32_t)(1<<10))

#define  CLKPWR_PCONP_PCAD      ((uint32_t)(1<<12))

#define  CLKPWR_PCONP_PCAN1     ((uint32_t)(1<<13))

#define  CLKPWR_PCONP_PCAN2     ((uint32_t)(1<<14))

#define CLKPWR_PCONP_PCGPIO     ((uint32_t)(1<<15))

#define CLKPWR_PCONP_PCRIT              ((uint32_t)(1<<16))

#define CLKPWR_PCONP_PCMC               ((uint32_t)(1<<17))

#define CLKPWR_PCONP_PCQEI              ((uint32_t)(1<<18))

#define  CLKPWR_PCONP_PCI2C1    ((uint32_t)(1<<19))

#define  CLKPWR_PCONP_PCSSP0    ((uint32_t)(1<<21))

#define  CLKPWR_PCONP_PCTIM2    ((uint32_t)(1<<22))

#define  CLKPWR_PCONP_PCTIM3    ((uint32_t)(1<<23))

#define  CLKPWR_PCONP_PCUART2   ((uint32_t)(1<<24))

#define  CLKPWR_PCONP_PCUART3   ((uint32_t)(1<<25))

#define  CLKPWR_PCONP_PCI2C2    ((uint32_t)(1<<26))

#define  CLKPWR_PCONP_PCI2S     ((uint32_t)(1<<27))

#define  CLKPWR_PCONP_PCGPDMA   ((uint32_t)(1<<29))

#define  CLKPWR_PCONP_PCENET    ((uint32_t)(1<<30))

#define  CLKPWR_PCONP_PCUSB     ((uint32_t)(1<<31))


/* Private Macros ------------------------------------------------------------- */
/* --------------------- BIT DEFINITIONS -------------------------------------- */
/*********************************************************************/
#define CLKPWR_CLKSRCSEL_CLKSRC_IRC                     ((uint32_t)(0x00))

#define CLKPWR_CLKSRCSEL_CLKSRC_MAINOSC         ((uint32_t)(0x01))

#define CLKPWR_CLKSRCSEL_CLKSRC_RTC                     ((uint32_t)(0x02))

#define CLKPWR_CLKSRCSEL_BITMASK                        ((uint32_t)(0x03))

/*********************************************************************/
/* Clock Output Configuration register definition */
#define CLKPWR_CLKOUTCFG_CLKOUTSEL_CPU          ((uint32_t)(0x00))

#define CLKPWR_CLKOUTCFG_CLKOUTSEL_MAINOSC      ((uint32_t)(0x01))

#define CLKPWR_CLKOUTCFG_CLKOUTSEL_RC           ((uint32_t)(0x02))

#define CLKPWR_CLKOUTCFG_CLKOUTSEL_USB          ((uint32_t)(0x03))

#define CLKPWR_CLKOUTCFG_CLKOUTSEL_RTC          ((uint32_t)(0x04))

#define CLKPWR_CLKOUTCFG_CLKOUTDIV(n)           ((uint32_t)((n&0x0F)<<4))

#define CLKPWR_CLKOUTCFG_CLKOUT_EN                      ((uint32_t)(1<<8))

#define CLKPWR_CLKOUTCFG_CLKOUT_ACT                     ((uint32_t)(1<<9))

#define CLKPWR_CLKOUTCFG_BITMASK                        ((uint32_t)(0x3FF))

/*********************************************************************/
#define CLKPWR_PLL0CON_ENABLE           ((uint32_t)(0x01))

#define CLKPWR_PLL0CON_CONNECT          ((uint32_t)(0x02))

#define CLKPWR_PLL0CON_BITMASK          ((uint32_t)(0x03))

/*********************************************************************/
#define CLKPWR_PLL0CFG_MSEL(n)          ((uint32_t)(n&0x7FFF))

#define CLKPWR_PLL0CFG_NSEL(n)          ((uint32_t)((n<<16)&0xFF0000))

#define CLKPWR_PLL0CFG_BITMASK          ((uint32_t)(0xFF7FFF))


/*********************************************************************/
#define CLKPWR_PLL0STAT_MSEL(n)         ((uint32_t)(n&0x7FFF))

#define CLKPWR_PLL0STAT_NSEL(n)         ((uint32_t)((n>>16)&0xFF))

#define CLKPWR_PLL0STAT_PLLE            ((uint32_t)(1<<24))

#define CLKPWR_PLL0STAT_PLLC            ((uint32_t)(1<<25))

#define CLKPWR_PLL0STAT_PLOCK           ((uint32_t)(1<<26))

/*********************************************************************/
#define CLKPWR_PLL0FEED_BITMASK                 ((uint32_t)0xFF)

/*********************************************************************/
#define CLKPWR_PLL1CON_ENABLE           ((uint32_t)(0x01))

#define CLKPWR_PLL1CON_CONNECT          ((uint32_t)(0x02))

#define CLKPWR_PLL1CON_BITMASK          ((uint32_t)(0x03))

/*********************************************************************/
#define CLKPWR_PLL1CFG_MSEL(n)          ((uint32_t)(n&0x1F))

#define CLKPWR_PLL1CFG_PSEL(n)          ((uint32_t)((n&0x03)<<5))

#define CLKPWR_PLL1CFG_BITMASK          ((uint32_t)(0x7F))

/*********************************************************************/
#define CLKPWR_PLL1STAT_MSEL(n)         ((uint32_t)(n&0x1F))

#define CLKPWR_PLL1STAT_PSEL(n)         ((uint32_t)((n>>5)&0x03))

#define CLKPWR_PLL1STAT_PLLE            ((uint32_t)(1<<8))

#define CLKPWR_PLL1STAT_PLLC            ((uint32_t)(1<<9))

#define CLKPWR_PLL1STAT_PLOCK           ((uint32_t)(1<<10))

/*********************************************************************/
#define CLKPWR_PLL1FEED_BITMASK         ((uint32_t)0xFF)

/*********************************************************************/
#define CLKPWR_CCLKCFG_BITMASK          ((uint32_t)(0xFF))

/*********************************************************************/
#define CLKPWR_USBCLKCFG_BITMASK        ((uint32_t)(0x0F))

/*********************************************************************/
#define CLKPWR_IRCTRIM_BITMASK          ((uint32_t)(0x0F))

/*********************************************************************/
#define CLKPWR_PCLKSEL0_BITMASK         ((uint32_t)(0xFFF3F3FF))

#define CLKPWR_PCLKSEL1_BITMASK         ((uint32_t)(0xFCF3F0F3))

#define CLKPWR_PCLKSEL_SET(p,n)         _SBF(p,n)

#define CLKPWR_PCLKSEL_BITMASK(p)       _SBF(p,0x03)

#define CLKPWR_PCLKSEL_GET(p, n)        ((uint32_t)((n>>p)&0x03))

/*********************************************************************/
#define CLKPWR_PCON_PM0                 ((uint32_t)(1<<0))

#define CLKPWR_PCON_PM1                 ((uint32_t)(1<<1))

#define CLKPWR_PCON_BODPDM              ((uint32_t)(1<<2))

#define CLKPWR_PCON_BOGD                ((uint32_t)(1<<3))

#define CLKPWR_PCON_BORD                ((uint32_t)(1<<4))

#define CLKPWR_PCON_SMFLAG              ((uint32_t)(1<<8))

#define CLKPWR_PCON_DSFLAG              ((uint32_t)(1<<9))

#define CLKPWR_PCON_PDFLAG              ((uint32_t)(1<<10))

#define CLKPWR_PCON_DPDFLAG             ((uint32_t)(1<<11))

/*********************************************************************/
#define CLKPWR_PCONP_BITMASK    0xEFEFF7DE

/* Public Functions ----------------------------------------------------------- */
void CLKPWR_SetPCLKDiv (uint32_t ClkType, uint32_t DivVal);
uint32_t CLKPWR_GetPCLKSEL (uint32_t ClkType);
uint32_t CLKPWR_GetPCLK (uint32_t ClkType);
void CLKPWR_ConfigPPWR (uint32_t PPType, FunctionalState NewState);
void CLKPWR_Sleep(void);
void CLKPWR_DeepSleep(void);
void CLKPWR_PowerDown(void);
void CLKPWR_DeepPowerDown(void);

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_CLKPWR_H_ */

/* --------------------------------- End Of File ------------------------------ */