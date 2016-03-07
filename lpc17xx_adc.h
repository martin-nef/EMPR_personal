/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_ADC_H_
#define LPC17XX_ADC_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Private macros ------------------------------------------------------------- */
/* -------------------------- BIT DEFINITIONS ----------------------------------- */
/*********************************************************************/
#define ADC_CR_CH_SEL(n)        ((1UL << n))

#define ADC_CR_CLKDIV(n)        ((n<<8))

#define ADC_CR_BURST            ((1UL<<16))

#define ADC_CR_PDN                      ((1UL<<21))

#define ADC_CR_START_MASK       ((7UL<<24))

#define ADC_CR_START_MODE_SEL(SEL)      ((SEL<<24))

#define ADC_CR_START_NOW        ((1UL<<24))

#define ADC_CR_START_EINT0      ((2UL<<24))

#define ADC_CR_START_CAP01      ((3UL<<24))

#define ADC_CR_START_MAT01      ((4UL<<24))

#define ADC_CR_START_MAT03      ((5UL<<24))

#define ADC_CR_START_MAT10      ((6UL<<24))

#define ADC_CR_START_MAT11      ((7UL<<24))

#define ADC_CR_EDGE                     ((1UL<<27))

/*********************************************************************/
#define ADC_GDR_RESULT(n)               (((n>>4)&0xFFF))

#define ADC_GDR_CH(n)                   (((n>>24)&0x7))

#define ADC_GDR_OVERRUN_FLAG    ((1UL<<30))

#define ADC_GDR_DONE_FLAG               ((1UL<<31))

#define ADC_GDR_CH_MASK         ((7UL<<24))
/*********************************************************************/
#define ADC_INTEN_CH(n)                 ((1UL<<n))

#define ADC_INTEN_GLOBAL                ((1UL<<8))

/*********************************************************************/
#define ADC_DR_RESULT(n)                (((n>>4)&0xFFF))

#define ADC_DR_OVERRUN_FLAG             ((1UL<<30))

#define ADC_DR_DONE_FLAG                ((1UL<<31))

/*********************************************************************/
#define ADC_STAT_CH_DONE_FLAG(n)                ((n&0xFF))

#define ADC_STAT_CH_OVERRUN_FLAG(n)             (((n>>8)&0xFF))

#define ADC_STAT_INT_FLAG                               ((1UL<<16))

/*********************************************************************/
#define ADC_ADCOFFS(n)          (((n&0xF)<<4))

#define ADC_TRIM(n)                 (((n&0xF)<<8))

/* ------------------- CHECK PARAM DEFINITIONS ------------------------- */
#define PARAM_ADCx(n)    (((uint32_t *)n)==((uint32_t *)LPC_ADC))

#define PARAM_ADC_START_ON_EDGE_OPT(OPT)    ((OPT == ADC_START_ON_RISING)||(OPT == ADC_START_ON_FALLING))

#define PARAM_ADC_DATA_STATUS(OPT)    ((OPT== ADC_DATA_BURST)||(OPT== ADC_DATA_DONE))

#define PARAM_ADC_RATE(rate)    ((rate>0)&&(rate<=200000))

#define PARAM_ADC_CHANNEL_SELECTION(SEL)        ((SEL == ADC_CHANNEL_0)||(ADC_CHANNEL_1)\
||(SEL == ADC_CHANNEL_2)|(ADC_CHANNEL_3)\
||(SEL == ADC_CHANNEL_4)||(ADC_CHANNEL_5)\
||(SEL == ADC_CHANNEL_6)||(ADC_CHANNEL_7))

#define PARAM_ADC_START_OPT(OPT)    ((OPT == ADC_START_CONTINUOUS)||(OPT == ADC_START_NOW)\
||(OPT == ADC_START_ON_EINT0)||(OPT == ADC_START_ON_CAP01)\
||(OPT == ADC_START_ON_MAT01)||(OPT == ADC_START_ON_MAT03)\
||(OPT == ADC_START_ON_MAT10)||(OPT == ADC_START_ON_MAT11))

#define PARAM_ADC_TYPE_INT_OPT(OPT)    ((OPT == ADC_ADINTEN0)||(OPT == ADC_ADINTEN1)\
||(OPT == ADC_ADINTEN2)||(OPT == ADC_ADINTEN3)\
||(OPT == ADC_ADINTEN4)||(OPT == ADC_ADINTEN5)\
||(OPT == ADC_ADINTEN6)||(OPT == ADC_ADINTEN7)\
||(OPT == ADC_ADGINTEN))

/* Public Types --------------------------------------------------------------- */
/*********************************************************************/
typedef enum
{
        ADC_CHANNEL_0  = 0, 
        ADC_CHANNEL_1,          
        ADC_CHANNEL_2,          
        ADC_CHANNEL_3,          
        ADC_CHANNEL_4,          
        ADC_CHANNEL_5,          
        ADC_CHANNEL_6,          
        ADC_CHANNEL_7           
}ADC_CHANNEL_SELECTION;

typedef enum
{
        ADC_START_CONTINUOUS =0,        
        ADC_START_NOW,                          
        ADC_START_ON_EINT0,                     
        ADC_START_ON_CAP01,                     
        ADC_START_ON_MAT01,                     
        ADC_START_ON_MAT03,                     
        ADC_START_ON_MAT10,                     
        ADC_START_ON_MAT11                      
} ADC_START_OPT;


typedef enum
{
        ADC_START_ON_RISING = 0,        
        ADC_START_ON_FALLING            
} ADC_START_ON_EDGE_OPT;

typedef enum
{
        ADC_ADINTEN0 = 0,               
        ADC_ADINTEN1,                   
        ADC_ADINTEN2,                   
        ADC_ADINTEN3,                   
        ADC_ADINTEN4,                   
        ADC_ADINTEN5,                   
        ADC_ADINTEN6,                   
        ADC_ADINTEN7,                   
        ADC_ADGINTEN                    
}ADC_TYPE_INT_OPT;

typedef enum
{
        ADC_DATA_BURST = 0,             /*Burst bit*/
        ADC_DATA_DONE            /*Done bit*/
}ADC_DATA_STATUS;

/* Public Functions ----------------------------------------------------------- */
/* Init/DeInit ADC peripheral ----------------*/
void ADC_Init(LPC_ADC_TypeDef *ADCx, uint32_t rate);
void ADC_DeInit(LPC_ADC_TypeDef *ADCx);

/* Enable/Disable ADC functions --------------*/
void ADC_BurstCmd(LPC_ADC_TypeDef *ADCx, FunctionalState NewState);
void ADC_PowerdownCmd(LPC_ADC_TypeDef *ADCx, FunctionalState NewState);
void ADC_StartCmd(LPC_ADC_TypeDef *ADCx, uint8_t start_mode);
void ADC_ChannelCmd (LPC_ADC_TypeDef *ADCx, uint8_t Channel, FunctionalState NewState);

/* Configure ADC functions -------------------*/
void ADC_EdgeStartConfig(LPC_ADC_TypeDef *ADCx, uint8_t EdgeOption);
void ADC_IntConfig (LPC_ADC_TypeDef *ADCx, ADC_TYPE_INT_OPT IntType, FunctionalState NewState);

/* Get ADC information functions -------------------*/
uint16_t ADC_ChannelGetData(LPC_ADC_TypeDef *ADCx, uint8_t channel);
FlagStatus ADC_ChannelGetStatus(LPC_ADC_TypeDef *ADCx, uint8_t channel, uint32_t StatusType);
uint32_t ADC_GlobalGetData(LPC_ADC_TypeDef *ADCx);
FlagStatus      ADC_GlobalGetStatus(LPC_ADC_TypeDef *ADCx, uint32_t StatusType);

#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_ADC_H_ */

/* --------------------------------- End Of File ------------------------------ */