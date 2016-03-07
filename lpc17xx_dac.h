/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_DAC_H_
#define LPC17XX_DAC_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
#define DAC_VALUE(n)            ((uint32_t)((n&0x3FF)<<6))

#define DAC_BIAS_EN                     ((uint32_t)(1<<16))

#define DAC_CCNT_VALUE(n)  ((uint32_t)(n&0xffff))

#define DAC_DBLBUF_ENA          ((uint32_t)(1<<1))

#define DAC_CNT_ENA                     ((uint32_t)(1<<2))

#define DAC_DMA_ENA                     ((uint32_t)(1<<3))

#define DAC_DACCTRL_MASK        ((uint32_t)(0x0F))

#define PARAM_DACx(n)   (((uint32_t *)n)==((uint32_t *)LPC_DAC))

#define PARAM_DAC_CURRENT_OPT(OPTION) ((OPTION == DAC_MAX_CURRENT_700uA)\
||(OPTION == DAC_MAX_CURRENT_350uA))

/* Public Types --------------------------------------------------------------- */
typedef enum
{
        DAC_MAX_CURRENT_700uA = 0,      
        DAC_MAX_CURRENT_350uA           
} DAC_CURRENT_OPT;

typedef struct
{

        uint8_t  DBLBUF_ENA;    
        uint8_t  CNT_ENA;               
        uint8_t  DMA_ENA;               
        uint8_t RESERVED;

} DAC_CONVERTER_CFG_Type;

/* Public Functions ----------------------------------------------------------- */
void    DAC_Init(LPC_DAC_TypeDef *DACx);
void    DAC_UpdateValue (LPC_DAC_TypeDef *DACx, uint32_t dac_value);
void    DAC_SetBias (LPC_DAC_TypeDef *DACx,uint32_t bias);
void    DAC_ConfigDAConverterControl (LPC_DAC_TypeDef *DACx,DAC_CONVERTER_CFG_Type *DAC_ConverterConfigStruct);
void    DAC_SetDMATimeOut(LPC_DAC_TypeDef *DACx,uint32_t time_out);

#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_DAC_H_ */

/* --------------------------------- End Of File ------------------------------ */
