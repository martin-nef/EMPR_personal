/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_PWM_H_
#define LPC17XX_PWM_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/* Private Macros ------------------------------------------------------------- */
/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
* IR register definitions
**********************************************************************/
#define PWM_IR_PWMMRn(n)        ((uint32_t)((n<4)?(1<<n):(1<<(n+4))))

#define PWM_IR_PWMCAPn(n)               ((uint32_t)(1<<(n+4)))

#define PWM_IR_BITMASK                  ((uint32_t)(0x0000073F))

/**********************************************************************
* TCR register definitions
**********************************************************************/
#define PWM_TCR_BITMASK                         ((uint32_t)(0x0000000B))
#define PWM_TCR_COUNTER_ENABLE      ((uint32_t)(1<<0)) 
#define PWM_TCR_COUNTER_RESET       ((uint32_t)(1<<1)) 
#define PWM_TCR_PWM_ENABLE          ((uint32_t)(1<<3)) 
/**********************************************************************
* CTCR register definitions
**********************************************************************/

#define PWM_CTCR_BITMASK                        ((uint32_t)(0x0000000F))

#define PWM_CTCR_MODE(n)                ((uint32_t)(n&0x03))

#define PWM_CTCR_SELECT_INPUT(n)        ((uint32_t)((n&0x03)<<2))

/**********************************************************************
* MCR register definitions
**********************************************************************/
#define PWM_MCR_BITMASK                         ((uint32_t)(0x001FFFFF))

#define PWM_MCR_INT_ON_MATCH(n)     ((uint32_t)(1<<(((n&0x7)<<1)+(n&0x07))))

#define PWM_MCR_RESET_ON_MATCH(n)   ((uint32_t)(1<<(((n&0x7)<<1)+(n&0x07)+1)))

#define PWM_MCR_STOP_ON_MATCH(n)    ((uint32_t)(1<<(((n&0x7)<<1)+(n&0x07)+2)))

/**********************************************************************
* CCR register definitions
**********************************************************************/
#define PWM_CCR_BITMASK                         ((uint32_t)(0x0000003F))

#define PWM_CCR_CAP_RISING(n)           ((uint32_t)(1<<(((n&0x2)<<1)+(n&0x1))))

#define PWM_CCR_CAP_FALLING(n)          ((uint32_t)(1<<(((n&0x2)<<1)+(n&0x1)+1)))

#define PWM_CCR_INT_ON_CAP(n)           ((uint32_t)(1<<(((n&0x2)<<1)+(n&0x1)+2)))

/**********************************************************************
* PCR register definitions
**********************************************************************/
#define PWM_PCR_BITMASK                 (uint32_t)0x00007E7C

#define PWM_PCR_PWMSELn(n)      ((uint32_t)(((n&0x7)<2) ? 0 : (1<<n)))

#define PWM_PCR_PWMENAn(n)      ((uint32_t)(((n&0x7)<1) ? 0 : (1<<(n+8))))

/**********************************************************************
* LER register definitions
**********************************************************************/
#define PWM_LER_BITMASK                         ((uint32_t)(0x0000007F))

#define PWM_LER_EN_MATCHn_LATCH(n)   ((uint32_t)((n<7) ? (1<<n) : 0))

/* ---------------- CHECK PARAMETER DEFINITIONS ---------------------------- */
#define PARAM_PWMx(n)   (((uint32_t *)n)==((uint32_t *)LPC_PWM1))

#define PARAM_PWM1_MATCH_CHANNEL(n)             ((n>=0) && (n<=6))

#define PARAM_PWM1_CHANNEL(n)                   ((n>=1) && (n<=6))

#define PARAM_PWM1_EDGE_MODE_CHANNEL(n)                 ((n>=2) && (n<=6))

#define PARAM_PWM1_CAPTURE_CHANNEL(n)   ((n==0) || (n==1))

#define PARAM_PWM_INTSTAT(n)    ((n==PWM_INTSTAT_MR0) || (n==PWM_INTSTAT_MR1) || (n==PWM_INTSTAT_MR2) \
|| (n==PWM_INTSTAT_MR3) || (n==PWM_INTSTAT_MR4) || (n==PWM_INTSTAT_MR5) \
|| (n==PWM_INTSTAT_MR6) || (n==PWM_INTSTAT_CAP0) || (n==PWM_INTSTAT_CAP1))

/* Public Types --------------------------------------------------------------- */
typedef struct {

        uint8_t PrescaleOption;         
        uint8_t Reserved[3];
        uint32_t PrescaleValue;         
} PWM_TIMERCFG_Type;

typedef struct {

        uint8_t CounterOption;          
        uint8_t CountInputSelect;       
        uint8_t Reserved[2];
} PWM_COUNTERCFG_Type;

typedef struct {
        uint8_t MatchChannel;   
        uint8_t IntOnMatch;             
        uint8_t StopOnMatch;    
        uint8_t ResetOnMatch;   
} PWM_MATCHCFG_Type;


typedef struct {
        uint8_t CaptureChannel; 
        uint8_t RisingEdge;             
        uint8_t FallingEdge;            
        uint8_t IntOnCaption;   
} PWM_CAPTURECFG_Type;

/* Timer/Counter in PWM configuration type definition -----------------------------------*/

typedef enum {
        PWM_MODE_TIMER = 0,             
        PWM_MODE_COUNTER,               
} PWM_TC_MODE_OPT;

#define PARAM_PWM_TC_MODE(n) ((n==PWM_MODE_TIMER) || (n==PWM_MODE_COUNTER))


typedef enum
{
        PWM_TIMER_PRESCALE_TICKVAL = 0,                 
        PWM_TIMER_PRESCALE_USVAL                                
} PWM_TIMER_PRESCALE_OPT;

#define PARAM_PWM_TIMER_PRESCALE(n) ((n==PWM_TIMER_PRESCALE_TICKVAL) || (n==PWM_TIMER_PRESCALE_USVAL))


typedef enum {
        PWM_COUNTER_PCAP1_0 = 0,                
        PWM_COUNTER_PCAP1_1                     
} PWM_COUNTER_INPUTSEL_OPT;

#define PARAM_PWM_COUNTER_INPUTSEL(n) ((n==PWM_COUNTER_PCAP1_0) || (n==PWM_COUNTER_PCAP1_1))

typedef enum {
    PWM_COUNTER_RISING = 1,             
    PWM_COUNTER_FALLING = 2,    
    PWM_COUNTER_ANY = 3                 
} PWM_COUNTER_EDGE_OPT;

#define PARAM_PWM_COUNTER_EDGE(n)       ((n==PWM_COUNTER_RISING) || (n==PWM_COUNTER_FALLING) \
|| (n==PWM_COUNTER_ANY))


/* PWM configuration type definition ----------------------------------------------------- */
typedef enum {
    PWM_CHANNEL_SINGLE_EDGE,    
    PWM_CHANNEL_DUAL_EDGE               
} PWM_CHANNEL_EDGE_OPT;

#define PARAM_PWM_CHANNEL_EDGE(n)       ((n==PWM_CHANNEL_SINGLE_EDGE) || (n==PWM_CHANNEL_DUAL_EDGE))


typedef enum {
        PWM_MATCH_UPDATE_NOW = 0,                       
        PWM_MATCH_UPDATE_NEXT_RST                       
} PWM_MATCH_UPDATE_OPT;

#define PARAM_PWM_MATCH_UPDATE(n)       ((n==PWM_MATCH_UPDATE_NOW) || (n==PWM_MATCH_UPDATE_NEXT_RST))


typedef enum
{
        PWM_INTSTAT_MR0 = PWM_IR_PWMMRn(0),     
        PWM_INTSTAT_MR1 = PWM_IR_PWMMRn(1),             
        PWM_INTSTAT_MR2 = PWM_IR_PWMMRn(2),             
        PWM_INTSTAT_MR3 = PWM_IR_PWMMRn(3),             
        PWM_INTSTAT_CAP0 = PWM_IR_PWMCAPn(0),   
        PWM_INTSTAT_CAP1 = PWM_IR_PWMCAPn(1),   
        PWM_INTSTAT_MR4 = PWM_IR_PWMMRn(4),             
        PWM_INTSTAT_MR6 = PWM_IR_PWMMRn(5),             
        PWM_INTSTAT_MR5 = PWM_IR_PWMMRn(6),             
}PWM_INTSTAT_TYPE;


/* Public Functions ----------------------------------------------------------- */
void PWM_PinConfig(LPC_PWM_TypeDef *PWMx, uint8_t PWM_Channel, uint8_t PinselOption);
IntStatus PWM_GetIntStatus(LPC_PWM_TypeDef *PWMx, uint32_t IntFlag);
void PWM_ClearIntPending(LPC_PWM_TypeDef *PWMx, uint32_t IntFlag);
void PWM_ConfigStructInit(uint8_t PWMTimerCounterMode, void *PWM_InitStruct);
void PWM_Init(LPC_PWM_TypeDef *PWMx, uint32_t PWMTimerCounterMode, void *PWM_ConfigStruct);
void PWM_DeInit (LPC_PWM_TypeDef *PWMx);
void PWM_Cmd(LPC_PWM_TypeDef *PWMx, FunctionalState NewState);
void PWM_CounterCmd(LPC_PWM_TypeDef *PWMx, FunctionalState NewState);
void PWM_ResetCounter(LPC_PWM_TypeDef *PWMx);
void PWM_ConfigMatch(LPC_PWM_TypeDef *PWMx, PWM_MATCHCFG_Type *PWM_MatchConfigStruct);
void PWM_ConfigCapture(LPC_PWM_TypeDef *PWMx, PWM_CAPTURECFG_Type *PWM_CaptureConfigStruct);
uint32_t PWM_GetCaptureValue(LPC_PWM_TypeDef *PWMx, uint8_t CaptureChannel);
void PWM_MatchUpdate(LPC_PWM_TypeDef *PWMx, uint8_t MatchChannel, \
                                        uint32_t MatchValue, uint8_t UpdateType);
void PWM_ChannelConfig(LPC_PWM_TypeDef *PWMx, uint8_t PWMChannel, uint8_t ModeOption);
void PWM_ChannelCmd(LPC_PWM_TypeDef *PWMx, uint8_t PWMChannel, FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_PWM_H_ */

/* --------------------------------- End Of File ------------------------------ */