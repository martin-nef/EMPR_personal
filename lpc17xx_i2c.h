/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
#ifndef LPC17XX_I2C_H_
#define LPC17XX_I2C_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif


/* Private Macros ------------------------------------------------------------- */
/* --------------------- BIT DEFINITIONS -------------------------------------- */
/*******************************************************************/
#define I2C_I2CONSET_AA                         ((0x04)) 
#define I2C_I2CONSET_SI                         ((0x08)) 
#define I2C_I2CONSET_STO                        ((0x10)) 
#define I2C_I2CONSET_STA                        ((0x20)) 
#define I2C_I2CONSET_I2EN                       ((0x40)) 
/*******************************************************************/
#define I2C_I2CONCLR_AAC                        ((1<<2))

#define I2C_I2CONCLR_SIC                        ((1<<3))

#define I2C_I2CONCLR_STAC                       ((1<<5))

#define I2C_I2CONCLR_I2ENC                      ((1<<6))

/********************************************************************/
/* Return Code in I2C status register */
#define I2C_STAT_CODE_BITMASK           ((0xF8))

/* I2C return status code definitions ----------------------------- */

#define I2C_I2STAT_NO_INF                                               ((0xF8))

/* Master transmit mode -------------------------------------------- */
#define I2C_I2STAT_M_TX_START                                   ((0x08))

#define I2C_I2STAT_M_TX_RESTART                                 ((0x10))

#define I2C_I2STAT_M_TX_SLAW_ACK                                ((0x18))

#define I2C_I2STAT_M_TX_SLAW_NACK                               ((0x20))

#define I2C_I2STAT_M_TX_DAT_ACK                                 ((0x28))

#define I2C_I2STAT_M_TX_DAT_NACK                                ((0x30))

#define I2C_I2STAT_M_TX_ARB_LOST                                ((0x38))

/* Master receive mode -------------------------------------------- */
#define I2C_I2STAT_M_RX_START                                   ((0x08))

#define I2C_I2STAT_M_RX_RESTART                                 ((0x10))

#define I2C_I2STAT_M_RX_ARB_LOST                                ((0x38))

#define I2C_I2STAT_M_RX_SLAR_ACK                                ((0x40))

#define I2C_I2STAT_M_RX_SLAR_NACK                               ((0x48))

#define I2C_I2STAT_M_RX_DAT_ACK                                 ((0x50))

#define I2C_I2STAT_M_RX_DAT_NACK                                ((0x58))

/* Slave receive mode -------------------------------------------- */
#define I2C_I2STAT_S_RX_SLAW_ACK                                ((0x60))

#define I2C_I2STAT_S_RX_ARB_LOST_M_SLA                  ((0x68))

//#define I2C_I2STAT_S_RX_SLAW_ACK                              ((0x68))

#define I2C_I2STAT_S_RX_GENCALL_ACK                             ((0x70))

#define I2C_I2STAT_S_RX_ARB_LOST_M_GENCALL              ((0x78))

//#define I2C_I2STAT_S_RX_GENCALL_ACK                           ((0x78))

#define I2C_I2STAT_S_RX_PRE_SLA_DAT_ACK                 ((0x80))

#define I2C_I2STAT_S_RX_PRE_SLA_DAT_NACK                ((0x88))

#define I2C_I2STAT_S_RX_PRE_GENCALL_DAT_ACK             ((0x90))

#define I2C_I2STAT_S_RX_PRE_GENCALL_DAT_NACK    ((0x98))

#define I2C_I2STAT_S_RX_STA_STO_SLVREC_SLVTRX   ((0xA0))

#define I2C_I2STAT_S_TX_SLAR_ACK                                ((0xA8))

#define I2C_I2STAT_S_TX_ARB_LOST_M_SLA                  ((0xB0))

//#define I2C_I2STAT_S_TX_SLAR_ACK                              ((0xB0))

#define I2C_I2STAT_S_TX_DAT_ACK                                 ((0xB8))

#define I2C_I2STAT_S_TX_DAT_NACK                                ((0xC0))

#define I2C_I2STAT_S_TX_LAST_DAT_ACK                    ((0xC8))

#define I2C_SLAVE_TIME_OUT                                              0x10000UL

/********************************************************************/
#define I2C_I2DAT_BITMASK                       ((0xFF))

#define I2C_I2DAT_IDLE_CHAR                     (0xFF)

/********************************************************************/
#define I2C_I2MMCTRL_MM_ENA                     ((1<<0))                
#define I2C_I2MMCTRL_ENA_SCL            ((1<<1))                
#define I2C_I2MMCTRL_MATCH_ALL          ((1<<2))                
#define I2C_I2MMCTRL_BITMASK            ((0x07))                
/********************************************************************/
#define I2DATA_BUFFER_BITMASK           ((0xFF))

/********************************************************************/
#define I2C_I2ADR_GC                            ((1<<0))

#define I2C_I2ADR_BITMASK                       ((0xFF))

/********************************************************************/
#define I2C_I2MASK_MASK(n)                      ((n&0xFE))

/********************************************************************/
#define I2C_I2SCLH_BITMASK                      ((0xFFFF))

/********************************************************************/
#define I2C_I2SCLL_BITMASK                      ((0xFFFF))

/* I2C status values */
#define I2C_SETUP_STATUS_ARBF   (1<<8)  
#define I2C_SETUP_STATUS_NOACKF (1<<9)  
#define I2C_SETUP_STATUS_DONE   (1<<10) 
/*********************************************************************/
#define I2C_MONITOR_CFG_SCL_OUTPUT      I2C_I2MMCTRL_ENA_SCL            
#define I2C_MONITOR_CFG_MATCHALL        I2C_I2MMCTRL_MATCH_ALL          
/* ---------------- CHECK PARAMETER DEFINITIONS ---------------------------- */
/* Macros check I2C slave address */
#define PARAM_I2C_SLAVEADDR_CH(n)       ((n>=0) && (n<=3))

#define PARAM_I2Cx(n)   ((((uint32_t *)n)==((uint32_t *)LPC_I2C0)) \
|| (((uint32_t *)n)==((uint32_t *)LPC_I2C1)) \
|| (((uint32_t *)n)==((uint32_t *)LPC_I2C2)))

/* Macros check I2C monitor configuration type */
#define PARAM_I2C_MONITOR_CFG(n) ((n==I2C_MONITOR_CFG_SCL_OUTPUT) || (I2C_MONITOR_CFG_MATCHALL))

/* Public Types --------------------------------------------------------------- */
typedef struct {
        uint8_t SlaveAddrChannel;       
        uint8_t SlaveAddr_7bit;         
        uint8_t GeneralCallState;       
        uint8_t SlaveAddrMaskValue;     
} I2C_OWNSLAVEADDR_CFG_Type;


typedef struct
{
  uint32_t          sl_addr7bit;                                
  uint8_t*          tx_data;                                    
  uint32_t          tx_length;                                  
  uint32_t          tx_count;                                   
  uint8_t*          rx_data;                                    
  uint32_t          rx_length;                                  
  uint32_t          rx_count;                                   
  uint32_t          retransmissions_max;                
  uint32_t          retransmissions_count;              
  uint32_t          status;                                             
  void                          (*callback)(void);                      
} I2C_M_SETUP_Type;


typedef struct
{
  uint8_t*          tx_data;
  uint32_t          tx_length;
  uint32_t          tx_count;
  uint8_t*          rx_data;
  uint32_t          rx_length;
  uint32_t          rx_count;
  uint32_t          status;
  void                          (*callback)(void);
} I2C_S_SETUP_Type;

typedef enum {
        I2C_TRANSFER_POLLING = 0,               
        I2C_TRANSFER_INTERRUPT                  
} I2C_TRANSFER_OPT_Type;


/* Public Functions ----------------------------------------------------------- */
/* I2C Init/DeInit functions ---------- */
void I2C_Init(LPC_I2C_TypeDef *I2Cx, uint32_t clockrate);
void I2C_DeInit(LPC_I2C_TypeDef* I2Cx);
//void I2C_SetClock (LPC_I2C_TypeDef *I2Cx, uint32_t target_clock);
void I2C_Cmd(LPC_I2C_TypeDef* I2Cx, FunctionalState NewState);

/* I2C transfer data functions -------- */
Status I2C_MasterTransferData(LPC_I2C_TypeDef *I2Cx, \
                I2C_M_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
Status I2C_SlaveTransferData(LPC_I2C_TypeDef *I2Cx, \
                I2C_S_SETUP_Type *TransferCfg, I2C_TRANSFER_OPT_Type Opt);
uint32_t I2C_MasterTransferComplete(LPC_I2C_TypeDef *I2Cx);
uint32_t I2C_SlaveTransferComplete(LPC_I2C_TypeDef *I2Cx);


void I2C_SetOwnSlaveAddr(LPC_I2C_TypeDef *I2Cx, I2C_OWNSLAVEADDR_CFG_Type *OwnSlaveAddrConfigStruct);
uint8_t I2C_GetLastStatusCode(LPC_I2C_TypeDef* I2Cx);

/* I2C Monitor functions ---------------*/
void I2C_MonitorModeConfig(LPC_I2C_TypeDef *I2Cx, uint32_t MonitorCfgType, FunctionalState NewState);
void I2C_MonitorModeCmd(LPC_I2C_TypeDef *I2Cx, FunctionalState NewState);
uint8_t I2C_MonitorGetDatabuffer(LPC_I2C_TypeDef *I2Cx);
BOOL_8 I2C_MonitorHandler(LPC_I2C_TypeDef *I2Cx, uint8_t *buffer, uint32_t size);

/* I2C Interrupt handler functions ------*/
void I2C_IntCmd (LPC_I2C_TypeDef *I2Cx, Bool NewState);
void I2C_MasterHandler (LPC_I2C_TypeDef *I2Cx);
void I2C_SlaveHandler (LPC_I2C_TypeDef *I2Cx);


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_I2C_H_ */

/* --------------------------------- End Of File ------------------------------ */