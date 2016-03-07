/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_i2c.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"


/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */


#ifdef _I2C


/* Private Types -------------------------------------------------------------- */
typedef struct
{
  uint32_t      txrx_setup;                                             /* Transmission setup */
  int32_t               dir;                                                            /* Current direction phase, 0 - write, 1 - read */
} I2C_CFG_T;

/* Private Variables ---------------------------------------------------------- */
static I2C_CFG_T i2cdat[3];

static uint32_t I2C_MasterComplete[3];
static uint32_t I2C_SlaveComplete[3];

static uint32_t I2C_MonitorBufferIndex;

/* Private Functions ---------------------------------------------------------- */

/* Get I2C number */
static int32_t I2C_getNum(LPC_I2C_TypeDef *I2Cx);

/* Generate a start condition on I2C bus (in master mode only) */
static uint32_t I2C_Start (LPC_I2C_TypeDef *I2Cx);

/* Generate a stop condition on I2C bus (in master mode only) */
static void I2C_Stop (LPC_I2C_TypeDef *I2Cx);

/* I2C send byte subroutine */
static uint32_t I2C_SendByte (LPC_I2C_TypeDef *I2Cx, uint8_t databyte);

/* I2C get byte subroutine */
static uint32_t I2C_GetByte (LPC_I2C_TypeDef *I2Cx, uint8_t *retdat, Bool ack);

/* I2C set clock (hz) */
static void I2C_SetClock (LPC_I2C_TypeDef *I2Cx, uint32_t target_clock);

/*--------------------------------------------------------------------------------*/
/********************************************************************/
static int32_t I2C_getNum(LPC_I2C_TypeDef *I2Cx){
        if (I2Cx == LPC_I2C0) {
                return (0);
        } else if (I2Cx == LPC_I2C1) {
                return (1);
        } else if (I2Cx == LPC_I2C2) {
                return (2);
        }
        return (-1);
}

/********************************************************************/
static uint32_t I2C_Start (LPC_I2C_TypeDef *I2Cx)
{
        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
        I2Cx->I2CONSET = I2C_I2CONSET_STA;

        // Wait for complete
        while (!(I2Cx->I2CONSET & I2C_I2CONSET_SI));
        I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
        return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}

/********************************************************************/
static void I2C_Stop (LPC_I2C_TypeDef *I2Cx)
{

        /* Make sure start bit is not active */
        if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
        {
                I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
        }
        I2Cx->I2CONSET = I2C_I2CONSET_STO;
        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
}

/********************************************************************/
static uint32_t I2C_SendByte (LPC_I2C_TypeDef *I2Cx, uint8_t databyte)
{
        /* Make sure start bit is not active */
        if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
        {
                I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
        }
        I2Cx->I2DAT = databyte & I2C_I2DAT_BITMASK;
        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;

        while (!(I2Cx->I2CONSET & I2C_I2CONSET_SI));
        return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}

/********************************************************************/
static uint32_t I2C_GetByte (LPC_I2C_TypeDef *I2Cx, uint8_t *retdat, Bool ack)
{
        if (ack == TRUE)
        {
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
        }
        else
        {
                I2Cx->I2CONCLR = I2C_I2CONCLR_AAC;
        }
        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;

        while (!(I2Cx->I2CONSET & I2C_I2CONSET_SI));
        *retdat = (uint8_t) (I2Cx->I2DAT & I2C_I2DAT_BITMASK);
        return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}

/*********************************************************************/
static void I2C_SetClock (LPC_I2C_TypeDef *I2Cx, uint32_t target_clock)
{
        uint32_t temp;

        CHECK_PARAM(PARAM_I2Cx(I2Cx));

        // Get PCLK of I2C controller
        if (I2Cx == LPC_I2C0)
        {
                temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C0) / target_clock;
        }
        else if (I2Cx == LPC_I2C1)
        {
                temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C1) / target_clock;
        }
        else if (I2Cx == LPC_I2C2)
        {
                temp = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_I2C1) / target_clock;
        }

        /* Set the I2C clock value to register */
        I2Cx->I2SCLH = (uint32_t)(temp / 2);
        I2Cx->I2SCLL = (uint32_t)(temp - I2Cx->I2SCLH);
}
/* End of Private Functions --------------------------------------------------- */


/* Public Functions ----------------------------------------------------------- */
/********************************************************************/
void I2C_Init(LPC_I2C_TypeDef *I2Cx, uint32_t clockrate)
{
        CHECK_PARAM(PARAM_I2Cx(I2Cx));

        if (I2Cx==LPC_I2C0)
        {
                /* Set up clock and power for I2C0 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, ENABLE);
                /* As default, peripheral clock for I2C0 module
                 * is set to FCCLK / 2 */
                CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C0, CLKPWR_PCLKSEL_CCLK_DIV_2);
        }
        else if (I2Cx==LPC_I2C1)
        {
                /* Set up clock and power for I2C1 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C1, ENABLE);
                /* As default, peripheral clock for I2C1 module
                 * is set to FCCLK / 2 */
                CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C1, CLKPWR_PCLKSEL_CCLK_DIV_2);
        }
        else if (I2Cx==LPC_I2C2)
        {
                /* Set up clock and power for I2C2 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C2, ENABLE);
                /* As default, peripheral clock for I2C2 module
                 * is set to FCCLK / 2 */
                CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_I2C2, CLKPWR_PCLKSEL_CCLK_DIV_2);
        }
        else {
                // Up-Support this device
                return;
        }

    /* Set clock rate */
    I2C_SetClock(I2Cx, clockrate);
    /* Set I2C operation to default */
    I2Cx->I2CONCLR = (I2C_I2CONCLR_AAC | I2C_I2CONCLR_STAC | I2C_I2CONCLR_I2ENC);
}

/*********************************************************************/
void I2C_DeInit(LPC_I2C_TypeDef* I2Cx)
{
        CHECK_PARAM(PARAM_I2Cx(I2Cx));

        /* Disable I2C control */
        I2Cx->I2CONCLR = I2C_I2CONCLR_I2ENC;

        if (I2Cx==LPC_I2C0)
        {
                /* Disable power for I2C0 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C0, DISABLE);
        }
        else if (I2Cx==LPC_I2C1)
        {
                /* Disable power for I2C1 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C1, DISABLE);
        }
        else if (I2Cx==LPC_I2C2)
        {
                /* Disable power for I2C2 module */
                CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCI2C2, DISABLE);
        }
}

/*********************************************************************/
void I2C_Cmd(LPC_I2C_TypeDef* I2Cx, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));
        CHECK_PARAM(PARAM_I2Cx(I2Cx));

        if (NewState == ENABLE)
        {
                I2Cx->I2CONSET = I2C_I2CONSET_I2EN;
        }
        else
        {
                I2Cx->I2CONCLR = I2C_I2CONCLR_I2ENC;
        }
}

/*********************************************************************/
void I2C_IntCmd (LPC_I2C_TypeDef *I2Cx, Bool NewState)
{
        if (NewState)
        {
                if(I2Cx == LPC_I2C0)
                {
                        NVIC_EnableIRQ(I2C0_IRQn);
                }
                else if (I2Cx == LPC_I2C1)
                {
                        NVIC_EnableIRQ(I2C1_IRQn);
                }
                else if (I2Cx == LPC_I2C2)
                {
                        NVIC_EnableIRQ(I2C2_IRQn);
                }
        }
        else
        {
                if(I2Cx == LPC_I2C0)
                {
                        NVIC_DisableIRQ(I2C0_IRQn);
                }
                else if (I2Cx == LPC_I2C1)
                {
                        NVIC_DisableIRQ(I2C1_IRQn);
                }
                else if (I2Cx == LPC_I2C2)
                {
                        NVIC_DisableIRQ(I2C2_IRQn);
                }
        }
    return;
}


/*********************************************************************/
void I2C_MasterHandler (LPC_I2C_TypeDef  *I2Cx)
{
        int32_t tmp;
        uint8_t returnCode;
        I2C_M_SETUP_Type *txrx_setup;

        tmp = I2C_getNum(I2Cx);
        txrx_setup = (I2C_M_SETUP_Type *) i2cdat[tmp].txrx_setup;

        returnCode = (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
        // Save current status
        txrx_setup->status = returnCode;
        // there's no relevant information
        if (returnCode == I2C_I2STAT_NO_INF){
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                return;
        }

        /* ----------------------------- TRANSMIT PHASE --------------------------*/
        if (i2cdat[tmp].dir == 0){
                switch (returnCode)
                {
                /* A start/repeat start condition has been transmitted -------------------*/
                case I2C_I2STAT_M_TX_START:
                case I2C_I2STAT_M_TX_RESTART:
                        I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
                        /*
                         * If there's any transmit data, then start to
                         * send SLA+W right now, otherwise check whether if there's
                         * any receive data for next state.
                         */
                        if ((txrx_setup->tx_data != NULL) && (txrx_setup->tx_length != 0)){
                                I2Cx->I2DAT = (txrx_setup->sl_addr7bit << 1);
                                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                        } else {
                                goto next_stage;
                        }
                        break;

                /* SLA+W has been transmitted, ACK has been received ----------------------*/
                case I2C_I2STAT_M_TX_SLAW_ACK:
                /* Data has been transmitted, ACK has been received */
                case I2C_I2STAT_M_TX_DAT_ACK:
                        /* Send more data */
                        if ((txrx_setup->tx_count < txrx_setup->tx_length) \
                                        && (txrx_setup->tx_data != NULL)){
                                I2Cx->I2DAT =  *(uint8_t *)(txrx_setup->tx_data + txrx_setup->tx_count);
                                txrx_setup->tx_count++;
                                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                        }
                        // no more data, switch to next stage
                        else {
next_stage:
                                // change direction
                                i2cdat[tmp].dir = 1;
                                // Check if any data to receive
                                if ((txrx_setup->rx_length != 0) && (txrx_setup->rx_data != NULL)){
                                                // check whether if we need to issue an repeat start
                                                if ((txrx_setup->tx_length != 0) && (txrx_setup->tx_data != NULL)){
                                                        // Send out an repeat start command
                                                        I2Cx->I2CONSET = I2C_I2CONSET_STA;
                                                        I2Cx->I2CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC;
                                                }
                                                // Don't need issue an repeat start, just goto send SLA+R
                                                else {
                                                        goto send_slar;
                                                }
                                }
                                // no more data send, the go to end stage now
                                else {
                                        // success, goto end stage
                                        txrx_setup->status |= I2C_SETUP_STATUS_DONE;
                                        goto end_stage;
                                }
                        }
                        break;

                /* SLA+W has been transmitted, NACK has been received ----------------------*/
                case I2C_I2STAT_M_TX_SLAW_NACK:
                /* Data has been transmitted, NACK has been received -----------------------*/
                case I2C_I2STAT_M_TX_DAT_NACK:
                        // update status
                        txrx_setup->status |= I2C_SETUP_STATUS_NOACKF;
                        goto retry;
                /* Arbitration lost in SLA+R/W or Data bytes -------------------------------*/
                case I2C_I2STAT_M_TX_ARB_LOST:
                        // update status
                        txrx_setup->status |= I2C_SETUP_STATUS_ARBF;
                default:
                        goto retry;
                }
        }

        /* ----------------------------- RECEIVE PHASE --------------------------*/
        else if (i2cdat[tmp].dir == 1){
                switch (returnCode){
                        /* A start/repeat start condition has been transmitted ---------------------*/
                case I2C_I2STAT_M_RX_START:
                case I2C_I2STAT_M_RX_RESTART:
                        I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
                        /*
                         * If there's any receive data, then start to
                         * send SLA+R right now, otherwise check whether if there's
                         * any receive data for end of state.
                         */
                        if ((txrx_setup->rx_data != NULL) && (txrx_setup->rx_length != 0)){
send_slar:
                                I2Cx->I2DAT = (txrx_setup->sl_addr7bit << 1) | 0x01;
                                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                        } else {
                                // Success, goto end stage
                                txrx_setup->status |= I2C_SETUP_STATUS_DONE;
                                goto end_stage;
                        }
                        break;

                /* SLA+R has been transmitted, ACK has been received -----------------*/
                case I2C_I2STAT_M_RX_SLAR_ACK:
                        if (txrx_setup->rx_count < (txrx_setup->rx_length - 1)) {
                                /*Data will be received,  ACK will be return*/
                                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                        }
                        else {
                                /*Last data will be received,  NACK will be return*/
                                I2Cx->I2CONCLR = I2C_I2CONSET_AA;
                        }
                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                        break;

                /* Data has been received, ACK has been returned ----------------------*/
                case I2C_I2STAT_M_RX_DAT_ACK:
                        // Note save data and increase counter first, then check later
                        /* Save data  */
                        if ((txrx_setup->rx_data != NULL) && (txrx_setup->rx_count < txrx_setup->rx_length)){
                                *(uint8_t *)(txrx_setup->rx_data + txrx_setup->rx_count) = (I2Cx->I2DAT & I2C_I2DAT_BITMASK);
                                txrx_setup->rx_count++;
                        }
                        if (txrx_setup->rx_count < (txrx_setup->rx_length - 1)) {
                                /*Data will be received,  ACK will be return*/
                                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                        }
                        else {
                                /*Last data will be received,  NACK will be return*/
                                I2Cx->I2CONCLR = I2C_I2CONSET_AA;
                        }

                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                        break;

                /* Data has been received, NACK has been return -------------------------*/
                case I2C_I2STAT_M_RX_DAT_NACK:
                        /* Save the last data */
                        if ((txrx_setup->rx_data != NULL) && (txrx_setup->rx_count < txrx_setup->rx_length)){
                                *(uint8_t *)(txrx_setup->rx_data + txrx_setup->rx_count) = (I2Cx->I2DAT & I2C_I2DAT_BITMASK);
                                txrx_setup->rx_count++;
                        }
                        // success, go to end stage
                        txrx_setup->status |= I2C_SETUP_STATUS_DONE;
                        goto end_stage;

                /* SLA+R has been transmitted, NACK has been received ------------------*/
                case I2C_I2STAT_M_RX_SLAR_NACK:
                        // update status
                        txrx_setup->status |= I2C_SETUP_STATUS_NOACKF;
                        goto retry;

                /* Arbitration lost ----------------------------------------------------*/
                case I2C_I2STAT_M_RX_ARB_LOST:
                        // update status
                        txrx_setup->status |= I2C_SETUP_STATUS_ARBF;
                default:
retry:
                        // check if retransmission is available
                        if (txrx_setup->retransmissions_count < txrx_setup->retransmissions_max){
                                // Clear tx count
                                txrx_setup->tx_count = 0;
                                I2Cx->I2CONSET = I2C_I2CONSET_STA;
                                I2Cx->I2CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC;
                                txrx_setup->retransmissions_count++;
                        }
                        // End of stage
                        else {
end_stage:
                                // Disable interrupt
                                I2C_IntCmd(I2Cx, 0);
                                // Send stop
                                I2C_Stop(I2Cx);

                                I2C_MasterComplete[tmp] = TRUE;
                        }
                        break;
                }
        }
}


/*********************************************************************/
void I2C_SlaveHandler (LPC_I2C_TypeDef  *I2Cx)
{
        int32_t tmp;
        uint8_t returnCode;
        I2C_S_SETUP_Type *txrx_setup;
        uint32_t timeout;

        tmp = I2C_getNum(I2Cx);
        txrx_setup = (I2C_S_SETUP_Type *) i2cdat[tmp].txrx_setup;

        returnCode = (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
        // Save current status
        txrx_setup->status = returnCode;
        // there's no relevant information
        if (returnCode == I2C_I2STAT_NO_INF){
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                return;
        }


        switch (returnCode)
        {

        /* No status information */
        case I2C_I2STAT_NO_INF:
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                break;

        /* Reading phase -------------------------------------------------------- */
        /* Own SLA+R has been received, ACK has been returned */
        case I2C_I2STAT_S_RX_SLAW_ACK:
        /* General call address has been received, ACK has been returned */
        case I2C_I2STAT_S_RX_GENCALL_ACK:
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                break;

        /* Previously addressed with own SLA;
         * DATA byte has been received;
         * ACK has been returned */
        case I2C_I2STAT_S_RX_PRE_SLA_DAT_ACK:
        /* DATA has been received, ACK hasn been return */
        case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_ACK:
                /*
                 * All data bytes that over-flow the specified receive
                 * data length, just ignore them.
                 */
                if ((txrx_setup->rx_count < txrx_setup->rx_length) \
                                && (txrx_setup->rx_data != NULL)){
                        *(uint8_t *)(txrx_setup->rx_data + txrx_setup->rx_count) = (uint8_t)I2Cx->I2DAT;
                        txrx_setup->rx_count++;
                }
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                break;

        /* Previously addressed with own SLA;
         * DATA byte has been received;
         * NOT ACK has been returned */
        case I2C_I2STAT_S_RX_PRE_SLA_DAT_NACK:
        /* DATA has been received, NOT ACK has been returned */
        case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_NACK:
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                break;

        /*
         * Note that: Return code only let us know a stop condition mixed
         * with a repeat start condition in the same code value.
         * So we should provide a time-out. In case this is really a stop
         * condition, this will return back after time out condition. Otherwise,
         * next session that is slave receive data will be completed.
         */

        /* A Stop or a repeat start condition */
        case I2C_I2STAT_S_RX_STA_STO_SLVREC_SLVTRX:
                // Temporally lock the interrupt for timeout condition
                I2C_IntCmd(I2Cx, 0);
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                // enable time out
                timeout = I2C_SLAVE_TIME_OUT;
                while(1){
                        if (I2Cx->I2CONSET & I2C_I2CONSET_SI){
                                // re-Enable interrupt
                                I2C_IntCmd(I2Cx, 1);
                                break;
                        } else {
                                timeout--;
                                if (timeout == 0){
                                        // timeout occur, it's really a stop condition
                                        txrx_setup->status |= I2C_SETUP_STATUS_DONE;
                                        goto s_int_end;
                                }
                        }
                }
                break;

        /* Writing phase -------------------------------------------------------- */
        /* Own SLA+R has been received, ACK has been returned */
        case I2C_I2STAT_S_TX_SLAR_ACK:
        /* Data has been transmitted, ACK has been received */
        case I2C_I2STAT_S_TX_DAT_ACK:
                /*
                 * All data bytes that over-flow the specified receive
                 * data length, just ignore them.
                 */
                if ((txrx_setup->tx_count < txrx_setup->tx_length) \
                                && (txrx_setup->tx_data != NULL)){
                        I2Cx->I2DAT = *(uint8_t *) (txrx_setup->tx_data + txrx_setup->tx_count);
                        txrx_setup->tx_count++;
                }
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                break;

        /* Data has been transmitted, NACK has been received,
         * that means there's no more data to send, exit now */
        /*
         * Note: Don't wait for stop event since in slave transmit mode,
         * since there no proof lets us know when a stop signal has been received
         * on slave side.
         */
        case I2C_I2STAT_S_TX_DAT_NACK:
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                txrx_setup->status |= I2C_SETUP_STATUS_DONE;
                goto s_int_end;

        // Other status must be captured
        default:
s_int_end:
                // Disable interrupt
                I2C_IntCmd(I2Cx, 0);
                I2Cx->I2CONCLR = I2C_I2CONCLR_AAC | I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC;
                I2C_SlaveComplete[tmp] = TRUE;
                break;
        }
}

/*********************************************************************/
Status I2C_MasterTransferData(LPC_I2C_TypeDef *I2Cx, I2C_M_SETUP_Type *TransferCfg, \
                                                                I2C_TRANSFER_OPT_Type Opt)
{
        uint8_t *txdat;
        uint8_t *rxdat;
        uint32_t CodeStatus;
        uint8_t tmp;

        // reset all default state
        txdat = (uint8_t *) TransferCfg->tx_data;
        rxdat = (uint8_t *) TransferCfg->rx_data;
        // Reset I2C setup value to default state
        TransferCfg->tx_count = 0;
        TransferCfg->rx_count = 0;
        TransferCfg->status = 0;

        if (Opt == I2C_TRANSFER_POLLING){

                /* First Start condition -------------------------------------------------------------- */
                TransferCfg->retransmissions_count = 0;
retry:
                // reset all default state
                txdat = (uint8_t *) TransferCfg->tx_data;
                rxdat = (uint8_t *) TransferCfg->rx_data;
                // Reset I2C setup value to default state
                TransferCfg->tx_count = 0;
                TransferCfg->rx_count = 0;
                CodeStatus = 0;

                // Start command
                CodeStatus = I2C_Start(I2Cx);
                if ((CodeStatus != I2C_I2STAT_M_TX_START) \
                                && (CodeStatus != I2C_I2STAT_M_TX_RESTART)){
                        TransferCfg->retransmissions_count++;
                        if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                // save status
                                TransferCfg->status = CodeStatus;
                                goto error;
                        } else {
                                goto retry;
                        }
                }

                /* In case of sending data first --------------------------------------------------- */
                if ((TransferCfg->tx_length != 0) && (TransferCfg->tx_data != NULL)){

                        /* Send slave address + WR direction bit = 0 ----------------------------------- */
                        CodeStatus = I2C_SendByte(I2Cx, (TransferCfg->sl_addr7bit << 1));
                        if (CodeStatus != I2C_I2STAT_M_TX_SLAW_ACK){
                                TransferCfg->retransmissions_count++;
                                if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                        // save status
                                        TransferCfg->status = CodeStatus | I2C_SETUP_STATUS_NOACKF;
                                        goto error;
                                } else {
                                        goto retry;
                                }
                        }

                        /* Send a number of data bytes ---------------------------------------- */
                        while (TransferCfg->tx_count < TransferCfg->tx_length)
                        {
                                CodeStatus = I2C_SendByte(I2Cx, *txdat);
                                if (CodeStatus != I2C_I2STAT_M_TX_DAT_ACK){
                                        TransferCfg->retransmissions_count++;
                                        if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                                // save status
                                                TransferCfg->status = CodeStatus | I2C_SETUP_STATUS_NOACKF;
                                                goto error;
                                        } else {
                                                goto retry;
                                        }
                                }

                                txdat++;
                                TransferCfg->tx_count++;
                        }
                }

                /* Second Start condition (Repeat Start) ------------------------------------------- */
                if ((TransferCfg->tx_length != 0) && (TransferCfg->tx_data != NULL) \
                                && (TransferCfg->rx_length != 0) && (TransferCfg->rx_data != NULL)){

                        CodeStatus = I2C_Start(I2Cx);
                        if ((CodeStatus != I2C_I2STAT_M_RX_START) \
                                        && (CodeStatus != I2C_I2STAT_M_RX_RESTART)){
                                TransferCfg->retransmissions_count++;
                                if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                        // Update status
                                        TransferCfg->status = CodeStatus;
                                        goto error;
                                } else {
                                        goto retry;
                                }
                        }
                }

                /* Then, start reading after sending data -------------------------------------- */
                if ((TransferCfg->rx_length != 0) && (TransferCfg->rx_data != NULL)){
                        /* Send slave address + RD direction bit = 1 ----------------------------------- */

                        CodeStatus = I2C_SendByte(I2Cx, ((TransferCfg->sl_addr7bit << 1) | 0x01));
                        if (CodeStatus != I2C_I2STAT_M_RX_SLAR_ACK){
                                TransferCfg->retransmissions_count++;
                                if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                        // update status
                                        TransferCfg->status = CodeStatus | I2C_SETUP_STATUS_NOACKF;
                                        goto error;
                                } else {
                                        goto retry;
                                }
                        }

                        /* Receive a number of data bytes ------------------------------------------------- */
                        while (TransferCfg->rx_count < TransferCfg->rx_length){

                                /*
                                 * Note that: if data length is only one, the master should not
                                 * issue an ACK signal on bus after reading to avoid of next data frame
                                 * on slave side
                                 */
                                if (TransferCfg->rx_count < (TransferCfg->rx_length - 1)){
                                        // Issue an ACK signal for next data frame
                                        CodeStatus = I2C_GetByte(I2Cx, &tmp, 1);
                                        if (CodeStatus != I2C_I2STAT_M_RX_DAT_ACK){
                                                TransferCfg->retransmissions_count++;
                                                if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                                        // update status
                                                        TransferCfg->status = CodeStatus;
                                                        goto error;
                                                } else {
                                                        goto retry;
                                                }
                                        }
                                } else {
                                        // Do not issue an ACK signal
                                        CodeStatus = I2C_GetByte(I2Cx, &tmp, 0);
                                        if (CodeStatus != I2C_I2STAT_M_RX_DAT_NACK){
                                                TransferCfg->retransmissions_count++;
                                                if (TransferCfg->retransmissions_count > TransferCfg->retransmissions_max){
                                                        // update status
                                                        TransferCfg->status = CodeStatus;
                                                        goto error;
                                                } else {
                                                        goto retry;
                                                }
                                        }
                                }
                                *rxdat++ = tmp;
                                TransferCfg->rx_count++;
                        }
                }

                /* Send STOP condition ------------------------------------------------- */
                I2C_Stop(I2Cx);
                return SUCCESS;

error:
                // Send stop condition
                I2C_Stop(I2Cx);
                return ERROR;
        }

        else if (Opt == I2C_TRANSFER_INTERRUPT){
                // Setup tx_rx data, callback and interrupt handler
                tmp = I2C_getNum(I2Cx);
                i2cdat[tmp].txrx_setup = (uint32_t) TransferCfg;
                // Set direction phase, write first
                i2cdat[tmp].dir = 0;

                /* First Start condition -------------------------------------------------------------- */
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                I2Cx->I2CONSET = I2C_I2CONSET_STA;
                I2C_IntCmd(I2Cx, 1);

                return (SUCCESS);
        }

        return ERROR;
}

/*********************************************************************/
Status I2C_SlaveTransferData(LPC_I2C_TypeDef *I2Cx, I2C_S_SETUP_Type *TransferCfg, \
                                                                I2C_TRANSFER_OPT_Type Opt)
{
        uint8_t *txdat;
        uint8_t *rxdat;
        uint32_t CodeStatus;
        uint32_t timeout;
        int32_t time_en;
        int32_t tmp;

        // reset all default state
        txdat = (uint8_t *) TransferCfg->tx_data;
        rxdat = (uint8_t *) TransferCfg->rx_data;
        // Reset I2C setup value to default state
        TransferCfg->tx_count = 0;
        TransferCfg->rx_count = 0;
        TransferCfg->status = 0;


        // Polling option
        if (Opt == I2C_TRANSFER_POLLING){

                /* Set AA bit to ACK command on I2C bus */
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                /* Clear SI bit to be ready ... */
                I2Cx->I2CONCLR = (I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC);

                time_en = 0;
                timeout = 0;

                while (1)
                {
                        /* Check SI flag ready */
                        if (I2Cx->I2CONSET & I2C_I2CONSET_SI)
                        {
                                time_en = 0;

                                switch (CodeStatus = (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK))
                                {

                                /* No status information */
                                case I2C_I2STAT_NO_INF:
                                        I2Cx->I2CONSET = I2C_I2CONSET_AA;
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        break;

                                /* Reading phase -------------------------------------------------------- */
                                /* Own SLA+R has been received, ACK has been returned */
                                case I2C_I2STAT_S_RX_SLAW_ACK:
                                /* General call address has been received, ACK has been returned */
                                case I2C_I2STAT_S_RX_GENCALL_ACK:
                                        I2Cx->I2CONSET = I2C_I2CONSET_AA;
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        break;

                                /* Previously addressed with own SLA;
                                 * DATA byte has been received;
                                 * ACK has been returned */
                                case I2C_I2STAT_S_RX_PRE_SLA_DAT_ACK:
                                /* DATA has been received, ACK hasn been return */
                                case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_ACK:
                                        /*
                                         * All data bytes that over-flow the specified receive
                                         * data length, just ignore them.
                                         */
                                        if ((TransferCfg->rx_count < TransferCfg->rx_length) \
                                                        && (TransferCfg->rx_data != NULL)){
                                                *rxdat++ = (uint8_t)I2Cx->I2DAT;
                                                TransferCfg->rx_count++;
                                        }
                                        I2Cx->I2CONSET = I2C_I2CONSET_AA;
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        break;

                                /* Previously addressed with own SLA;
                                 * DATA byte has been received;
                                 * NOT ACK has been returned */
                                case I2C_I2STAT_S_RX_PRE_SLA_DAT_NACK:
                                /* DATA has been received, NOT ACK has been returned */
                                case I2C_I2STAT_S_RX_PRE_GENCALL_DAT_NACK:
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        break;

                                /*
                                 * Note that: Return code only let us know a stop condition mixed
                                 * with a repeat start condition in the same code value.
                                 * So we should provide a time-out. In case this is really a stop
                                 * condition, this will return back after time out condition. Otherwise,
                                 * next session that is slave receive data will be completed.
                                 */

                                /* A Stop or a repeat start condition */
                                case I2C_I2STAT_S_RX_STA_STO_SLVREC_SLVTRX:
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        // enable time out
                                        time_en = 1;
                                        timeout = 0;
                                        break;

                                /* Writing phase -------------------------------------------------------- */
                                /* Own SLA+R has been received, ACK has been returned */
                                case I2C_I2STAT_S_TX_SLAR_ACK:
                                /* Data has been transmitted, ACK has been received */
                                case I2C_I2STAT_S_TX_DAT_ACK:
                                        /*
                                         * All data bytes that over-flow the specified receive
                                         * data length, just ignore them.
                                         */
                                        if ((TransferCfg->tx_count < TransferCfg->tx_length) \
                                                        && (TransferCfg->tx_data != NULL)){
                                                I2Cx->I2DAT = *txdat++;
                                                TransferCfg->tx_count++;
                                        }
                                        I2Cx->I2CONSET = I2C_I2CONSET_AA;
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        break;

                                /* Data has been transmitted, NACK has been received,
                                 * that means there's no more data to send, exit now */
                                /*
                                 * Note: Don't wait for stop event since in slave transmit mode,
                                 * since there no proof lets us know when a stop signal has been received
                                 * on slave side.
                                 */
                                case I2C_I2STAT_S_TX_DAT_NACK:
                                        I2Cx->I2CONSET = I2C_I2CONSET_AA;
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        // enable time out
                                        time_en = 1;
                                        timeout = 0;
                                        break;

                                // Other status must be captured
                                default:
                                        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
                                        goto s_error;
                                }
                        } else if (time_en){
                                if (timeout++ > I2C_SLAVE_TIME_OUT){
                                        // it's really a stop condition, goto end stage
                                        goto s_end_stage;
                                }
                        }
                }

s_end_stage:
                /* Clear AA bit to disable ACK on I2C bus */
                I2Cx->I2CONCLR = I2C_I2CONCLR_AAC;
                // Check if there's no error during operation
                // Update status
                TransferCfg->status = CodeStatus | I2C_SETUP_STATUS_DONE;
                return SUCCESS;

s_error:
                /* Clear AA bit to disable ACK on I2C bus */
                I2Cx->I2CONCLR = I2C_I2CONCLR_AAC;
                // Update status
                TransferCfg->status = CodeStatus;
                return ERROR;
        }

        else if (Opt == I2C_TRANSFER_INTERRUPT){
                // Setup tx_rx data, callback and interrupt handler
                tmp = I2C_getNum(I2Cx);
                i2cdat[tmp].txrx_setup = (uint32_t) TransferCfg;
                // Set direction phase, read first
                i2cdat[tmp].dir = 1;

                // Enable AA
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC;
                I2C_IntCmd(I2Cx, 1);

                return (SUCCESS);
        }

        return ERROR;
}

/*********************************************************************/
void I2C_SetOwnSlaveAddr(LPC_I2C_TypeDef *I2Cx, I2C_OWNSLAVEADDR_CFG_Type *OwnSlaveAddrConfigStruct)
{
        uint32_t tmp;
        CHECK_PARAM(PARAM_I2Cx(I2Cx));
        CHECK_PARAM(PARAM_I2C_SLAVEADDR_CH(OwnSlaveAddrConfigStruct->SlaveAddrChannel));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(OwnSlaveAddrConfigStruct->GeneralCallState));

        tmp = (((uint32_t)(OwnSlaveAddrConfigStruct->SlaveAddr_7bit << 1)) \
                        | ((OwnSlaveAddrConfigStruct->GeneralCallState == ENABLE) ? 0x01 : 0x00))& I2C_I2ADR_BITMASK;
        switch (OwnSlaveAddrConfigStruct->SlaveAddrChannel)
        {
        case 0:
                I2Cx->I2ADR0 = tmp;
                I2Cx->I2MASK0 = I2C_I2MASK_MASK((uint32_t) \
                                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
                break;
        case 1:
                I2Cx->I2ADR1 = tmp;
                I2Cx->I2MASK1 = I2C_I2MASK_MASK((uint32_t) \
                                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
                break;
        case 2:
                I2Cx->I2ADR2 = tmp;
                I2Cx->I2MASK2 = I2C_I2MASK_MASK((uint32_t) \
                                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
                break;
        case 3:
                I2Cx->I2ADR3 = tmp;
                I2Cx->I2MASK3 = I2C_I2MASK_MASK((uint32_t) \
                                (OwnSlaveAddrConfigStruct->SlaveAddrMaskValue));
                break;
        }
}


/*********************************************************************/
void I2C_MonitorModeConfig(LPC_I2C_TypeDef *I2Cx, uint32_t MonitorCfgType, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_I2Cx(I2Cx));
        CHECK_PARAM(PARAM_I2C_MONITOR_CFG(MonitorCfgType));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

        if (NewState == ENABLE)
        {
                I2Cx->MMCTRL |= MonitorCfgType;
        }
        else
        {
                I2Cx->MMCTRL &= (~MonitorCfgType) & I2C_I2MMCTRL_BITMASK;
        }
}


/*********************************************************************/
void I2C_MonitorModeCmd(LPC_I2C_TypeDef *I2Cx, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_I2Cx(I2Cx));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

        if (NewState == ENABLE)
        {
                I2Cx->MMCTRL |= I2C_I2MMCTRL_MM_ENA;
                I2Cx->I2CONSET = I2C_I2CONSET_AA;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC;
        }
        else
        {
                I2Cx->MMCTRL &= (~I2C_I2MMCTRL_MM_ENA) & I2C_I2MMCTRL_BITMASK;
                I2Cx->I2CONCLR = I2C_I2CONCLR_SIC | I2C_I2CONCLR_STAC | I2C_I2CONCLR_AAC;
        }
        I2C_MonitorBufferIndex = 0;
}


/*********************************************************************/
uint8_t I2C_MonitorGetDatabuffer(LPC_I2C_TypeDef *I2Cx)
{
        CHECK_PARAM(PARAM_I2Cx(I2Cx));
        return ((uint8_t)(I2Cx->I2DATA_BUFFER));
}

/*********************************************************************/
BOOL_8 I2C_MonitorHandler(LPC_I2C_TypeDef *I2Cx, uint8_t *buffer, uint32_t size)
{
        BOOL_8 ret=FALSE;

        I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;

        buffer[I2C_MonitorBufferIndex] = (uint8_t)(I2Cx->I2DATA_BUFFER);
        I2C_MonitorBufferIndex++;
        if(I2C_MonitorBufferIndex >= size)
        {
                ret = TRUE;
        }
        return ret;
}
/*********************************************************************/
uint32_t I2C_MasterTransferComplete(LPC_I2C_TypeDef *I2Cx)
{
        uint32_t retval, tmp;
        tmp = I2C_getNum(I2Cx);
        retval = I2C_MasterComplete[tmp];
        I2C_MasterComplete[tmp] = FALSE;
        return retval;
}

/*********************************************************************/
uint32_t I2C_SlaveTransferComplete(LPC_I2C_TypeDef *I2Cx)
{
        uint32_t retval, tmp;
        tmp = I2C_getNum(I2Cx);
        retval = I2C_SlaveComplete[tmp];
        I2C_SlaveComplete[tmp] = FALSE;
        return retval;
}



#endif /* _I2C */

/* --------------------------------- End Of File ------------------------------ */