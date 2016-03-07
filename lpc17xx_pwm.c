/***********************************************************************/
/* Peripheral group ----------------------------------------------------------- */
/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_pwm.h"
#include "lpc17xx_clkpwr.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */


#ifdef _PWM


/* Public Functions ----------------------------------------------------------- */
/*********************************************************************/
IntStatus PWM_GetIntStatus(LPC_PWM_TypeDef *PWMx, uint32_t IntFlag)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM_INTSTAT(IntFlag));

        return ((PWMx->IR & IntFlag) ? SET : RESET);
}



/*********************************************************************/
void PWM_ClearIntPending(LPC_PWM_TypeDef *PWMx, uint32_t IntFlag)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM_INTSTAT(IntFlag));
        PWMx->IR = IntFlag;
}



/*****************************************************************************/
void PWM_ConfigStructInit(uint8_t PWMTimerCounterMode, void *PWM_InitStruct)
{
        PWM_TIMERCFG_Type *pTimeCfg;
        PWM_COUNTERCFG_Type *pCounterCfg;
        CHECK_PARAM(PARAM_PWM_TC_MODE(PWMTimerCounterMode));

        pTimeCfg = (PWM_TIMERCFG_Type *) PWM_InitStruct;
        pCounterCfg = (PWM_COUNTERCFG_Type *) PWM_InitStruct;

        if (PWMTimerCounterMode == PWM_MODE_TIMER )
        {
                pTimeCfg->PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
                pTimeCfg->PrescaleValue = 1;
        }
        else if (PWMTimerCounterMode == PWM_MODE_COUNTER)
        {
                pCounterCfg->CountInputSelect = PWM_COUNTER_PCAP1_0;
                pCounterCfg->CounterOption = PWM_COUNTER_RISING;
        }
}


/*********************************************************************/
void PWM_Init(LPC_PWM_TypeDef *PWMx, uint32_t PWMTimerCounterMode, void *PWM_ConfigStruct)
{
        PWM_TIMERCFG_Type *pTimeCfg;
        PWM_COUNTERCFG_Type *pCounterCfg;
        uint64_t clkdlycnt;

        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM_TC_MODE(PWMTimerCounterMode));

        pTimeCfg = (PWM_TIMERCFG_Type *)PWM_ConfigStruct;
        pCounterCfg = (PWM_COUNTERCFG_Type *)PWM_ConfigStruct;


        CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCPWM1, ENABLE);
        CLKPWR_SetPCLKDiv (CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_4);
        // Get peripheral clock of PWM1
        clkdlycnt = (uint64_t) CLKPWR_GetPCLK (CLKPWR_PCLKSEL_PWM1);


        // Clear all interrupts pending
        PWMx->IR = 0xFF & PWM_IR_BITMASK;
        PWMx->TCR = 0x00;
        PWMx->CTCR = 0x00;
        PWMx->MCR = 0x00;
        PWMx->CCR = 0x00;
        PWMx->PCR = 0x00;
        PWMx->LER = 0x00;

        if (PWMTimerCounterMode == PWM_MODE_TIMER)
        {
                CHECK_PARAM(PARAM_PWM_TIMER_PRESCALE(pTimeCfg->PrescaleOption));

                /* Absolute prescale value */
                if (pTimeCfg->PrescaleOption == PWM_TIMER_PRESCALE_TICKVAL)
                {
                        PWMx->PR   = pTimeCfg->PrescaleValue - 1;
                }
                /* uSecond prescale value */
                else
                {
                        clkdlycnt = (clkdlycnt * pTimeCfg->PrescaleValue) / 1000000;
                        PWMx->PR = ((uint32_t) clkdlycnt) - 1;
                }

        }
        else if (PWMTimerCounterMode == PWM_MODE_COUNTER)
        {
                CHECK_PARAM(PARAM_PWM_COUNTER_INPUTSEL(pCounterCfg->CountInputSelect));
                CHECK_PARAM(PARAM_PWM_COUNTER_EDGE(pCounterCfg->CounterOption));

                PWMx->CTCR |= (PWM_CTCR_MODE((uint32_t)pCounterCfg->CounterOption)) \
                                                | (PWM_CTCR_SELECT_INPUT((uint32_t)pCounterCfg->CountInputSelect));
        }
}

/*********************************************************************/
void PWM_DeInit (LPC_PWM_TypeDef *PWMx)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));

        // Disable PWM control (timer, counter and PWM)
        PWMx->TCR = 0x00;
        CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCPWM1, DISABLE);

}


/*********************************************************************/
void PWM_Cmd(LPC_PWM_TypeDef *PWMx, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

        if (NewState == ENABLE)
        {
                PWMx->TCR       |=  PWM_TCR_PWM_ENABLE;
        }
        else
        {
                PWMx->TCR &= (~PWM_TCR_PWM_ENABLE) & PWM_TCR_BITMASK;
        }
}


/*********************************************************************/
void PWM_CounterCmd(LPC_PWM_TypeDef *PWMx, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));
        if (NewState == ENABLE)
        {
                PWMx->TCR       |=  PWM_TCR_COUNTER_ENABLE;
        }
        else
        {
                PWMx->TCR &= (~PWM_TCR_COUNTER_ENABLE) & PWM_TCR_BITMASK;
        }
}


/*********************************************************************/
void PWM_ResetCounter(LPC_PWM_TypeDef *PWMx)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        PWMx->TCR |= PWM_TCR_COUNTER_RESET;
        PWMx->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
}


/*********************************************************************/
void PWM_ConfigMatch(LPC_PWM_TypeDef *PWMx, PWM_MATCHCFG_Type *PWM_MatchConfigStruct)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_MATCH_CHANNEL(PWM_MatchConfigStruct->MatchChannel));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_MatchConfigStruct->IntOnMatch));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_MatchConfigStruct->ResetOnMatch));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_MatchConfigStruct->StopOnMatch));

        //interrupt on MRn
        if (PWM_MatchConfigStruct->IntOnMatch == ENABLE)
        {
                PWMx->MCR |= PWM_MCR_INT_ON_MATCH(PWM_MatchConfigStruct->MatchChannel);
        }
        else
        {
                PWMx->MCR &= (~PWM_MCR_INT_ON_MATCH(PWM_MatchConfigStruct->MatchChannel)) \
                                        & PWM_MCR_BITMASK;
        }

        //reset on MRn
        if (PWM_MatchConfigStruct->ResetOnMatch == ENABLE)
        {
                PWMx->MCR |= PWM_MCR_RESET_ON_MATCH(PWM_MatchConfigStruct->MatchChannel);
        }
        else
        {
                PWMx->MCR &= (~PWM_MCR_RESET_ON_MATCH(PWM_MatchConfigStruct->MatchChannel)) \
                                        & PWM_MCR_BITMASK;
        }

        //stop on MRn
        if (PWM_MatchConfigStruct->StopOnMatch == ENABLE)
        {
                PWMx->MCR |= PWM_MCR_STOP_ON_MATCH(PWM_MatchConfigStruct->MatchChannel);
        }
        else
        {
                PWMx->MCR &= (~PWM_MCR_STOP_ON_MATCH(PWM_MatchConfigStruct->MatchChannel)) \
                                        & PWM_MCR_BITMASK;
        }
}


/*********************************************************************/
void PWM_ConfigCapture(LPC_PWM_TypeDef *PWMx, PWM_CAPTURECFG_Type *PWM_CaptureConfigStruct)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_CAPTURE_CHANNEL(PWM_CaptureConfigStruct->CaptureChannel));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_CaptureConfigStruct->FallingEdge));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_CaptureConfigStruct->IntOnCaption));
        CHECK_PARAM(PARAM_FUNCTIONALSTATE(PWM_CaptureConfigStruct->RisingEdge));

        if (PWM_CaptureConfigStruct->RisingEdge == ENABLE)
        {
                PWMx->CCR |= PWM_CCR_CAP_RISING(PWM_CaptureConfigStruct->CaptureChannel);
        }
        else
        {
                PWMx->CCR &= (~PWM_CCR_CAP_RISING(PWM_CaptureConfigStruct->CaptureChannel)) \
                                        & PWM_CCR_BITMASK;
        }

        if (PWM_CaptureConfigStruct->FallingEdge == ENABLE)
        {
                PWMx->CCR |= PWM_CCR_CAP_FALLING(PWM_CaptureConfigStruct->CaptureChannel);
        }
        else
        {
                PWMx->CCR &= (~PWM_CCR_CAP_FALLING(PWM_CaptureConfigStruct->CaptureChannel)) \
                                        & PWM_CCR_BITMASK;
        }

        if (PWM_CaptureConfigStruct->IntOnCaption == ENABLE)
        {
                PWMx->CCR |= PWM_CCR_INT_ON_CAP(PWM_CaptureConfigStruct->CaptureChannel);
        }
        else
        {
                PWMx->CCR &= (~PWM_CCR_INT_ON_CAP(PWM_CaptureConfigStruct->CaptureChannel)) \
                                        & PWM_CCR_BITMASK;
        }
}


/*********************************************************************/
uint32_t PWM_GetCaptureValue(LPC_PWM_TypeDef *PWMx, uint8_t CaptureChannel)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_CAPTURE_CHANNEL(CaptureChannel));

        switch (CaptureChannel)
        {
        case 0:
                return PWMx->CR0;

        case 1:
                return PWMx->CR1;

        default:
                return (0);
        }
}


/********************************************************************/
void PWM_MatchUpdate(LPC_PWM_TypeDef *PWMx, uint8_t MatchChannel, \
                                        uint32_t MatchValue, uint8_t UpdateType)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_MATCH_CHANNEL(MatchChannel));
        CHECK_PARAM(PARAM_PWM_MATCH_UPDATE(UpdateType));

        switch (MatchChannel)
        {
        case 0:
                PWMx->MR0 = MatchValue;
                break;

        case 1:
                PWMx->MR1 = MatchValue;
                break;

        case 2:
                PWMx->MR2 = MatchValue;
                break;

        case 3:
                PWMx->MR3 = MatchValue;
                break;

        case 4:
                PWMx->MR4 = MatchValue;
                break;

        case 5:
                PWMx->MR5 = MatchValue;
                break;

        case 6:
                PWMx->MR6 = MatchValue;
                break;
        }

        // Write Latch register
        PWMx->LER |= PWM_LER_EN_MATCHn_LATCH(MatchChannel);

        // In case of update now
        if (UpdateType == PWM_MATCH_UPDATE_NOW)
        {
                PWMx->TCR |= PWM_TCR_COUNTER_RESET;
                PWMx->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;
        }
}


/********************************************************************/
void PWM_ChannelConfig(LPC_PWM_TypeDef *PWMx, uint8_t PWMChannel, uint8_t ModeOption)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_EDGE_MODE_CHANNEL(PWMChannel));
        CHECK_PARAM(PARAM_PWM_CHANNEL_EDGE(ModeOption));

        // Single edge mode
        if (ModeOption == PWM_CHANNEL_SINGLE_EDGE)
        {
                PWMx->PCR &= (~PWM_PCR_PWMSELn(PWMChannel)) & PWM_PCR_BITMASK;
        }
        // Double edge mode
        else if (PWM_CHANNEL_DUAL_EDGE)
        {
                PWMx->PCR |= PWM_PCR_PWMSELn(PWMChannel);
        }
}



/********************************************************************/
void PWM_ChannelCmd(LPC_PWM_TypeDef *PWMx, uint8_t PWMChannel, FunctionalState NewState)
{
        CHECK_PARAM(PARAM_PWMx(PWMx));
        CHECK_PARAM(PARAM_PWM1_CHANNEL(PWMChannel));

        if (NewState == ENABLE)
        {
                PWMx->PCR |= PWM_PCR_PWMENAn(PWMChannel);
        }
        else
        {
                PWMx->PCR &= (~PWM_PCR_PWMENAn(PWMChannel)) & PWM_PCR_BITMASK;
        }
}

#endif /* _PWM */

/* --------------------------------- End Of File ------------------------------ */
