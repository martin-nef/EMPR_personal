/***********************************************************************/
#include "lpc17xx_gpio.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_adc.h"
#include "mylpclib.h"
#include "lpc17xx_rtc.h"
#include "lpc17xx_wdt.h"

static uint32_t BGLCDADDR= 34;

//Run Main Program
void main (void){
	uint8_t* dataPtr;
	uint8_t* rx_dat;
	uint8_t bglcd_rsp[1] = {0x00};
	rx_dat = bglcd_rsp;
	char msg[BUFFLENGTH] = "";
	char* strPtr = &msg;
	//Setup external device communication, I2C1 Bus && UAR
	setupUART();
	setupI2C1();
	setupGPIO(2,5,1);
	displayBits(2, 3);

	//Send Status Check Commands to LCD
	uint32_t i2caddress = BGLCDADDR;
	uint8_t lcd_init[38] = {0x03, 0x00, 0x03, 0x00, 0x03, 0x40, 0x03, 0x20, 0x03, 0x00, 0x03, 0x41, 0x03, }
	dataPtr = lcd_init;
	I2C_M_SETUP_Type TransferCfg;
	TransferCfg.sl_addr7bit = i2caddress;
	TransferCfg.tx_data = dataPtr;
	TransferCfg.tx_length = sizeof(uint8_t)*38;
	TransferCfg.rx_data = rx_dat;
	TransferCfg.rx_length = sizeof(uint8_t);

	I2C_MasterTransferData((LPC_I2C_TypeDef *)LPC_I2C1, &TransferCfg, I2C_TRANSFER_POLLING);
	memset(msg,0,sizeof(msg));
	sprintf(strPtr, "\r\nData back: %D", *rx_dat);
	UART_SendMSG(strPtr, msg);
	SystickTimer(30);

}
