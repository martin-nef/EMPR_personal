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


//Run Main Program
void main (void){


	//Setup external device communication, I2C1 Bus && UAR
	setupUART();
	setupI2C1();
	setupGPIO(2,5,1);

	while(1){
		float dist = usScan();
		char msg[BUFFLENGTH] = "";
		char* strPtr = &msg;
		if(dist > 0){
			sprintf(strPtr, "\r\nRaw Value: %.3f", dist);
			UART_SendMSG(strPtr, msg);
		}else{
			sprintf(strPtr, "\r\nReconnect UltraSound (Error:%.1f)", dist);
			UART_SendMSG(strPtr, msg);
		}
	}
}
