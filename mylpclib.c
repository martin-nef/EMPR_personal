#include "lpc17xx_gpio.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pwm.h"
#include "lpc17xx_adc.h"
#include "mylpclib.h"
#include "math.h"

//Global Vars
static uint32_t LCDADDR = 59;
static uint32_t KEYADDR = 33;
int numChars = 0;
int currline = 1;
char msg[BUFFLENGTH] = "";
char* strPtr = &msg;
uint8_t* dataPtr;

int userposition = 0;
int whichSensor = 0;
double ir_multiplier_cal = 26530;
double us_multiplier_cal = 0;
// Servo globals
uint8_t servo_channel; // pwm channel used by the servo
uint32_t servo_position = 0; // current servo position. will be between sweepstartpos and sweepstoppos
const uint32_t SERVO_LOW_BOUND = 110;
const uint32_t SERVO_HIGH_BOUND = 190;
volatile int tim0_flag = 0;
volatile int upflag = 0;
volatile float risVal = 0;
volatile float ultradist = 0;
double sweep[100];
double currentIr = 0;
double currentUs = 0;//does nothing for now
double currentRawIr = 0;
double currentRawUr = 0;//does nothing for now
double averageDistance = 0;

//Required parameters
uint8_t sweepspeed = 1; // at samplespersweep = 20 can be 1-4
uint8_t samplespersweep = 20; // at sweepspeed = 4 can be 1-20
//These two should be 0 and 80 respectively to get 20 scans per sweep at speeds 1-4
uint32_t sweepstartpos = 0; // min 0
uint32_t sweepstoppos = 80; // max 80

static char* generalusetext[8] = {
  "Welcome User.Use","* and # to move","Enter a number","To access option","Press 0 to exit"," ","1.Functions",
  "2.Parameters"
};

static char* functionusetext[11] = {
  "Functions : ","1. Calibration", "2.Tape-measure", "3.Scan","4.Multi-view","Last Estimate :","Place object","# when ready","Place at 10cm","Place at 50cm","Calibration done"
};

static char* parameterusetext[8] = {
  "Parameters :", "1.Sweep speed", "2.No of samples", "3.Start pos", "4.Stop pos", "5.Choose Sensor", "1.Infrared", "2.Ultrasound"
};

static char KEYPAD[4][4] = {
   {'1', '2', '3', 'A'},
   {'4', '5', '6', 'B'},
   {'7', '8', '9', 'C'},
   {'*', '0', '#', 'D'}
};

static char CHARSET[68] = {
							'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
							'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
							'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
							'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd',
							'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
							'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x',
							'y', 'z', '#', '*', ' ', '.', ':', '-'
						};

static uint8_t CHARVALS[68] = {
								0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,
								0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,
								0xCB,0xCC,0xCD,0xCE,0xCF,0xD0,0xD1,0xD2,0xD3,0xD4,
								0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0x61,0x62,0x63,0x64,
								0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,
								0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,
								0x79,0x7A,0xA3,0xAA,0xA0,0xAE,0xBA,0xAD
							};

/*
Pre-Reqs: SetupPWM() with prescale = 10.
Desc: Initialise PWM match0 register to 2048 for correct servo operation.
Inputs: int servo_channel_ - ranges from 1 to 6, sets the match register
							 to be used as the output for the servo.
*/
void initServo(int servo_channel_){
	uint8_t match_channel = 0;
	uint32_t match_value = 2048;
	SetRawPWM(match_channel, match_value);
	servo_channel = servo_channel_;
}

/*
Pre-Reqs:initServo()
Desc: Set the servo position.
Inputs: uint32_t position - ranges from 0 to 80
*/
void setServo(uint32_t position){
	uint32_t match_value = SERVO_LOW_BOUND + position;
	if (match_value>SERVO_HIGH_BOUND){
		match_value = SERVO_HIGH_BOUND;
	}
	if (match_value<SERVO_LOW_BOUND){
		match_value = SERVO_LOW_BOUND;
	}
	SetRawPWM(servo_channel, match_value);
	servo_position = match_value - SERVO_LOW_BOUND;
}

/*
Pre-Reqs:initServo()
Desc: Move the servo by specified amount
Inputs: int amount - negative for right, positive for left.
*/
void turnServo(int amount){
	servo_position = servo_position + amount;
	setServo(servo_position);
}

/*
Pre-Reqs: Servo has to have been set or moved at least once.
Desc: Detects whether the servo is at an edge or not.
Output: 1 if at edge, 0 otherwise.
*/
int servoAtEdge(){
	if(servo_position == 0 | servo_position == 80){
		return 1;
	}else{
		return 0;
	}
}

/*
Pre-Reqs:SetupPWM()
Desc: Sets PWM's match registers
Inputs: uint8_t match_channel - match register to update
		uint32_t match_value - value to update the match register with
*/
void SetRawPWM(uint8_t match_channel, uint32_t match_value){
	PWM_Cmd(LPC_PWM1, DISABLE);
	PWM_MatchUpdate(LPC_PWM1, match_channel, match_value, PWM_MATCH_UPDATE_NOW);
	PWM_Cmd(LPC_PWM1, ENABLE);
}

/*
Desc:Configures PINSEL to setup P2.0 with Function 1 which is PWM channel 1 (mbed pin 26),
					 		and P2.1 with Function 1 which is PWM channel 2 (mbed pin 25).
	 NB! Does not set match registers to any default values.
Inputs: uint32_t prescale - sets the PWM prescale value.
*/
void SetupPWM(uint32_t prescale){
	PINSEL_CFG_Type PinCfg;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Funcnum = 1;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	PWM_TIMERCFG_Type pwm_countercfg;
	pwm_countercfg.PrescaleOption = PWM_TIMER_PRESCALE_USVAL;
	pwm_countercfg.PrescaleValue = prescale;
	PWM_Init(LPC_PWM1, PWM_MODE_TIMER, &pwm_countercfg);	// Initializes the PWMx peripheral

	PWM_MATCHCFG_Type pwm_matchcfg;
	pwm_matchcfg.IntOnMatch = DISABLE;
	pwm_matchcfg.ResetOnMatch = DISABLE;
	pwm_matchcfg.StopOnMatch = DISABLE;
	pwm_matchcfg.MatchChannel = 0;
	PWM_ConfigMatch(LPC_PWM1, &pwm_matchcfg);

	pwm_matchcfg.MatchChannel = 1;
	PWM_ConfigMatch(LPC_PWM1, &pwm_matchcfg);

	pwm_matchcfg.MatchChannel = 2;
	PWM_ConfigMatch(LPC_PWM1, &pwm_matchcfg);

	PWM_CounterCmd(LPC_PWM1, ENABLE);

	PWM_ChannelCmd(LPC_PWM1, 1, ENABLE);	// Enable PWM channel 1 output
	PWM_ChannelCmd(LPC_PWM1, 2, ENABLE);	// Enable PWM channel 2 output

	PWM_Cmd(LPC_PWM1, ENABLE);	// Enable PWM peripheral
}

/*
Desc:Configures the PINSEL data structure to setup pin P0.2
	 for use with the I2C bus and enables it
*/
void setupUART(){
	//UART works with 4 main steps
	PINSEL_CFG_Type PinCfg; //Configure Pin select register
	UART_CFG_Type UARTConfigStruct;	//Configure the UART Register
	UART_FIFO_CFG_Type FifoCfg; //Configure the UART FIFO Register

	//Init UART
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);


	UART_ConfigStructInit(&UARTConfigStruct);
	UART_Init((LPC_UART_TypeDef *)LPC_UART0, &UARTConfigStruct);

	UART_FIFOConfigStructInit(&FifoCfg);
	UART_FIFOConfig((LPC_UART_TypeDef *) LPC_UART0, &FifoCfg);

	//Enable UART Transmit
	UART_TxCmd((LPC_UART_TypeDef *)LPC_UART0, ENABLE);
}

/*
Desc:Configures the PINSEL data structure to setup pins P0.0 & P0.1
	 for use with the I2C bus and enables it
*/
void setupI2C1(){
	PINSEL_CFG_Type PinCfg;

	//Init I2C1 - P0.0
	PinCfg.Funcnum = 3;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 1;

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	//Init I2C1 - P0.1
	PinCfg.Funcnum = 3;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 1;

	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	I2C_Init(LPC_I2C1, 30000);
	I2C_Cmd(LPC_I2C1, ENABLE);
}

/*
Pre-Reqs:setupI2C1()
Desc: Sends some data of a given size to a given address on the I2C bus
Inputs: char* strPtr - Pointer to the string
		char[BUFFLENGTH] - String of length BUFFLENGTH
*/
void sendToI2C1(uint32_t i2caddress, uint8_t* tx_dat, uint8_t size){
	char* strPtr;
	char msg[BUFFLENGTH] = "";
	strPtr = &msg;
	int num = 0;

	I2C_M_SETUP_Type TransferCfg;

	TransferCfg.sl_addr7bit = i2caddress;
	TransferCfg.tx_data = tx_dat;
	//TransferCfg.retransmissions_max = 5;
	TransferCfg.tx_length = sizeof(uint8_t)*size;
	TransferCfg.rx_data = NULL;
	TransferCfg.rx_length = 0;

	I2C_MasterTransferData((LPC_I2C_TypeDef *)LPC_I2C1, &TransferCfg, I2C_TRANSFER_POLLING);
}

/*
Desc: Displays a given bit pattern using the 4 LEDs and holds it for a given period
Inputs: int number - Number of to be lit
		long delay - Duration of the delay in s
*/
void displayBits(int number, long delay){
	int bitpos = 1;
	int num = 0;
	delay= delay*1000;
	/*test bit position 0 - if number is odd*/
	if((bitpos & number) == 1){
		lightLED(1);
	}

	/*test bit position 1 - AND with bin 2*/
	bitpos = 2;
	if((bitpos & number) == 2){
		lightLED(2);
	}

	/*test bit position 2 - AND with bin 3*/
	bitpos = 4;
	if((bitpos & number) == 4){
		lightLED(3);
	}

	/*test bit position 3 - AND with bin 4*/
	bitpos = 8;
	if((bitpos & number)== 8){
		lightLED(4);
	}

	SystickTimer(delay);

	killLED(1);
	killLED(2);
	killLED(3);
	killLED(4);
}

/*
Desc: Kills the LED given a number for it
Inputs: int ledNum - Number of the led to be killed
*/
void killLED(int ledNum){
	uint32_t led;
	switch(ledNum){
		case 1:
			led = (1<<18);
			break;
		case 2:
			led = (1<<20);
			break;
		case 3:
			led = (1<<21);
			break;
		case 4:
			led = (1<<23);
			break;
	}
	GPIO_ClearValue(1, led);
}

/*
Desc: Lights the LED given a number for it
Inputs: int ledNum - Number of the led to be lit
*/
void lightLED(int ledNum){
	uint32_t led;
	switch(ledNum){
		case 1:
			led = (1<<18);
			break;
		case 2:
			led = (1<<20);
			break;
		case 3:
			led = (1<<21);
			break;
		case 4:
			led = (1<<23);
			break;
	}
	GPIO_SetDir(1, led, 1);
	GPIO_ClearValue(1, led);
	GPIO_SetValue(1, led);
}


void SysTick_Handler(void) {
	SysTickCnt++;
}


void SystickTimer(unsigned long tick){
	SysTick_Config(SystemCoreClock/1000);
	unsigned long systickcnt;
	systickcnt = SysTickCnt;
	while ((SysTickCnt - systickcnt) < tick);
}

/*
Pre-Reqs:setupUART()
Desc: Sends a string down to the UART
Inputs: char* strPtr - Pointer to the string
		char[BUFFLENGTH] - String of length BUFFLENGTH
*/
void UART_SendMSG(char* strPtr, char msg[BUFFLENGTH]){
	sprintf(strPtr, msg);
	UART_RS485SendData (LPC_UART0, strPtr, BUFFLENGTH);
}

/*
Pre-Reqs: setupI2C1(), setupUART()
Desc: Polls the keyboard to detect key presses.
Outputs: char key - the button that was pressed
*/
char KeypadTest(){
	uint8_t* rx_dat;
	uint32_t i2caddress = KEYADDR;
	char buttonPressed;

	uint8_t cols[4] = {0x7F, 0xBF, 0xDF, 0xEF};
	int i;
	int row = -1;
	int column = -1;
	for (i=0; i<4; ++i) {
		uint8_t key_init[1] = {cols[i]};
		uint8_t key_prsd[1] = {0x00};
		dataPtr = key_init;
		rx_dat = key_prsd;
		sendToI2C1(i2caddress, dataPtr, 1);


		I2C_M_SETUP_Type TransferCfg;
		TransferCfg.sl_addr7bit = i2caddress;
		TransferCfg.tx_data = NULL;
		TransferCfg.tx_length = 0;
		TransferCfg.rx_data = rx_dat;
		TransferCfg.rx_length = sizeof(uint8_t);

		if(I2C_MasterTransferData((LPC_I2C_TypeDef *)LPC_I2C1, &TransferCfg, I2C_TRANSFER_POLLING) != 0){
			switch (*rx_dat&0xf){
				case 0x7:
					column = 0;
					row = i;
					break;
				case 0xB:
					column = 1;
					row = i;
					break;
				case 0xD:
					column = 2;
					row = i;
					break;
				case 0xE:
					column = 3;
					row = i;
					break;
			}
		} else {
			return NULL;
		}
		//30ms Polling Delay
		// SystickTimer(30);
	}
	if (column>=0) {
		return KEYPAD[column][row];
	} else {
		return NULL;
	}
}

/*
Pre-Reqs: setupI2C1(), setupUART()
Desc: Initializes the LCD and prints confirmation to the UART once it is done
*/
void StartLCD(){
	//Send Init Commands to LCD
	uint32_t i2caddress = LCDADDR;
	uint8_t lcd_init[11] = {0x00, 0x34, 0x0C, 0x06, 0x35, 0x04, 0x10, 0x42,
		0x9F, 0x34, 0x02};
	dataPtr = lcd_init;
	sendToI2C1(i2caddress, dataPtr, 11);
	sprintf(strPtr, "\r\nInit Done");
	UART_SendMSG(strPtr, msg);

	ClearLCD();
}

/*
Pre-Reqs: setupI2C1(), setupUART()
Desc: Clears the LCD screen by filling each line with blanks
*/
void ClearLCD(){
	uint32_t i2caddress = LCDADDR;

	//Command to print blank char
	uint8_t lcd_clr[2] = {0x40, 0xA0};
	//Command to shift the cursor
	uint8_t lcd_shft[2] = {0x00, 0x14};

	//Fill line with blank chars
	int num = 0;
	while(num < 16){
		dataPtr = lcd_clr;
		sendToI2C1(i2caddress, dataPtr, 2);
		num++;
	}

	//Shift cursor to next line
	while(num < 40){
		dataPtr = lcd_shft;
		sendToI2C1(i2caddress, dataPtr, 2);
		num++;
	}

	//Fill line with blank chars
	num = 0;
	while(num < 16){
		dataPtr = lcd_clr;
		sendToI2C1(i2caddress, dataPtr, 2);
		num++;
	}

	//Return to starting point in LCD Screen
	uint8_t lcd_home[2] = {0x00, 0x02};
	dataPtr = lcd_home;
	sendToI2C1(i2caddress, dataPtr, 2);

	//sendToI2C1(i2caddress, dataPtr, 2);
	sprintf(strPtr, "\r\nClr Screen Done");
	UART_SendMSG(strPtr, msg);
}


/*
Pre-Reqs: setupI2C1(), setupUART()
Desc: Prints a given char to the LCD screen. If the current line is full wraps
around to the next line by calling LSDShftLine()
Inputs: char ch - Character to print to LCD
*/
void PrintToLCD(char ch){
	uint32_t i2caddress = LCDADDR;

	uint8_t lcd_cmd[2] = {0x00, 0x80};

	uint8_t lcd_crsr[1] = {0x14};

	int validChar = 1;
	uint8_t lcd_print[2] = {0x40, CharToHex(ch)};
	dataPtr = lcd_print;

	if (lcd_print[1]==NULL) {
		validChar = 0;
	}

	if (validChar){
		// sendToI2C1(i2caddress, lcd_cmd, 2);
		sendToI2C1(i2caddress, dataPtr, 2);
		sendToI2C1(i2caddress, lcd_crsr, 1);

		//Check if EOL on LCD
		numChars++;
		if(numChars == 16){
			LCDShftLine();
			if(currline == 1){
				currline = 2;
			}else if(currline == 2){
				currline = 1;
				SystickTimer(1600);
				ClearLCD();
			}
		}
	}
}


/*
Pre-Reqs: setupUART(), setupI2C1()
Desc: Shifts the display until it reaches a new line. Wraps-around LCD Screen
*/
void LCDShftLine(){
	uint32_t i2caddress = LCDADDR;
	//Command to shift the cursor
	uint8_t lcd_shft[2] = {0x00, 0x14};
	//Command to print blank char
	uint8_t lcd_clr[2] = {0x40, 0xA0};
	int num = 0;

	//Shift cursor
	dataPtr = lcd_shft;
	numChars = 0;
	while(numChars < 24 ){
		sendToI2C1(i2caddress, dataPtr, 2);
		numChars++;
	}

	numChars = 0;
}


/*
Pre-Reqs: None
Desc: takes a character and converts it into a hex value
	  corresponding to that character in the LCD character set.
	  If the char is not in CHARSET, return NULL.
Input: char ch.
Output: uint8_t hex code corresponding to input character
*/
uint8_t CharToHex(char ch){
	int i;
	for (i = 0; i < sizeof(CHARSET); ++i)
	{
		if (ch==CHARSET[i])
		{
			return CHARVALS[i];
		}
	}
	return NULL;
}



/*
Pre-Reqs: setupI2C1(), setupUART()
Desc: Queries all the addresses on the I2C bus to see how many devices are connected
and their addresses
*/
void I2CSniffer(){
	uint8_t tx_dat = 0;
	uint32_t i2caddress = 0x00;
	int addrcount = 0;
	I2C_M_SETUP_Type TransferCfg;
	for(i2caddress = 0x00; i2caddress < 128; i2caddress++){
		TransferCfg.sl_addr7bit = i2caddress;
		TransferCfg.tx_data = &tx_dat;
		TransferCfg.tx_length = 1;
		TransferCfg.rx_data = NULL;
		TransferCfg.rx_length = 0;
		TransferCfg.retransmissions_max = 3;
		if(I2C_MasterTransferData(LPC_I2C1, &TransferCfg, I2C_TRANSFER_POLLING) != 0){
			//displayBits(15, 5);
			sprintf(strPtr, "\r\nAddress: %X\t", i2caddress);
			UART_SendMSG(strPtr, msg);
			addrcount++;
			SystickTimer(5);
		}else{
		 	//sprintf(msg, "\r\nFailure\t");
		 	//UART_SendMSG(strPtr, msg);
		}
		sprintf(strPtr, "\r\nTotal Addresses: %d\t", addrcount);
		UART_SendMSG(strPtr, msg);
	}
}

void setupDAC(){
    //Setup mbed pin 18, for analogue output
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PINSEL_ConfigPin(&PinCfg);
    //

    DAC_Init(LPC_DAC);      //Initialise and set to 0
    DAC_UpdateValue(LPC_DAC, 0x0000);
}

/*
 -Pre-Reqs: None
 -Desc: Initalise ADC at pin 30, mbed pin 19
 -Input: None
 -Output: None
 -*/
void setupADC(){
    //Setup for channel 0
    PINSEL_CFG_Type ADCPinCfg;
    ADCPinCfg.Funcnum = 3;
    ADCPinCfg.OpenDrain = 0;
    ADCPinCfg.Pinmode = 0;
    ADCPinCfg.Pinnum = 30;
    ADCPinCfg.Portnum = 1;
    PINSEL_ConfigPin(&ADCPinCfg);

    ADC_Init(LPC_ADC,100);
    ADC_ChannelCmd(LPC_ADC,4,ENABLE);
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
}


/*
 -Pre-Reqs: SetupADC()
 -Desc: Takes a voltage input and returns the raw value
 -Input: None
 -Output: uint32
-*/
uint32_t adcInput(void){
    uint32_t adcValue = 0;
	adcValue = ADC_ChannelGetData(LPC_ADC,4);
	return adcValue;
}

/*
 -Pre-Reqs: SetupADC(), adcInput(),
 -Desc: Calculates distance reading from ir sensor using value of adc,
        outputs -1 if distance calculated is < 10cm
 -Input: None
 -Output: double
-*/
double irDistance(void){
    double distance;
    double exponent = (1/-0.979);
    double adcValue = (double) adcInput();
    double voltage = (3.3 * (adcValue / 4095));

    distance = pow((adcValue/ir_multiplier_cal), exponent);
    //y = 26530x^-0.979
    //(y/26530)^(-1000/979) = x

    //Old function:
    //distance = pow((voltage/multiplier), exponent);
    //  multiplier = 15.274
    //  exponent = 1/-0.825
    //15.274*V^0.825
    //(y/15.274)^(-40/33) = x

    if (distance >= 10){
        return roundf(distance * 10) / 10;
    } else{
        return -1;
    }
}

double irDistanceRaw(void){
    double distance;
    double exponent = (1/-0.979);
    double adcValue = (double) adcInput();
    double voltage = (3.3 * (adcValue / 4095));

    distance = pow((adcValue/ir_multiplier_cal), exponent);
    //y = 26530x^-0.979
    //(y/26530)^(-1000/979) = x

    //Old function:
    //distance = pow((voltage/multiplier), exponent);
    //  multiplier = 15.274
    //  exponent = 1/-0.825
    //15.274*V^0.825
    //(y/15.274)^(-40/33) = x

    if (distance >= 10){
        return roundf(distance * 10) / 10;
    } else{
        return roundf(distance * 10) / 10;
    }
}

/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(),
 -Desc: Calls irDistance until a valid value is found, returns
        this value.
 -Input: None
 -Output: double
-*/
double irDistanceValid(){
    double distance;
    while(1){
        distance = irDistance();
        if (distance != -1){
            return distance;
        }
    }
}


/*
	-Input:	uint32_t cap - max iterations allowd
	-Desc:	Same as irDistanceValid(), but with iteration cap.
			If valid value not found , returns -1
*/
double irTryDistanceValid(uint32_t cap, int delay){
    double distance;
    uint32_t x = 1;
    while(1){
        distance = irDistance();
        if (delay > 0) {
        	SystickTimer(delay);
        }
        x++;
        if (x>cap) {
		    return distance;
		}
        if (distance != -1)	{
            return distance;
        }
    }
}

/*
 -Pre-Reqs: None
 -Desc: Compare function for qsort()
 -Input: erm
 -Output: int
-*/
int cmpfunc(const void *a,const void *b) {
    double *x = (double *) a;
    double *y = (double *) b;
    if (*x < *y) return -1;
    else if (*x > *y) return 1; return 0;
}

/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(), irDistanceValid()
 -Desc: Calls irDistanceValid 10 times and returns the median
 -Input: None
 -Output: double
-*/
double irMedian(void){
    int i;
    double v[10];
    double distance;

    for(i=0;i<10;i++){
        v[i] = irDistanceValid();
    }

    qsort(v, 10, sizeof(double), cmpfunc);

    return v[4];
}

/*
 -Pre-Reqs: SetupADC(), adcInput()
 -Desc: Gets raw IR value 10 times and returns the median
 -Input: None
 -Output: double
-*/
double irMedianRaw(void){
  int i;
  double v[10];

  for(i=0;i<10;i++){
      v[i] = (double) adcInput();
  }

  qsort(v, 10, sizeof(double), cmpfunc);
  return v[4];
}
/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(), irDistanceValid()
 -Desc: Calibrates the IR function based on two
 -Input: 2 doubles
 -Output: None
-*/
void irCalibrate(double adcValue10, double adcValue50){
    double multiplier = 26530;
    double exponent = -0.979;       //base values
    double distance, adcValue, multiplier10, multiplier50;

    //distance = pow(( adcValue / multiplier), 1/exponent);
    //x = (y/26530)^(1/-0.979)

    //adcValue = multiplier * pow(distance, exponent);
    //y = 26530*x^(-0.979)

    multiplier10 = adcValue10 / pow(10, exponent);  //10cm
    multiplier50 = adcValue50 / pow(50, exponent);  //50cm

    multiplier = (multiplier10 + multiplier50) / 2; //Take average of the two

    ir_multiplier_cal = multiplier; //Set global to calibrated value

}

/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(), irDistanceValid()
 -Desc: Calibrates the IR function based on two
 -Input: 2 doubles
 -Output: None
-*/
void usCalibrate(double usValue10, double usValue50){
    double usValue, multiplier10, multiplier50;
    double multiplier = 0;
    //distance = pow(( usValue / multiplier), 1/exponent);
    //x = (y/26530)^(1/-0.979)

    //usValue = multiplier * pow(distance, exponent);
    //y = 26530*x^(-0.979)

    multiplier10 = usValue10 / 10;  //10cm
    multiplier50 = usValue50 / 50;  //50cm

    multiplier = (multiplier10 + multiplier50) / 2; //Take average of the two

    us_multiplier_cal = multiplier; //Set global to calibrated value

}


/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(), irDistanceValid(), irMedian
 -Desc: Creates a string with the irMedian value
 -Input: None
 -Output: None
-*/
void usString(void){
    double distance;
    char* output;
    char src[50];
	  distance = usScan();
    sprintf(strPtr, "\r\nDistance: %.2fcm", distance);
    UART_SendMSG(strPtr, msg);
    sprintf(src, "%.2fcm", distance);
    PrintLineToLCD(src);
}

/*
 -Pre-Reqs: SetupADC(), adcInput(), irDistance(), irDistanceValid(), irMedian
 -Desc: Creates a string with the irMedian value
 -Input: None
 -Output: None
-*/
void irString(void){
    double distance;
    char* output;
    char src[50];
	  distance = irMedian();
    sprintf(strPtr, "\r\nDistance: %.1fcm", distance);
    UART_SendMSG(strPtr, msg);
    sprintf(src, "%.1fcm", distance);
    PrintLineToLCD(src);
}

/*
Pre-Reqs: PrintToLCD2(), LCDShftLine()
Desc: Cycles through a line of text sending each char to the function PrintToLCD2()
Inputs: line to be displayed MUST BE LESS THAN 16 CHARS
*/
void PrintLineToLCD(char* Testing){
	int counter = 0;
  uint32_t i2caddress = LCDADDR;
  uint8_t lcd_clr[2] = {0x40, 0xA0};
  dataPtr = lcd_clr;
	while (1){
		PrintToLCD2(Testing[counter]);
		if (counter == strlen(Testing)-1){
			break;
		}
		counter ++;
	}
  if (counter < 16){
    while (counter<=14){
      PrintToLCD2(' ');
      counter++;
    }
  }
  LCDShftLine();
}

/*
Pre-Reqs: SystickTimer(), sendToI2C1()
Desc: Another ClearLCD(), which wipes entire screen and returns to original position when called
*/
void ActualLCDClear(){
  SystickTimer(4000);
  numChars = 0;
	uint32_t i2caddress = LCDADDR;

	//Command to print blank char
	uint8_t lcd_clr[2] = {0x40, 0xA0};

	//Fill screen with blank chars
	int num = 0;
	while(num  < 80){
		dataPtr = lcd_clr;
		sendToI2C1(i2caddress, dataPtr, 2);
		num++;
	}

}
/*
Pre-Reqs: CharToHex(), sendToI2C1()
Desc: Copy of PrintToLCD() but with no reset of lcd on line end or screen end
Inputs: char to be displayed
*/
void PrintToLCD2(char ch){
	uint32_t i2caddress = LCDADDR;

	uint8_t lcd_cmd[2] = {0x00, 0x80};

	uint8_t lcd_crsr[1] = {0x14};

	int validChar = 1;
	uint8_t lcd_print[2] = {0x40, CharToHex(ch)};
	dataPtr = lcd_print;

	if (lcd_print[1]==NULL) {
		validChar = 0;
	}

	if (validChar){
		// sendToI2C1(i2caddress, lcd_cmd, 2);
		sendToI2C1(i2caddress, dataPtr, 2);
		sendToI2C1(i2caddress, lcd_crsr, 1);
	}
}
/*
Pre-Reqs:PrintLineToLCD(), ActualLCDClear(), KeypadTest(), functiontext(),parametertext(), global generalusetext
Desc: Sends greetings text and displays options to user. Depending on button presses calls corresponding function or returns
*/
void greetingtext(){
  int counter = 0;
  char button;
  while (1){
    PrintLineToLCD(generalusetext[counter]);
    counter++;
    PrintLineToLCD(generalusetext[counter]);
    counter++;
    if (counter >=8){
      break;
    }
    ActualLCDClear();
  }
  while (1){
    PrintLineToLCD(generalusetext[6]);
    PrintLineToLCD(generalusetext[7]);
    SystickTimer(400);
    while(1){
      button = KeypadTest();
      if (button == '1'){
        functiontext();
        break;
      }
      else if (button == '2'){
        parametertext();
        break;
      }
      else if (button == '0'){
        return;
      }
    }
  }
}

/*
Pre-Reqs:PrintLineToLCD(), KeypadTest(), global functionusetext, global userposition
Desc: Bunch of loops. First displays 1st and 2nd lines, then checks button presses to send corresponding lines
*/
void functiontext(){
      char button;
      PrintLineToLCD(functionusetext[0]);
      PrintLineToLCD(functionusetext[1]);
      SystickTimer(400);
      while (1){
        button = KeypadTest();
        if (button == '#'){
          userposition++;
          if (userposition>3){
            userposition = 0;
          }
          PrintLineToLCD(functionusetext[0]);
          PrintLineToLCD(functionusetext[userposition+1]);
        }
        else if (button == '*'){
          userposition-- ;
          if (userposition<0){
            userposition = 3;
          }
          PrintLineToLCD(functionusetext[0]);
          PrintLineToLCD(functionusetext[userposition+1]);
        }
        else if (button == '0'){
          userposition =0;
          return;
        }
        else if (button == '1'){
          double ircal10, ircal50;
          float uscal10, uscal50;
          userposition=0;
          PrintLineToLCD(functionusetext[6]);
          PrintLineToLCD(functionusetext[7]);
          SystickTimer(1500);
          PrintLineToLCD(functionusetext[8]);
          PrintLineToLCD(functionusetext[7]);
          while(1){
            button = KeypadTest();
            if (button == '#'){
              ircal10 = irMedianRaw();
              uscal10 = usScan();
              break;
            }
          }
          PrintLineToLCD(functionusetext[9]);
          PrintLineToLCD(functionusetext[7]);
          while(1){
            button = KeypadTest();
            if (button == '#'){
              ircal50 = irMedianRaw();
              uscal50 = usScan();
              break;
            }
          }
          if (whichSensor == 0){
            irCalibrate(ircal10, ircal50);
          }
          else{
            usCalibrate(uscal10, uscal50);
          }
          PrintLineToLCD(functionusetext[10]);
          PrintLineToLCD(" ");
          SystickTimer(1500);
          return;
        }
        else if (button == '2'){
          userposition =0;
          while (1){
            button = KeypadTest();
            PrintLineToLCD(functionusetext[5]);
            if (whichSensor == 0){
              irString();
              killLED(2);
              lightLED(3);
            }
            else if(whichSensor == 1){
              usString();
              killLED(3);
              lightLED(2);
            }
            if (button == '0'){
              break;
            }
        }
        }
        else if (button == '3'){
          userposition =0;
          return;
        }
        else if (button == '4'){
          userposition = 0;
          return;
        }

      }
}
/*
Pre-Reqs: PrintLineToLCD(), KeypadTest(), global userposition, global parameterusetext
Desc: Bunch of loops, first displays 1st and 2nd lines, then enters loop where accepts button presses and gives corresponding text
*/
void parametertext(){
  char button;
    PrintLineToLCD(parameterusetext[0]);
    PrintLineToLCD(parameterusetext[1]);
    SystickTimer(400);
      while (1){
        button = KeypadTest();
        if (button == '#'){
          userposition++;
          if (userposition>4){
            userposition = 0;
          }
          PrintLineToLCD(parameterusetext[0]);
          PrintLineToLCD(parameterusetext[userposition+1]);
        }
        else if (button == '*'){
          userposition-- ;
          if (userposition<0){
            userposition = 4;
          }
          PrintLineToLCD(parameterusetext[0]);
          PrintLineToLCD(parameterusetext[userposition+1]);
        }
        else if (button == '0'){
          userposition =0;
          return;
        }
        else if (button == '1'){
          userposition=0;
          return;
        }
        else if (button == '2'){
          userposition =0;
          return;
        }
        else if (button == '3'){
          userposition =0;
          return;
        }
        else if (button == '4'){
          userposition = 0;
          return;
        }
        else if (button == '5'){
          userposition = 0;
          PrintLineToLCD(parameterusetext[6]);
          PrintLineToLCD(parameterusetext[7]);
          while (1){
            button = KeypadTest();
            if (button == '1'){
              whichSensor = 0;
              break;
            }
            else if (button == '2'){
              whichSensor = 1;
              break;
            }

          }
          return;
        }

}
}


/*
	-Pre-Reqs:	SetupPWM(uint32_t prescale) with prescale = 10
				setupI2C1()
				setupUART()
				setupADC()
	-Globals:	sweepspeed
				samplespersweep
				sweepstartpos
				sweepstoppos
	-Desc:	Moves servo between servo start and end position at a set speed,
			scans a set number of times.
			If '0' is pressed on the keypad, quits.
*/
void scanMode(void){
	initServo(1);
	setServo(sweepstartpos);
	SystickTimer(100);

	// gap between scans = scan range / sample speed
	uint32_t scangap = (uint32_t)((float)(sweepstoppos - sweepstartpos)/(float)samplespersweep);
	char button;
	int turnAmt;
	int dir = 1;
	int i = samplespersweep - 1;
	while(1){
		// If servo at edge, change direction
		if(servoAtEdge()){
			dir = dir*-1;
			startSweep();
			i = samplespersweep - 1;
			averageDistance = _averageSweep();
		}

		// Turn servo in the "dir" direction by "sweepspeed"
		turnAmt = dir*sweepspeed;
		turnServo(turnAmt);

		if(servo_position % scangap < sweepspeed){
			currentIr = irMedian();
			sendScanData(currentIr);
			sweep[i] = currentIr;
			currentRawIr = irMedianRaw();
		}

		// If 0 was pressed on the keypad, break out of the loop.
		button = KeypadTest();
		if(button == '0'){
			break;
		};

		SystickTimer(10);
	}
}

void sendUART(char strPtr[]){
	UART_RS485SendData(LPC_UART0, strPtr, strlen(strPtr));
}

void startSweep(void){
	char mystring[100] = "";
	sprintf(mystring, "\nsweep_start;");
	sendUART(mystring);
	sprintf(mystring, "\nsweep_size;%d;", samplespersweep);
	sendUART(mystring);
	sprintf(mystring, "\nscan_range;%d;", sweepstoppos-sweepstartpos);
	sendUART(mystring);
}

void sendScanData(double sample){
	char mystring[100] = "";
	sprintf(mystring, "\nsample;%f;", sample);
	sendUART(mystring);
}

void setupGPIO(int port, int pin, int dir){
	PINSEL_CFG_Type PinCfg;
	uint32_t bitVal;
	//Init IO - P2.5
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Portnum = port;
	PinCfg.Pinnum = pin;
	PINSEL_ConfigPin(&PinCfg);

	if(dir)
		bitVal = 0xFFFFFFFF;
	else
		bitVal = 0;

	GPIO_SetDir(port, 0xFFFFFFFF, dir);
}

void TIMER0_IRQHandler(void){
	char msg[BUFFLENGTH] = "";
	char* strPtr = &msg;
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)== SET){
		TIM_Cmd(LPC_TIM0,DISABLE);
		TIM_ResetCounter(LPC_TIM0);
		tim0_flag = 1;
		TIM_Cmd(LPC_TIM0,ENABLE);
	}
  TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

void Timer(uint32_t period){
	TIM_MATCHCFG_Type matchcfg;
	matchcfg.MatchChannel = 0;
	matchcfg.IntOnMatch = ENABLE;
	matchcfg.ResetOnMatch = DISABLE;
	matchcfg.StopOnMatch = DISABLE;
	matchcfg.MatchValue = period;
	matchcfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

	TIM_TIMERCFG_Type timercfg;
	timercfg.PrescaleOption = TIM_PRESCALE_USVAL;
	timercfg.PrescaleValue = 1;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timercfg);
	TIM_ConfigMatch(LPC_TIM0, &matchcfg);

	tim0_flag = 0;
	NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
	NVIC_EnableIRQ(TIMER0_IRQn);
	TIM_Cmd(LPC_TIM0, ENABLE);

	while(!tim0_flag);
	TIM_DeInit(LPC_TIM0);
	NVIC_DisableIRQ(TIMER0_IRQn);
}

float TimerCapture(void){
  PINSEL_CFG_Type PinCfg;
  PinCfg.Funcnum = 3;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
  PinCfg.Portnum = 0;
  PinCfg.Pinnum = 4;
  PINSEL_ConfigPin(&PinCfg);


  TIM_CAPTURECFG_Type captcfg;
  captcfg.CaptureChannel = 0;
  captcfg.IntOnCaption = ENABLE;
  captcfg.FallingEdge = ENABLE;
  captcfg.RisingEdge = ENABLE;

  TIM_TIMERCFG_Type timercfg;
  timercfg.PrescaleOption = TIM_PRESCALE_USVAL;
  timercfg.PrescaleValue = 1;

  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &timercfg);
  TIM_ConfigCapture(LPC_TIM2, &captcfg);


  NVIC_SetPriority(TIMER2_IRQn, ((0x01<<3)|0x01));
  NVIC_EnableIRQ(TIMER2_IRQn);
  TIM_Cmd(LPC_TIM2, ENABLE);

  return ultradist;
}

float usScan(void){
  GPIO_SetValue(2, 0xFFFFFFFF);
  TimerCapture();
  Timer(59998);
  GPIO_ClearValue(2, 0xFFFFFFFF);
  Timer(2);
  return ultradist/58;
}

void TIMER2_IRQHandler(void){
    if(upflag == 0){
      risVal = TIM_GetCaptureValue(LPC_TIM2, TIM_COUNTER_INCAP0);
      upflag = 1;
    }else{
      upflag = 0;
      ultradist = TIM_GetCaptureValue(LPC_TIM2, TIM_COUNTER_INCAP0) - risVal;
      if(ultradist < 0){
        ultradist = -1;
      }
      TIM_ResetCounter(LPC_TIM2);
    }
  TIM_ClearIntCapturePending(LPC_TIM2, TIM_CR0_INT);
}

uint32_t getServoAngle(void){
	return (uint32_t)((9.0f*(float)servo_position)/8.0f);
}

uint32_t getServo(void){
	return servo_position;
}



// do not use on its own
// part of scanMode()
double _averageSweep(void){
	double average = 0;
	int i = samplespersweep - 1;
	while(i>=0){
		average += sweep[i];
	}
	average = average/(double)samplespersweep;
	return average;
}


