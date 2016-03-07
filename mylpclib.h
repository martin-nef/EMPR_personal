#define BUFFLENGTH 31


volatile unsigned long SysTickCnt;


void SysTick_Handler(void);	//deals with SYSTICK interrupts
void SystickTimer(unsigned long tick);
void sendToI2C1(uint32_t i2caddress, uint8_t* tx_dat, uint8_t size);
void lightLED(int ledNum);
void displayBits(int number, long delay);
void killLED(int ledNum);
void setupUART();
void setupI2C1();
void LCDTest();
char KeypadTest();
void ClearLCD();
void PrintToLCD(char ch);
uint8_t CharToHex(char ch);
void SetupPWM(uint32_t prescale);
void SetPWM(uint8_t servo_angle, uint8_t increment);
void SetRawPWM(uint8_t match_channel, uint32_t match_value);
uint32_t adcInput();
void setupDAC();
void setupADC();
void I2CSniffer();
void StartLCD();
void LCDShftLine();
void UART_SendMSG(char* strPtr, char msg[BUFFLENGTH]);
int servoAtEdge();
void turnServo(int amount);
void setServo(uint32_t position);
void initServo(int servo_channel_);
double irDistance(void);
double irDistanceValid();
int cmpfunc(const void *a,const void *b);
double irMedian(void);
void irCalibrate(double adcValue10, double adcValue50);
void irString(void);
void PrintLineToLCD(char* Testing);
void ActualLCDClear();
void PrintToLCD2(char ch);
void greetingtext();
void functiontext();
void parametertext();
void TapeMeasure();
void scanMode(void);
void Timer(uint32_t period);
float usScan(void);
void setupGPIO(int port, int pin, int dir);
void TIMER0_IRQHandler(void);
float TimerCapture(void);
void TIMER2_IRQHandler(void);
double _averageSweep(void);
uint32_t getServoAngle(void);
uint32_t getServo(void);
double irTryDistanceValid(uint32_t cap, int delay);
double irDistanceRaw(void);



