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
#include "myproject.h"

const int LOW_BOUND = 110;
const int HIGH_BOUND = 190;


// range of the scanning cone in servo distance
int conerange;

// Servo globals
const uint8_t servo_channel; // pwm channel used by the servo
int my_servo_position = 0; // current servo position. will be between sweepstartpos and sweepstoppos
int edgeone; // right edge (min 0)
int edgetwo; // left edge(max 80)

// Main configuration
// range of the scanning cone in degrees
const int coneangle = 30;
// amount to turn by after each scan
int speed = 2;
// delay after servo is moved
unsigned long servo_delay = 35;

// Vars used for debugging/testing
// internally used by timer
int timerstart = 0;
// internally used by debugPrintNum()
int debugPrintInt = 1;

/*
Inputs:	double sweep[][2] - the sweep array
		int noscans - number of scans made in last sweep
Desc:	Looks for the group of adjacent scans with the smallest average distance
Output:	Position of the scan in the middle of the group, i.e. object position
*/
int whereIsObject(double sweep[][2], int noscans){
	// char string[50] = "";
	// sprintf(string, "\r\nmin vals:");
	// sendUART(string);

	double minAry[noscans/3][2];
	int i;

	for (i = 0; i < noscans/3; ++i) {
		minAry[i][0] = 999999999999;
		minAry[i][1] = -1;
	}

	for (i = 0; i < (noscans - noscans/3); ++i) {
		if (avg(minAry, noscans/3, 0, noscans/3) > avg(sweep, noscans, i, (i + noscans/3))) {
			// sprintf(string, "\r\n averages: %.2f, %.2f   ", avg(minAry, noscans/3, 0, noscans/3), avg(sweep, noscans, i, (i + noscans/3)));
			// sendUART(string);
			// sprintf(string, "\r\n %.2f > %.2f", avg(minAry, noscans/3, 0, noscans/3), avg(sweep, noscans, i, (i + noscans/3)));
			// sendUART(string);
			int x;
			for (x = 0; x < (noscans/3); ++x) {
				minAry[x][0] = sweep[i+x][0];
				minAry[x][1] = sweep[i+x][1];
			}
			// sprintf(string, "\r\nminAry:");
			// sendUART(string);
			// x = 0;
			// while(1){
			// 	if(x == noscans/3){
			// 		break;
			// 	}

			// 	sprintf(string, "\r\n %d: %.2f, at: %d", x, minAry[x][0], (int)minAry[x][1]);
			// 	sendUART(string);
			// 	x++;
			// }
			// sprintf(string, "\r\nsweep:");
			// sendUART(string);
			// x = i;
			// while(1){
			// 	if(x == (i + noscans/3)){
			// 		break;
			// 	}
			// 	sprintf(string, "\r\n %d: %.2f, at: %d", x-i, sweep[x][0], (int)sweep[x][1]);
			// 	sendUART(string);
			// 	x++;
			// }
			// sprintf(string, "\r\n %.2f == %.2f", avg(minAry, noscans/3, 0, noscans/3), avg(sweep, noscans, i, (i + noscans/3)));
			// sendUART(string);
			// sprintf(string, "\r\n min average: %.2f, at: %d", avg(minAry, noscans/3, 0, noscans/3), minAry[noscans/3/2][1]);
			// sendUART(string);
		}
	}

	// int x = 0;
	// while(1){
	// 	if(x == noscans/3){
	// 		break;
	// 	}
	// 	sprintf(string, "\r\n %d: %.2f, at: %d", x, minAry[x][0], (int)minAry[x][1]);
	// 	sendUART(string);
	// 	x++;
	// }
	// sprintf(string, "\r\n  avg: %.2f", avg(minAry, noscans/3, 0, noscans/3));
	// sendUART(string);

	return avgPos(minAry, noscans/3, 0, noscans/3);
}

double avg(double array[][2], int arySize, int start, int end){
	// char string[50] = "";
	double sum = 0;
	double count = 0;
	int i;
	for (i = start; i < end; ++i) {
		sum += array[i][0];
		count++;
	}
	// sprintf(string, "\r\n avg: %.2f", sum/count);
	// sendUART(string);
	return sum/count;
}

int avgPos(double array[][2], int arySize, int start, int end) {
	double sum = 0;
	double count = 0;
	int i;
	for (i = start; i < end; ++i) {
		sum += array[i][1];
		count++;
	}
	return (sum/count);
}

/*
Outputs:	-1 if at first edge
			1 if at second edge
			0 if not at edge
*/
int getEdge(void){
	if (my_servo_position == edgeone) {
		return -1;
	} else if (my_servo_position == edgetwo) {
		return 1;
	} else {
		return 0;
	}
}

/*
Desc:	Converts degrees to serv distance
*/
int degrees_to_servo_dist(int angle){
	return (int)((8.0f * (float)angle)/9.0f);
}

/*
Desc:	Lets you calibrate the ir sensor
*/
void calibrateSensors(void){
	double usValue10;
	double usValue50;
	char button;
	char string[100] = "";
	
	sprintf(string, "\r\nSet object at 10 cm and press #\n");
	sendUART(string);
	while(1){
		usValue10 = (double)adcInput();
		// sprintf(string, "\r         ");
		// sendUART(string);
		sprintf(string, "\r%f", (float)usValue10);
		sendUART(string);
		button = KeypadTest();
		if(button == '#'){
			break;
		}
	}
	SystickTimer(200);
	sprintf(string, "\r\nSet object at 50 cm and press #\n");
	sendUART(string);
	while(1){
		usValue50 = (double)adcInput();
		sprintf(string, "\r%f", usValue50);
		sendUART(string);
		button = KeypadTest();
		if(button == '#'){
			break;
		}
	}
	sprintf(string, "\r\nusValue10: %f", usValue10);
	sendUART(string);
	sprintf(string, "\r\nusValue50: %f", usValue50);
	sendUART(string);
	irCalibrate(usValue10, usValue50);
}

/*
Pre-Reqs:initServo()
Desc: Set the servo position. Claps the position given to current servo range
Inputs: int position - ranges from edgeone to edgetwo
*/
void mySetServo(int position){

	int my_servo_position_orig = my_servo_position;
	int match_value = LOW_BOUND + position;
	if (match_value > (LOW_BOUND + edgetwo)){

		match_value = LOW_BOUND + edgetwo;
	}
	if (match_value < (LOW_BOUND + edgeone)){

		match_value = LOW_BOUND + edgeone;
	}
	SetRawPWM(servo_channel, match_value);
	my_servo_position = match_value - LOW_BOUND;
	SystickTimer(servo_delay);
}

/*
Pre-Reqs:initServo()
Desc: Set the servo position. Moves gradually, 1 step at a time.
Inputs: int position - ranges from edgeone to edgetwo
*/
void servoMove(int position){
	while (1) {
		if (my_servo_position < position){
			myTurnServo(speed);
		} else if (my_servo_position > position) {
			myTurnServo(-speed);
		} else {
			break;
		}
	}
}

/*
Pre-Reqs:initServo()
Desc: Move the servo by specified amount
Inputs: int amount - negative for right, positive for left.
*/
void myTurnServo(int amount){
	mySetServo(my_servo_position + amount);
}

/*
Pre-Reqs:initServo()
Desc: Move the servo by specified amount. Moves gradually, 1 step at a time.
Inputs: int amount - negative for right, positive for left.
*/
void myTurnServoMove(int amount){
	servoMove(my_servo_position + amount);
}

/*
Desc:	Used for debugging, intended for printing the sweep after it is complete.
*/
void printSweep(double sweep[][2], int noscans){		
	char string[50] = "";
	sprintf(string, "\r\nsweep:");
	sendUART(string);

	int x = 0;
	while(1){
		if(x == noscans){
			break;
		}
		sprintf(string, "\r\n %d: %.2f, at: %d", x, sweep[x][0], (int)sweep[x][1]);
		sendUART(string);
		x++;
	}
	// while(1){
	// 	sprintf(string, "%.2f,", sweep[x][0]);
	// 	sendUART(string);
	// 	if(x == noscans - 1){
	// 		break;
	// 	}
	// 	x++;
	// }
}

/*
Desc:	Used for timing sections of code.
		Starts the timer.
*/
void timerStart(void){
	timerstart = SysTickCnt;
}

/*
Desc:	Used for timing sections of code.
		Ends the timer. 
Output:	Returns an int representing milliseconds 
		since timerStart() was called.
*/
int timerEnd(void){
	return SysTickCnt - timerstart;
}

/*
Desc:	Used for timing sections of code.
		Ends the timer and prints time taken to UART.
*/
void timerEndPrint(void){
	char string[40] = "";
	sprintf(string, "\r\ntime: %d ms", timerEnd());
	sendUART(string);
}

/*
Desc:	Recenters the current scanning cone around a given servo position.
		Clamps the edges to the servo's range.
Input:	int servo_pos - servo position to center araound
*/
void recenter(int servo_pos) {
	int edgeoneorig = edgeone;
	int edgetwoorig = edgetwo;

	edgeone = servo_pos - conerange/2;
	edgetwo = edgeone + conerange;
	if (edgetwo > 80) {
		edgetwo = 80;
		edgeone = edgetwo - conerange;
	}
	if (edgeone < 0) {
		edgeone = 0;
		edgetwo = edgeone + conerange;
	}
}

/*
Desc:	Temporary function used for testing.
*/
void tempTest(void){
	// int samplesize = 200;
	// char button;
	// char string[100] = "";
	// int x = 0;
	// sprintf(string, "\r\nTest WITH capacitor\r\nPress # to continue\r\nsweep:");
	// sendUART(string);
	// while(1){
	// 	button = KeypadTest();
	// 	if(button == '#'){
	// 		break;
	// 	}
	// }
	// while(x<samplesize){

	// 	sprintf(string, "%f,", irDistance());
	// 	sendUART(string);
	// 	x++;
	// }
	// sprintf(string, "\r\nTest WITHOUT capacitor\r\nPress # to continue\r\nsweep:");
	// sendUART(string);
	// while(1){
	// 	button = KeypadTest();
	// 	if(button == '#'){
	// 		break;
	// 	}
	// }
	// x = 0;
	// while(x<samplesize){

	// 	sprintf(string, "%f,", irDistance());
	// 	sendUART(string);
	// 	x++;
	// }

	// while(1){
	// 	sprintf(string, "\r\nTest done\n");
	// 	sendUART(string);
	// }

	int x = 0;
	char button = "";
	char string[20] = "";
	sprintf(string, "\r\ntemptest");
	sendUART(string);
	double readings1[5];
	double readings2[5];
	double min1 = -1;
	double min2 = -1;
	double max1 = 9999;
	double max2 = 9999;

	while(x<5){
		x++;
		readings1[x] = irDistance();
		if (min1 > readings1[x]) {
			min1 = readings1[x];
		}
		if (max1 < readings1[x]) {
			max1 = readings1[x];
		}
		// sprintf(string, "\r reading %d: %f    ", x, irMedian_());
		// sendUART(string);
		// SystickTimer(80);
		// button = KeypadTest();
		// if(button == '0'){
			// break;
		// };
	};
	while(1){
		button = KeypadTest();
		if(button == '0'){
			break;
		};
	}
	x = 0;
	while(x<5){
		readings2[x] = irDistance();
		if (min2 > readings2[x]) {
			min2 = readings2[x];
		}
		if (max2 < readings2[x]) {
			max2 = readings2[x];
		}
	}
	double range1 = max1 - min1;
	double range2 = max2 - min2;
	sprintf(string, "range1: %f, \r\nrange2: %f  ", range1, range2);
	sendUART(string);
}

/*
	Desc:	irMedian() rewritten to suit this project.
			Tries to get nine infrared readings.
			Tries to get a valid reading up to ten times for each reading
			with a delay of 1 ms.
	Output:	Infrared distance as a double. The reading is the median value of
			all valid readings. If there are no valid readings, return 999999  
*/
double irMedian_(void){
	char string[30] = "";
    int i;
    double irArray[9];
    int validnum = 0;

    for(i=0; i<9; i++){
        irArray[i] = irTryDistanceValid(10, 1);
        if (irArray[i] != -1) {
        	validnum++;
        }
    }

    qsort(irArray, 9, sizeof(double), cmpfunc);
    if (validnum == 0){
		return 999999;
    }
    return irArray[(int)((8 - validnum) + round((float)validnum / (float)2))];
}

/*
Desc: deleteme
*/
double irMedianRawTimed(int time_){
    double rawArray[1000];
    int arySize = 0;
    int startTime = SysTickCnt;

	while(1){
		uint32_t adcValue = ADC_ChannelGetData(LPC_ADC,4);

		rawArray[arySize] = (double)adcValue;

		arySize++;
		if ((SysTickCnt - startTime) >= time_) {
			break;
		}
	}

	qsort(rawArray, arySize, sizeof(double), cmpfunc);
	
	return rawArray[arySize/2];
}

/*
Desc:	Prints debugPrintInt to UART and increments it.
*/
void debugPrintNum(void){
	char string[100] = "";
	sprintf(string, "\r\ndebug int %d", debugPrintInt);
	sendUART(string);
	debugPrintInt++;
}

/*
Desc:	The main tracking function. Sweeps left and right
		within the boundaries of the current scanning cone.
		Upon reaching an edge, tries to recenter the scanning
		cone on the object position.
*/
void track(void){
	// gap between scans = scan range / sample speed
	conerange = degrees_to_servo_dist(coneangle);
	conerange = conerange - (conerange % speed);
	edgeone = 0;
	edgetwo = edgeone + conerange;

	recenter(40);

	// set the servo at the right edge.
	mySetServo(edgeone);


	// number of scans made in current sweep
	int noscans = 0;
	// current scan number in sweep
	int currscan = 0;
	// current sweep
	double sweep[100][2];
	char button;
	// amount to turn servo by
	int turnAmt;
	// direction of sweep
	int dir = 1;
	// object position
	int objpos = -1;

	char string[40] = "";
	sprintf(string, "\r\n");
	sendUART(string);

	while(1){

		// Turn servo in the dir direction by speed
		turnAmt = dir*speed;
		myTurnServo(turnAmt);

		// Scan the distance to sensor with IR
		double ir = irMedian_();

		// sprintf(string, "\r distance: %f, pos: %d  ", ir, my_servo_position);
		// sendUART(string);

		sweep[currscan][0] = ir;
		sweep[currscan][1] = my_servo_position;
		currscan += 1;
		noscans += 1;

		// if (mindist > ir) {
		// 	mindist = ir;
		// 	objpos = my_servo_position;
		// }


		int edge = getEdge();
		// If servo at edge
		if (edge) {
			// get object position from the sweep
			objpos = whereIsObject(sweep, noscans);
			
			// printSweep(sweep, noscans);
			
			// if object position valid
			if (objpos > 0) {
				// recenter around it
				recenter(objpos);
			}

			sprintf(string, "\r\nobjpos %d   ", objpos);
			sendUART(string);
			sprintf(string, "\r\nspeed %d   ", speed);
			sendUART(string);
			sprintf(string, "\r\nservo_delay %d   ", servo_delay);
			sendUART(string);

			// if was at left edge before recentering
			if (edge > 0) {
				// move the servo to the new left edge
				servoMove(edgetwo);
				// set the direction to right
				dir = -1;
			// if was at right edge before recentering
			} else if (edge < 0) {
				// move the servo to the new right edge
				servoMove(edgeone);
				// set the direction to left
				dir = 1;
			}

			// reset current scan number
			currscan = 0;
			// reset number of scans made
			noscans = 0;
			// reset object position
			objpos = -1;
		}

		// If 0 was pressed on the keypad, break out of the loop.
		button = KeypadTest();
		if(button == '0' ){
			break;
		};
		if(button == '1' ){
			speed--;
			if (speed < 1) {
				speed = 1;
			}
			SystickTimer(75);
		};
		if(button == '2' ){
			speed++;
			if (speed > 5) {
				speed = 5;
			}
			SystickTimer(75);
		};
		if(button == '4' ){
			servo_delay -= 5;
			if (servo_delay < 10) {
				servo_delay = 10;
			}
			SystickTimer(75);
		};
		if(button == '5' ){
			servo_delay += 5;
			if (servo_delay > 60) {
				servo_delay = 60;
			}
			SystickTimer(75);
		};
	}
}

/*
Desc: Initializes required devices, calibrates the IR, runs the tracking function.
*/
void main(void){
	setupI2C1();
	setupUART();
	setupADC();
	uint32_t prescale = 10;
	SetupPWM(prescale);
	initServo(1);

	// calibrateSensors();
	irCalibrate(2934.000000, 776.000000);
	tempTest();
	
	// tempTest();
	
	// track();
}
