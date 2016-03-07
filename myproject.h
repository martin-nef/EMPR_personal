int whereIsObject(double sweep[][2], int noscans);
double avg(double array[][2], int arySize, int start, int end);
int getEdge(void);
int degrees_to_servo_dist(int angle);
void calibrateSensors(void);
void mySetServo(int position);
void servoMove(int position);
void myTurnServo(int amount);
void myTurnServoMove(int amount);
// void printSweep(void);
void timerStart(void);
int timerEnd(void);
void timerEndPrint(void);
void recenter(int servo_pos);
void tempTest(void);
double irMedian_(void);
double irMedianRawTimed(int time_);
void debugPrintNum(void);
void track(void);
void move(void);