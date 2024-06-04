/******************************************************************************
* Class::       CSC-615-01 Spring 2024
* Name::        Bryan Yan, Natalie Yam, Westley Cho, Griffin Evans
* Student ID::  916956188, 920698945, 922841683,922498210 
* Github-Name:: yan-bryan, ynataliee, westleyc30, griffinevans 
* Project::     Term Project 
* 
* File::        assignment3.c
* 
* Description:: This code defines an algorithm enabling a raspberry pi
* -controlled robotic vehicle to navigate an obstacle course following a path.
*
******************************************************************************/
#include "assignment3.h"

uint32_t fdOne;
uint32_t fdTwo;

#define LINE_LEFT 14
#define LINE_RIGHT 15
#define LINE_MIDDLE_LEFT 23
#define LINE_MIDDLE_RIGHT 24
#define LINE_MIDDLE 21

#define OBSTACLE_FRONT_LEFT 26
#define OBSTACLE_FRONT_RIGHT 19
#define OBSTACLE_SIDE_RIGHT 20

#define MAX_SPEED 100

#define BUTTON_START 17

//Obstacle sensor calculations
volatile int obstacleSensorFlags[] = {1,1,1,1};
//int timeError = 0;
int ko = 40;

//Line sensor calculations
volatile int lineSensorFlags[] ={0,0,0,0,0};
int error = 0;
int previousError = 0;
double kp = 45;
double kd = 20;
double ki = 0;
int iTerm = 0;

//threads for components
pthread_t threadLineLeft, threadLineRight;// IR and Line theads to do work
pthread_t threadLineMiddleLeft, threadLineMiddleRight;
pthread_t threadLineMiddle;
pthread_t threadObstacleMiddleRight, threadObstacleMiddleLeft;
pthread_t threadObstacleSideRight;
//pthread_t threadObstacleSideLeft;

//sensor pins
int lineSensorLeftPin = LINE_LEFT;
int lineSensorRightPin = LINE_RIGHT;
int lineSensorMiddleLeftPin = LINE_MIDDLE_LEFT;
int lineSensorMiddleRightPin = LINE_MIDDLE_RIGHT;
int lineSensorMiddlePin = LINE_MIDDLE;

int obstacleSensorMiddleLeftPin = OBSTACLE_FRONT_LEFT;
int obstacleSensorMiddleRightPin = OBSTACLE_FRONT_RIGHT;
int obstacleSensorSideRightPin = OBSTACLE_SIDE_RIGHT;
//int obstacleSensorSideLeftPin = OBSTACLE_SIDE_LEFT;

//turn timings
int timeStartSec, timeStartMicro;
int timeEndSec, timeEndMicro;
clock_t timeStart, timeEnd;
int turnAdjustment;

//flags
int isDone = 0;
int turnFlag = 0;
int leftForward = 1;
int rightForward = 1;

void Handler(int signo)
{
  //System Exit
  printf("\r\nHandler:Motor Stop\r\n");
  PCA9685_SetPwmDutyCycle(fdOne, PWMA, 0);
  PCA9685_SetPwmDutyCycle(fdOne, PWMB, 0);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMA, 0);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMB, 0);

  i2cClose(fdOne);
  i2cClose(fdTwo);

  isDone = 1;
  if(turnFlag == 1) {
    pthread_join(threadObstacleSideRight, NULL);	
    pthread_join(threadLineMiddle, NULL);

  } else {
    pthread_join(threadLineLeft, NULL);
    pthread_join(threadLineRight, NULL);
    pthread_join(threadLineMiddleLeft, NULL);
    pthread_join(threadLineMiddleRight, NULL);
    pthread_join(threadLineMiddle, NULL);

    pthread_join(threadObstacleMiddleRight, NULL);
    pthread_join(threadObstacleMiddleLeft, NULL);
    //    pthread_join(threadObstacleSideRight, NULL);
  }
  gpioTerminate();

  exit(0);
}

void* checkSensors(void* num) 
{
  int *threadData = (int *) (num);

  if ( *threadData == LINE_LEFT) {
    while ( isDone == 0 && turnFlag == 0) {
      if ( gpioRead(LINE_LEFT) == 0 ) {
        lineSensorFlags[0] = 0;
      } else {
        lineSensorFlags[0] = 1;
      }
    }
  } else if( *threadData == LINE_RIGHT ) {
    while( isDone == 0 && turnFlag == 0 ) {
      if( gpioRead(LINE_RIGHT) == 0 ) {
        lineSensorFlags[4] = 0;
      } else {
        lineSensorFlags[4] = 1;
      }
    }
  } else if( *threadData == LINE_MIDDLE_LEFT ) {
    while( isDone == 0 && turnFlag == 0 ) {
      if( gpioRead(LINE_MIDDLE_LEFT) == 0 ) {
        lineSensorFlags[1] = 0;
      } else {
        lineSensorFlags[1] = 1;
      }
    }
  } else if( *threadData == LINE_MIDDLE_RIGHT ) {
    while( isDone == 0 && turnFlag == 0 ) {
      if( gpioRead(LINE_MIDDLE_RIGHT) == 0 ) {
        lineSensorFlags[3] = 0;
      } else {
        lineSensorFlags[3] = 1;
      }
    }
  } else if( *threadData == LINE_MIDDLE ) {
    while( isDone == 0 ) {
      if( gpioRead(LINE_MIDDLE) == 0 ) {
        lineSensorFlags[2] = 0;
      } else {
        lineSensorFlags[2] = 1;
      }
    }

  } else if( *threadData == OBSTACLE_FRONT_LEFT ) {
    while( isDone == 0 && turnFlag == 0 ) {
      if( gpioRead(OBSTACLE_FRONT_LEFT) == 0 ) {
        obstacleSensorFlags[1] = 0;
      } else {
        obstacleSensorFlags[1] = 1;
      }
    }

  } else if( *threadData == OBSTACLE_FRONT_RIGHT ) {
    while( isDone == 0 && turnFlag == 0 ) {
      if( gpioRead(OBSTACLE_FRONT_RIGHT) == 0 ) {
        obstacleSensorFlags[2] = 0;
      } else {
        obstacleSensorFlags[2] = 1;
      }
    }

  }  else if( *threadData == OBSTACLE_SIDE_RIGHT ) {
    while( isDone == 0 && turnFlag == 1 ) { //use turnFlag == 1 for real runs
      if( gpioRead(OBSTACLE_SIDE_RIGHT) == 0 ) {
        obstacleSensorFlags[3] = 0;
      } else {
        obstacleSensorFlags[3] = 1;
      }
    }
  } 
}

/*
fdOne AIN = Front left, BIN = Back left
fdTwo AIN = Back right, BIN = Front right
*/

int CALCULATE_PID_LINE()
{
  //We calculate the error of where we are versus where we want to be
  if( lineSensorFlags[0] == 1 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = -4;
  } else if (lineSensorFlags[0] == 1 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = -3;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = -2;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = -1;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = 0;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 0 ){
    error = 1;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 0 ){
    error = 2;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 1 ){
    error = 3;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 0
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 1 ){
    error = 4;

  } else if (lineSensorFlags[0] == 1 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = -4;

  } else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 1 ){
    error = 4;

  }  else if (lineSensorFlags[0] == 1 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 0 ){
    error = -4;

  }  else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 1 ){
    error = 4;

  } else if (lineSensorFlags[0] == 1 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 1 ){
    error = 0;

  }  else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 1 && lineSensorFlags[2] == 1
    && lineSensorFlags[3] == 1 && lineSensorFlags[4] == 0 ){
    error = previousError;
  } 

  /*	else if (lineSensorFlags[0] == 0 && lineSensorFlags[1] == 0 && lineSensorFlags[2] == 0
      && lineSensorFlags[3] == 0 && lineSensorFlags[4] == 0 ){
    error = 0;
    isDone = 1;

  }

*/
  previousError = error;

  //Based on the error, we calculate the PID
  return kp*error + kd*(error - previousError) + ki*(iTerm + error);

}	

/* To do's
*	Should really turn set level into it's own function
*/
void MOVE_MOTOR_FOLLOW_LINE(int speed)
{
  int setLeftSpeed = MAX_SPEED + speed;
  int setRightSpeed = MAX_SPEED - speed;
  //    printf("setLeftSpeed is: %d\n", setLeftSpeed);
  //    printf("setRightSpeed is %d\n", setRightSpeed);

  if(setLeftSpeed > 100){
    setLeftSpeed = 100;
  }

  if(setRightSpeed > 100){
    setRightSpeed = 100;
  }

  if(setLeftSpeed < -100 ) { //this is what you need to adjust after wheel changes
    setLeftSpeed = -100; // values cap at zero after
  }

  if(setRightSpeed < -100 ){
    setRightSpeed = -100;
  }

  //	printf("setLeftSpeed after adjustments is: %d\n", setLeftSpeed);
  //	printf("setRightSpeed after adjustments is %d\n", setRightSpeed);

  if( setLeftSpeed < 0 ) {
    if( leftForward == 1 ){
      PCA9685_SetLevel(fdOne, AIN1, 0);
      PCA9685_SetLevel(fdOne, AIN2, 1);
      PCA9685_SetLevel(fdOne, BIN1, 0);
      PCA9685_SetLevel(fdOne, BIN2, 1);

      leftForward = 0;
    }

    if( rightForward == 0 ){
      PCA9685_SetLevel(fdTwo, AIN1, 0);
      PCA9685_SetLevel(fdTwo, AIN2, 1);
      PCA9685_SetLevel(fdTwo, BIN1, 0);
      PCA9685_SetLevel(fdTwo, BIN2, 1);

      rightForward = 1;
    }

    PCA9685_SetPwmDutyCycle(fdOne, PWMA, ( -1 * setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdOne, PWMB, ( -1 * setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMA, ( setRightSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMB, ( setRightSpeed ) );
  } else if( setRightSpeed < 0 ) {
    if( leftForward == 0 ){
      PCA9685_SetLevel(fdOne, AIN1, 1);
      PCA9685_SetLevel(fdOne, AIN2, 0);
      PCA9685_SetLevel(fdOne, BIN1, 1);
      PCA9685_SetLevel(fdOne, BIN2, 0);

      leftForward = 1;
    }

    if( rightForward == 1 ){
      PCA9685_SetLevel(fdTwo, AIN1, 1);
      PCA9685_SetLevel(fdTwo, AIN2, 0);
      PCA9685_SetLevel(fdTwo, BIN1, 1);
      PCA9685_SetLevel(fdTwo, BIN2, 0);

      rightForward = 0;
    }

    PCA9685_SetPwmDutyCycle(fdOne, PWMA, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdOne, PWMB, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMA, ( -1 * setRightSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMB, ( -1 * setRightSpeed ) );
  } else {
    if( leftForward == 0 ){
      PCA9685_SetLevel(fdOne, AIN1, 1);
      PCA9685_SetLevel(fdOne, AIN2, 0);
      PCA9685_SetLevel(fdOne, BIN1, 1);
      PCA9685_SetLevel(fdOne, BIN2, 0);

      leftForward = 1;
    }

    if( rightForward == 0 ){
      PCA9685_SetLevel(fdTwo, AIN1, 0);
      PCA9685_SetLevel(fdTwo, AIN2, 1);
      PCA9685_SetLevel(fdTwo, BIN1, 0);
      PCA9685_SetLevel(fdTwo, BIN2, 1);

      rightForward = 1;
    }

    PCA9685_SetPwmDutyCycle(fdOne, PWMA, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdOne, PWMB, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMA, ( setRightSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMB, ( setRightSpeed ) );
  }

  previousError = error;

}

void MOVE_MOTOR_FORWARD(int speed)
{
  int setSpeed;
  if(speed > 100){
    setSpeed = 100;
  } else if(speed < 0){
    setSpeed = 0;
  } else {
    setSpeed = speed;
  }

  if( leftForward == 0 ){
    PCA9685_SetLevel(fdOne, AIN1, 1);
    PCA9685_SetLevel(fdOne, AIN2, 0);
    PCA9685_SetLevel(fdOne, BIN1, 1);
    PCA9685_SetLevel(fdOne, BIN2, 0);

    leftForward = 1;
  }

  if( rightForward == 0 ){
    PCA9685_SetLevel(fdTwo, AIN1, 0);
    PCA9685_SetLevel(fdTwo, AIN2, 1);
    PCA9685_SetLevel(fdTwo, BIN1, 0);
    PCA9685_SetLevel(fdTwo, BIN2, 1);

    rightForward = 1;
  }

  PCA9685_SetPwmDutyCycle(fdOne, PWMA, setSpeed);
  PCA9685_SetPwmDutyCycle(fdOne, PWMB, setSpeed);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMA, setSpeed);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMB, setSpeed);
}

void MOVE_MOTOR_PIVOT_LEFT(int speed)
{
  int setSpeed;
  if(speed > 100){
    setSpeed = 100;
  }else{
    setSpeed = speed;
  }

  if( leftForward == 1 ){
    PCA9685_SetLevel(fdOne, AIN1, 0);
    PCA9685_SetLevel(fdOne, AIN2, 1);
    PCA9685_SetLevel(fdOne, BIN1, 0);
    PCA9685_SetLevel(fdOne, BIN2, 1);

    leftForward = 0;
  }

  if( rightForward == 0 ){
    PCA9685_SetLevel(fdTwo, AIN1, 0);
    PCA9685_SetLevel(fdTwo, AIN2, 1);
    PCA9685_SetLevel(fdTwo, BIN1, 0);
    PCA9685_SetLevel(fdTwo, BIN2, 1);

    rightForward = 1;
  }

  PCA9685_SetPwmDutyCycle(fdOne, PWMA, setSpeed );
  PCA9685_SetPwmDutyCycle(fdOne, PWMB, setSpeed );
  PCA9685_SetPwmDutyCycle(fdTwo, PWMA, setSpeed );
  PCA9685_SetPwmDutyCycle(fdTwo, PWMB, setSpeed );
}

void MOVE_MOTOR_TURN_RIGHT_PROPORTIONAL(int speed)
{
  int setRightSpeed = MAX_SPEED - speed;
  int setLeftSpeed = MAX_SPEED + speed;

  if(setLeftSpeed > 100){
    setLeftSpeed = 100;
  }

  if(setRightSpeed > 100){
    setRightSpeed = 100;
  }

  if(setLeftSpeed < -100 ) { //this is what you need to adjust after wheel changes
    setLeftSpeed = -100; // values cap at zero after
  }

  if(setRightSpeed < -100 ){
    setRightSpeed = -100;
  }

  //	printf("setRightSpeed after adjustment is: %d\n", setRightSpeed);
  //	printf("setLeftSpeed after adjustment is: %d\n", setLeftSpeed);

  if( setRightSpeed < 0 ) {
    if( leftForward == 0 ){
      PCA9685_SetLevel(fdOne, AIN1, 1);
      PCA9685_SetLevel(fdOne, AIN2, 0);
      PCA9685_SetLevel(fdOne, BIN1, 1);
      PCA9685_SetLevel(fdOne, BIN2, 0);

      leftForward = 1;
    }

    if( rightForward == 1 ){
      PCA9685_SetLevel(fdTwo, AIN1, 1);
      PCA9685_SetLevel(fdTwo, AIN2, 0);
      PCA9685_SetLevel(fdTwo, BIN1, 1);
      PCA9685_SetLevel(fdTwo, BIN2, 0);

      rightForward = 0;
    }

    PCA9685_SetPwmDutyCycle(fdOne, PWMA, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdOne, PWMB, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMA, ( -1 * setRightSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMB, ( -1 * setRightSpeed ) );
  } else {
    if( leftForward == 0 ){
      PCA9685_SetLevel(fdOne, AIN1, 1);
      PCA9685_SetLevel(fdOne, AIN2, 0);
      PCA9685_SetLevel(fdOne, BIN1, 1);
      PCA9685_SetLevel(fdOne, BIN2, 0);

      leftForward = 1;
    }

    if( rightForward == 0 ){
      PCA9685_SetLevel(fdTwo, AIN1, 0);
      PCA9685_SetLevel(fdTwo, AIN2, 1);
      PCA9685_SetLevel(fdTwo, BIN1, 0);
      PCA9685_SetLevel(fdTwo, BIN2, 1);

      rightForward = 1;
    }

    PCA9685_SetPwmDutyCycle(fdOne, PWMA, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdOne, PWMB, ( setLeftSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMA, ( setRightSpeed ) );
    PCA9685_SetPwmDutyCycle(fdTwo, PWMB, ( setRightSpeed ) );
  }
}

void MOVE_MOTOR_AROUND_OBJECT_LEFT()
{
  turnFlag = 1;
  pthread_join(threadLineLeft, NULL);
  pthread_join(threadLineRight, NULL);
  pthread_join(threadLineMiddleLeft, NULL);
  pthread_join(threadLineMiddleRight, NULL);
  pthread_join(threadObstacleMiddleLeft, NULL);
  pthread_join(threadObstacleMiddleRight, NULL);
  pthread_create(&threadObstacleSideRight, NULL, &checkSensors, (void*) (&obstacleSensorSideRightPin) );

  MOVE_MOTOR_PIVOT_LEFT(MAX_SPEED);
  gpioSleep(PI_TIME_RELATIVE, 0, 400000);

  while(lineSensorFlags[2] == 0) {
    //		printf("I'm in lineSensorFlags[2] == 0\n");
    if(obstacleSensorFlags[3] == 1) {
      //			printf("Side Sensor is off....turning\n");
      timeStart = clock() / CLOCKS_PER_SEC;
      while(obstacleSensorFlags[3] == 1){
        //				MOVE_MOTOR_PIVOT_LEFT(0);
        timeEnd = clock() / CLOCKS_PER_SEC;
        turnAdjustment = ko * (timeEnd - timeStart + 1 );
        MOVE_MOTOR_TURN_RIGHT_PROPORTIONAL(turnAdjustment);

        if(lineSensorFlags[2] == 1){
          break;
        }
      }
    } else {
      MOVE_MOTOR_FORWARD(MAX_SPEED);
    }
  }

  turnFlag = 0;
  pthread_create(&threadLineLeft, NULL, &checkSensors, (void*) (&lineSensorLeftPin));
  pthread_create(&threadLineRight, NULL, &checkSensors, (void*) (&lineSensorRightPin));
  pthread_create(&threadLineMiddleLeft, NULL, &checkSensors, (void*) (&lineSensorMiddleLeftPin));
  pthread_create(&threadLineMiddleRight, NULL, &checkSensors, (void*) (&lineSensorMiddleRightPin));
  pthread_create(&threadObstacleMiddleLeft, NULL, &checkSensors, (void*) (&obstacleSensorMiddleLeftPin));
  pthread_create(&threadObstacleMiddleRight, NULL, &checkSensors, (void*) (&obstacleSensorMiddleRightPin));
  pthread_join(threadObstacleSideRight, NULL);	

  //	printf("We are exiting turning...\n");
}


void MOVE_MOTOR_BACKWARD(int speed)
{
  int setSpeed;
  if(speed > 100){
    setSpeed = 100;
  }else{
    setSpeed = speed;
  }

  PCA9685_SetPwmDutyCycle(fdOne, PWMA, setSpeed);
  PCA9685_SetLevel(fdOne, AIN1, 0);
  PCA9685_SetLevel(fdOne, AIN2, 1);

  PCA9685_SetPwmDutyCycle(fdOne, PWMB, setSpeed);
  PCA9685_SetLevel(fdOne, BIN1, 0);
  PCA9685_SetLevel(fdOne, BIN2, 1);

  PCA9685_SetPwmDutyCycle(fdTwo, PWMA, setSpeed);
  PCA9685_SetLevel(fdTwo, AIN1, 1);
  PCA9685_SetLevel(fdTwo, AIN2, 0);

  PCA9685_SetPwmDutyCycle(fdTwo, PWMB, setSpeed);
  PCA9685_SetLevel(fdTwo, BIN1, 1);
  PCA9685_SetLevel(fdTwo, BIN2, 0);
}

int main(void)
{
  //1.System Initialization
  if(gpioInitialise() < 0){
    printf("gpio init failed!\n");
    return 1;
  }else{
    printf("gpio init success!\n");
  }

  int lineSensorLeftPin = LINE_LEFT;
  int lineSensorRightPin = LINE_RIGHT;
  int lineSensorMiddleLeftPin = LINE_MIDDLE_LEFT;
  int lineSensorMiddleRightPin = LINE_MIDDLE_RIGHT;
  int lineSensorMiddlePin = LINE_MIDDLE;

  gpioSetMode(LINE_LEFT, PI_INPUT);
  gpioSetMode(LINE_RIGHT, PI_INPUT);
  gpioSetMode(LINE_MIDDLE_LEFT, PI_INPUT);
  gpioSetMode(LINE_MIDDLE_RIGHT, PI_INPUT);
  gpioSetMode(LINE_MIDDLE, PI_INPUT);
  gpioSetMode(OBSTACLE_FRONT_LEFT, PI_INPUT);
  gpioSetMode(OBSTACLE_FRONT_RIGHT, PI_INPUT);
  gpioSetMode(OBSTACLE_SIDE_RIGHT, PI_INPUT);

  gpioSetMode(BUTTON_START, PI_INPUT);
  gpioSetPullUpDown(BUTTON_START, PI_PUD_UP);

  //////////

  //2.Motor Initialization

  fdOne = PCA9685_Init(SLAVE_ADD_ONE); //Our slave address is 0x40 for this particular hat
  PCA9685_SetPWMFreq(fdOne, 1000); //Set the frequency to 100Hz

  fdTwo = PCA9685_Init(SLAVE_ADD_TWO);//Our slave address is 0x50 for this particular hat
  PCA9685_SetPWMFreq(fdTwo, 1000);//Set the frequency to 100Hz

  //////////

  //Exception Handling: Ctrl+c
  signal(SIGINT, Handler);

  pthread_create(&threadLineLeft, NULL, &checkSensors, (void*) (&lineSensorLeftPin));
  pthread_create(&threadLineRight, NULL, &checkSensors, (void*) (&lineSensorRightPin));
  pthread_create(&threadLineMiddleLeft, NULL, &checkSensors, (void*) (&lineSensorMiddleLeftPin));
  pthread_create(&threadLineMiddleRight, NULL, &checkSensors, (void*) (&lineSensorMiddleRightPin));
  pthread_create(&threadLineMiddle, NULL, &checkSensors, (void*) (&lineSensorMiddlePin) );
  pthread_create(&threadObstacleMiddleLeft, NULL, &checkSensors, (void*) (&obstacleSensorMiddleLeftPin));
  pthread_create(&threadObstacleMiddleRight, NULL, &checkSensors, (void*) (&obstacleSensorMiddleRightPin));
  //    pthread_create(&threadObstacleSideRight, NULL, &checkSensors, (void*) (&obstacleSensorSideRightPin));

  //pthread_create(&threadObstacleSideLeft, NULL, &checkSensors, (void*) (&obstacleSideLeftPin) );
  //pthread_create(&threadObstacleSideRight, NULL, &checkSensors, (void*) (&obstacleSideRightPin) );

  //    gpioServo(SERVO, neutral);

  PCA9685_SetLevel(fdOne, AIN1, 1);
  PCA9685_SetLevel(fdOne, AIN2, 0);
  PCA9685_SetLevel(fdOne, BIN1, 1);
  PCA9685_SetLevel(fdOne, BIN2, 0);

  PCA9685_SetLevel(fdTwo, AIN1, 0);
  PCA9685_SetLevel(fdTwo, AIN2, 1);
  PCA9685_SetLevel(fdTwo, BIN1, 0);
  PCA9685_SetLevel(fdTwo, BIN2, 1);

  //    gpioTime(PI_TIME_RELATIVE, &timeStartSec, &timeStartMicro);
  //	timeStart = clock() / CLOCKS_PER_SEC;

  while( gpioRead(BUTTON_START) == 1 ) {
    //		printf("We are in button\n");
    if(gpioRead(BUTTON_START) == 0){
      //			printf("Button got pressed\n");
      break;
    }
  }
  gpioSleep(PI_TIME_RELATIVE, 2, 0);

  while( isDone == 0 ){
    //    gpioTime(PI_TIME_RELATIVE, &timeEndSec, &timeEndMicro);

    //	    timeEnd = clock() / CLOCKS_PER_SEC;

    //	    printf("Difference in time is: %d\n", timeEnd - timeStart );

    //	    turnAdjustment = ko * (timeEnd - timeStart + 1);

    //	    printf("turnAdjustment is: %d\n", turnAdjustment);

    //	    MOVE_MOTOR_TURN_RIGHT_PROPORTIONAL(turnAdjustment); 


    //    printf("time difference is: %d\n", (timeEnd - timeStart) );	

    int adjustment = CALCULATE_PID_LINE();    
    MOVE_MOTOR_FOLLOW_LINE(adjustment);

    if( obstacleSensorFlags[1] == 0 && obstacleSensorFlags[2] == 0 ){
      //			MOVE_MOTOR_FORWARD(0);		
      //			timeStart = clock() / CLOCKS_PER_SEC;
      //			timeEnd = (clock() / CLOCKS_PER_SEC) + 10;
      //			while( timeStart < timeEnd ){
      //				timeStart = clock() / CLOCKS_PER_SEC;
      //			}
      /*
      if( obstacleSensorFlags[1] == 0 && obstacleSensorFlags[2] == 0 ) {
        MOVE_MOTOR_AROUND_OBJECT_LEFT(); 
      }
      */

      MOVE_MOTOR_AROUND_OBJECT_LEFT();
    }

    //	previousError = error;

    //	printf("obstacle side right is: %d\n", obstacleSensorFlags[3]);

  }

  PCA9685_SetPwmDutyCycle(fdOne, PWMA, 0);
  PCA9685_SetPwmDutyCycle(fdOne, PWMB, 0);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMA, 0);
  PCA9685_SetPwmDutyCycle(fdTwo, PWMB, 0);

  i2cClose(fdOne);
  i2cClose(fdTwo);

  //    isDone = 1;
  if(turnFlag == 1) {
    pthread_join(threadObstacleSideRight, NULL);	
    pthread_join(threadLineMiddle, NULL);

  } else {
    pthread_join(threadLineLeft, NULL);
    pthread_join(threadLineRight, NULL);
    pthread_join(threadLineMiddleLeft, NULL);
    pthread_join(threadLineMiddleRight, NULL);
    pthread_join(threadLineMiddle, NULL);

    pthread_join(threadObstacleMiddleRight, NULL);
    pthread_join(threadObstacleMiddleLeft, NULL);
    //    pthread_join(threadObstacleSideRight, NULL);
  }

  gpioTerminate();

  /*    isDone = 1;
    pthread_join(threadLineLeft, NULL);
    pthread_join(threadLineRight, NULL);
    pthread_join(threadLineMiddleLeft, NULL);
    pthread_join(threadLineMiddleRight, NULL);
    pthread_join(threadLineMiddle, NULL);
  */
  //    pthread_join(threadObstacleSonar, NULL);

  //3.System Exit

  //    i2cClose(fdOne);
  //    i2cClose(fdTwo);
  //DEV_ModuleExit();// gpioTerminate() in our case
  return 0;
}
