#ifndef MOTORS_H
#define MOTORS_H

#include <Adafruit_PWMServoDriver.h>
#include "display.h"
// Servos Definitions
extern Adafruit_PWMServoDriver pwm;
extern bool handState; // Hand state
#define SERVOMIN 75 // Minimum pulse length count
#define SERVOMAX 500  // Maximum pulse length count
// ANALOG 50Hz
// DIGITAL 60Hz
#define SERVO_FREQ 50 // Servo frequency (60 Hz)
// Feedback Pins
#define FEED_1_BACK_INDEX 13 // Feedback pin
#define FEED_2_BACK_INDEX 14 // Feedback pin
#define FEED_3_BACK_INDEX 15 // Feedback pin
#define STOP_FEEDBACK 82
#define MED_FEEDBACK 85
#define MAX_FEEDBACK 90

void initPCA9685();
void initHand();
void movServoByAngle(int servo, int angle);
void activeFeedBack(int value, int threshold);
void openHand();
void closeHand();
void toogleHandState(bool state);
void openFinger(int finger);
void closeFinger(int finger);
void openFingers(int *fingers);
void closeFingers(int *fingers);
void checkHandState(bool enableServo);

void toggleGestureHand(int gesture);

#endif // MOTORS_H