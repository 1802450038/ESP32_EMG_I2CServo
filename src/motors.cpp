#include <Adafruit_PWMServoDriver.h>
#include "motors.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool handState = false; // Hand state

void initHand()
{
  movServoByAngle(0, 30);
  delay(500);
  movServoByAngle(1, 30);
  delay(500);
  movServoByAngle(2, 30);
  delay(500);
  movServoByAngle(3, 30);
  delay(500);
  movServoByAngle(4, 30);
  delay(500);
  movServoByAngle(0, 170);
  delay(500);
  movServoByAngle(1, 170);
  delay(500);
  movServoByAngle(2, 170);
  delay(500);
  movServoByAngle(3, 170);
  delay(500);
  movServoByAngle(4, 170);
  delay(500);
}

void initPCA9685()
{
  pwm.begin();                // Initialize PCA9685
  pwm.setPWMFreq(SERVO_FREQ); // Set frequency to 60 Hz
  delay(1000);
}

void toggleHandStateOpen()
{
  handState = true;
}

void toggleHandStateClose()
{
  handState = false;
}

void movServoByAngle(int servo, int angle)
{
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  Serial.println("Pulse length: " + String(pulseLength) + " Angle: " + String(angle) + " Servo: " + String(servo));
  pwm.setPWM(servo, 0, pulseLength);
}

void activeFeedBack(int value, int threshold)
{
  if (value > threshold)
  {
    movServoByAngle(FEED_1_BACK_INDEX, MED_FEEDBACK);
    movServoByAngle(FEED_2_BACK_INDEX, MED_FEEDBACK);
    movServoByAngle(FEED_3_BACK_INDEX, MED_FEEDBACK);
  }
  else
  {

    movServoByAngle(FEED_1_BACK_INDEX, STOP_FEEDBACK);
    movServoByAngle(FEED_2_BACK_INDEX, STOP_FEEDBACK);
    movServoByAngle(FEED_3_BACK_INDEX, STOP_FEEDBACK);
  }
}

void openHand()
{

  Serial.println("Opening hand");
  movServoByAngle(0, 115);
  movServoByAngle(1, 115);
  movServoByAngle(2, 115);
  movServoByAngle(3, 115);
  movServoByAngle(4, 115);
}

void closeHand()
{
  Serial.println("Closing hand");
  movServoByAngle(0, 30);
  movServoByAngle(1, 30);
  movServoByAngle(2, 30);
  movServoByAngle(3, 30);
  movServoByAngle(4, 30);
}

void toogleHandState(bool state)
{
  if (state)
  {
    openHand();
  }
  else
  {
    closeHand();
  }
}
void openFinger(int finger)
{
  // pwm.setPWM(finger, 0, SERVOMIN);
  movServoByAngle(finger, 180);
}

void closeFinger(int finger)
{
  // pwm.setPWM(finger, 0, SERVOMAX);
  movServoByAngle(finger, 0);
}

void openFingers(int *fingers)
{
  for (int i = 0; i < sizeof(fingers) / sizeof(fingers[0]); i++)
  {
    pwm.setPWM(fingers[i], 0, SERVOMIN);
  }
}

void closeFingers(int *fingers)
{
  for (int i = 0; i < sizeof(fingers) / sizeof(fingers[0]); i++)
  {
    pwm.setPWM(fingers[i], 0, SERVOMAX);
  }
}

void toggleGestureHand(int gesture)
{
  if (gesture == 0)
  {
    Serial.println("Gesture - OPEN");
    movServoByAngle(0, 180);
    openFinger(0);
    openFinger(1);
    openFinger(2);
    openFinger(3);
    openFinger(4);
  }
  else if (gesture == 1)
  {
    Serial.println("Gesture - ONE");
    movServoByAngle(0, 0);
    closeFinger(0);
    openFinger(1);
    closeFinger(2);
    closeFinger(3);
    closeFinger(4);
  }
  else if (gesture == 2)
  {
    Serial.println("Gesture - TWO");
    closeFinger(0);
    openFinger(1);
    openFinger(2);
    closeFinger(3);
    closeFinger(4);
  }
  else if (gesture == 3)
  {
    Serial.println("Gesture - THREE");
    closeFinger(0);
    openFinger(1);
    openFinger(2);
    openFinger(3);
    closeFinger(4);
  }
  else if (gesture == 4)
  {
    Serial.println("Gesture - OK");
    closeFinger(0);
    closeFinger(1);
    openFinger(2);
    openFinger(3);
    openFinger(4);
  }
  else if (gesture == 5)
  {
    Serial.println("Gesture - THUMB");
    openFinger(0);
    closeFinger(1);
    closeFinger(2);
    closeFinger(3);
    closeFinger(4);
  }
}
