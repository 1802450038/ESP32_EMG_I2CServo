#include <Arduino.h>
#include "hall.h"


int HALL_SENSOR_PIN_THUMB_THRESHOLD = 2360;
int HALL_SENSOR_PIN_INDEX_THRESHOLD = 2350;
int HALL_SENSOR_PIN_MIDDLE_THRESHOLD = 2270;
int HALL_SENSOR_PIN_RING_THRESHOLD = 2620;
int HALL_SENSOR_PIN_LITTLE_THRESHOLD = 2630;


int readings[NUM_READINGS];
int readIndex = 0;
long total = 0;
int aisVal = 0;

void initHallSensor()
{
  pinMode(HALL_SENSOR_PIN_THUMB, INPUT);
  pinMode(HALL_SENSOR_PIN_INDEX, INPUT);
  pinMode(HALL_SENSOR_PIN_MIDDLE, INPUT);
  pinMode(HALL_SENSOR_PIN_RING, INPUT);
  pinMode(HALL_SENSOR_PIN_LITTLE, INPUT);
}

int readAnalogHall(int pin)
{
  int hallSensorValue = analogRead(pin);
  return hallSensorValue;
}

bool checkPress(int pin, int threshold)
{
  int hallSensorValue = analogRead(pin);
  if (hallSensorValue < threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

long smooth(int vPin, int threshold)
{ /* function smooth */
  long average;
  total = total - readings[readIndex];

  int reading = analogRead(vPin);
  readings[readIndex] = threshold - reading;

  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= NUM_READINGS)
  {
    readIndex = 0;
  }
  average = total / NUM_READINGS;

  if (average < 0)
  {
    return 0;
  }
  else
  {
    return average;
  }
  return 0; // Default return value to ensure all paths return a value
}

void displayHallSensorGraph()
{
  if (checkPress(HALL_SENSOR_PIN_THUMB, HALL_SENSOR_PIN_THUMB_THRESHOLD))
  {
    printDisplayLine(0, "Thum -> " + String(smooth(HALL_SENSOR_PIN_THUMB, HALL_SENSOR_PIN_THUMB_THRESHOLD)));
    drawLineGraph(smooth(HALL_SENSOR_PIN_THUMB, HALL_SENSOR_PIN_THUMB_THRESHOLD));
  }
  else if (checkPress(HALL_SENSOR_PIN_INDEX, HALL_SENSOR_PIN_INDEX_THRESHOLD))
  {
    printDisplayLine(0, "Inde -> " + String(smooth(HALL_SENSOR_PIN_INDEX, HALL_SENSOR_PIN_INDEX_THRESHOLD)));
    drawLineGraph(smooth(HALL_SENSOR_PIN_INDEX, HALL_SENSOR_PIN_INDEX_THRESHOLD));
  }
  else if (checkPress(HALL_SENSOR_PIN_MIDDLE, HALL_SENSOR_PIN_MIDDLE_THRESHOLD))
  {
    printDisplayLine(0, "Midd -> " + String(smooth(HALL_SENSOR_PIN_MIDDLE, HALL_SENSOR_PIN_MIDDLE_THRESHOLD)));
    drawLineGraph(smooth(HALL_SENSOR_PIN_MIDDLE, HALL_SENSOR_PIN_MIDDLE_THRESHOLD));
  }
  else if (checkPress(HALL_SENSOR_PIN_RING, HALL_SENSOR_PIN_RING_THRESHOLD))
  {
    printDisplayLine(0, "Ring -> " + String(smooth(HALL_SENSOR_PIN_RING, HALL_SENSOR_PIN_RING_THRESHOLD)));
    drawLineGraph(smooth(HALL_SENSOR_PIN_RING, HALL_SENSOR_PIN_RING_THRESHOLD));
  }
  else if (checkPress(HALL_SENSOR_PIN_LITTLE, HALL_SENSOR_PIN_LITTLE_THRESHOLD))
  {
    printDisplayLine(0, "Litt -> " + String(smooth(HALL_SENSOR_PIN_LITTLE, HALL_SENSOR_PIN_LITTLE_THRESHOLD)));
    drawLineGraph(smooth(HALL_SENSOR_PIN_LITTLE, HALL_SENSOR_PIN_LITTLE_THRESHOLD));
  }
  else
  {
    printDisplayLine(0, "Press a Finger");
  }
}

void debugAnalog(int number)
{
  long smoothValue = 0; // Declare smoothValue outside the switch
  switch (number)
  {
  case 0:
    smoothValue = smooth(HALL_SENSOR_PIN_THUMB, HALL_SENSOR_PIN_THUMB_THRESHOLD);
    printDisplayLine(1, "Thum -> " + String(smoothValue));
    drawLineGraph(smoothValue);
    activeFeedBack(smoothValue, 15);
    break;
  case 1:
    smoothValue = smooth(HALL_SENSOR_PIN_INDEX, HALL_SENSOR_PIN_INDEX_THRESHOLD);
    printDisplayLine(1, "Inde -> " + String(smoothValue));
    drawLineGraph(smoothValue);
    activeFeedBack(smoothValue, 15);
    break;
  case 2:
    smoothValue = smooth(HALL_SENSOR_PIN_MIDDLE, HALL_SENSOR_PIN_MIDDLE_THRESHOLD);
    printDisplayLine(1, "Midd -> " + String(smoothValue));
    drawLineGraph(smoothValue);
    activeFeedBack(smoothValue, 15);
    break;
  case 3:
    smoothValue = smooth(HALL_SENSOR_PIN_RING, HALL_SENSOR_PIN_RING_THRESHOLD);
    printDisplayLine(1, "Ring -> " + String(smoothValue));
    drawLineGraph(smoothValue);
    activeFeedBack(smoothValue, 15);
    break;
  case 4:
    smoothValue = smooth(HALL_SENSOR_PIN_LITTLE, HALL_SENSOR_PIN_LITTLE_THRESHOLD);
    printDisplayLine(1, "Litt -> " + String(smoothValue));
    drawLineGraph(smoothValue);
    activeFeedBack(smoothValue, 15);
    break;
  default:
    printDisplayLine(1, "Press a Finger");
    break;
  }
}
