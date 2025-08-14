
#include <Arduino.h>
#include <Wire.h>
#include "display.h"
#include "buttons.h"
#include "motors.h"
#include "hall.h"
#include <esp_now.h>
#include <WiFi.h>

#define OLED_ENABLE FALSE

int inc_thumb_pos = 0;
int inc_index_pos = 0;
int inc_middle_pos = 0;
int inc_ring_pos = 0;
int inc_little_pos = 0;
int inc_gesture = 0;
int inc_action_mode = 0;
bool inc_hand_state = false;
bool inc_feedback = false;

typedef struct struct_inc_message
{
  int thumb_pos;
  int index_pos;
  int middle_pos;
  int ring_pos;
  int little_pos;
  int gesture;
  int action_mode;
  bool hand_state;
  bool feedback;
} struct_inc_message;

typedef struct struct_out_message
{
  int thumb_hall;
  int index_hall;
  int middle_hall;
  int ring_hall;
  int little_hall;
} struct_out_message;

String success;
struct_out_message OUT_DATA;
struct_inc_message INC_DATA;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
  }
}
// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&INC_DATA, incomingData, sizeof(INC_DATA));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  inc_thumb_pos = INC_DATA.thumb_pos;
  inc_index_pos = INC_DATA.index_pos;
  inc_middle_pos = INC_DATA.middle_pos;
  inc_ring_pos = INC_DATA.ring_pos;
  inc_little_pos = INC_DATA.little_pos;
  inc_gesture = INC_DATA.gesture;
  inc_action_mode = INC_DATA.action_mode;
  // inc_hand_state = INC_DATA.hand_state;
  inc_hand_state = true;
  inc_feedback = INC_DATA.feedback;
}

void prepareDataToSend()
{
  OUT_DATA.thumb_hall = analogRead(HALL_SENSOR_PIN_THUMB);
  OUT_DATA.index_hall = analogRead(HALL_SENSOR_PIN_INDEX);
  OUT_DATA.middle_hall = analogRead(HALL_SENSOR_PIN_MIDDLE);
  OUT_DATA.ring_hall = analogRead(HALL_SENSOR_PIN_RING);
  OUT_DATA.little_hall = analogRead(HALL_SENSOR_PIN_LITTLE);
}

void setupNow()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  delay(1000);
  Serial.println("Initializing");
  analogReadResolution(12);
  initDisplay(); // Initialize OLED display
  delay(500);
  button.attachClick(click);                   // Attach click event
  button.attachDoubleClick(doubleClick);       // Attach double click event
  button.attachLongPressStart(longPressStart); // Attach double click event
  button.attachLongPressStop(longPressStop);   // Attach double click event
  Serial.println("Init PCA");
  initPCA9685(); // Initialize PCA9685
  delay(500);
  Serial.println("Init Hand");
  initHand();
  delay(500);
  Serial.println("Init Hall");
  initHallSensor();
  delay(1000);
  Serial.println("Setup HALL");
  setupNow(); // Initialize ESP-NOW
}

void updateHallValues()
{
  OUT_DATA.thumb_hall = analogRead(HALL_SENSOR_PIN_THUMB);
  OUT_DATA.index_hall = analogRead(HALL_SENSOR_PIN_INDEX);
  OUT_DATA.middle_hall = analogRead(HALL_SENSOR_PIN_MIDDLE);
  OUT_DATA.ring_hall = analogRead(HALL_SENSOR_PIN_RING);
  OUT_DATA.little_hall = analogRead(HALL_SENSOR_PIN_LITTLE);
}

void action_mode()
{
  switch (inc_action_mode)
  {
  case 0:
    Serial.println("Modo EMG");
    toogleHandState(inc_hand_state);
    break;
  case 1:
    Serial.println("Modo angulo");
    movServoByAngle(0, inc_thumb_pos);
    movServoByAngle(1, inc_index_pos);
    movServoByAngle(2, inc_middle_pos);
    movServoByAngle(3, inc_ring_pos);
    movServoByAngle(4, inc_little_pos);
    break;
  case 2:
    toggleGestureHand(inc_gesture);
    break;
  case 3:
    Serial.print("Modo Toggle");
    if (!inc_hand_state)
    {
      openHand();
    }
    else
    {
      closeHand();
    }

  case 4:
    Serial.print("Modo debug");
  }
}

String getActionModeName(int action)
{
  if (action == 0)
  {
    return String("EMG - MODE");
  }
  else if (action == 1)
  {
    return String("ANGLE - MODE");
  }
  else if (action == 2)
  {
    return String("GESTURE - MODE");
  }
  else if (action == 3)
  {
    return String("TOGGLE - MODE");
  }
  else if (action == 4)
  {
    return String("DEBUG - MODE");
  }
  return "ERROR NOT DEFINED: " + String(action);
}

// // Display Functions
void updateValues(int freqMillis)
{
  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= freqMillis)
  {
    action_mode();
    prepareDataToSend();
    Serial.println("Thumb pos: " + String(inc_thumb_pos));
    Serial.println("Index pos: " + String(inc_index_pos));
    Serial.println("Middle pos: " + String(inc_middle_pos));
    Serial.println("Ring pos: " + String(inc_ring_pos));
    Serial.println("Little pos: " + String(inc_little_pos));
    Serial.println("Gesture : " + String(inc_gesture));
    Serial.println("Action mode: " + String(inc_action_mode) + " " + getActionModeName(inc_action_mode));
    Serial.println("Hand state: " + String(inc_hand_state));
    Serial.println("Feedback: " + String(inc_feedback));
    lastUpdate = currentMillis;
  }
}

void loop()
{
  updateValues(300);
}
