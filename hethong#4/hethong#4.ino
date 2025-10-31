#include <SPI.h>
#include <mcp2515.h>
#include <Servo.h>
#include "STM32FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ------------------- Define Pin -------------------
#define SENSOR_PIN PB12
#define SERVO_COP_PIN PB6
#define SERVO_GATMUA_PIN PB13
#define POT_GATMUA_PIN PB0
#define SERVO_PIN PB8

const int sw_left = PA15, sw_right = PA12, sw_hazard = PA11;
const int sw_den1 = PA10, sw_den2 = PA9, sw_pha_cos = PA8, sw_flash = PB15;
const int led_left = PA2, led_right = PA3, led_tail = PC15, led_pha = PC14, led_cos = PC13;
const int motorPWM = PB5, motorIN1 = PA0, motorIN2 = PA1;
const int motorPWMB = PB1, motorIN3 = PB10, motorIN4 = PB11;

// ------------------- Servo -------------------
Servo myServo_cop, Servo_gatmua, myServo;
int angle = 0, direction = 1;
bool servoOpened = false;
unsigned long openTime = 0;
int servoDelayTime = 50;


// ------------------- CAN -------------------
MCP2515 mcp2515(PA4);
struct can_frame canMsg;

// ------------------- RTOS -------------------
QueueHandle_t canQueue;
SemaphoreHandle_t servoMutex;

// ------------------- Task declarations -------------------
void Task_CANMotorHandler(void *pvParameters);
void Task_LEDHandler(void *pvParameters);
void Task_LightControl(void *pvParameters);
void Task_ServoCop(void *pvParameters);
void Task_MotorControl(void *pvParameters);

void setup() {
  Serial.begin(115200);

  pinMode(sw_left, INPUT_PULLUP);
  pinMode(sw_right, INPUT_PULLUP);
  pinMode(sw_hazard, INPUT_PULLUP);
  pinMode(sw_den1, INPUT_PULLUP);
  pinMode(sw_den2, INPUT_PULLUP);
  pinMode(sw_pha_cos, INPUT_PULLUP);
  pinMode(sw_flash, INPUT_PULLUP);

  pinMode(led_left, OUTPUT);
  pinMode(led_right, OUTPUT);
  pinMode(led_tail, OUTPUT);
  pinMode(led_cos, OUTPUT);
  pinMode(led_pha, OUTPUT);

  pinMode(SENSOR_PIN, INPUT);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  pinMode(motorPWMB, OUTPUT);
  pinMode(motorIN3, OUTPUT);
  pinMode(motorIN4, OUTPUT);

  myServo_cop.attach(SERVO_COP_PIN);
  Servo_gatmua.attach(SERVO_GATMUA_PIN);
  myServo.attach(SERVO_PIN);
  myServo_cop.write(145);

  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  canQueue = xQueueCreate(5, sizeof(can_frame));
  servoMutex = xSemaphoreCreateMutex();

  // xTaskCreate(Task_CANMotorHandler, "CANMotor", 128, NULL, 2, NULL);
  xTaskCreate(Task_MotorControl, "Motor", 128, NULL, 1, NULL);
  xTaskCreate(Task_LEDHandler, "LEDHandler", 128, NULL, 1, NULL);
  xTaskCreate(Task_LightControl, "Light", 128, NULL, 1, NULL);
  xTaskCreate(Task_ServoCop, "ServoCop", 128, NULL, 1, NULL);


  vTaskStartScheduler();
}

void loop() {}

void Task_MotorControl(void *pvParameters) {
  while (1) {
    digitalWrite(motorIN3, HIGH);
    digitalWrite(motorIN4, LOW);
    analogWrite(motorPWMB, 200);
  }
}

void Task_CANMotorHandler(void *pvParameters) {
  struct can_frame msg;
  while (1) {
    // Đọc dữ liệu CAN nếu có
    if (mcp2515.readMessage(&msg) == MCP2515::ERROR_OK) {
      if (msg.can_id == 0x100 && msg.can_dlc == 3) {
        uint8_t weightVal = msg.data[0];
        uint8_t potVal = msg.data[1];
        uint8_t switchState = msg.data[2];

        // Điều khiển hướng động cơ
        if (switchState == 1) {
          digitalWrite(motorIN1, HIGH);
          digitalWrite(motorIN2, LOW);
          digitalWrite(motorIN3, HIGH);
          digitalWrite(motorIN4, LOW);
        } else {
          digitalWrite(motorIN1, LOW);
          digitalWrite(motorIN2, HIGH);
          digitalWrite(motorIN3, LOW);
          digitalWrite(motorIN4, HIGH);
        }

        // Điều khiển tốc độ động cơ
        analogWrite(motorPWM, weightVal);
        analogWrite(motorPWMB, weightVal);

        // Điều khiển góc servo
        myServo.write(map(potVal, 0, 255, 0, 180));
      }
    }
    vTaskDelay(10);
  }
}


void Task_LEDHandler(void *pvParameters) {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  while (1) {
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      blinkState = !blinkState;

      bool left = digitalRead(sw_left) == LOW;
      bool right = digitalRead(sw_right) == LOW;
      bool hazard = digitalRead(sw_hazard) == LOW;

      digitalWrite(led_left, (hazard || left) ? blinkState : LOW);
      digitalWrite(led_right, (hazard || right) ? blinkState : LOW);
    }
    vTaskDelay(50);
  }
}

void Task_LightControl(void *pvParameters) {
  while (1) {
    bool den1 = digitalRead(sw_den1) == LOW;
    bool den2 = digitalRead(sw_den2) == LOW;
    bool pha_cos = digitalRead(sw_pha_cos) == LOW;
    bool flash = digitalRead(sw_flash) == LOW;

    if (flash) {
      digitalWrite(led_pha, HIGH);
      digitalWrite(led_cos, LOW);
    } else if (pha_cos) {
      digitalWrite(led_tail, HIGH);
      digitalWrite(led_cos, LOW);
      digitalWrite(led_pha, HIGH);
    } else if (den2) {
      digitalWrite(led_tail, HIGH);
      digitalWrite(led_cos, HIGH);
      digitalWrite(led_pha, LOW);
    } else if (den1) {
      digitalWrite(led_tail, HIGH);
      digitalWrite(led_cos, LOW);
      digitalWrite(led_pha, LOW);
    } else {
      digitalWrite(led_tail, LOW);
      digitalWrite(led_cos, LOW);
      digitalWrite(led_pha, LOW);
    }
    vTaskDelay(50);
  }
}


void Task_ServoCop(void *pvParameters) {
  while (1) {
    int sensorState = digitalRead(SENSOR_PIN);
    if (sensorState == LOW && !servoOpened) {
      if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
        myServo_cop.write(0);
        servoOpened = true;
        openTime = millis();
        xSemaphoreGive(servoMutex);
      }
    }

    if (servoOpened && millis() - openTime > 5000) {
      if (xSemaphoreTake(servoMutex, portMAX_DELAY) == pdTRUE) {
        myServo_cop.write(145);
        servoOpened = false;
        xSemaphoreGive(servoMutex);
      }
    }
    vTaskDelay(50);
  }
}