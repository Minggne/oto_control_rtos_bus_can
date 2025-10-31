#define configTOTAL_HEAP_SIZE         ( ( size_t ) ( 6 * 1024 ) )
#define configUSE_MALLOC_FAILED_HOOK  1
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define INCLUDE_xPortGetFreeHeapSize  1

#include <STM32FreeRTOS.h>
#include <SPI.h>
#include "CANManager.h"
#include "CANMessage.h"
#include "HX711.h"
#include <SimpleDHT.h>

// ====== Định nghĩa chân ======
QueueHandle_t canQueue;
SemaphoreHandle_t canSem;

CANManager can(PA4);
#define LED_PIN PC13

SimpleDHT11 dht(PA3);
HX711 scale;
#define CALIBRATION_FACTOR 420.0983

#define TRIG1 PB12
#define ECHO1 PB13
#define TRIG2 PB14
#define ECHO2 PB15
#define IR_SENSOR PA8
#define POT_PIN PA2
#define SWITCH_PIN PC15

#define FSR_PIN PB0
#define FSR_MAX_FORCE_N 20.0
#define FSR_CONTACT_AREA_M2 0.0001

// ====== Đo khoảng cách ======
long readDistanceCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 10000);
  if (duration == 0) return 255;
  return constrain(duration * 0.034 / 2, 0, 255);
}

// ====== Task gửi CAN ======
void TaskCAN(void* param) {
  pinMode(LED_PIN, OUTPUT);
  CANMessage* msg;
  struct can_frame frame;

  while (1) {
    if (xSemaphoreTake(canSem, portMAX_DELAY) == pdTRUE) {
      while (xQueueReceive(canQueue, &msg, 0) == pdTRUE) {
        frame.can_id = msg->can_id;
        frame.can_dlc = msg->can_dlc;
        memcpy(frame.data, msg->data, msg->can_dlc);
        can.send(frame);
        vPortFree(msg);
        digitalWrite(LED_PIN, LOW); delay(10); digitalWrite(LED_PIN, HIGH);
      }
    }
  }
}

// ====== Task đo Loadcell, POT, SWITCH ======
void CtrlTaskRunner(void* param) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t loopDelay = pdMS_TO_TICKS(50); // vòng lặp nhanh hơn

  float accWeight = 0;
  int sampleCount = 0;
  const int avgSampleTarget = 10;

  while (1) {
    if (scale.is_ready()) {
      accWeight += scale.get_units(1);  // lấy từng mẫu 1
      sampleCount++;
    }

    if (sampleCount >= avgSampleTarget) {
      float weight = accWeight / sampleCount;
      accWeight = 0;
      sampleCount = 0;

      if (weight < 0) weight = 0;
      uint8_t weightVal = (uint8_t)constrain(weight, 0, 255);

      int potRaw = analogRead(POT_PIN);
      uint8_t potVal = map(potRaw, 0, 1023, 0, 255);
      uint8_t sw = digitalRead(SWITCH_PIN) == LOW ? 1 : 0;

      CANMessage* msg = (CANMessage*)pvPortMalloc(sizeof(CANMessage));
      if (msg) {
        msg->can_id = 0x100;
        msg->can_dlc = 3;
        msg->data[0] = weightVal;
        msg->data[1] = potVal;
        msg->data[2] = sw;

        if (xQueueSendToBack(canQueue, &msg, 0) == pdPASS) {
          xSemaphoreGive(canSem);
        } else {
          vPortFree(msg);
        }
      }
    }

    vTaskDelayUntil(&lastWake, loopDelay);
  }
}


// ====== Task đo nhiệt độ + áp suất FSR ======
void TempTaskRunner(void* param) {
  TickType_t lastWake = xTaskGetTickCount();
  byte temp = 0, hum = 0;

  while (1) {
    if (dht.read(&temp, &hum, NULL) == SimpleDHTErrSuccess) {
      int adcValue = analogRead(FSR_PIN);
      float force = (adcValue / 4095.0) * FSR_MAX_FORCE_N;
      float pressure_Pa = force / FSR_CONTACT_AREA_M2;
      float pressure_Bar = pressure_Pa / 100000.0;
      uint8_t pressureVal = (uint8_t)constrain(pressure_Bar * 100, 0, 255);

      CANMessage* msg = (CANMessage*)pvPortMalloc(sizeof(CANMessage));
      if (msg) {
        msg->can_id = 0x300;
        msg->can_dlc = 2;
        msg->data[0] = temp;
        msg->data[1] = pressureVal;
        if (xQueueSendToBack(canQueue, &msg, 0) == pdPASS) {
          xSemaphoreGive(canSem);
        } else {
          vPortFree(msg);
        }
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// ====== Task đo khoảng cách & IR ======
void DistTaskRunner(void* param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    uint8_t d1 = readDistanceCM(TRIG1, ECHO1);
    uint8_t d2 = readDistanceCM(TRIG2, ECHO2);
    uint8_t ir = digitalRead(IR_SENSOR) == LOW ? 1 : 0;

    CANMessage* msg = (CANMessage*)pvPortMalloc(sizeof(CANMessage));
    if (msg) {
      msg->can_id = 0x200;
      msg->can_dlc = 3;
      msg->data[0] = d1;
      msg->data[1] = d2;
      msg->data[2] = ir;
      xQueueSendToBack(canQueue, &msg, 0);
      xSemaphoreGive(canSem);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1500));
  }
}

// ====== setup ======
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  scale.begin(PA0, PA1);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(FSR_PIN, INPUT);

  canQueue = xQueueCreate(15, sizeof(CANMessage*));
  canSem = xSemaphoreCreateBinary();
  can.begin();

  xTaskCreate(TaskCAN, "CAN", 128, NULL, 2, NULL);
  xTaskCreate(CtrlTaskRunner, "Ctrl", 140, NULL, 2, NULL);
  xTaskCreate(TempTaskRunner, "Temp", 128, NULL, 3, NULL);
  xTaskCreate(DistTaskRunner, "Dist", 128, NULL, 4, NULL);

  delay(1000);
  vTaskStartScheduler();
  while (1);
}

void loop() {}

// ====== Hook lỗi malloc và stack overflow ======
extern "C" void vApplicationMallocFailedHook() {
  digitalWrite(LED_PIN, LOW);
  while (1);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  digitalWrite(LED_PIN, LOW);
  while (1);
}
