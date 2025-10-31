#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <mcp2515.h>
#include "ScreenBase.h"
#include "DashboardScreen.h"
#include "EngineInfoScreen.h"
#include "ObstacleScreen.h"
#include "AirConditionScreen.h"
#include "PlaceholderScreen.h"
#include "GoogleMapScreen.h"

// ==== OLED & CAN ====
#define CAN_CS_PIN 5
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_MIRROR, U8X8_PIN_NONE);
MCP2515 mcp2515(CAN_CS_PIN);

// ==== Button ====
#define BUTTON_UP_PIN     14
#define BUTTON_SELECT_PIN 12
#define BUTTON_DOWN_PIN   13

QueueHandle_t buttonQueue;
SemaphoreHandle_t oledMutex;

// ==== Debounce ====
volatile uint32_t lastDebounceTimeUp = 0;
volatile uint32_t lastDebounceTimeDown = 0;
volatile uint32_t lastDebounceTimeSelect = 0;
const uint32_t debounceDelay = 200;

// ==== Motor ====
#define ENA_PIN 26
#define IN1_PIN 25
#define IN2_PIN 33
#define POT_PIN 27
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RES 8

// ==== Screens ====
ScreenBase* screens[6];
int currentScreenIndex = 0;
bool inMenu = true;
bool hudMirrorEnabled = false;

void applyHUDMirrorSetting() {
  u8g2.setFlipMode(hudMirrorEnabled ? 1 : 0);
}


// ==== ISR Buttons ====
void IRAM_ATTR isrButtonUp() {
  uint32_t now = millis();
  if (now - lastDebounceTimeUp > debounceDelay) {
    uint8_t val = 1;
    xQueueSendFromISR(buttonQueue, &val, NULL);
    lastDebounceTimeUp = now;
  }
}

void IRAM_ATTR isrButtonDown() {
  uint32_t now = millis();
  if (now - lastDebounceTimeDown > debounceDelay) {
    uint8_t val = 2;
    xQueueSendFromISR(buttonQueue, &val, NULL);
    lastDebounceTimeDown = now;
  }
}

void IRAM_ATTR isrButtonSelect() {
  uint32_t now = millis();
  if (now - lastDebounceTimeSelect > debounceDelay) {
    uint8_t val = 3;
    xQueueSendFromISR(buttonQueue, &val, NULL);
    lastDebounceTimeSelect = now;
  }
}

// ==== TASKS ====

void TaskMotor(void* pvParameters) {
  for (;;) {
    int potValue = analogRead(POT_PIN);
    int pwmValue = map(potValue, 0, 4095, 0, 255);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(PWM_CHANNEL, pwmValue);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void TaskButton(void* pvParameters) {
  uint8_t btn;
  for (;;) {
    if (xQueueReceive(buttonQueue, &btn, portMAX_DELAY)) {
      if (btn == 1)
        currentScreenIndex = (currentScreenIndex - 1 + 6) % 6;
      else if (btn == 2)
        currentScreenIndex = (currentScreenIndex + 1) % 6;
      else if (btn == 3)
        inMenu = !inMenu;
    }
  }
}

void TaskCAN(void* pvParameters) {
  struct can_frame frame;
  for (;;) {
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      for (int i = 0; i < 6; i++) {
        if (screens[i]) screens[i]->update(frame);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskOLED(void* pvParameters) {
  for (;;) {
    applyHUDMirrorSetting();  
    if (xSemaphoreTake(oledMutex, portMAX_DELAY)) {
      u8g2.clearBuffer();

      if (inMenu) {
        u8g2.setFont(u8g2_font_6x13_tf);
        u8g2.drawStr(10, 20, screens[(currentScreenIndex - 1 + 6) % 6]->name());

        u8g2.drawBox(0, 25, 128, 18);
        u8g2.setDrawColor(0);
        u8g2.drawStr(10, 40, screens[currentScreenIndex]->name());
        u8g2.setDrawColor(1);

        u8g2.drawStr(10, 60, screens[(currentScreenIndex + 1) % 6]->name());
      } else {
        if (screens[currentScreenIndex]) {
          screens[currentScreenIndex]->draw(u8g2);
        } else {
          u8g2.setFont(u8g2_font_ncenB08_tr);
          u8g2.drawStr(15, 25, "Coming soon...");
          u8g2.setFont(u8g2_font_6x13_tf);
          u8g2.drawStr(0, 45, "Screen: ");
          u8g2.drawStr(60, 45, "Unknown");
        }
      }

      xSemaphoreGive(oledMutex);
      u8g2.sendBuffer();
    }

    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  u8g2.begin();

  // Button setup
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_UP_PIN, isrButtonUp, FALLING);
  attachInterrupt(BUTTON_DOWN_PIN, isrButtonDown, FALLING);
  attachInterrupt(BUTTON_SELECT_PIN, isrButtonSelect, FALLING);

  // Motor setup
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL);

  // CAN setup
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  // Screen setup
  screens[0] = new DashboardScreen();
  screens[1] = new EngineInfoScreen();
  screens[2] = new ObstacleScreen();
  screens[3] = new AirConditionScreen(POT_PIN);
  screens[4] = new PlaceholderScreen("Tire Pressure");
  screens[5] = new GoogleMapScreen();

  // RTOS
  buttonQueue = xQueueCreate(10, sizeof(uint8_t));
  oledMutex = xSemaphoreCreateMutex();

  xTaskCreate(TaskMotor,  "Motor", 2048, NULL, 1, NULL);
  xTaskCreate(TaskButton, "Button", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCAN,    "CAN", 2048, NULL, 2, NULL);
  xTaskCreate(TaskOLED,   "OLED", 4096, NULL, 1, NULL);
}

void loop() {
  // Không cần gì ở đây
}
