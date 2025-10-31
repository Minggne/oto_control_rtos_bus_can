#include <SPI.h>
#include <mcp2515.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==== MCP2515 CAN ====
#define CAN_CS_PIN 5
#define SPI_MOSI 7
#define SPI_MISO 6
#define SPI_SCK  4
SPIClass CAN_SPI(FSPI);
MCP2515 mcp2515(CAN_CS_PIN, 10000000, &CAN_SPI);

// ==== BLE UUID ====
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ==== Dữ liệu dùng chung ====
volatile uint8_t weight = 0, pot = 0, sw = 0;
volatile uint8_t d1 = 0, d2 = 0, ir = 0;
volatile float temp = 0.0, hum = 0.0;
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// ==== BLE ====
BLECharacteristic* pCharacteristic = nullptr;
BLEServer* pServer = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
  }
};

// ==== Task nhận dữ liệu từ MCP2515 ====
void TaskCANReceive(void* param) {
  struct can_frame frame;

  for (;;) {
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      taskENTER_CRITICAL(&dataMux);
      switch (frame.can_id) {
        case 0x100: // Weight, Pot, Switch
          if (frame.can_dlc >= 3) {
            weight = frame.data[0];
            pot = frame.data[1];
            sw = frame.data[2];
          }
          break;
        case 0x200: // Distance + IR
          if (frame.can_dlc >= 3) {
            d1 = frame.data[0];
            d2 = frame.data[1];
            ir = frame.data[2];
          }
          break;
        case 0x300: // Temp + Pressure
          if (frame.can_dlc >= 2) {
            temp = frame.data[0];            // temp đơn vị °C
            hum = frame.data[1] / 1.0;       // áp suất, có thể là phần trăm hoặc bar x100
          }
          break;
      }
      taskEXIT_CRITICAL(&dataMux);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==== Task gửi BLE ====
void BLESendTask(void* param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    if (deviceConnected) {
      uint8_t w, p, s, _d1, _d2, _ir;
      float t, h;

      taskENTER_CRITICAL(&dataMux);
      w = weight; p = pot; s = sw;
      _d1 = d1; _d2 = d2; _ir = ir;
      t = temp; h = hum;
      taskEXIT_CRITICAL(&dataMux);

      String str = String(w) + "," + String(p) + "," + String(s) + "," +
                   String(_d1) + "," + String(_d2) + "," + String(_ir) + "," +
                   String(t, 1) + "," + String(h, 1);

      pCharacteristic->setValue(str.c_str());
      pCharacteristic->notify();
      Serial.println("[BLE] Sent: " + str);
    }

    if (!deviceConnected && oldDeviceConnected) {
      vTaskDelay(pdMS_TO_TICKS(500));
      pServer->startAdvertising();
      Serial.println("[BLE] Restarted advertising");
      oldDeviceConnected = false;
    }

    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = true;
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
  }
}

// ==== setup BLE và CAN ====
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // SPI MCP2515
  CAN_SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CAN_CS_PIN);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  // BLE setup
  BLEDevice::init("ESP32 BLE FreeRTOS");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println("✅ BLE & CAN Receiver Started");

  // Tạo task
  xTaskCreatePinnedToCore(TaskCANReceive, "CANRecv", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(BLESendTask, "BLESend", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Không cần dùng loop khi dùng FreeRTOS
}
