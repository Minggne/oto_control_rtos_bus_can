#ifndef GOOGLEMAPSCREEN_H
#define GOOGLEMAPSCREEN_H

#include "ScreenBase.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

class GoogleMapScreen : public ScreenBase {
private:
  BLECharacteristic* pWriteCharacteristic;
  bool deviceConnected = false;
  bool hasData = false;

  uint32_t initialDistance = 0;
  uint32_t currentDistance = 0;
  uint8_t latestSpeed = 0;
  uint8_t latestDirection = 0;
  std::string latestValue = "";

public:
  GoogleMapScreen() {
    BLEDevice::init("ESP32_BLE_HUD");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    BLEService *pService = pServer->createService("DD3F0AD1-6239-4E1F-81F1-91F6C9F01D86");
    pWriteCharacteristic = pService->createCharacteristic(
      "DD3F0AD3-6239-4E1F-81F1-91F6C9F01D86",
      BLECharacteristic::PROPERTY_WRITE
    );
    pWriteCharacteristic->setCallbacks(new CharacteristicCallbacks(this));
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("DD3F0AD1-6239-4E1F-81F1-91F6C9F01D86");
    pAdvertising->start();
  }

  void update(const struct can_frame&) override {}

  void draw(U8G2& u8g2) override {
  if (!deviceConnected) {
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 10, "BLE: Waiting");
    return;
  }

  if (!hasData) {
    u8g2.setFont(u8g2_font_6x13_tf);
    u8g2.drawStr(0, 10, "BLE: Connected");
    return;
  }

    if (!deviceConnected) return;

    drawSpeedCircle(u8g2, latestSpeed);
    drawDirectionIcon(u8g2, latestDirection);
    if (initialDistance > 0)
      drawDistanceBar(u8g2, initialDistance - currentDistance, initialDistance);
  }

  const char* name() override {
    return "Google Map";
  }

private:
  class ServerCallbacks : public BLEServerCallbacks {
    GoogleMapScreen* parent;
  public:
    ServerCallbacks(GoogleMapScreen* p) : parent(p) {}
    void onConnect(BLEServer*) override { parent->deviceConnected = true; }
    void onDisconnect(BLEServer*) override {
      parent->deviceConnected = false;
      BLEDevice::startAdvertising();
    }
  };

  class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    GoogleMapScreen* parent;
  public:
    CharacteristicCallbacks(GoogleMapScreen* p) : parent(p) {}
    void onWrite(BLECharacteristic *pChar) override {
      std::string value = pChar->getValue();
      parent->latestValue = value;
      if (value.length() >= 3 && value[0] == 0x01) {
        parent->hasData = true;
        parent->latestSpeed = value[1];
        parent->latestDirection = value[2];
        if (value.length() >= 7) {
          int distance = atoi(value.substr(3, 3).c_str());
          if (distance > 0) {
            if (parent->initialDistance == 0 || distance > parent->initialDistance)
              parent->initialDistance = distance;
            parent->currentDistance = distance;
          }
        }
      }
    }
  };

  void drawSpeedCircle(U8G2& u8g2, uint8_t speed) {
    int cx = 30, cy = 30, r = 16;
    char buf[4];
    sprintf(buf, "%d", speed);
    u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
    u8g2.setFont(u8g2_font_ncenB14_tr);
    int tw = u8g2.getStrWidth(buf);
    u8g2.drawStr(cx - tw / 2, cy + 6, buf);
  }

  void drawDirectionIcon(U8G2& u8g2, uint8_t dir) {
    int x = 60;
    if (dir == 0x08) {
      u8g2.drawBox(x + 13, 5, 11, 25);
      u8g2.drawBox(x, 5, 13, 10);
      u8g2.drawTriangle(x - 13, 10, x, 0, x, 20);
    } else if (dir == 0x0A) {
      u8g2.drawBox(x, 5, 11, 25);
      u8g2.drawBox(x + 11, 5, 13, 10);
      u8g2.drawTriangle(x + 39, 10, x + 26, 0, x + 26, 20);
    } else if (dir == 0x04) {
      u8g2.drawBox(x, 10, 11, 19);
      u8g2.drawTriangle(x + 5, 2, x - 11, 14, x + 21, 14);
    }
  }

  void drawDistanceBar(U8G2& u8g2, uint32_t travelled, uint32_t max) {
    const int x = 10, y = 52, w = 108, h = 12;
    int filled = map(travelled, 0, max, 0, w - 2);
    filled = constrain(filled, 0, w - 2);
    u8g2.drawFrame(x, y, w, h);
    u8g2.drawBox(x + 1, y + 1, filled, h - 2);

    char distStr[10];
    sprintf(distStr, "%dm", max - travelled);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(x + w - 50, y - 5, distStr);
  }
};

#endif
