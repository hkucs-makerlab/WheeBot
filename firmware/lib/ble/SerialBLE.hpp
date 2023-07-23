#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "QueueArray.hpp"

// #define __DEBUG__

// server
class ServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer *pServer) {
#ifdef __DEBUG__
    Serial.println("[connected]");
#endif
  };
  void onDisconnect(BLEServer *pServer) {
#ifdef __DEBUG__
    Serial.println("[disconnected]");
    Serial.println("start advertising again!");
#endif
    pServer->startAdvertising();
  }
};

// characteristic
#define BUFF_LEN 64
class DataCallbacks : public BLECharacteristicCallbacks {
 private:
  uint8_t outBuffer[BUFF_LEN];
  volatile size_t dataLen;
  volatile bool locked;
  QueueArray<byte> dataQueue;

 public:
  DataCallbacks() : dataLen(0),
                    locked(false),
                    dataQueue(QueueArray<byte>()) {}

  void onWrite(BLECharacteristic *pCharacteristic);
  void onRead(BLECharacteristic *pCharacteristic);
  void onNotify(BLECharacteristic *pCharacteristic) {
    // Serial.print("onNotify call back ");
  }
  int read();
  int write(uint8_t *outBuffer, size_t len);
  bool avaliable();
};

class SerialBLE {
 private:
  BLEServer *pServer;
  BLECharacteristic *pRxTxCharacteristic;
  ServerCallbacks serverCallbacks;
  DataCallbacks dataCallbacks;
  bool inited;

 public:
  SerialBLE()
      : pServer(NULL),
        serverCallbacks(ServerCallbacks()),
        dataCallbacks(DataCallbacks()),inited(false) {}

  void begin(String id = "");
  bool isConnected() { return pServer->getConnectedCount(); }
  int read();
  size_t write(uint8_t *buff, size_t len);
  bool available();
  //
  virtual ~SerialBLE() {
    delete pServer;
  }
};
