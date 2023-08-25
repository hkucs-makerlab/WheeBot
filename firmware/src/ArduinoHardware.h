#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#include <Arduino.h>

#define __ROS_USE_SPP_HANDLE__
//
#ifdef __ROS_USE_SPP_HANDLE__
#include <BluetoothSerial.h>
static BluetoothSerial bluetoothSPP;
#define SERIAL_CLASS BluetoothSerial
#else 
#include <HardwareSerial.h>
#define SERIAL_CLASS HardwareSerial
#endif
//
class ArduinoHardware {
 public:
  ArduinoHardware() {
#ifdef __ROS_USE_SPP_HANDLE__
    iostream = &bluetoothSPP;
#else
    iostream = &Serial2;
#endif
    baud_ = 38400;
  }

  void setBaud(long baud) { this->baud_ = baud; }

  int getBaud() { return baud_; }

  void init() {
#ifdef __ROS_USE_SPP_HANDLE__
    uint64_t chipid = ESP.getEfuseMac();
    String id = String((uint16_t)(chipid >> 32), HEX);
    id.toUpperCase();
    iostream->begin("ROS-" + id);
    Serial.println(String("Node handle on BT-SPP, address ") +
                   iostream->getBtAddressString());
#else
    iostream->begin(baud_);
    Serial.println("Node handle on Serial2");
#endif
    // Serial.println("init called");
  }

  int read() { return iostream->read(); };

  void write(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      iostream->write(data[i]);
    }
  }

  unsigned long time() { return millis(); }

 protected:
  SERIAL_CLASS* iostream;
  long baud_;
};
#endif  // ROS_ARDUINO_HARDWARE_H_
