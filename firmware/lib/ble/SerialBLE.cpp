#include "SerialBLE.hpp"

#define SERVICE_UUID "DFB0"
#define CHARACTERISTIC_UUID "DFB1"

// DataCallbacks class
void DataCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  if (rxValue.length() > 0) {
    locked = true;
    // Serial.println("Received data len: " + String(rxValue.length()));
    for (int i = 0; i < rxValue.length(); i++) {
      dataQueue.push(rxValue[i]);
    }
    locked = false;
  }
}

void DataCallbacks::onRead(BLECharacteristic *pCharacteristic) {
  // Serial.println("onRead call back " +String(dataLen));
  if (dataLen > 0) {
    pCharacteristic->setValue(outBuffer, dataLen);
    dataLen = 0;
  }
}

int DataCallbacks::write(uint8_t *outBuffer, size_t len) {
  if (len > BUFF_LEN) len = BUFF_LEN;
  memcpy(this->outBuffer, outBuffer, len);
  this->dataLen = len;
  return len;
}

int DataCallbacks::read() {
  if (locked || dataQueue.isEmpty()) return -1;
  return dataQueue.pop();
}

bool DataCallbacks::avaliable() {
  if (locked || dataQueue.isEmpty()) return false;
  return true;
}

// BLEDescriptor classes
// Characteristic User Description
class BLE2901 : public BLEDescriptor {
 public:
  BLE2901() : BLEDescriptor(BLEUUID((uint16_t)0x2901)) {
    std::string data = "TX & RX";
    setValue(data);
  }
};

// Client Characteristic Configuration
class BLE2902 : public BLEDescriptor {
 public:
  BLE2902() : BLEDescriptor(BLEUUID((uint16_t)0x2902)) {
    uint8_t data[] = {0};
    setValue(data, 1);
  }
};

// PnP ID
class BLE2A50 : public BLEDescriptor {
 public:
  BLE2A50() : BLEDescriptor(BLEUUID((uint16_t)0x2A50)) {
    uint8_t data[7] = {0x01, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x01};
    setValue(data, 7);
  }
};

// SerialBLE class
bool SerialBLE::available() {
  if (!isConnected()) {
    return false;
  }
  return dataCallbacks.avaliable();
}

int SerialBLE::read() {
  if (!isConnected()) {
    return -1;
  }
  return dataCallbacks.read();
}

size_t SerialBLE::write(uint8_t *buff, size_t len) {
  dataCallbacks.write(buff, len);
  return len;
}

void SerialBLE::begin(String id) {
  static BLE2902 ble2902;
  static BLE2901 ble2901;

  if (inited) return;

  uint64_t chipid = ESP.getEfuseMac();
  if (id.length() == 0) {
    id = "BLE" + String((uint16_t)(chipid >> 32), HEX);
  }
  id.toUpperCase();
  // Create the BLE Device
  BLEDevice::init(id.c_str());
  //
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create the Characteristic
  pRxTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR |
          BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  //
  pRxTxCharacteristic->addDescriptor(&ble2901);
  pRxTxCharacteristic->addDescriptor(&ble2902);
  pRxTxCharacteristic->setCallbacks(&dataCallbacks);
  // Start the service
  pService->start();

  /* Start advertising */
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();
  // pAdvertising->addServiceUUID(SERVICE_UUID);
  // pServer->startAdvertising();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  //
/*   
  static BLEAdvertisementData oAdvertisementData;
  oAdvertisementData.setShortName("short name");
  oAdvertisementData.setName("name");
  oAdvertisementData.setManufacturerData("makerlab");
  pAdvertising->setAdvertisementData(oAdvertisementData);
 */
  inited = true;
}
