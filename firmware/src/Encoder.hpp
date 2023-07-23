#pragma once
#include <Arduino.h>
#include <FunctionalInterrupt.h>

class Encoder {
 private:
  int gpioPin;

 private:
  void IRAM_ATTR isr();

 public:
  volatile unsigned long counter;

  Encoder(int pin) : gpioPin(pin), counter(0) {
  }

  void begin() {
    pinMode(gpioPin, INPUT_PULLUP);
    attachInterrupt(gpioPin, std::bind(&Encoder::isr, this), FALLING);
  }

  virtual ~Encoder() {
    detachInterrupt(gpioPin);
  }
};