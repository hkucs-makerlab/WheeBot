#ifndef __BUZZER__
#define __BUZZER__
#include <ESP32Tone.h>

class Buzzer {
 private:
  int pin;
  bool flag;
  long nextT;

  bool wait(long interval) {
    long currT = millis();
    if (currT < nextT) {
      return false;
    }
    return true;
  }

  bool tone2(int frequency, int duration) {
    if (flag) {
      nextT =  millis() + duration;
      tone(pin, frequency);
    }
    if ((flag = wait(duration))) {
      noTone(pin);
    }
    return flag;
  }

 public:
  Buzzer(int pin) : pin(pin), flag(true) {
  }

  void beepStart() {
    while (!beepShort()) {
    }
  }
  bool beepShort() {
    return tone2(20, 100);
  }

  bool beep() {
    return tone2(500, 100);
  }
  
  bool beepError() {
    return tone2(100, 100);
  }
};

#endif  //__BUZZER__