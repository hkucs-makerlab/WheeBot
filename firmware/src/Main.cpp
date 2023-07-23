#include <Arduino.h>

#include "Config.hpp"
#include "Mobile.hpp"
//
Motor motorLeft(MOTOR_LEFT_GPIO_PIN1, MOTOR_LEFT_GPIO_PIN2);
Motor motorRight(MOTOR_RIGHT_GPIO_PIN1, MOTOR_RIGHT_GPIO_PIN2);
Mobile mobile(motorRight, motorLeft);

#if HAS_SPEED_SENSOR
#include "Encoder.hpp"
Encoder leftSpeedSensor(GPIO_PIN_SPEED_L);
Encoder rightSpeedSensor(GPIO_PIN_SPEED_R);
#endif

#ifdef __ROS__
#include <geometry_msgs/Twist.h>

#include "ROSserial.hpp"
ros::MyNodeHandle nh;

float demandx = 0;
float demandz = 0;
volatile bool demandUpdate = false;
void cmdVelCallback(const geometry_msgs::Twist &cmd_vel) {
  float x = constrain(cmd_vel.linear.x, -1, 1) * 255;
  float z = constrain(cmd_vel.angular.z, -1, 1) * 150;
  if (demandx != x) {
    demandUpdate = true;
    demandx = x;
  }
  if (demandz != z) {
    demandUpdate = true;
    demandz = z;
  }
}
ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel",
                                                       cmdVelCallback);
#endif //  __ROS__

#ifdef __ESP32_BLE__
#include <GoBLE.hpp>

bool handleInput(float *_valueLinear, float *_valueAngular) {
  bool rc = false;
  float valueLinear = 0, valueAngular = 0;

  if (Goble.available()) {
    int joystickY = Goble.readJoystickY();
    int joystickX = Goble.readJoystickX();

    valueLinear = map(joystickY, 0, 255, -100, 100);
    valueAngular = map(joystickX, 0, 255, 100, -100);

    if (Goble.readSwitchLeft() == PRESSED) {
      valueLinear = 10;
      valueAngular = 70;
    } else if (Goble.readSwitchRight() == PRESSED) {
      valueLinear = 10;
      valueAngular = -70;
    } else if (Goble.readSwitchUp() == PRESSED) {
      valueLinear = 80;
      valueAngular = 0;
    } else if (Goble.readSwitchDown() == PRESSED) {
      valueLinear = -80;
      valueAngular = 0;
    }
    valueLinear = (255 * 3 / 4) * (valueLinear / 100);
    valueAngular = (255 * 3 / 4) * (valueAngular / 100);

    rc = true;
  }
  *_valueLinear = valueLinear;
  *_valueAngular = valueAngular;
  return rc;
}
#endif  // __ESP32_BLE__

void loop() {
  float rightWheelPwm = 0;
  float leftWheelPwm = 0;

#ifdef __ESP32_BLE__
  float valueLinear = 0, valueAngular = 0;
  if (handleInput(&valueLinear, &valueAngular)) {
    rightWheelPwm = valueLinear + valueAngular;
    leftWheelPwm = valueLinear - valueAngular;
    mobile.updatePWM(rightWheelPwm, leftWheelPwm);
  }
#endif  // __ESP32_BLE__

#ifdef __ROS__
  if (1) {
    static long timeout = 0;
    if (timeout < millis()) {
      timeout = millis() + 50;
      if (demandUpdate) {
        rightWheelPwm = demandx + demandz;
        leftWheelPwm = demandx - demandz;
        mobile.updatePWM(rightWheelPwm, leftWheelPwm);
        demandUpdate = false;
      }
    }
    nh.spinOnce();
  }
#endif

#if HAS_SPEED_SENSOR
  if (1) {
    static long timeout = 0;
    if (timeout < millis()) {
      timeout = millis() + 1000;
      Console.println(leftSpeedSensor.counter);
      Console.println(rightSpeedSensor.counter);
      Console.println();
      if (leftWheelPwm == 0) leftSpeedSensor.counter = 0;
      if (rightWheelPwm == 0) rightSpeedSensor.counter = 0;
    }
  }
#endif
}

void setup() {
  Console.begin(CONSOLE_BAUD_RATE);
#ifdef __DEBUG__
  Console.println("in debugging mode");
#endif
  //
  leftSpeedSensor.begin();
  rightSpeedSensor.begin();
  //
  mobile.stop();
  //
#ifdef __ESP32_BLE__
  Goble.begin();
#endif
  //
#ifdef __ROS__
  nh.initNode();
  nh.subscribe(cmdVelSubscriber);
  nh.loginfo("[started]");
#endif
  //

  // led.update(150);
  Console.println("[started]");
}
