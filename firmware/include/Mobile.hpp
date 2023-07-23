#pragma once

class Motor {
 protected:
  int pwmPin1;
  int pwmPin2;
  int pwmValue;

 public:
  Motor(int pwmPin1, int pwmPin2) : pwmPin1(pwmPin1), pwmPin2(pwmPin2) {
    update(0);
  }

  void update(int pwmValue) {
    if (pwmValue > 0) {
      analogWrite(pwmPin1, pwmValue);
      analogWrite(pwmPin2, 0);
      this->pwmValue = pwmValue;
    } else if (pwmValue < 0) {
      analogWrite(pwmPin1, 0);
      analogWrite(pwmPin2, -pwmValue);
      this->pwmValue = pwmValue;
    } else {  // Motor brake
      analogWrite(pwmPin1, 10);
      analogWrite(pwmPin2, 10);
      this->pwmValue = 0;
    }
  }
  int getpwmValue() { return pwmValue; }
};

class Mobile {
 private:
  Motor rightMotor;
  Motor leftMotor;

 public:
  Mobile(Motor rightMotor, Motor leftMotor)
      : rightMotor(rightMotor), leftMotor(leftMotor) {}

  void updatePWM(int right, int left) {
    rightMotor.update(right);
    leftMotor.update(left);
  }

  void getPWM(int *right, int *left) {
    if (right != nullptr) {
      *right = rightMotor.getpwmValue();
    }
    if (left != nullptr) {
      *left = leftMotor.getpwmValue();
    }
  }
  inline void stop() { updatePWM(0, 0); }
};