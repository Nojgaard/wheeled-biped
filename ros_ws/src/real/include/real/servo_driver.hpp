#pragma once

#include <scservo/SCServo.h>

class ServoDriver {
public:
  bool initialize();
private:
  SMS_STS servo_;
};