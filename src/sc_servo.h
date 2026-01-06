#include "servo.h"

// SC series servo (big-endian)
class SCServo : public Servo {
public:
  using Servo::Servo;
  ServoBusApi::ServoType type() const override { return ServoBusApi::ServoType::SC; }
};
