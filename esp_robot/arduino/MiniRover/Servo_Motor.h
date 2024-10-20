#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <stdint.h>


class Servo_Motor
{
public:
  Servo_Motor();
  ~Servo_Motor();

  void initialize(int pin, int pwm_channel);
  void setPwmDuty(float duty);

private:

  double frequency = 50;
  uint8_t resolution_bits = 13;

  int pin;
  int pwm_channel;

};

#endif
