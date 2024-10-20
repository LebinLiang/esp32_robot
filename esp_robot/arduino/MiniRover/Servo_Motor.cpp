#include "Servo_Motor.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h>



void Servo_Motor::initialize(int pin, int pwm_channel)
{
  this->pin = pin;
  this->pwm_channel = pwm_channel;

  ledcSetup(pwm_channel,frequency,resolution_bits);
  ledcAttachPin(pin,pwm_channel);
 
}

void Servo_Motor::setPwmDuty(float duty)
{

  duty = constrain(duty, -1, 1);

  int value = duty*3+19;

  uint32_t out_duty = (8191 / 255) * min(value, 255);

ledcWrite(this->pwm_channel, out_duty);

}

Servo_Motor::Servo_Motor()
{
}

Servo_Motor::~Servo_Motor()
{
}
