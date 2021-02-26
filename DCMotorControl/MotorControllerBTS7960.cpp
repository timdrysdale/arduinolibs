/******************************************************************************
MotorController.cpp
Commands for the MotorController for PID Controller Remote Lab.
David Reid
14/09/20

Modified Tim Drysdale
18 Feb 2021
Use SAMD21 PWM

Contains commands for the DC Motor for the PID Controller remote lab.
******************************************************************************/

#include "MotorControllerBTS7960.h"
#include <Arduino.h>

MotorBTS7960::MotorBTS7960(int wavePin, int enableLPin, int enableRPin, int offset, long prescale)
{
  wave = wavePin;
  enableL = enableLPin;
  enableR = enableRPin;
  
  Offset = offset;
  
  prevSpeed = 0;
  
  pinMode(wave, OUTPUT);
  pinMode(enableL, OUTPUT);
  pinMode(enableR, OUTPUT);
 
  servo.setClockDivider(1, false);  // Input clock is divided by 1 and 48MHz is sent to Generic Clock, Turbo is off
  servo.timer(0, 1, prescale, true); //timer0 for pin6 https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM, was 960000 for 48MHz/960000=50Hz

  minSpeed = 0;
  maxSpeed = MAX_ABS_SPEED;
    
}

void MotorBTS7960::setPrescale(long prescale) {
  servo.timer(0, 1, prescale, true);
}


unsigned int MotorBTS7960::speedToDuty(float speed) {
  
  // convert
  // -1.0 <= speed <= +1.0
  // to duty cycle unsigned int 0 - 1000 (0 - 100% in 0.1% steps?)
  speed = abs(speed);
  
  if (speed > 1) {
	speed = 1.0;
  }

  speed = minSpeed + (speed * (maxSpeed - minSpeed));
  
  return (unsigned int) (1000.0f * speed);
}

void MotorBTS7960::pwm(float speed) {
    
  servo.analogWrite(wave, speedToDuty(speed)); 


}

void MotorBTS7960::drive(float speed)
{
  if(speed / prevSpeed < 0) free();	//H bridge cannot have enable pin high when direction changes 

  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
  
  prevSpeed = speed;
}

void MotorBTS7960::drive(float speed, int duration)
{
  drive(speed);
  delay(duration);
}

void MotorBTS7960::fwd(float speed)
{
   digitalWrite(enableR, HIGH);
   pwm(speed);
}

void MotorBTS7960::rev(float speed)
{
   digitalWrite(enableL, HIGH);
   pwm(speed);
}

void MotorBTS7960::brake()
{
  free();
}

void MotorBTS7960::free()
{
  // must set enable to zero or else changing direction will
  // damage the h-bridge
  digitalWrite(enableL, LOW);
  digitalWrite(enableR, LOW);
}

void MotorBTS7960::standby()
{
  free();
}
