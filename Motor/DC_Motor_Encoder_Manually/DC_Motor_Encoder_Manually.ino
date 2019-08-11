#include "src/DC_motor_driver/DC_motor_driver.h"
#include <Ewma.h>

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 53

// Encoder output to Arduino Interrupt pin
#define ENC_IN 3

// Analog pin for potentiometer
int speedcontrol = 0;

// Pulse count from encoder
volatile long encoderValue = 0;

// One-second interval for measurements
int interval = 50;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
int rpm = 0;
int rpmPrev = 0;

// Variable for PWM motor speed output
int motorPwm = 0;

DC_motor motor( 11, 12, 128);  // PWM = Pin 11, DIR = Pin 12.

Ewma adcFilter(0.2);  // More smoothing - less prone to noise, but slower to detect changes
float filtered;

void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600);

  // Set encoder as input with internal pullup
  pinMode(ENC_IN, INPUT_PULLUP);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, CHANGE);

  // Setup initial values for timer
  previousMillis = millis();
}

void loop()
{

  // Control motor with potentiometer
  motorPwm = map(analogRead(speedcontrol), 0, 1023, 0, 120);

  // Write PWM to controller
  motor.setSpeed(motorPwm);

  if (millis() - previousMillis > interval) {
    currentMillis = millis();
    rpm = (float)(encoderValue * 60000 / (ENC_COUNT_REV * (currentMillis - previousMillis)));
    previousMillis = currentMillis;


    // Calculate RPM

    filtered = adcFilter.filter(rpm);

    Serial.print(rpm);
    Serial.print("\t");
    
    rpm = rpm * 0.3 + rpmPrev * (1-0.3);
    rpmPrev = rpm;
    Serial.print(rpm);
    Serial.print("\t");
    Serial.println(filtered);

    encoderValue = 0;
  }
}

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
}
