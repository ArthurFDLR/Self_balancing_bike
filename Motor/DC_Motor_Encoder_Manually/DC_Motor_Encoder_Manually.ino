#include "src/DC_motor_driver/DC_motor_driver.h"

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 53

// Encoder output to Arduino Interrupt pin
#define ENC_IN 3 

// Analog pin for potentiometer
int speedcontrol = 0;

// Pulse count from encoder
volatile long encoderValue = 0;

// One-second interval for measurements
int interval = 25;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
int rpm = 0;

// Variable for PWM motor speed output
int motorPwm = 0;

DC_motor motor( 11, 12, 128);  // PWM = Pin 11, DIR = Pin 12.

void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN, INPUT_PULLUP); 
  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder, RISING);
  
  // Setup initial values for timer
  previousMillis = millis();
}

void loop()
{
  
    // Control motor with potentiometer
    motorPwm = map(analogRead(speedcontrol), 0, 1023, 0, 120);
    
    // Write PWM to controller
    motor.setSpeed(motorPwm);
  
  // Update RPM value every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;


    // Calculate RPM
    rpm = (float)(encoderValue * 600 / ENC_COUNT_REV);

    // Only update display when there is a reading
    if (motorPwm > 0 || rpm > 0) {
      Serial.print("PWM VALUE: ");
      Serial.print(motorPwm);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue);
      Serial.print('\t');
      Serial.print(" SPEED: ");
      Serial.print(rpm);
      Serial.println(" RPM");
    }
    
    encoderValue = 0;
  }
}

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
}
