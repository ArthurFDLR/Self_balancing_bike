#include "Encoder.h"

EncoderQuad::EncoderQuad(uint8_t pinEncoderChA, uint8_t pinEncoderChB, float paramExpFilter, uint8_t encoderCountRev)
{
    _pinEncoderAInterrupt = pinEncoderChA;
    _pinEncoderB = pinEncoderChB;
    _registerMaskPind = ((1 << pinEncoderChA) | (1 << pinEncoderChB));
    _encoderCountRev = encoderCountRev;
    pinMode(_pinEncoderAInterrupt, INPUT_PULLUP);
    pinMode(_pinEncoderB, INPUT_PULLUP);
    

    _motorTimePrev = millis();
}

void EncoderQuad::ISR_updateEncoder(void) //Direct access to register to optimize time consumption
{
    if (((_registerMaskPind & PIND) == (1 << _pinEncoderAInterrupt)) or ((_registerMaskPind & PIND) == (1 << _pinEncoderB)))
    {
        _motorTickCount++;
    }
    else if (((_registerMaskPind & PIND) == 0) or ((_registerMaskPind & PIND) == _registerMaskPind))
    {
        _motorTickCount--;
    }
}

void EncoderQuad::Update_MotorData(void)
{
    //Get motor rotation speed from tick count
    _motorTimeNow = millis();
    _motorTimeDiff = _motorTimeNow - _motorTimePrev;
    _motorTimePrev = _motorTimeNow;
    _motorSpeedRpmPrev = motorSpeedRpm;
    _motorSpeedRpmRaw = ((_motorTickCount > 0) ? _motorTickCount : -_motorTickCount) * 60000 / (_encoderCountRev * _motorTimeDiff); // 60000 => ms to s
    _motorTickCount = 0;

    //Set motor rotation direction
    if (motorSpeedRpm == 0)
    {
        motorDirection = 0;
    }
    else if (_motorTickCount > 0)
    {
        motorDirection = 1;
    }
    else
    {
        motorDirection = -1;
    }

    //Filtering
    motorSpeedRpm = _motorSpeedRpmRaw * _paramExpFilter + _motorSpeedRpmPrev * (1.0 - _paramExpFilter);
}