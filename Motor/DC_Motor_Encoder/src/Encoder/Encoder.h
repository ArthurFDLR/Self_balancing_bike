/**
 * Read data from quadrature encoder with 
 * direct access to arduino nano's register
 *
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

/*
Encoder pins have to be on register D !
*/
class EncoderQuad
{
    public:
	    EncoderQuad(uint8_t pinEncoderChA, uint8_t pinEncoderChB, float paramExpFilter, uint8_t encoderCountRev);
        void ISR_updateEncoder(void);
	    void Update_MotorData(void);

        int8_t motorDirection = 0; // {-1;0;1} with 0 => stopped
        uint16_t motorSpeedRpm = 0;

    private:
    
        uint8_t _pinEncoderAInterrupt;
        uint8_t _pinEncoderB;
	    char _registerMaskPind; //Show pin 3 and 4
        uint8_t _encoderCountRev;
        volatile int16_t _motorTickCount = 0;
        uint16_t _motorTimePrev = 0; //Some of those variables can be ignored/simplify
        uint16_t _motorTimeNow = 0;
        uint16_t _motorTimeDiff = 0;
        uint16_t _motorSpeedRpmPrev = 0;
        uint16_t _motorSpeedRpmRaw = 0;
        float _paramExpFilter;
};


#endif