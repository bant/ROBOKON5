//
//
//
#ifndef CONFIGDATA_H__
#define CONFIGDATA_H__

#include "Arduino.h"
#include "EEPROMAnything.h"

static const uint16_t cfg_addr = 0;

typedef struct
{
    float P, I, D;     // PID variables
    float targetAngle; // Resting angle of the robot
    bool backToSpot;   // Set whenever the robot should stay in the same spot
    uint16_t controlAngleLimit; // Set the maximum tilting angle of the
                                //    robot
    uint16_t turningLimit;      // Set the maximum turning value
    //   float accYzero, accZzero;  // Accelerometer zero values
    //    bool bindSpektrum;
} cfg_t;

class ConfigData
{
  private:
    cfg_t cfg;

  protected:
    uint16_t setTargetAngle(float targetAngle);
    uint16_t setPValue(float P);
    uint16_t setIValue(float I);
    uint16_t setDValue(float D);
    uint16_t setBackToSpotFlag(bool backToSpot);
    uint16_t setControlAngleLimit(uint16_t controlAngleLimit);
    uint16_t setTurningLimit(uint16_t turningLimit);

  public:
    ConfigData();

    float getTargetAngle();
    float getPValue();
    float getIValue();
    float getDValue();
    bool getBackToSpotFlag();
    uint16_t getControlAngleLimit();
    uint16_t getTurningLimit();
};

#endif // CONFIGDATA_H__