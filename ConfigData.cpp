#include "ConfigData.h"
#include "Arduino.h"

// Constructors ////////////////////////////////////////////////////////////////

ConfigData::ConfigData()
{
    EEPROM_readAnything(cfg_addr, cfg);
}

// Public Methods //////////////////////////////////////////////////////////////
//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setTargetAngle(float targetAngle)
{
    cfg.targetAngle = targetAngle;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
float ConfigData::getTargetAngle()
{
    return cfg.targetAngle;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setPValue(float P)
{
    cfg.P = P;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
float ConfigData::getPValue()
{
    return cfg.P;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setIValue(float I)
{
    cfg.I = I;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
float ConfigData::getIValue()
{
    return cfg.I;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setDValue(float D)
{
    cfg.D = D;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
float ConfigData::getDValue()
{
    return cfg.D;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setBackToSpotFlag(bool backToSpot)
{
    cfg.backToSpot = backToSpot;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
bool ConfigData::getBackToSpotFlag()
{
    return cfg.backToSpot;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setControlAngleLimit(uint16_t controlAngleLimit)
{
    cfg.controlAngleLimit = controlAngleLimit;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
uint16_t ConfigData::getControlAngleLimit()
{
    return cfg.controlAngleLimit;
}

//==============================================================================
//
//==============================================================================
// setter
uint16_t ConfigData::setTurningLimit(uint16_t turningLimit)
{
    cfg.turningLimit = turningLimit;
    return EEPROM_updateAnything(cfg_addr, cfg);
}

// getter
uint16_t ConfigData::getTurningLimit()
{
    return cfg.turningLimit;
}
