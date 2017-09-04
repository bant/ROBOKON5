//
//
//
#ifndef CONTROLDATA_h__
#define CONTROLDATA_h__

#include "Arduino.h"
#include "ConfigData.h"

int Serial1_available(void);
unsigned char Serial1_read(void);
void Serial1_write(uint8_t c);
void Serial1_print(const char str[]);
void Serial1_println(const char str[]);
void Serial1_flush();
float scale(float input, float inputMin, float inputMax, float outputMin,
            float outputMax);

static const int8_t control_data = 0;
static const int8_t config_data = 1;

typedef enum {
    NO_COMMAND,
    UPDATED_CFG_DATA,
    POSTURE_REQ,
    FORWARD_LEFT_CTRL,
    LEFT_CTRL,
    BACKWARD_LEFT_CTRL,
    FORWARD_CTRL,
    BACKWARD_CTRL,
    FORWARD_RIGHT_CTRL,
    RIGHT_CTRL,
    BACKWARD_RIGHT_CTRL,
    STOP_CTRL
} command_type;

class ControlData_Class : public ConfigData
{
  private:
    // UPD input buffer
    char UDPBuffer[7];
    uint8_t bytesRead;
    int8_t packet_data;
    float targetOffset;  // Offset for going forward and backward
    float turningOffset; // Offset for turning left and right
    command_type command;

    float extractParamFloat();
    uint16_t extractParamUInt16();

  public:
    ControlData_Class();

    void sendMessage(uint8_t command, float p);
    void sendMessage(uint8_t command, uint16_t p);
    void sendMessage(float yaw, float pitch, float roll, float pid,
                     int32_t wheelVelocity);

    command_type readMessage();
    float getTargetOffset();
    float getTurningOffset();
};

#endif