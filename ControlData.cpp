#include "ControlData.h"
#include <avr/interrupt.h>
#include "Arduino.h"
#include "EEPROMAnything.h"

// We reescribe the ISR for USART1
// Same definitions as in HardwareSerial.cpp
#define SERIAL_BUFFER_SIZE 128

struct ring_buffer
{
    unsigned char buffer[SERIAL_BUFFER_SIZE];
    volatile unsigned int head;
    volatile unsigned int tail;
};

ring_buffer rx_bufferS1 = {{0}, 0, 0};
uint8_t rx_bufferS1_overflow = 0;

// =======================================
// ==                                   ==
// =======================================
inline void store_char(unsigned char c, ring_buffer *buffer)
{
    int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

    // Always store new byte on ring buffer.
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
    //  Overflow? => Discard old bytes
    if (i == buffer->tail)
    {
        buffer->tail = (unsigned int)(buffer->tail + 1) %
                       SERIAL_BUFFER_SIZE; // Move tail to discard old bytes
        rx_bufferS1_overflow = 1;          // Buffer overflow flag
    }
}

// =======================================
// ==                                   ==
// =======================================
ISR(USART1_RX_vect)
{
    if (bit_is_clear(UCSR1A, UPE1))
    {
        unsigned char c = UDR1;
        store_char(c, &rx_bufferS1);
    }
    else
    {
        unsigned char c = UDR1;
    }
}

// =======================================
// ==                                   ==
// =======================================
int Serial1_available(void)
{
    return (unsigned int)(SERIAL_BUFFER_SIZE + rx_bufferS1.head -
                          rx_bufferS1.tail) %
           SERIAL_BUFFER_SIZE;
}

// =======================================
// ==                                   ==
// =======================================
unsigned char Serial1_read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (rx_bufferS1.head == rx_bufferS1.tail)
    {
        return -1;
    }
    else
    {
        unsigned char c = rx_bufferS1.buffer[rx_bufferS1.tail];
        rx_bufferS1.tail =
            (unsigned int)(rx_bufferS1.tail + 1) % SERIAL_BUFFER_SIZE;
        return c;
    }
}

// Write to serial1 (without interrupts)
// =======================================
// ==                                   ==
// =======================================
void Serial1_write(uint8_t c)
{
    while (!((UCSR1A) & (1 << UDRE1)))
        ;

    UDR1 = c;
}

// =======================================
// ==                                   ==
// =======================================
void Serial1_flush()
{
    rx_bufferS1.head = rx_bufferS1.tail;
}

// This are simplified versions of print and println (only for strings)
// =======================================
// ==                                   ==
// =======================================
void Serial1_print(const char str[])
{
    while (*str)
        Serial1_write(*str++);
}

// =======================================
// ==                                   ==
// =======================================
void Serial1_println(const char str[])
{
    while (*str)
        Serial1_write(*str++);

    Serial1_write('\r');
    Serial1_write('\n');
}

// =======================================
// ==                                   ==
// =======================================
void Serial1_begin(unsigned long baud)
{
    uint16_t baud_setting;

    UCSR1A = (1 << U2X1);
    baud_setting = (F_CPU / 4 / baud - 1) / 2;

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    UBRR1H = baud_setting >> 8;
    UBRR1L = baud_setting;

    // transmitting = false;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
}

// =======================================
// ==                                   ==
// =======================================
float scale(float input, float inputMin, float inputMax, float outputMin,
            float outputMax)
{
    // Like map() just returns a float
    float output;

    if (inputMin < inputMax)
        output = (input - inputMin) /
                 ((inputMax - inputMin) / (outputMax - outputMin));
    else
        output = (inputMin - input) /
                 ((inputMin - inputMax) / (outputMax - outputMin));

    if (output > outputMax)
        output = outputMax;
    else if (output < outputMin)
        output = outputMin;

    return output;
}

// Private Methods

// =======================================
// ==                                   ==
// =======================================
float ControlData_Class::extractParamFloat()
{
    union {
        unsigned char Buff[4];
        float d;
    } u;

    u.Buff[0] = (unsigned char)UDPBuffer[4];
    u.Buff[1] = (unsigned char)UDPBuffer[3];
    u.Buff[2] = (unsigned char)UDPBuffer[2];
    u.Buff[3] = (unsigned char)UDPBuffer[1];

    return (u.d);
}

// =======================================
// ==                                   ==
// =======================================
uint16_t ControlData_Class::extractParamUInt16()
{
    union {
        unsigned char Buff[2];
        uint16_t d;
    } u;

    u.Buff[0] = (unsigned char)UDPBuffer[2];
    u.Buff[1] = (unsigned char)UDPBuffer[1];

    return (u.d);
}
// Constructors ////////////////////////////////////////////////////////////////

ControlData_Class::ControlData_Class()
{
    Serial1_begin(115200);

    targetOffset =
        scale(5.0f, 0.0f, 10.0f, 0.0f, (float)getControlAngleLimit());
    turningOffset = scale(5.0f, 0.0f, 10.0f, 0.0f, (float)getTurningLimit());
}

// Public Methods //////////////////////////////////////////////////////////////
// This function lets us send simple one param messages (float param)
// No overflow check!
void ControlData_Class::sendMessage(uint8_t command, float p)
{
    uint8_t i;
    uint8_t c[7];
    union {
        uint8_t Buff[4];
        float d;
    } u;

    // 送信パケット構築
    u.d = p;
    c[0] = '>';
    c[1] = command;
    c[2] = u.Buff[0];
    c[3] = u.Buff[1];
    c[4] = u.Buff[2];
    c[5] = u.Buff[3];
    c[6] = ';';

    for (i = 0; i < 7; i++)
    {
        Serial1_write(c[i]);
    }
}

void ControlData_Class::sendMessage(uint8_t command, uint16_t p)
{
    uint8_t i;
    uint8_t c[7];
    union {
        uint8_t Buff[2];
        uint16_t d;
    } u;

    // 送信パケット構築
    u.d = p;
    c[0] = '>';
    c[1] = command;
    c[2] = u.Buff[0];
    c[3] = u.Buff[1];
    c[4] = 0x00;
    c[5] = 0x00;
    c[6] = ';';

    for (i = 0; i < 7; i++)
    {
        Serial1_write(c[i]);
    }
}

void ControlData_Class::sendMessage(float yaw, float pitch, float roll,
                                    float pid, int32_t wheelVelocity)
{
    uint8_t i;
    uint8_t c[23];
    union {
        uint8_t Buff[4];
        float d;
    } u_float;
    union {
        uint8_t Buff[4];
        int32_t d;
    } u_int32_t;

    // 送信パケット構築
    c[0] = '>';
    c[1] = '@';

    u_float.d = yaw;
    c[2] = u_float.Buff[0];
    c[3] = u_float.Buff[1];
    c[4] = u_float.Buff[2];
    c[5] = u_float.Buff[3];

    u_float.d = pitch;
    c[6] = u_float.Buff[0];
    c[7] = u_float.Buff[1];
    c[8] = u_float.Buff[2];
    c[9] = u_float.Buff[3];

    u_float.d = roll;
    c[10] = u_float.Buff[0];
    c[11] = u_float.Buff[1];
    c[12] = u_float.Buff[2];
    c[13] = u_float.Buff[3];

    u_float.d = pid;
    c[14] = u_float.Buff[0];
    c[15] = u_float.Buff[1];
    c[16] = u_float.Buff[2];
    c[17] = u_float.Buff[3];

    u_int32_t.d = wheelVelocity;
    c[18] = u_float.Buff[0];
    c[19] = u_float.Buff[1];
    c[20] = u_float.Buff[2];
    c[21] = u_float.Buff[3];

    c[22] = ';';

    for (i = 0; i < 23; i++)
    {
        Serial1_write(c[i]);
    }
}

command_type ControlData_Class::readMessage()
{
    uint8_t i;
    uint8_t tmp;
    //    float value;
    //    float value2;

    command = NO_COMMAND;

    // Overflow on rx buffer?
    if (rx_bufferS1_overflow)
    {
        // We lost old bytes so we need to reset the parser and discard old
        // bytes until we reach a message start character '/'
        //        readStatus = 0;
        rx_bufferS1_overflow = 0;
        while (Serial1_available())
        {
            tmp = Serial1_read();
            if (tmp == '<' || tmp == '=')
            {
                UDPBuffer[0] = tmp;
                return;
            }
        }
    }

    // New byteas available to process?
    if (Serial1_available() > 0)
    {
        // Serial.print("B:");
        // Serial.println(Serial1_available());
        // We rotate the Buffer (we could implement a ring buffer in future)
        for (i = 6; i > 0; i--)
        {
            UDPBuffer[i] = UDPBuffer[i - 1];
        }
        UDPBuffer[0] = Serial1_read();

        if (UDPBuffer[0] == '<' || UDPBuffer[0] == '=')
        {
            packet_data = config_data;
        }

        if (config_data == packet_data)
        {
            if (UDPBuffer[0] == ';' && (UDPBuffer[2] == '<'))
            {
                switch (UDPBuffer[1])
                {
                    case 'A':
                        sendMessage('A', getTargetAngle());
                        break;

                    case 'P':
                        sendMessage('P', getPValue());
                        break;

                    case 'I':
                        sendMessage('I', getIValue());
                        break;

                    case 'D':
                        sendMessage('D', getDValue());
                        break;

                    case 'O':
                        sendMessage('O', getControlAngleLimit());
                        break;

                    case 'T':
                        sendMessage('T', getTurningLimit());
                        break;

                    case '#':
                        uint16_t send_data;
                        if (getBackToSpotFlag())
                        {
                            send_data = 1;
                        }
                        else
                        {
                            send_data = 0;
                        }
                        sendMessage('#', send_data);
                        break;

                    case '@':
                        command = POSTURE_REQ;
                        break;
                }
                packet_data = control_data;
            }

            else if ((UDPBuffer[4] == '=') && (UDPBuffer[0] == ';'))
            {
                switch (UDPBuffer[3])
                {
                    case 'O':
                        if (setControlAngleLimit(extractParamUInt16()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;

                    case 'T':
                        if (setTurningLimit(extractParamUInt16()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;

                    case 'N': // turn
                        turningOffset = scale((float)extractParamUInt16(), 0,
                                              10, 0, (float)getTurningLimit());
                        break;

                    case 'G': // turn
                        targetOffset = scale((float)extractParamUInt16(), 0, 10,
                                             0, (float)getControlAngleLimit());
                        break;

                    case '#':
                        if (extractParamUInt16() != 0)
                        {
                            if (setBackToSpotFlag(true) != 0)
                            {
                                command = UPDATED_CFG_DATA;
                            }
                        }
                        else
                        {
                            if (setBackToSpotFlag(false) != 0)
                            {
                                command = UPDATED_CFG_DATA;
                            }
                        }
                        break;
                }
                packet_data = control_data;
            }

            else if (UDPBuffer[6] == '=' && (UDPBuffer[0] == ';'))
            {
                switch (UDPBuffer[5])
                {
                    case 'A':
                        if (setTargetAngle(extractParamFloat()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;

                    case 'P':
                        if (setPValue(extractParamFloat()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;

                    case 'I':
                        if (setIValue(extractParamFloat()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;

                    case 'D':
                        if (setDValue(extractParamFloat()) != 0)
                        {
                            command = UPDATED_CFG_DATA;
                        }
                        break;
                }
                packet_data = control_data;
            }
        }
        else
        {
            switch (UDPBuffer[0])
            {
                case 'g':
                case 'G':
                    command = FORWARD_LEFT_CTRL;
                    break;

                case 'l':
                case 'L':
                    command = LEFT_CTRL;
                    break;

                case 'h':
                case 'H':
                    command = BACKWARD_LEFT_CTRL;
                    break;

                case 'f':
                case 'F':
                    command = FORWARD_CTRL;
                    break;

                case 'b':
                case 'B':
                    command = BACKWARD_CTRL;
                    break;

                case 'i':
                case 'I':
                    command = FORWARD_RIGHT_CTRL;
                    break;

                case 'r':
                case 'R':
                    command = RIGHT_CTRL;
                    break;

                case 'j':
                case 'J':
                    command = BACKWARD_RIGHT_CTRL;
                    break;

                case 's':
                case 'S':
                    command = STOP_CTRL;
                    break;
            }
        }
    }

    return command;
}

float ControlData_Class::getTargetOffset()
{
    return targetOffset;
}

float ControlData_Class::getTurningOffset()
{
    return turningOffset;
}

// make one instance for the user to use
