//=============================================================================
// File Name    : ROBOKON5.ino
//
// Title        : ROBOKON5 メインルーチン
// Revision     : 0.1
// Notes        :
// Target MCU   : Arduino(ATMega32U4)
// Tool Chain   :
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 2013/02/06   ばんと      修正完了
//=============================================================================

/* Includes ------------------------------------------------------------------*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "ControlData.h"

/* local typedef -------------------------------------------------------------*/
/* local define --------------------------------------------------------------*/
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define ZERO_SPEED 65535
#define MAX_ACCEL \
    7 // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)

#define MAX_CONTROL_OUTPUT 500

static const uint16_t zoneA = 8000 * 2;
static const uint16_t zoneB = 4000 * 2;
static const uint16_t zoneC = 1000 * 2;
static const float positionScaleA = 600.0f * 2.0f; // One resolution is
                                                   // 1856 pulses per encoder
static const float positionScaleB = 800.0f * 2.0f;
static const float positionScaleC = 1000.0f * 2.0f;
static const float positionScaleD = 500.0f * 2.0f;
static const float velocityScaleMove = 70.0f * 2.0f;
static const float velocityScaleStop = 60.0f * 2.0f;
static const float velocityScaleTurning = 70.0f * 2.0f;

static const int16_t speed_ratio = 16;

/* local macro ---------------------------------------------------------------*/
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))

/* local variables -----------------------------------------------------------*/
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float
    ypr[3]; // [yaw, roll, pitch]   yaw/pitch/roll container and gravity vector
float yaw = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;

int16_t speed_M1 = 0;
int16_t speed_M2 = 0; // Actual speed of motors
int8_t dir_M1 = 0;
int8_t dir_M2 = 0; // Actual direction of steppers motors
int32_t motor1Counter = 0;
int32_t motor2Counter = 0;

float P, I, D;     // PID variables
float targetAngle; // Resting angle of the robot
float targetOffset;
float turningOffset;
bool backToSpot; // Set whenever the robot should stay in the same spot
float PIDValue;
int16_t motor1;
int16_t motor2;

int32_t lastWheelPosition = 0; // Used to calculate the wheel velocity
int32_t targetPosition = 0;
int32_t wheelVelocity = 0;
float lastRestAngle = 0;
float lastError; // Store last angle error
float iTerm;     // Store iTerm

uint32_t pidTimer;
uint32_t encoderTimer;

//コントーロール
bool steerStop = true;
bool layingDown = false;
bool stopped = false;

/* local function prototypes -------------------------------------------------*/

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation
// board)
// AD0 high = 0x69
MPU6050 mpu;

//=======================
//
ControlData_Class ctrl_data;

// ================================================================
// ===      16 single cycle instructions = 1us at 16Mhz         ===
// ================================================================
void delay_1us()
{
    __asm__ __volatile__(
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop"
        "\n\t"
        "nop");
}

// ================================================================
// ===          TIMER 1 : STEPPER MOTOR1 SPEED CONTROL          ===
// ================================================================
ISR(TIMER1_COMPA_vect)
{
    if (dir_M1 == 0) // If we are not moving we dont generate a pulse
        return;
    else if (dir_M1 > 0)
        motor1Counter++;
    else
        motor1Counter--;

    // We generate 1us STEP pulse
    SET(PORTE, 6); // STEP MOTOR 1
    delay_1us();
    CLR(PORTE, 6);
}

// ================================================================
// ===          TIMER 3 : STEPPER MOTOR2 SPEED CONTROL          ===
// ================================================================
ISR(TIMER3_COMPA_vect)
{
    if (dir_M2 == 0) // If we are not moving we dont generate a pulse
        return;
    else if (dir_M2 > 0)
        motor2Counter++;
    else
        motor2Counter--;

    // We generate 1us STEP pulse
    SET(PORTD, 6); // STEP MOTOR 2
    delay_1us();
    CLR(PORTD, 6);
}

// ================================================================
// ===          モータ初期化                                    ===
// ================================================================
void initializeMotors()
{
    // ポート設定
    pinMode(4, OUTPUT);  // ENABLE MOTORS
    pinMode(7, OUTPUT);  // STEP MOTOR 1 PORTE,6
    pinMode(8, OUTPUT);  // DIR MOTOR 1  PORTB,4
    pinMode(12, OUTPUT); // STEP MOTOR 2 PORTD,6
    pinMode(5, OUTPUT);  // DIR MOTOR 2  PORTC,6

    digitalWrite(4, HIGH); // Disbale motors

    // STEPPER MOTORS INITIALIZATION
    //    Serial.println("Steper motors initialization...");
    // MOTOR1 => TIMER1
    TCCR1A = 0; // Timer1 CTC mode 4, OCxA,B outputs disconnected
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
    OCR1A = ZERO_SPEED;                  // Motor stopped
    dir_M1 = 0;
    TCNT1 = 0;

    // MOTOR2 => TIMER3
    TCCR3A = 0; // Timer3 CTC mode 4, OCxA,B outputs disconnected
    TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
    OCR3A = ZERO_SPEED;                  // Motor stopped
    dir_M2 = 0;
    TCNT3 = 0;
}

// ================================================================
// ===    Set speed of Stepper Motor1                           ===
// ===    tspeed could be positive or negative (reverse)        ===
// ================================================================
void setMotorSpeedM1(int16_t tspeed)
{
    long timer_period;
    int16_t speed;

    // Limit max speed?

    // WE LIMIT MAX ACCELERATION of the motors
    if ((speed_M1 - tspeed) > MAX_ACCEL)
        speed_M1 -= MAX_ACCEL;
    else if ((speed_M1 - tspeed) < -MAX_ACCEL)
        speed_M1 += MAX_ACCEL;
    else
        speed_M1 = tspeed;

    speed = speed_ratio * speed_M1; // Adjust factor from control output speed
                                    // to real motor speed in steps/second

    if (speed == 0)
    {
        timer_period = ZERO_SPEED;
        dir_M1 = 0;
    }
    else if (speed > 0)
    {
        timer_period = 2000000 / speed; // 2Mhz timer
        dir_M1 = 1;
        SET(PORTB, 4); // DIR Motor 1 (Forward)
    }
    else
    {
        timer_period = 2000000 / -speed;
        dir_M1 = -1;
        CLR(PORTB, 4); // Dir Motor 1
    }
    if (timer_period >
        65535) // Check for minimun speed (maximun period without overflow)
        timer_period = ZERO_SPEED;

    OCR1A = timer_period;
    // Check  if we need to reset the timer...
    if (TCNT1 > OCR1A)
        TCNT1 = 0;
}

// ================================================================
// ===    Set speed of Stepper Motor2                           ===
// ===    tspeed could be positive or negative (reverse)        ===
// ================================================================
void setMotorSpeedM2(int16_t tspeed)
{
    long timer_period;
    int16_t speed;

    // Limit max speed?

    // WE LIMIT MAX ACCELERATION of the motors
    if ((speed_M2 - tspeed) > MAX_ACCEL)
        speed_M2 -= MAX_ACCEL;
    else if ((speed_M2 - tspeed) < -MAX_ACCEL)
        speed_M2 += MAX_ACCEL;
    else
        speed_M2 = tspeed;

    speed = speed_ratio * speed_M2; // Adjust factor from control output speed
                                    // to real motor speed in steps/second

    if (speed == 0)
    {
        timer_period = ZERO_SPEED;
        dir_M2 = 0;
    }
    else if (speed > 0)
    {
        timer_period = 2000000 / speed; // 2Mhz timer
        dir_M2 = 1;
        CLR(PORTC, 6); // Dir Motor2 (Forward)
    }
    else
    {
        timer_period = 2000000 / -speed;
        dir_M2 = -1;
        SET(PORTC, 6); // DIR Motor 2
    }
    if (timer_period >
        65535) // Check for minimun speed (maximun period without overflow)
        timer_period = ZERO_SPEED;

    OCR3A = timer_period;
    // Check  if we need to reset the timer...
    if (TCNT3 > OCR3A)
        TCNT3 = 0;
}

// ================================================================
// ===      Enable stepper drivers and TIMER interrupts         ===
// ================================================================
void enableMotors()
{
    digitalWrite(4, LOW); // Enable stepper drivers
    // Enable TIMERs interrupts
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
    TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

// ================================================================
// ===      Disbale stepper drivers and TIMER interrupts        ===
// ================================================================
void disableMotors()
{
    digitalWrite(4, HIGH); // Disbale motors
    // Disbale TIMERs interrupts
    TIMSK1 &= ~(1 << OCIE1A); // Disbale Timer1 interrupt
    TIMSK3 &= ~(1 << OCIE1A); // Disbale Timer1 interrupt
}

// ================================================================
// ===     Set speed of Stepper Motor2                          ===
// ================================================================
float updatePID(float actualAngle, float targetAngle, float offset,
                float turning, float dt)
{
    float restAngle = targetAngle;
#if 0
    /* Brake */
    if (steerStop)
    {
        int32_t wheelPosition = (motor1Counter + motor2Counter) / 16;
        int32_t positionError = wheelPosition - targetPosition;

        if (backToSpot)
        {
            if (abs(positionError) > zoneA) // Inside zone A
                restAngle -= (float)positionError / positionScaleA;
            else if (abs(positionError) > zoneB) // Inside zone B
                restAngle -= (float)positionError / positionScaleB;
            else if (abs(positionError) > zoneC) // Inside zone C
                restAngle -= (float)positionError / positionScaleC;
            else // Inside zone D
                restAngle -= (float)positionError / positionScaleD;
        }
        else
        {
            if (abs(positionError) < zoneC)
                restAngle -= (float)positionError / positionScaleD;
            else
                targetPosition = wheelPosition;
        }
        restAngle -= (float)wheelVelocity / velocityScaleStop;

        restAngle = constrain(restAngle, targetAngle - 10,
                              targetAngle + 10); // Limit rest Angle
    }
    /* Drive forward and backward */
    else
    {
        if ((offset > 0 && wheelVelocity < 0) ||
            (offset < 0 && wheelVelocity > 0) ||
            offset == 0) // Scale down offset at high speed - wheel velocity is
                         // negative when driving forward and positive when
                         // driving backward
            offset += (float)wheelVelocity /
                      velocityScaleMove; // We will always compensate if the
                                         // offset is 0, but steerStop is not
                                         // set
        restAngle -= offset;
    }
#endif

    // Don't change restAngle with more than 1 degree in each loop
    restAngle = constrain(restAngle, lastRestAngle - 1, lastRestAngle + 1);
    lastRestAngle = restAngle;

    /* Update PID values */
    float error = restAngle - actualAngle;
    float pTerm = P * error;
    iTerm += I * 100.0f * error * dt; // Multiplication with Ki is done
                                      // before integration limit, to make
                                      // it independent from integration
                                      // limit value

    // Limit the integrated error - prevents windup
    iTerm = constrain(iTerm, -100.0f, 100.0f);
    float dTerm = (D / 100.0f) * (error - lastError) / dt;
    lastError = error;
    PIDValue = pTerm + iTerm + dTerm;

#if 0
    /* Steer robot sideways */
    if (turning < 0)
    { // Left

        // Scale down at high speed
        turning += abs((float)wheelVelocity / velocityScaleTurning);
        if (turning > 0)
            turning = 0;
    }
    else if (turning > 0)
    { // Right

        // Scale down at high speed
        turning -= abs((float)wheelVelocity / velocityScaleTurning);
        if (turning < 0)
            turning = 0;
    }
#endif

    return constrain(PIDValue, -MAX_ACCEL, MAX_ACCEL);
}

// ================================================================
// ===      LEDブリンク                                         ===
// ================================================================
void ledBlink()
{
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt =
    false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // 標準シリアルポートの初期化
    Serial.begin(115200);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(
        F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read())
        ; // empty buffer
    while (!Serial.available())
        ; // wait for data
    while (Serial.available() && Serial.read())
        ; // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F(
            "Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to
        // use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    Serial.println(F("Initializing Motors..."));
    initializeMotors();
    enableMotors();

    P = ctrl_data.getPValue();
    I = ctrl_data.getIValue();
    D = ctrl_data.getDValue();
    targetAngle = ctrl_data.getTargetAngle();
    targetOffset = ctrl_data.getTargetOffset();
    turningOffset = ctrl_data.getTurningOffset();
    backToSpot = ctrl_data.getBackToSpotFlag();

    delay(15000);
    // Little motor vibration and servo move to indicate that robot is ready
    for (uint8_t k = 0; k < 5; k++)
    {
        setMotorSpeedM1(5);
        setMotorSpeedM2(5);
        delay(200);
        setMotorSpeedM1(-5);
        setMotorSpeedM2(-5);
        delay(200);
        ledBlink();
    }

    pidTimer = millis();
    encoderTimer = pidTimer;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    uint32_t timer_value;
    command_type command;

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }

    switch (ctrl_data.readMessage())
    {
        case UPDATED_CFG_DATA:
            P = ctrl_data.getPValue();
            I = ctrl_data.getIValue();
            D = ctrl_data.getDValue();
            targetAngle = ctrl_data.getTargetAngle();
            targetOffset = ctrl_data.getTargetOffset();
            turningOffset = ctrl_data.getTurningOffset();
            backToSpot = ctrl_data.getBackToSpotFlag();
            break;

        case FORWARD_LEFT_CTRL:
            steerStop = false;
            targetOffset = ctrl_data.getTargetOffset();
            turningOffset = ctrl_data.getTurningOffset();
            break;

        case LEFT_CTRL:
            steerStop = false;
            targetOffset = 0.0f;
            turningOffset = ctrl_data.getTurningOffset();
            break;

        case BACKWARD_LEFT_CTRL:
            steerStop = false;
            targetOffset = -1.0f * ctrl_data.getTargetOffset();
            turningOffset = ctrl_data.getTurningOffset();
            break;

        case FORWARD_CTRL:
            steerStop = false;
            targetOffset = ctrl_data.getTargetOffset();
            turningOffset = 0.0f;
            break;

        case BACKWARD_CTRL:
            steerStop = false;
            targetOffset = -1.0f * ctrl_data.getTargetOffset();
            turningOffset = 0.0f;
            break;

        case FORWARD_RIGHT_CTRL:
            steerStop = false;
            targetOffset = ctrl_data.getTargetOffset();
            turningOffset = -1.0f * ctrl_data.getTurningOffset();
            break;

        case RIGHT_CTRL:
            steerStop = false;
            targetOffset = 0.0f;
            turningOffset = -1.0f * ctrl_data.getTurningOffset();
            break;

        case BACKWARD_RIGHT_CTRL:
            steerStop = false;
            targetOffset = -1.0f * ctrl_data.getTargetOffset();
            turningOffset = -1.0f * ctrl_data.getTurningOffset();
            break;

        case STOP_CTRL:
            steerStop = true;
            targetOffset = 0.0f;
            turningOffset = 0.0f;
            break;

        case POSTURE_REQ:
            ctrl_data.sendMessage(yaw, pitch, roll, PIDValue, wheelVelocity);
            break;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too
    // inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen
        // frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw = ypr[0] * 180 / M_PI;
        roll = ypr[1] * 180 / M_PI;
        pitch = ypr[2] * 180 / M_PI;

        timer_value = millis();
        if ((layingDown &&
             (pitch < (targetAngle - 10) || pitch > (targetAngle + 10))) ||
            (!layingDown &&
             (pitch < (targetAngle - 45) || roll > (targetAngle + 45))))

        {
            layingDown = true;

            digitalWrite(4, HIGH); // Disable stepper drivers
            //            disableMotors(); // Disbale motors
            dir_M1 = 0;
            dir_M2 = 0;
            motor1Counter = 0;
            motor2Counter = 0;
        }
        else
        {
            layingDown = false;

            float accel =
                updatePID(pitch, targetAngle, targetOffset, turningOffset,
                          (float)(timer_value - pidTimer) / 1000000.0f);
            digitalWrite(4, LOW); // Enable stepper drivers
                                  //            enableMotors();
            //       setMotorSpeedM1(ctrl_data.getTurningLimit());
            //       setMotorSpeedM2(ctrl_data.getTurningLimit());

            setMotorSpeedM1(motor1);
            setMotorSpeedM2(motor1);
            Serial.print(F("("));
            Serial.print(motor1);
            Serial.print(F(","));
            Serial.print(motor2);
            Serial.println(F(")"));
            //            setMotorSpeedM1(ctrl_data.getTurningLimit());
            //            setMotorSpeedM2(ctrl_data.getTurningLimit());
            ledBlink();
        }
        pidTimer = timer_value;

        // Update encoder values every 100ms
        timer_value = millis();
        if (timer_value - encoderTimer >= 100)
        {
            encoderTimer = timer_value;

            int32_t wheelPosition = (motor1Counter + motor2Counter) / 16;
            wheelVelocity = wheelPosition - lastWheelPosition;
            lastWheelPosition = wheelPosition;

            if (abs(wheelVelocity) <= 40 && !stopped)
            { // Set new targetPosition if braking
                targetPosition = wheelPosition;
                stopped = true;
            }
        }

        // blink LED to indicate activity
        //        blinkState = !blinkState;
        //        digitalWrite(LED_PIN, blinkState);
    }
}
