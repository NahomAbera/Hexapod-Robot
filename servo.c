/*
 * servo.c
 */

#include "driverlib.h"
#include "delaySys.h"
#include "i2c_driver.h"
#include "servo.h"
#include "motions.h"
#include "config.h"

uint8_t checkServos = 0;
int servosDisconnected = 0;
short MotorPos[12];


// set PWM for specific servo position
void setPWM(int channel, int pos)
{
    MotorPos[channel] = pos;

    // convert the position to pulse length
    int p = pos * (SERVO_MAX_PULSE_LEN - SERVO_MIN_PULSE_LEN) / 180 + SERVO_MIN_PULSE_LEN;

    if (!checkServos)
    {
        driver_setPWM(channel, 0, p);
    }
}

void checkServosFirst()
{
    checkServos = 1;
}

void setServosPWM()
{
    checkForServosCrash();
    checkServos = 0;
    int channel;
    for (channel = 0; channel < 12; channel++) // 12 servo motors
    {
        setPWM(channel, MotorPos[channel]);
    }
}

// reset the servo PWM driver
void resetPWMDriver()
{
    driver_init();
    driver_setPWMFreq(120);  // 120 Hz Frequency
}

unsigned long sampleTime = 0;

// the servo driver may go to sleep if there is a voltage drop
void checkPWMDriverSleep()
{
    if (millisec > sampleTime)
    {
        // read the servo driver mode to see if it is asleep
        int mode1 = i2c_read8(DRIVER_MODE1);

        // 5th bit indicates sleep
        if (mode1 & 0x10)
        {
            // wake up the driver
            resetPWMDriver();
            // error led
            MAP_GPIO_setOutputLowOnPin(LED1_PIN);
        }
        sampleTime = millisec + 100;
    }
}
