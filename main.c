/*
 * Hexapod Robot
 * main.c
 */

#include "msp.h"
#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>
#include "delaySys.h"
#include "i2c_driver.h"
#include "config.h"
#include "macros.h"
#include "servo.h"
#include "motions.h"
#include "gaits.h"


volatile uint32_t switchState; // robot state set by MSP push buttons
unsigned long loopTime = 0;

volatile unsigned long state_walk_timer = 0;
volatile uint32_t demo_flag = 0;

uint8_t state = STATE_WALK; // walking state by default

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
    2000000,
    EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
    0,
    EUSCI_B_I2C_NO_AUTO_STOP
};


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer

    /* Clock configuration */
    MAP_CS_setDCOFrequency(16E+6); // Set DCO clock source frequency to 16MHz
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8); // Tie SMCLK to DCO, Set to 2MHz

    /* Enabling FPU */
    MAP_FPU_enableModule();

    MAP_Interrupt_disableMaster(); // Disable all interrupts

    /* Enabling SysTick */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(16000); // Configuring SysTick to trigger at 16000 (MCLK is 16MHz so this will make it toggle every 1 ms)
    MAP_SysTick_enableInterrupt();

    /* GPIO configuration */
    MAP_GPIO_setAsOutputPin(LED1_PIN);
    MAP_GPIO_setAsOutputPin(LEDR_PIN);
    MAP_GPIO_setAsOutputPin(LEDG_PIN);
    MAP_GPIO_setAsOutputPin(LEDB_PIN);
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN1_PIN);
    MAP_GPIO_setAsInputPinWithPullUpResistor(BTN2_PIN);
    MAP_GPIO_clearInterruptFlag(BTN1_PIN);
    MAP_GPIO_enableInterrupt(BTN1_PIN);
    MAP_GPIO_clearInterruptFlag(BTN2_PIN);
    MAP_GPIO_enableInterrupt(BTN2_PIN);
    MAP_Interrupt_enableInterrupt(INT_PORT1);

    MAP_GPIO_setOutputLowOnPin(LED1_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDR_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
    MAP_GPIO_setOutputLowOnPin(LEDB_PIN);

    /* I2C configuration */
    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function, (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL). */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Initializing I2C Master to SMCLK at 400kbs with no autostop */
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);

    MAP_Interrupt_enableMaster();

    // LED Blinking for power-up indication
    MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
    delaySys(300);
    MAP_GPIO_setOutputLowOnPin(LEDR_PIN);
    delaySys(150);
    MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
    delaySys(150);;
    MAP_GPIO_setOutputLowOnPin(LEDR_PIN);

    uint8_t demo_status = 0;

    resetPWMDriver();
    delaySys(250);

    switchState = 0;
    robot_stand();
    delaySys(300);

    while (1)
    {
        checkPWMDriverSleep();
        checkForServosCrash();

        // switchState is set by GPIO interrupt by pressing the push button
        if (switchState == SWSTATE_STAND) // Stand mode
        {
            // blink Red LED stand state
            MAP_GPIO_setOutputHighOnPin(LEDR_PIN);

            delaySys(250);
            robot_stand();

            // print the mode every 1 second for debugging purposes
            if (millisec > loopTime)
            {
                loopTime = millisec + 1000;
                printf("Stand Mode\n");
            }
        }
        else if (switchState == SWSTATE_DEMO) // demo state
        {
            // blink Green LED in demo state
            MAP_GPIO_setOutputHighOnPin(LEDG_PIN);

            demo_status = robot_random(demo_flag);
            demo_flag = 0;

            // print the mode every 1 second for debugging purposes
            if (millisec > loopTime)
            {
                loopTime = millisec + 1000;
                printf("Demo Mode\n");
            }

            if (demo_status == 1)
            {
                switchState = SWSTATE_STAND;
                demo_status = 0;
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDB_PIN);
            }
        }
        else if (switchState == SWSTATE_WALK) // walk state
        {
            MAP_GPIO_setOutputHighOnPin(LEDB_PIN); // Blue LED is set to on in walk state

            if (millisec > loopTime)
            {
                loopTime = millisec + 2000;
                printf("Walk Mode\n");
            }

            state = STATE_WALK;

            // forward
            robot_forward(1, HIP_BACKWARD, HIP_FORWARD, KNEE_TRIPOD_UP+KNEE_TRIPOD_ADJ, KNEE_DOWN, TRIPOD_CYCLE_TIME);

            if (millisec > state_walk_timer + 4000)
            {
                switchState = SWSTATE_STAND;
                state_walk_timer = 0;
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDB_PIN);
            }
        }
        else if (switchState == SWSTATE_ADJUST) // adjust state, put all servos at 90 degrees
        {
            if (led_blink(500)) // blink Red LED in adjust state
                MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
            else
                MAP_GPIO_setOutputLowOnPin(LEDR_PIN);

            delaySys(250);
            robot_stand_90_deg();

            // print the mode every 1 second for debugging purposes
            if (millisec > loopTime)
            {
                loopTime = millisec + 1000;
                printf("Adjust Mode\n");
            }
        }
        else if (switchState == SWSTATE_TEST) // Test each servo one by one
        {
            if (led_blink(500)) // blink Green LED in test state
                MAP_GPIO_setOutputHighOnPin(LEDG_PIN);
            else
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);

            int channel; //variable for for loop
            for (channel = 0; channel < 12; channel++) // 12 Legs
            {
                if (switchState != SWSTATE_TEST)
                {
                    break;
                }

                setPWM(channel, 140);
                delaySys(500);

                if (switchState != SWSTATE_TEST)
                {
                    break;
                }

                setPWM(channel, 40);
                delaySys(500);
                setPWM(channel, 90);
                delaySys(100);
                printf("CHANNEL: %d\n",channel);
            }
        }
        else // error
        {
            // White LED is set to steady on in error
            MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
            MAP_GPIO_setOutputHighOnPin(LEDG_PIN);
            MAP_GPIO_setOutputHighOnPin(LEDB_PIN);
        }
    }
}

void SysTick_Handler(void)
{
    millisec++;
}

/* GPIO ISR */
void PORT1_IRQHandler(void)
{
    uint32_t status;
    static uint32_t last_interrupt_time = 0;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    unsigned long interrupt_time = millisec;
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time > 250)
    {
        if(status & GPIO_PIN1)
        {
            // good time to turn off error led if it is on
            MAP_GPIO_setOutputLowOnPin(LED1_PIN);

            // change the robot mode to demo
            if (switchState != SWSTATE_DEMO)
            {
                switchState = SWSTATE_DEMO;
                MAP_GPIO_setOutputLowOnPin(LEDR_PIN);
                MAP_GPIO_setOutputHighOnPin(LEDG_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDB_PIN);
                demo_flag = 1;
            }
            else
            {
                switchState = SWSTATE_STAND;
                MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDB_PIN);
                demo_flag = 0;
            }
        }
        else if (status & GPIO_PIN4)
        {
            // good time to turn off error led if it is on
            MAP_GPIO_setOutputLowOnPin(LED1_PIN);

            // change the robot mode to walking
            if (switchState != SWSTATE_WALK)
            {
                switchState = SWSTATE_WALK;
                MAP_GPIO_setOutputLowOnPin(LEDR_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
                MAP_GPIO_setOutputHighOnPin(LEDB_PIN);

                state_walk_timer = millisec;
            }
            else
            {
                switchState = SWSTATE_STAND;
                MAP_GPIO_setOutputHighOnPin(LEDR_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDG_PIN);
                MAP_GPIO_setOutputLowOnPin(LEDB_PIN);
            }
        }

        last_interrupt_time = interrupt_time;
    }
}
