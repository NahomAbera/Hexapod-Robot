/*
 * i2c_driver.cpp
 */

#include <stdint.h>
#include <math.h>
#include "driverlib.h"
#include "delaySys.h"
#include "i2c_driver.h"


/* Read a byte from I2C */
uint8_t i2c_read8(uint8_t ui8Reg)
{
    /* Wait until ready */
    while (MAP_I2C_isBusBusy(EUSCI_B0_BASE));

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, DRIVER_I2C_ADDR);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE,ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT)) & EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
        return(0xFF);
    }

    /* Turn off TX and generate RE-Start */
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);

    /* Wait for start bit to complete */
    while(MAP_I2C_masterIsStartSent(EUSCI_B0_BASE));

    if((MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT)) & EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
        return(0xFF);
    }

    /* Read one more byte */
    /* If reading 1 byte (or last byte), generate the stop to meet the spec */
    uint8_t data = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_B0_BASE);

    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
    return(data);
}

/* Write a byte to I2C */
uint8_t i2c_write8(uint8_t ui8Reg, uint8_t data)
{
    /* Wait until ready to write */
    while (MAP_I2C_isBusBusy(EUSCI_B0_BASE));

    /* Load device slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, DRIVER_I2C_ADDR);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE,ui8Reg);

    /* Wait for tx to complete */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Check if slave ACK/NACK */
    if((MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT)) & EUSCI_B_I2C_NAK_INTERRUPT)
    {
        /* If NACK, set stop bit and exit */
        MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
        return(false);
    }

    /* Now write one more data byte */
    /* Wait for next INT */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* Send the next byte */
    MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, data);

    /* Wait for next INT */
    while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    /* we are done */
    MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
    return(true);
}

/* Initialize the PWM Driver */
void driver_init(void) {
    driver_reset();
    // set the default frequency
    driver_setPWMFreq(1000);
}

/* Send a reset command to the driver via I2C */
void driver_reset(void)
{
  i2c_write8(DRIVER_MODE1, 0x80);
  delaySys(10);
}

/* Set the PWM Driver frequency */
void driver_setPWMFreq(float freq)
{
  freq = freq * 0.9; //correct for an error
  float prescale_value = 25000000;
  prescale_value /= 4096;
  prescale_value /= freq;
  prescale_value -= 1;

  uint8_t prescale = floor(prescale_value + 0.5);

  uint8_t mode0 = i2c_read8(DRIVER_MODE1);
  uint8_t mode1 = (mode0&0x7F) | 0x10; // sleep
  i2c_write8(DRIVER_MODE1, mode1); // go to sleep
  i2c_write8(DRIVER_PRESCALE, prescale); // set the prescaler
  i2c_write8(DRIVER_MODE1, mode0);
  delaySys(5);
  i2c_write8(DRIVER_MODE1, mode0 | 0xa0);  //  This sets the MODE1 register to turn on auto increment.
}

/* Set the PWM output of one of the driver pins */
void driver_setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
  /* Wait until ready to write */
  while (MAP_I2C_isBusBusy(EUSCI_B0_BASE));

  /* Load device slave address */
  MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, DRIVER_I2C_ADDR);

  /* Send start bit and register */
  MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE,DRIVER_LED_ON+4*channel);

  /* Wait for tx to complete */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

  /* Check if slave ACK/NACK */
  if((MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT)) & EUSCI_B_I2C_NAK_INTERRUPT)
  {
      /* If NACK, set stop bit and exit */
      MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
      return(false);
  }

  /* Now write one more data byte */
  /* Wait for next INT */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

  MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, on);
  /* Wait for next INT */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));
  MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, on>>8);
  /* Wait for next INT */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));
  MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, off);
  /* Wait for next INT */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));
  MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, off>>8);
  /* Wait for next INT */
  while(!(MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0) & EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

  /* we are done */
  MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
  return(true);
}

/* Set the output for a PWM Driver pin */
void driver_setPin(uint8_t channel, uint16_t value)
{
    // saturate the value at 4095
    value = ((value) < ((uint16_t)4095) ? (value) : ((uint16_t)4095));

    if (value == 4095)
    {
        // 100% Duty Cylce
        driver_setPWM(channel, 4096, 0);
    }
    else if (value == 0)
    {
        // 0% Duty Cylce
        driver_setPWM(channel, 0, 4096);
    }
    else
    {
        driver_setPWM(channel, 0, value);
    }

}
