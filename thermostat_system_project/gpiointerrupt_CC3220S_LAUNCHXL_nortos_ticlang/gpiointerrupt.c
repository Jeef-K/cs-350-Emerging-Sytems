/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART2_write(uart, &output, x, NULL);
// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t             txBuffer[1];
uint8_t             rxBuffer[2];
I2C_Transaction     i2cTransaction;

// UART Global Variables
char                output[64];
int                 bytesToSend;

// Driver Handles - Global variables
I2C_Handle      i2c;
UART2_Handle     uart;
Timer_Handle    timer0;


volatile uint32_t elapsedTime = 0;
volatile bool checkButtonsFlag = false;
volatile bool checkTemperatureFlag = false;
volatile bool updateUARTFlag = false;

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    elapsedTime += 200000;  // Update elapsed time in microseconds (200ms intervals)

    if (elapsedTime % 200000 == 0) {
        checkButtonsFlag = true;  // Set flag every 200ms
    }
    if (elapsedTime % 500000 == 0) {
        checkTemperatureFlag = true;  // Set flag every 500ms
    }
    if (elapsedTime % 1000000 == 0) {
        updateUARTFlag = true;  // Set flag every 1 second
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
volatile int setPointTemperature = 25; // Initial set-point temperature
volatile unsigned char Button0Flag = 0;

void gpioButtonFxn0(uint_least8_t index) {
    Button0Flag = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
volatile unsigned char Button1Flag = 0;
void gpioButtonFxn1(uint_least8_t index) {
    Button1Flag = 1;  // Set flag for Button 1
}

void initUART(void)
{
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t              i, found;
    I2C_Params          i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}


void initTimer(void)
{
    Timer_Params    params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }

    DISPLAY( snprintf(output, 64, "Timer Configured\n\r"))
}

int16_t readTemp(void)
{
    int     j;
    int16_t temperature = 0;

    i2cTransaction.readCount  = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
 }





void controlHeater(int currentTemperature) {
    if (currentTemperature < setPointTemperature) {
        GPIO_write(CONFIG_GPIO_LED_0, 1);  // Turn on heater (LED)
    } else {
        GPIO_write(CONFIG_GPIO_LED_0, 0);  // Turn off heater (LED)
    }
}



void sendUARTMessage(int currentTemperature, int setPointTemperature, int heaterState, uint32_t seconds) {
    snprintf(output, 64, "<%02d,%02d,%d,%04d>", currentTemperature, setPointTemperature, heaterState, seconds);
    UART2_write(uart, output, strlen(output), NULL);  // Adding the NULL for the callback argument
}




/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {
    int currentTemperature = 0;
    uint32_t secondsSinceReset = 0;

    /* Initialize the GPIO, Timer, I2C, and UART drivers */
    GPIO_init();
    initTimer();
    initI2C();
    initUART();

    /* Configure the LED pins */
        GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);  // LED as output

        /* Configure Button 0 pin with interrupt */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

        /* Configure Button 1 pin with interrupt */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    while (1) {
        if (checkButtonsFlag) {
            checkButtonsFlag = false;

            // Handle Button 0: Increase set-point temperature
            if (Button0Flag) {
                setPointTemperature++;
                Button0Flag = 0;
                DISPLAY(snprintf(output, 64, "Set Point Temperature Increased: %d\n\r", setPointTemperature));
            }

            // Handle Button 1: Decrease set-point temperature
            if (Button1Flag) {
                setPointTemperature--;
                Button1Flag = 0;
                DISPLAY(snprintf(output, 64, "Set Point Temperature Decreased: %d\n\r", setPointTemperature));
            }
        }

        if (checkTemperatureFlag) {
            checkTemperatureFlag = false;
            // Read the temperature sensor (every 500ms)
            currentTemperature = readTemp();
            // Control the heater/LED based on the temperature
            controlHeater(currentTemperature);
        }

        if (updateUARTFlag) {
            updateUARTFlag = false;
            secondsSinceReset++;
            // Send the UART message every 1 second
            int heaterState = (currentTemperature < setPointTemperature) ? 1 : 0;
            sendUARTMessage(currentTemperature, setPointTemperature, heaterState, secondsSinceReset);
        }
    }

    return (NULL);
}


