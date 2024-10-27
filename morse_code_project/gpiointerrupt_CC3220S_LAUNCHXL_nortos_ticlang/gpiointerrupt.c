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

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Global variables to keep track of the state machine
volatile bool sosMode = true;        // Default to SOS mode
volatile bool buttonPressed = false; // Flag for button press

// Morse code timings in microseconds
#define DOT_DURATION     500000   // 500 ms
#define DASH_DURATION    1500000  // 1500 ms
#define CHAR_GAP         1500000  // 3 * 500 ms
#define WORD_GAP         3500000  // 7 * 500 ms

// Morse code patterns (durations and LED colors)
typedef struct {
    uint32_t duration;  // Duration of dot/dash/gap
    uint8_t  led;       // LED to turn on: 0 for none, 1 for red, 2 for green
} MorseElement;

// SOS pattern: S (dot dot dot), O (dash dash dash), S (dot dot dot)
MorseElement sosPattern[] = {
    {DOT_DURATION, 1}, {DOT_DURATION, 0}, // S
    {DOT_DURATION, 1}, {DOT_DURATION, 0},
    {DOT_DURATION, 1}, {CHAR_GAP,    0},
    {DASH_DURATION, 2}, {DOT_DURATION, 0}, // O
    {DASH_DURATION, 2}, {DOT_DURATION, 0},
    {DASH_DURATION, 2}, {CHAR_GAP,    0},
    {DOT_DURATION, 1}, {DOT_DURATION, 0}, // S
    {DOT_DURATION, 1}, {DOT_DURATION, 0},
    {DOT_DURATION, 1}, {WORD_GAP,    0}
};
const int sosPatternLength = sizeof(sosPattern) / sizeof(MorseElement);

// OK pattern: O (dash dash dash), K (dash dot dash)
MorseElement okPattern[] = {
    {DASH_DURATION, 2}, {DOT_DURATION, 0}, // O
    {DASH_DURATION, 2}, {DOT_DURATION, 0},
    {DASH_DURATION, 2}, {CHAR_GAP,    0},
    {DASH_DURATION, 2}, {DOT_DURATION, 0}, // K
    {DOT_DURATION, 1},  {DOT_DURATION, 0},
    {DASH_DURATION, 2}, {WORD_GAP,    0}
};
const int okPatternLength = sizeof(okPattern) / sizeof(MorseElement);

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    buttonPressed = true; // Set flag to indicate button was pressed
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    static uint32_t elapsedTime = 0;
    static int patternIndex = 0;

    // Get the current pattern (SOS or OK)
    MorseElement *currentPattern;
    int patternLength;

    if (sosMode) {
        currentPattern = sosPattern;
        patternLength = sosPatternLength;
    } else {
        currentPattern = okPattern;
        patternLength = okPatternLength;
    }

    // Handle button press to toggle message after current message completes
    if (buttonPressed && patternIndex == 0 && elapsedTime == 0) {
        sosMode = !sosMode;  // Toggle between SOS and OK mode
        buttonPressed = false;
        return; // Wait for next cycle to load new pattern
    }

    // Check if we've completed the entire message
    if (patternIndex >= patternLength) {
        patternIndex = 0;     // Reset pattern index to loop the message
        elapsedTime = 0;      // Reset elapsed time

        // Handle button press after message completion
        if (buttonPressed) {
            sosMode = !sosMode;  // Toggle between SOS and OK mode
            buttonPressed = false;
        }
        return;
    }

    // Get the current Morse code element
    MorseElement element = currentPattern[patternIndex];

    // Turn LEDs on or off based on the element
    if (element.led == 1) {
        // Red LED on
        GPIO_write(CONFIG_GPIO_LED_0, 1);
        GPIO_write(CONFIG_GPIO_LED_1, 0);
    } else if (element.led == 2) {
        // Green LED on
        GPIO_write(CONFIG_GPIO_LED_0, 0);
        GPIO_write(CONFIG_GPIO_LED_1, 1);
    } else {
        // Both LEDs off
        GPIO_write(CONFIG_GPIO_LED_0, 0);
        GPIO_write(CONFIG_GPIO_LED_1, 0);
    }

    // Update elapsed time
    elapsedTime += 500000; // Timer callback every 500 ms

    // Check if the duration for the current element is complete
    if (elapsedTime >= element.duration) {
        // Move to the next element
        patternIndex++;
        elapsedTime = 0;
    }
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000; // 500 ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialize timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Initialize the GPIO and Timer drivers */
    GPIO_init();
    initTimer();

    /* Configure the LED pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); // Red LED
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); // Green LED

    /* Configure the button pin with interrupt on falling edge */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable button interrupt */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    return (NULL);
}
