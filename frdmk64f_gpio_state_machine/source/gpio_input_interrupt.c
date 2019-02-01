/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* LED RED definitions*/
#define BOARD_LED_GPIO_RED BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN_RED BOARD_LED_RED_GPIO_PIN

/* LED BLUE definitions*/
#define BOARD_LED_GPIO_BLUE BOARD_LED_BLUE_GPIO
#define BOARD_LED_GPIO_PIN_BLUE BOARD_LED_BLUE_GPIO_PIN

/* LED GREEN definitions*/
#define BOARD_LED_GPIO_GREEN BOARD_LED_GREEN_GPIO
#define BOARD_LED_GPIO_PIN_GREEN BOARD_LED_GREEN_GPIO_PIN

/* SW3 definitions*/
#define BOARD_SW_GPIO_SW3 BOARD_SW3_GPIO
#define BOARD_SW_PORT_SW3 BOARD_SW3_PORT
#define BOARD_SW_GPIO_PIN_SW3 BOARD_SW3_GPIO_PIN
#define BOARD_SW_IRQ_SW3 BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER_SW3 BOARD_SW3_IRQ_HANDLER
#define BOARD_SW_NAME_SW3 BOARD_SW3_NAME

/* SW2 definitions*/
#define BOARD_SW_GPIO_SW2 BOARD_SW2_GPIO
#define BOARD_SW_PORT_SW2 BOARD_SW2_PORT
#define BOARD_SW_GPIO_PIN_SW2 BOARD_SW2_GPIO_PIN
#define BOARD_SW_IRQ_SW2 BOARD_SW2_IRQ
#define BOARD_SW_IRQ_HANDLER_SW2 BOARD_SW2_IRQ_HANDLER
#define BOARD_SW_NAME_SW2 BOARD_SW2_NAME


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW button is pressed */
volatile bool g_ButtonPress_SW3 = false;
volatile bool g_ButtonPress_SW2 = false;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Interrupt service fuction of switch 3.
 *
 * This function toggles the LED
 */
void BOARD_SW_IRQ_HANDLER_SW3(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO_SW3, 1U << BOARD_SW_GPIO_PIN_SW3);
    /* Change state of button. */
    PRINTF("\r\n H_SW3\r\n");
    g_ButtonPress_SW3 = true;
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Interrupt service fuction of switch 2.
 *
 * This function toggles the LED
 */
void BOARD_SW_IRQ_HANDLER_SW2(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO_SW2, 1U << BOARD_SW_GPIO_PIN_SW2);
    /* Change state of button. */
    PRINTF("\r\n H_SW2\r\n");
    g_ButtonPress_SW2 = true;
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

enum states { CYAN_ON, MAGENTA_ON, YELLOW_ON, RED_ON, GREEN_ON, BLUE_ON, MAX_STATES } current_state;
enum events { SW2_ON, SW3_ON, MAX_EVENTS } new_event;

void state_cyan_sw2(void) {
	printf("\r\n cyan sw 2\r\n");
	current_state = MAGENTA_ON;
}

void state_cyan_sw3(void) {
	printf("\r\n cyan sw 3\r\n");
	current_state = RED_ON;
}

void state_magenta_sw2(void) {
	printf("\r\n magenta sw 2\r\n");
	current_state = YELLOW_ON;
}

void state_magenta_sw3(void) {
	printf("\r\n magenta sw 3\r\n");
	current_state = GREEN_ON;
}

void state_yellow_sw2(void) {
	printf("\r\n yellow sw 2\r\n");
	current_state = CYAN_ON;
}

void state_yellow_sw3(void) {
	printf("\r\n yellow sw 3\r\n");
	current_state = BLUE_ON;
}
void state_red_sw2(void) {
	printf("\r\n red sw 2\r\n");
	current_state = GREEN_ON;
}

void state_red_sw3(void) {
	printf("\r\n red sw 3\r\n");
	current_state = CYAN_ON;
}

void state_green_sw2(void) {
	printf("\r\n green sw 2\r\n");
	current_state = BLUE_ON;
}

void state_green_sw3(void) {
	printf("\r\n green sw 3\r\n");
	current_state = MAGENTA_ON;
}

void state_blue_sw2(void) {
	printf("\r\n blue sw 2\r\n");
	current_state = RED_ON;
}

void state_blue_sw3(void) {
	printf("\r\n blue sw 3\r\n");
	current_state = YELLOW_ON;

}


void (*const state_table [MAX_STATES][MAX_EVENTS]) (void) = {

    { state_cyan_sw2, state_cyan_sw3 }, /* procedures for state 1 */
    { state_magenta_sw2, state_magenta_sw3 }, /* procedures for state 1 */
    { state_yellow_sw2, state_yellow_sw3 }, /* procedures for state 1 */
    { state_red_sw2, state_red_sw3 }, /* procedures for state 1 */
    { state_green_sw2, state_green_sw3 }, /* procedures for state 1 */
    { state_blue_sw2, state_blue_sw3 } /* procedures for state 1 */
};


/*!
 * @brief Main function
 */
int main(void)
{
	int count = 0;
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };

    /* Define the init structure for the output LED pin */
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Print a note to terminal. */
    PRINTF("\r\n GPIO Driver example\r\n");
    PRINTF("\r\n Press %s to turn on/off a LED \r\n", BOARD_SW_NAME_SW3);

/* Init input switch 2 and 3 GPIO. */
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    GPIO_SetPinInterruptConfig(BOARD_SW_GPIO_SW3, BOARD_SW_GPIO_PIN_SW3, kGPIO_InterruptFallingEdge);
    GPIO_SetPinInterruptConfig(BOARD_SW_GPIO_SW2, BOARD_SW_GPIO_PIN_SW2, kGPIO_InterruptFallingEdge);
#else
    PORT_SetPinInterruptConfig(BOARD_SW_PORT_SW3, BOARD_SW_GPIO_PIN_SW3, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_SW_PORT_SW2, BOARD_SW_GPIO_PIN_SW2, kPORT_InterruptFallingEdge);
#endif
    EnableIRQ(BOARD_SW_IRQ_SW3);
    GPIO_PinInit(BOARD_SW_GPIO_SW3, BOARD_SW_GPIO_PIN_SW3, &sw_config);
    EnableIRQ(BOARD_SW_IRQ_SW2);
    GPIO_PinInit(BOARD_SW_GPIO_SW2, BOARD_SW_GPIO_PIN_SW2, &sw_config);


    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO_RED, BOARD_LED_GPIO_PIN_RED, &led_config);
    GPIO_PinInit(BOARD_LED_GPIO_BLUE, BOARD_LED_GPIO_PIN_BLUE, &led_config);
    GPIO_PinInit(BOARD_LED_GPIO_GREEN, BOARD_LED_GPIO_PIN_GREEN, &led_config);

    /* Turn Off LEDs before start */
    GPIO_PortSet(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
    GPIO_PortSet(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
    GPIO_PortSet(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);

    new_event = RED_ON;
    current_state = SW2_ON;
    while (1)
    {
        if (g_ButtonPress_SW3)
        {
            PRINTF(" %s is pressed \r\n", BOARD_SW_NAME_SW3);
            /* Toggle LED. */
            GPIO_PortToggle(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
            /* Reset state of button. */
            g_ButtonPress_SW3 = false;
            count++;
        }
        if (g_ButtonPress_SW2)
        {
            PRINTF(" %s is pressed \r\n", BOARD_SW_NAME_SW2);
            /* Toggle LED. */
            GPIO_PortToggle(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
            /* Reset state of button. */
            g_ButtonPress_SW2 = false;
            count++;
        }
        if( count == 4 )
        {
			PRINTF(" Counter reached 4!\r\n");
			GPIO_PortSet(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
			GPIO_PortSet(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
			/* Toggle LED. */
			GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
			delay();
			delay();
			delay();
			GPIO_PortSet(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
			/* Reset state of button. */
			count = 0;
        }

    }
}
