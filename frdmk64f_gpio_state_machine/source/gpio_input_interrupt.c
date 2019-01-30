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
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

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

/*!
 * @brief Main function
 */
int main(void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config_sw3 = {
        kGPIO_DigitalInput, 0,
    };

    gpio_pin_config_t sw_config_sw2 = {
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
    GPIO_PinInit(BOARD_SW_GPIO_SW3, BOARD_SW_GPIO_PIN_SW3, &sw_config_sw3);
    EnableIRQ(BOARD_SW_IRQ_SW2);
    GPIO_PinInit(BOARD_SW_GPIO_SW2, BOARD_SW_GPIO_PIN_SW2, &sw_config_sw2);


    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);


    while (1)
    {
        if (g_ButtonPress_SW3)
        {
            PRINTF(" %s is pressed \r\n", BOARD_SW_NAME_SW3);
            /* Toggle LED. */
            GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
            /* Reset state of button. */
            g_ButtonPress_SW3 = false;
        }
        if (g_ButtonPress_SW2)
        {
            PRINTF(" %s is pressed \r\n", BOARD_SW_NAME_SW2);
            /* Toggle LED. */
            GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
            /* Reset state of button. */
            g_ButtonPress_SW2 = false;
        }

    }
}
