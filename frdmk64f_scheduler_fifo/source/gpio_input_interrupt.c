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

/* Tasks */
void task_red(void);
void tast_green(void);
void task_blue(void);
void task_cyan(void);
void task_magenta(void);
void task_yellow(void);

/* Functions */
void turn_off_leds(void);
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Whether the SW button is pressed */
volatile bool g_ButtonPress_SW3 = false;
volatile bool g_ButtonPress_SW2 = false;

/* Scheduler Variables */
#define MAXTASKS	6
typedef void (*mTaskType)();
mTaskType mTaskArray[MAXTASKS] = { NULL };
int pos = 0;
typedef enum {
	R_OK,
	R_NOT_OK
} Response;

/*******************************************************************************
 * Code
 ******************************************************************************/

Response mTaskCreate(mTaskType newTask){
	if(pos < MAXTASKS){
		mTaskArray[pos] = newTask;
		pos++;
		return R_OK;
	} else {
		return R_NOT_OK;
	}
}

void mSchedulerStart(){
	int localPos = 0;
	while(1){
		if(localPos == MAXTASKS) {
			localPos = 0;
		}
		mTaskArray[localPos]();
		localPos++;

	}
}

void delay(void) {
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void turn_off_leds(void){
	GPIO_PortSet(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	GPIO_PortSet(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	GPIO_PortSet(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
}

void task_red(void) {
	printf("\r\n RED \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	delay();
	delay();
}

void task_green(void) {
	printf("\r\n GREEN \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	delay();
	delay();
}

void task_blue(void) {
	printf("\r\n BLUE \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	delay();
}

void task_cyan(void) {
	printf("\r\n CYAN \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	delay();
}

void task_magenta(void) {
	printf("\r\n MAGENTA \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	delay();
}

void task_yellow(void) {
	printf("\r\n YELLOW \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	delay();
	delay();
}


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
    PRINTF("\r\n RGB/CMY State Machine\r\n");
    PRINTF("\r\n SW2 to switch color in the color space \r\n");
    PRINTF("\r\n SW3 to switch RGB<->CMY \r\n");

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
    turn_off_leds();

    /* Create Tasks */
    if(mTaskCreate(task_red) == R_NOT_OK){
        printf("Error creating task /n/r");
    }
    if(mTaskCreate(task_green) == R_NOT_OK){
        printf("Error creating task /n/r");
    }
    if(mTaskCreate(task_blue) == R_NOT_OK){
        printf("Error creating task /n/r");
    }
    if(mTaskCreate(task_cyan) == R_NOT_OK){
        printf("Error creating task /n/r");
    }
    if(mTaskCreate(task_magenta) == R_NOT_OK){
        printf("Error creating task /n/r");
    }
    if(mTaskCreate(task_yellow) == R_NOT_OK){
        printf("Error creating task /n/r");
    }

    /* Start Scheduler */
    mSchedulerStart();
}
