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
int localPos = 0;
typedef enum {
	R_OK,
	R_NOT_OK
} Response;


typedef struct mTaskNode {
	struct mTaskNode* next;
	struct mTaskNode* prev;
	int value;
	mTaskType func;
}mTaskNode;

struct mTaskNode *head = NULL;
int coolVar = 0;
struct mTaskNode *currentTask = NULL;

/*******************************************************************************
 * Code
 ******************************************************************************/

struct mTaskNode* create_node(mTaskType fnt){
	struct mTaskNode *node = (struct mTaskNode*)malloc(sizeof(struct mTaskNode));
	if(node == NULL){
		return NULL;
	}
	(*node).next = NULL;
	(*node).prev = NULL;
	(*node).value = coolVar;
	(*node).func = fnt;
	coolVar++;
	return node;
}

Response mTaskCreate(mTaskType newTask) {

	struct mTaskNode *node = create_node(newTask);
	if(node == NULL){
		printf("Error creating task.\n\r");
		return R_NOT_OK;
	}

	if(head == NULL){
		/* First node to add */
		head = node;
	} else {
		/* Many more nodes */
		struct mTaskNode *aux = head;
		while( (*aux).next != NULL ){
				aux = (*aux).next;
		}
		(*aux).next = node;
		(*node).prev = aux;
	}

	return R_OK;

}

void mSchedulerStart(){
	currentTask = head;
	while(1){
		printf("N(%p) = %d ( prev(%d) ) \n\r", currentTask, (*currentTask).value, (*(*currentTask).prev).value );
		(*currentTask).func();
		currentTask = (*currentTask).next;
		if( currentTask == NULL ){
			currentTask = head;
		}
	}
	printf("End of scheduler");
}

Response mTaskDelete(struct mTaskNode *task){
	printf("Deleting Value %d \n\r", (*task).value);

	printf("Now (%d) points to (%d) \n\r", (*(*task).prev).value,  (*(*task).next).value);
	printf("Deleting N(%p) = %d \n\r", task, (*task).value );


	if( (*task).next == NULL && (*task).prev == NULL ){
		/* It's the only node */
		printf("only node\n\r");
		head = NULL;
	} else if((*task).next == NULL){
		/* It's the last node */
		printf("last node\n\r");
		(*(*task).prev).next = NULL;
	} else if((*task).prev != NULL ){
		printf("middle node\n\r");
		/* let task's prev next point to task's next */
		(*(*task).prev).next = (*task).next;
		/* let task's next prev point to task's prev */
		(*(*task).next).prev = (*task).prev;
	} else {
		printf("first node\n\r");
		/* It's the first node. Now task's next is the head and its prev points to NULL */
		(*(*task).next).prev = NULL;
		head = (*task).next;
	}

	free(task);

	return R_OK;

}

void delete_this_task(){
	/**/
	mTaskDelete(currentTask);
}

void add_red_task(){
	printf("Add extra RED task /n/r");
	if(mTaskCreate(task_red) == R_NOT_OK){
		printf("Error creating task /n/r");
	}
}

void delay(void) {
    volatile uint32_t i = 0;
    for (i = 0; i < 8000000; ++i)
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
	printf("RED \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	delay();
	turn_off_leds();
}

void task_green(void) {
	printf("GREEN \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	delay();
	turn_off_leds();
}

void task_blue(void) {
	printf("BLUE \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	turn_off_leds();
}

void task_cyan(void) {
	printf("CYAN \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	turn_off_leds();
}

void task_magenta(void) {
	printf("MAGENTA \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	GPIO_PortClear(BOARD_LED_GPIO_BLUE, 1U << BOARD_LED_GPIO_PIN_BLUE);
	delay();
	turn_off_leds();
}

void task_yellow(void) {
	printf("YELLOW \r\n");
	turn_off_leds();
	GPIO_PortClear(BOARD_LED_GPIO_RED, 1U << BOARD_LED_GPIO_PIN_RED);
	GPIO_PortClear(BOARD_LED_GPIO_GREEN, 1U << BOARD_LED_GPIO_PIN_GREEN);
	delay();
	turn_off_leds();
}



/*!
 * @brief Interrupt service fuction of switch 3.
 *
 * This function toggles the LED
 */
void BOARD_SW_IRQ_HANDLER_SW3(void)
{
	DisableIRQ(BOARD_SW_IRQ_SW3);
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO_SW3, 1U << BOARD_SW_GPIO_PIN_SW3);
    /* Change state of button. */
    PRINTF("\r\n H_SW3\r\n");
    g_ButtonPress_SW3 = true;
    delete_this_task();
    EnableIRQ(BOARD_SW_IRQ_SW3);
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
	DisableIRQ(BOARD_SW_IRQ_SW2);
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO_SW2, 1U << BOARD_SW_GPIO_PIN_SW2);
    /* Change state of button. */
    PRINTF("\r\n H_SW2\r\n");
    g_ButtonPress_SW2 = true;
    add_red_task();
    EnableIRQ(BOARD_SW_IRQ_SW2);
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

    return 0;
}

