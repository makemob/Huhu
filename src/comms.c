/*
 * comms.c
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */

#include "chip.h"
#include "hardware.h"
#include <string.h>
#include "timer.h"

/* Transmit and receive ring buffers */
STATIC volatile RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 256	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

/* Transmit and receive buffers */
static volatile uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

static uint8_t txEnableDelay = 0;   //  Keep tx enable line asserted for txEnableDelay ms after transmission
static volatile uint8_t triggerTxEnableDelay = 0;     // Signaling from interrupt to main loop
static uint8_t lastTriggerTxEnableDelay = 0;

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART_IRQHandler(void)
{
	uint32_t id = Chip_UART_ReadIntIDReg(LPC_USART);

	if ((id & UART_IIR_INTID_RLS) || (id & UART_IIR_INTID_RDA)) {
		/* Use default ring buffer handler. Override this with your own
		   code if you need more capability. */
		Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
	}

	if ((id & UART_IIR_INTID_THRE) && RingBuffer_IsEmpty(&txring)) {
		// Clear tx enable
		if (txEnableDelay) {
			triggerTxEnableDelay++;  // Signal back to main loop (Timer code not interrupt safe)
		} else {
			Chip_GPIO_SetPinOutLow(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
			Chip_GPIO_SetPinOutLow(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
		}
	}
}

void CommsInit(uint32_t baud) {

	/*** UART Init ***/
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, baud);
	// Chip_UART_SetRS485Flags(LPC_USART, (UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1)); // Doesn't appear to work..
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);


	RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT | UART_IER_THREINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);
}

void CommsSendString(uint8_t* str) {

	// Assert tx enable (needs to happen before tx triggered or start bit can be truncated)
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);

	if (!Chip_UART_SendRB(LPC_USART, &txring, str, strlen(str))) {
		// Abort tx if buffer full, as packet will be corrupt
		Chip_GPIO_SetPinOutLow(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
	}
}

void CommsSendChars(uint8_t* str, uint8_t numChars) {

	// Assert tx enable (needs to happen before tx triggered or start bit can be truncated)
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);

	if (!Chip_UART_SendRB(LPC_USART, &txring, str, numChars)) {
		// Abort tx if buffer full, as packet will be corrupt
		Chip_GPIO_SetPinOutLow(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
	}
}

int16_t CommsReadKey(uint8_t *result) {
	return(Chip_UART_ReadRB(LPC_USART, &rxring, result, 1));
}

int16_t CommsReadChars(uint8_t *buffer, uint8_t bufferSize) {
	return(Chip_UART_ReadRB(LPC_USART, &rxring, buffer, bufferSize));
}

uint8_t CommsGetNumRxChars(void) {
	return(RingBuffer_GetCount(&rxring));
}

void CommsFlushRxBuffer(void) {
	RingBuffer_Flush(&rxring);
}

// Keep tx enable line asserted for delay milliseconds after transmission
void CommsSetTxEnableDelay(uint8_t delay) {
	txEnableDelay = delay;
}

void CommsPoll(void) {

	// Pick up signal from interrupt, trigger tx enable delay
	if (triggerTxEnableDelay != lastTriggerTxEnableDelay) {
		TimerSetDurationMs(TIMER_TX_ENABLE_DELAY, txEnableDelay);
		lastTriggerTxEnableDelay = triggerTxEnableDelay;

	// Deassert tx enable once delay over
	} else if (TimerCheckExpired(TIMER_TX_ENABLE_DELAY)) {
		Chip_GPIO_SetPinOutLow(LPC_GPIO, RS485_TX_EN_PORT, RS485_TX_EN_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
	}
}
