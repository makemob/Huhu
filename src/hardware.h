/*
 * hardware.h
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 *
 *	Configured for V1.0 Scarab PCB
 *	github.com/scarab
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

#define BOARD_VERSION (1)
#define FW_VERSION_MAJOR (0)
#define FW_VERSION_MINOR (3)



#define BLUE_LED_PORT  (0)
#define BLUE_LED_PIN   (7)
#define GREEN_LED_PORT (0)
#define GREEN_LED_PIN  (3)

#define BRIDGE_1_PORT (1)
#define BRIDGE_1_PIN  (8)
#define BRIDGE_2_PORT (1)
#define BRIDGE_2_PIN  (9)
#define BRIDGE_3_PORT (0)
#define BRIDGE_3_PIN  (9)
#define BRIDGE_4_PORT (0)
#define BRIDGE_4_PIN  (8)

#define RS485_TX_EN_PORT (1)
#define RS485_TX_EN_PIN  (5)

#define UART_RX_PIN IOCON_PIO1_6
#define UART_TX_PIN IOCON_PIO1_7

#define EXT_1_PORT    (1)
#define EXT_1_PIN     (1)
#define EXT_2_PORT    (1)
#define EXT_2_PIN     (0)
#define EXT_3_PORT    (0)
#define EXT_3_PIN     (6)
#define EXT_4_PORT    (0)
#define EXT_4_PIN     (2)
#define EXT_5_PORT    GREEN_LED_PORT
#define EXT_5_PIN     GREEN_LED_PIN
#define EXT_6_PORT    BLUE_LED_PORT
#define EXT_6_PIN     BLUE_LED_PIN

#define INWARD_ENDSTOP_PORT EXT_3_PORT  // Fully-in microswitch
#define INWARD_ENDSTOP_PIN EXT_3_PIN
#define OUTWARD_ENDSTOP_PORT EXT_4_PORT // Fully-out microswitch
#define OUTWARD_ENDSTOP_PIN EXT_4_PIN

#define TEMPERATURE_ADC ADC_CH0
#define BATT_V_ADC      ADC_CH3
#define CURR_SENSE_ADC  ADC_CH5
#define EXT_1_ADC       ADC_CH2
#define EXT_2_ADC       ADC_CH1

#define TEMPERATURE_ADC_PIN IOCON_PIO0_11
#define CURR_SENSE_ADC_PIN  IOCON_PIO1_4
#define BATT_V_ADC_PIN      IOCON_PIO1_2
#define EXT_1_ADC_PIN		IOCON_PIO1_1
#define EXT_2_ADC_PIN		IOCON_PIO1_0


extern const uint8_t ScarabID[];


void HardwareInit(void);

void HardwarePoll(void);

uint16_t HardwareGetMaxBattVoltage(void);

uint16_t HardwareGetMinBattVoltage(void);

bool HardwareGetInwardEndstop(void);

bool HardwareGetOutwardEndstop(void);

#endif /* HARDWARE_H_ */
