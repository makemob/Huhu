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
#define FW_VERSION_MINOR (4)



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
#define OUTWARD_ENDSTOP_PORT EXT_1_PORT // Fully-out microswitch
#define OUTWARD_ENDSTOP_PIN EXT_1_PIN
#define POSITION_ENCODER_A_PORT EXT_4_PORT  // A channel of position encoder
#define POSITION_ENCODER_A_PIN EXT_4_PIN
#define POSITION_ENCODER_B_PORT EXT_2_PORT  // B channel of position encoder
#define POSITION_ENCODER_B_PIN EXT_2_PIN
#define POSITION_ENCODER_C_PORT EXT_3_PORT  // C channel of position encoder, shared with inward endstop microswitch
#define POSITION_ENCODER_C_PIN EXT_3_PIN

#define ENABLE_EXT_ADCS (FALSE)    // Set TRUE to use EXT_1 and EXT_2 as analog inputs

#define TEMPERATURE_ADC ADC_CH0
#define BATT_V_ADC      ADC_CH3
#define CURR_SENSE_ADC  ADC_CH5
#define EXT_1_ADC       ADC_CH2
#define EXT_2_ADC       ADC_CH1

#define TEMPERATURE_ADC_IOCON IOCON_PIO0_11
#define CURR_SENSE_ADC_IOCON  IOCON_PIO1_4
#define BATT_V_ADC_IOCON      IOCON_PIO1_2
#define EXT_1_ADC_IOCON       IOCON_PIO1_1
#define EXT_2_ADC_IOCON       IOCON_PIO1_0
#define POSITION_ENCODER_A_IOCON IOCON_PIO0_2
#define POSITION_ENCODER_B_IOCON IOCON_PIO1_0
#define POSITION_ENCODER_C_IOCON IOCON_PIO0_6

#define POSITION_ENCODER_UNCALIBRATED (0x7FFF)   // Value to return when querying uncalibrated position encoder

extern const uint8_t ScarabID[];


void HardwareInit(void);

void HardwarePoll(void);

uint16_t HardwareGetMaxBattVoltage(void);

uint16_t HardwareGetMinBattVoltage(void);

bool HardwareGetInwardEndstop(void);

bool HardwareGetOutwardEndstop(void);

int16_t HardwareGetPositionEncoderCounts(void);

int16_t HardwareGetPositionEncoderDistance(void);  // tenths of a mm

void HardwareZeroPositionEncoder(void);

uint16_t HardwareGetPositionEncoderScaling(void);

void HardwareSetPositionEncoderScaling(uint16_t tenthMillimetrePerCount);

#endif /* HARDWARE_H_ */
