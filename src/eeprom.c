/*
 * eeprom.c
 *
 *  Created on: 26 Jun 2016
 *
 *  https://github.com/jdesbonnet/LPC8xx_Flash_EEPROM
 *
 */

#include "chip.h"
#include "iap_driver.h"

// Allocate a 64 byte aligned 64 byte block in flash memory for "EEPROM" storage
const uint8_t eeprom_flashpage[64] __attribute__ ((aligned (64))) = {0};

int32_t eeprom_write (uint8_t *data) {

	uint32_t iap_status;

	// iap_init();

	uint32_t flash_page = (uint32_t)&eeprom_flashpage >> 6;
	uint32_t flash_sector = flash_page >> 4;

	// Example code checks MCU part ID, bootcode revision number and serial number. There are some
	// differences in behavior across silicon revisions (in particular to do with ability
	// to erase multiple sectors at the same time). In this case we require to be able to program
	// just one sector, so this does not concern us.

	/* Prepare the page for erase */
	iap_status = (__e_iap_status) iap_prepare_sector(flash_sector, flash_sector);
	if (iap_status != CMD_SUCCESS) return -4;

	/* Erase the page */
	iap_status = (__e_iap_status) iap_erase_page(flash_page, flash_page);
	if (iap_status != CMD_SUCCESS) return -5;

	/* Prepare the page for writing */
	iap_status = (__e_iap_status) iap_prepare_sector(flash_sector, flash_sector);
	if (iap_status != CMD_SUCCESS) return -6;

	/* Write data to page */
	iap_status = (__e_iap_status) iap_copy_ram_to_flash(data,
			(void *)&eeprom_flashpage, 64);
	if (iap_status != CMD_SUCCESS) return -7;

	return 0;
}
