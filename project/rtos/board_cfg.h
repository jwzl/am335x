/*
* board_cfg.h
*/
#ifndef BOARD_CFG_H_
#define BOARD_CFG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Board ID information */
#define BOARD_INFO_CPU_NAME     "AM335x"
#define BOARD_INFO_BOARD_NAME   "EVMAM335x"

/* Mmeory Sections */
#define BOARD_DDR3_START_ADDR           0x80010000
#define BOARD_DDR3_SIZE                 ((1 * 1024 * 1024 * 1024UL) - 0x00010000)
#define BOARD_DDR3_END_ADDR             (BOARD_DDR3_START_ADDR + BOARD_DDR3_SIZE - 1)


/* UART LLD instance number */
#define BOARD_UART_INSTANCE				0

/* EEPROM Data read length */
#define I2C_EEPROM_RX_LENGTH            10U

/* Port and pin number mask for MMCSD Card Detect pin.
   Bits 7-0: Pin number  and Bits 15-8: (Port number + 1) */
#define GPIO_MMC_SDCD_PIN_NUM          (0x6)
#define GPIO_MMC_SDCD_PORT_NUM         (0x1)
#define GPIO_PIN_MMC_SDCD_ACTIVE_STATE (0x0)

/* I2C instance connected to EEPROM */
#define BOARD_I2C_EEPROM_INSTANCE       0

/* I2C address for EEPROM */
#define BOARD_I2C_EEPROM_ADDR           (0x50)

/* EEPROM board ID information */
#define BOARD_EEPROM_HEADER_LENGTH      4
#define BOARD_EEPROM_BOARD_NAME_LENGTH  8
#define BOARD_EEPROM_VERSION_LENGTH     4
#define BOARD_EEPROM_SERIAL_NO_LENGTH   12
#define BOARD_EEPROM_HEADER_ADDR        0
#define BOARD_EEPROM_BOARD_NAME_ADDR    (BOARD_EEPROM_HEADER_ADDR + BOARD_EEPROM_HEADER_LENGTH)
#define BOARD_EEPROM_VERSION_ADDR       (BOARD_EEPROM_BOARD_NAME_ADDR + BOARD_EEPROM_BOARD_NAME_LENGTH)
#define BOARD_EEPROM_SERIAL_NO_ADDR     (BOARD_EEPROM_VERSION_ADDR + BOARD_EEPROM_VERSION_LENGTH)

#define CPLD_CTRLREG_OFFSET             (0x10)
#define CPLD_I2C_ADDR                   (0x35)

#define AM335x_GPEVM_PROFILE_2          (2)

/* Instance for interfaces connected to MMCSD */
#define BOARD_MMCSD_SD_INSTANCE         (0)

#ifdef __cplusplus
}
#endif

#endif
