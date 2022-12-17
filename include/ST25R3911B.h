#ifndef ST25R3911B_H
#define ST25R3911B_H

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define ST25R3911B_PIN_MOSI CONFIG_ST25R3911B_PIN_MOSI
#define ST25R3911B_PIN_MISO CONFIG_ST25R3911B_PIN_MISO
#define ST25R3911B_PIN_SCLK CONFIG_ST25R3911B_PIN_SCLK
#define ST25R3911B_PIN_CSEN CONFIG_ST25R3911B_PIN_CSEN

/**
 * Command Modes
 */
#define ST25R3911B_CMD_REG_READ 0b01
#define ST25R3911B_CMD_REG_WRITE 0b00
#define ST25R3911B_CMD_FIFO_LOAD 0b10
#define ST25R3911B_CMD_FIFO_READ 0b10
#define ST25R3911B_CMD_DIRECT_COMMAND 0b11

/**
 * Registers
 */
#define ST25R3911B_REG_IO1 0x00
#define ST25R3911B_REG_IO2 0x01
#define ST25R3911B_REG_OP_CONTROL 0x02
#define ST25R3911B_REG_MODE 0x03
#define ST25R3911B_REG_BITRATE 0x04
#define ST25R3911B_REG_MASK_IRQ_MAIN 0x14
#define ST25R3911B_REG_MASK_IRQ_NFC 0x15
#define ST25R3911B_REG_IRQ_MAIN 0x17
#define ST25R3911B_REG_IRQ_NFC 0x18
#define ST25R3911B_REG_IC_IDENTITY 0x3F
#define ST25R3911B_REG_FIFO_STATUS_1 0x1A
#define ST25R3911B_REG_FIFO_STATUS_2 0x1B
#define ST25R3911B_REG_COLLISION_DISPLAY 0x1B
#define ST25R3911B_REG_TX_BYTES_1 0x1D
#define ST25R3911B_REG_TX_BYTES_2 0x1E

#define ST25R3911B_BIT_OP_CONTROL_RX_EN (1 << 6)
#define ST25R3911B_BIT_OP_CONTROL_TX_EN (1 << 3)

#define ST25R3911B_BIT_IRQ_MAIN_RXE (1 << 4)

/**
 * Direct Commands
 */
#define ST25R3911B_DCMD_SET_DEFAULT 0xC1
#define ST25R3911B_DCMD_CLEAR1 0xC2
#define ST25R3911B_DCMD_CLEAR2 0xC3
#define ST25R3911B_DCMD_TRANSMIT_CRC 0xC4
#define ST25R3911B_DCMD_TRANSMIT_NO_CRC 0xC5
#define ST25R3911B_DCMD_ADJUST_REGULATORS 0xD6
#define ST25R3911B_DCMD_NF_FIELD_ON 0xC8
#define ST25R3911B_DCMD_SEND_REQA 0xC6
#define ST25R3911B_DCMD_SEND_WUPA 0xC7
#define ST25R3911B_DCMD_ANALOG_PRESET 0xCC
#define ST25R3911B_DCMD_ADJUST_REGULATORS 0xD6

typedef struct
{
    spi_device_handle_t handle;
} st25r3911b_t;

uint8_t st25r3911b_read_register(st25r3911b_t *dev, uint8_t addr);
void st25r3911b_write_register(st25r3911b_t *dev, uint8_t addr, uint8_t data);
void st25r3911b_direct_command(st25r3911b_t *dev, uint8_t command);

void st25r3911b_init(st25r3911b_t *dev);
bool st25r3911b_check(st25r3911b_t *spi);
void st25r3911b_powerup(st25r3911b_t *spi);
void st25r3911b_set_mode_nfca(st25r3911b_t *dev);

// iso14443a compatible functions
void st25r3911b_transmit_reqa(void *dev_ptr);
void st25r3911b_transmit_crc(void *dev_ptr, uint16_t bytes, uint8_t *data);
void st25r3911b_transmit_no_crc(void *dev_ptr, uint16_t bytes, uint8_t *data);
uint16_t st25r3911b_receive(void *dev_ptr, uint16_t bufsize, uint8_t *buf);

#endif // ST25R3911B_H