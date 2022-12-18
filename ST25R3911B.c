#include "./include/ST25R3911B.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

void enable_cs(spi_transaction_t *trans)
{
    gpio_set_level(5, 0U);
}

void disable_cs(spi_transaction_t *trans)
{
    gpio_set_level(5, 1U);
}

void st25r3911b_init(st25r3911b_t *dev)
{
    esp_err_t ret;

    ret = gpio_set_direction(5, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);
    ret = gpio_set_level(ST25R3911B_PIN_CSEN, 1U);
    ESP_ERROR_CHECK(ret);

    gpio_config_t iocfg = {};
    iocfg.mode = GPIO_MODE_INPUT;
    iocfg.intr_type = GPIO_INTR_ANYEDGE;
    iocfg.pull_down_en = false;
    iocfg.pull_down_en = false;
    iocfg.pin_bit_mask = 1U << 2;
    ret = gpio_config(&iocfg);
    ESP_ERROR_CHECK(ret);

    spi_bus_config_t buscfg = {
        .miso_io_num = ST25R3911B_PIN_MISO,
        .mosi_io_num = ST25R3911B_PIN_MOSI,
        .sclk_io_num = ST25R3911B_PIN_SCLK,
        .max_transfer_sz = 32,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    const spi_device_interface_config_t reader_cfg = {
        .mode = 1,
        .clock_speed_hz = 4 * 1000 * 1000, // Source: Trust me bro
        .queue_size = 128,
        .address_bits = 6,
        .command_bits = 2,
        // .spics_io_num = 5,
        // Due to a silicon bug, we can't use pretrans and posttrans. We
        // instead rely on callback delay to give us a reliable CS signal.
        // .spics_io_num = ST25R3911B_PIN_CSEN,
        .pre_cb = enable_cs,
        .post_cb = disable_cs,
    };

    ret = spi_bus_add_device(SPI3_HOST, &reader_cfg, &dev->handle);
    ESP_ERROR_CHECK(ret);
}

uint8_t st25r3911b_read_register(st25r3911b_t *dev, uint8_t addr)
{
    esp_err_t ret;

    spi_transaction_t xact = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = ST25R3911B_CMD_REG_READ,
        .addr = addr,
        .length = 8,
        .rxlength = 8,
    };

    ret = spi_device_polling_transmit(dev->handle, &xact);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("st25r3911b", "Read Register %02X: %02X", addr, xact.rx_data[0]);
    // vTaskDelay(10 / portTICK_RATE_MS);
    return xact.rx_data[0];
}

void st25r3911b_write_register(st25r3911b_t *dev, uint8_t addr, uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t xact = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = ST25R3911B_CMD_REG_WRITE,
        .addr = addr,
        .length = 8,
        .rxlength = 0,
    };

    xact.tx_data[0] = data;

    ret = spi_device_polling_transmit(dev->handle, &xact);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("st25r3911b", "Write Register %02X: %02X", addr, data);
}

void st25r3911b_direct_command(st25r3911b_t *dev, uint8_t command)
{
    esp_err_t ret;
    spi_transaction_t xact = {
        .cmd = ST25R3911B_CMD_DIRECT_COMMAND,
        .addr = command,
        .length = 0,
        .rxlength = 0,
    };

    ret = spi_device_transmit(dev->handle, &xact);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("st25r3911b", "Sent Direct Command: %02X", command);
}

void st25r3911b_load_fifo(st25r3911b_t *dev, uint8_t bytes, uint8_t *data)
{
    esp_err_t ret;

    spi_transaction_t xact = {
        .cmd = ST25R3911B_CMD_FIFO_LOAD,
        .addr = 0x00,
        .length = 8 * bytes,
        .tx_buffer = (void *)data,
    };

    ret = spi_device_polling_transmit(dev->handle, &xact);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("st25r3911b", "Load FIFO: %i bytes", bytes);
}

void st25r3911b_read_fifo(st25r3911b_t *dev, uint8_t bytes, uint8_t *data)
{
    esp_err_t ret;

    spi_transaction_t xact = {
        .cmd = ST25R3911B_CMD_FIFO_READ,
        .addr = 0xFF,
        .length = 8 * bytes,
        .rxlength = 8 * bytes,
        .rx_buffer = (void *)data,
    };

    ret = spi_device_polling_transmit(dev->handle, &xact);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("st25r3911b", "Read FIFO: %i bytes", bytes);
}

bool st25r3911b_check(st25r3911b_t *dev)
{
    uint8_t ic_identity = st25r3911b_read_register(dev, ST25R3911B_REG_IC_IDENTITY);

    uint8_t ic_type = ic_identity >> 3;
    ESP_LOGI("st253911b", "IC Type: %d", ic_type);

    uint8_t ic_revision_bits = ic_identity & 0b00000111;
    ESP_LOGI("st253911b", "IC Revision: %d", ic_revision_bits);

    if (ic_type != 1)
    {
        return false;
    }

    switch (ic_revision_bits)
    {
    case 0b010:
        return true;
        break;
    case 0b011:
        return true;
        break;
    case 0b100:
        return true;
        break;
    case 0b101:
        return true;
        break;
    }

    return false;
}

/**
 * Stolen from https://github.com/stm32duino/ST25R3911B/blob/a4976ad8cf9e06b0fba89a0e880b0edc7857b315/src/st25r3911.cpp#L109
 */
void st25r3911b_powerup(st25r3911b_t *dev)
{
    // Reset
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_SET_DEFAULT);

    // Zero out Op Control
    st25r3911b_write_register(dev, ST25R3911B_REG_OP_CONTROL, 0b00000000);

    // Enable MISO Pull-Downs
    uint8_t io_conf_2 = st25r3911b_read_register(dev, ST25R3911B_REG_IO2);
    io_conf_2 |= 0b00011000;
    st25r3911b_write_register(dev, ST25R3911B_REG_IO2, io_conf_2);
    uint8_t i;
    // Disable all Interrupts
    for (i = 0; i < 3; i++)
    {
        st25r3911b_write_register(dev, 0x14 + i, 0xFF);
    }

    // Clear Interrupts
    for (i = 0; i < 3; i++)
    {
        st25r3911b_read_register(dev, 0x17 + i);
    }

    // Set Modes
    st25r3911b_write_register(dev, ST25R3911B_REG_MODE, 0b00001000);
    st25r3911b_write_register(dev, ST25R3911B_REG_BITRATE, 0b00000000);
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_ANALOG_PRESET);

    // Set Oscillators on
    // - first check if we're not already enabled
    uint8_t op_cntrl = st25r3911b_read_register(dev, ST25R3911B_REG_OP_CONTROL);
    if ((op_cntrl & (1 << 7)) != (1 << 7))
    {
        // - clear osc_en interrupt
        st25r3911b_read_register(dev, ST25R3911B_REG_IRQ_MAIN);
        // - enable interrupt
        uint8_t mask_irq_main = st25r3911b_read_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN);
        mask_irq_main &= ~(1 << 7);
        st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN, mask_irq_main);

        // - enable oscillators
        op_cntrl |= (1 << 7);
        st25r3911b_write_register(dev, ST25R3911B_REG_OP_CONTROL, op_cntrl);
    }
    else
    {
        ESP_LOGE("st25t3911b", "OpCntrl_en already enabled.");
        return;
    }

    uint8_t irq_main;
    while (true)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        irq_main = st25r3911b_read_register(dev, ST25R3911B_REG_IRQ_MAIN);
        if (irq_main != 0)
        {
            break;
        }
    }
    // - disable interrupt
    uint8_t mask_irq_main = st25r3911b_read_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN);
    mask_irq_main &= ~(1 << 7);
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN, mask_irq_main);
}

void st25r3911b_set_mode_nfca(st25r3911b_t *dev)
{
    // Adjust Regulators
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_NFC, 0b01111111);
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_ADJUST_REGULATORS);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    st25r3911b_read_register(dev, ST25R3911B_REG_IRQ_NFC);
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_NFC, 0xFF);

    // Switch NFC Field On
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_NFC, 0b11111001);
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_NF_FIELD_ON);
    while (true)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        uint8_t result = st25r3911b_read_register(dev, ST25R3911B_REG_IRQ_NFC);
        if (result == 2)
        {
            st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_NFC, 0xFF);
            break;
        }
    }

    // Enable rx & tx
    uint8_t op_cntrl = st25r3911b_read_register(dev, ST25R3911B_REG_OP_CONTROL);
    op_cntrl |= ST25R3911B_BIT_OP_CONTROL_RX_EN;
    op_cntrl |= ST25R3911B_BIT_OP_CONTROL_TX_EN;
    st25r3911b_write_register(dev, ST25R3911B_REG_OP_CONTROL, op_cntrl);
}

void st25r3911b_transmit(st25r3911b_t *dev, uint16_t bytes, uint8_t *data, bool crc)
{
    // Send Clear Command
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_CLEAR1);
    // st25r3911b_direct_command(dev, ST25R3911B_DCMD_CLEAR2);

    // Load Bytes into FIFO
    st25r3911b_load_fifo(dev, bytes, data);

    // Set TX Bytes Register
    uint8_t tx_bytes[2] = {0, 0};
    *((uint16_t *)tx_bytes) = (bytes << 3);
    st25r3911b_write_register(dev, ST25R3911B_REG_TX_BYTES_2, tx_bytes[0]);
    st25r3911b_write_register(dev, ST25R3911B_REG_TX_BYTES_1, tx_bytes[1]);

    // Transmit
    if (crc)
    {
        st25r3911b_direct_command(dev, ST25R3911B_DCMD_TRANSMIT_CRC);
    }
    else
    {
        st25r3911b_direct_command(dev, ST25R3911B_DCMD_TRANSMIT_NO_CRC);
    }
}

void st25r3911b_transmit_reqa(void *dev_ptr)
{
    st25r3911b_t *dev = dev_ptr;

    // Send REQA
    st25r3911b_direct_command(dev, ST25R3911B_DCMD_SEND_REQA);
}

void st25r3911b_transmit_crc(void *dev_ptr, uint16_t bytes, uint8_t *data)
{
    st25r3911b_t *dev = dev_ptr;

    st25r3911b_transmit(dev, bytes, data, true);
}

void st25r3911b_transmit_no_crc(void *dev_ptr, uint16_t bytes, uint8_t *data)
{
    st25r3911b_t *dev = dev_ptr;

    st25r3911b_transmit(dev, bytes, data, false);
}

uint16_t st25r3911b_receive(void *dev_ptr, uint16_t bufsize, uint8_t *buf)
{
    st25r3911b_t *dev = dev_ptr;

    // Unmask rxe (end of receive) interrupt
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN, ~ST25R3911B_BIT_IRQ_MAIN_RXE);

    // TODO: we should really wait for an Interrupt here, waiting is not ideal.
    uint8_t irq_main;
    bool timeout = true;
    for (uint8_t i = 0; i < 10; i++)
    {
        irq_main = st25r3911b_read_register(dev, ST25R3911B_REG_IRQ_MAIN);
        if ((irq_main >> 4) & 1U)
        {
            timeout = false;
            break;
        }
    }

    // Mask rxe (end of receive) interrupt
    st25r3911b_write_register(dev, ST25R3911B_REG_MASK_IRQ_MAIN, 0xFF);

    if (timeout)
    {
        return 0;
    }

    // Determine how many bytes we should read
    uint16_t bytes_available = st25r3911b_read_register(dev, ST25R3911B_REG_FIFO_STATUS_1);
    st25r3911b_read_register(dev, ST25R3911B_REG_FIFO_STATUS_2);
    uint8_t bytes_rx = bytes_available < bufsize ? bytes_available : bufsize;

    // Load Bytes from FIFO
    st25r3911b_read_fifo(dev, bytes_rx, buf);

    return bytes_rx;
}
