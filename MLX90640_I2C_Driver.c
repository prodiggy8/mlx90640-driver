/**
 * MLX90640 I2C Driver for Jetson Nano
 * Implements Repeated Start using Linux I2C-dev ioctl
 */

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "MLX90640_I2C_Driver.h"

#define I2C_DEVICE "/dev/i2c-1"

static int i2c_fd = -1;

void MLX90640_I2CInit(void)
{
    if (i2c_fd >= 0) close(i2c_fd);

    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        fprintf(stderr, "error: could not open %s - %s\n", I2C_DEVICE, strerror(errno));
        return;
    }
    // The Linux driver handles the initial high state and stop condition.
    printf("i2c initialized on %s\n", I2C_DEVICE);
}

void MLX90640_I2CFreqSet(int freq)
{
    // On Jetson Nano, frequency is managed by the kernel.
    // To set to 400kHz, run: sudo sh -c 'echo 400000 > /sys/class/i2c-adapter/i2c-1/bus_clk_rate'
    printf("i2c frequency setting to %d hz requested (set via sysfs on jetson)\n", freq);
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t addr_buf[2];
    addr_buf[0] = startAddress >> 8;
    addr_buf[1] = startAddress & 0xFF;

    // Buffer to hold raw bytes (2 bytes per 16-bit word)
    uint8_t rx_buf[nMemAddressRead * 2];

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    // Message 1: Write the register address
    msgs[0].addr = slaveAddr;
    msgs[0].flags = 0; // Write
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;

    // Message 2: Read data with REPEATED START
    msgs[1].addr = slaveAddr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len = nMemAddressRead * 2;
    msgs[1].buf = rx_buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        return -1; // NACK or communication failure
    }

    // Convert Big-Endian from sensor to Little-Endian for Jetson Nano
    for (int i = 0; i < nMemAddressRead; i++) {
        data[i] = (uint16_t)((rx_buf[i * 2] << 8) | rx_buf[i * 2 + 1]);
    }

    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t tx_buf[4];
    uint16_t dataCheck;

    tx_buf[0] = writeAddress >> 8;
    tx_buf[1] = writeAddress & 0xFF;
    tx_buf[2] = data >> 8;
    tx_buf[3] = data & 0xFF;

    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;

    msg.addr = slaveAddr;
    msg.flags = 0; // Write
    msg.len = 4;
    msg.buf = tx_buf;

    msgset.msgs = &msg;
    msgset.nmsgs = 1;

    // Perform Write
    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    // Per specs 2.1.4: Read back and verify
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &dataCheck) != 0) {
        return -1;
    }

    if (dataCheck != data) {
        return -2; // Data mismatch
    }

    return 0;
}

int MLX90640_I2CGeneralReset(void)
{
    uint8_t reset_cmd = 0x06;
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;

    msg.addr = 0x00; // General call address
    msg.flags = 0;
    msg.len = 1;
    msg.buf = &reset_cmd;

    msgset.msgs = &msg;
    msgset.nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    usleep(50000); // 50ms delay for sensor reboot
    return 0;
}
