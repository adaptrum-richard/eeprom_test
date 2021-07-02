#ifndef _EEPROM_IO_H
#define _EEPROM_IO_H

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

typedef struct i2c_rdwr_ioctl_data ioctl_st;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
#define TEMP_BUFF_SIZE 10
enum temp_address {
    temp_addr0 = 0x49,
};

#define DEV_PATH "/dev/i2c-1" 
#define I2C_M_WR          0
#define MAX_MSG_NR        2    
#define I2C_MSG_SIZE (sizeof(struct i2c_msg))

typedef struct temp_data {
    u8 slave_addr;
    u16 byte_addr;
    u16 len;
    u8 *buf;
} temp_st;

extern u8 temp_byte_read(u16 pos);
extern int temp_byte_write(u16 pos, u8 data);
extern int temp_data_write(u16 pos, u8 *data, int size);
extern int temp_data_read(u16 pos, u8 *data, int size);



#endif