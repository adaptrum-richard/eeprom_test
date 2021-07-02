#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include "temp_io.h"

/* 对设备进行初始化设置, 设置超时时间及重发次数, 参数为设备文件描述符 */
static int temp_init(int fd)
{
    ioctl(fd, I2C_TIMEOUT, 4);
    ioctl(fd, I2C_RETRIES, 2);
    return 0;
}

/* 初始化一个ioctl_st结构, 并预分配空间 */
static ioctl_st *ioctl_st_init(void)
{
    ioctl_st *iocs;

    iocs = malloc(sizeof(ioctl_st));
    if (!(iocs)) {
        perror("malloc iocs:");
        return NULL;
    }

    iocs->msgs = malloc(I2C_MSG_SIZE * MAX_MSG_NR);
    if (!(iocs->msgs)) {
        perror("malloc iocs->msgs");
        return NULL;
    }

    iocs->msgs[0].buf = malloc(TEMP_BUFF_SIZE + 2);
    if (!(iocs->msgs[0].buf)) {
        perror("malloc iocs->msgs[0].buf");
        return NULL;
    }

    iocs->msgs[1].buf = malloc(TEMP_BUFF_SIZE + 2);
    if (!(iocs->msgs[1].buf)) {
        perror("malloc iocs->msgs[1].buf");
        return NULL;
    }

    return iocs;
}

/* 销毁ioctl_st结构, 释放空间 */
static void ioctl_st_release(ioctl_st *iocs)
{
    free(iocs->msgs[0].buf);
    free(iocs->msgs[1].buf);
    free(iocs->msgs);
    free(iocs);
}

/* 根据eeprom_st结构生成page_read时序所需的ioctl_st结构 */
static void temp_read_st_gen(ioctl_st *iocs, temp_st *temps)
{
    int size = temps->len;

    iocs->nmsgs = 2;  //read需2次start信号

    /* 第1次信号 */
    iocs->msgs[0].addr = temps->slave_addr;  //填入slave_addr
    iocs->msgs[0].flags = I2C_M_WR;         //write标志
    iocs->msgs[0].len = 1;                  //信号长度1字节
    iocs->msgs[0].buf[0] = temps->byte_addr;

    /* 第2次信号 */
    iocs->msgs[1].addr = temps->slave_addr;  //填入slave_addr
    iocs->msgs[1].flags = I2C_M_RD;         //read标志
    iocs->msgs[1].len = size;               //信号长度: 待读数据长度
    memset(iocs->msgs[1].buf, 0, size);     //先清零, 待读数据将自动存放于此
}

/* 用ioctl方法从eeprom中读取数据 */
static int temp_read(int fd, ioctl_st *iocs, temp_st *temps)
{
    int ret;
    int size = temps->len;

    temp_read_st_gen(iocs, temps);
    ret = ioctl(fd, I2C_RDWR, (u32)iocs);
    if (ret == -1) {
        perror("ioctl");
        return ret;
    }

    /* 将读取的数据从ioctl结构中复制到用户buf */
    memcpy(temps->buf, iocs->msgs[1].buf, size);
    //    printf("read byte ioctl ret = %d\n", ret);

    return ret;
}

int temp_data_read(u16 pos, u8 *data, int size)
{
    int fd;
    u8 *buf = data;
    temp_st temps;
    ioctl_st *iocs = ioctl_st_init();

    fd = open(DEV_PATH, O_RDONLY);
    if (fd < 0) {
        perror("open eeprom");
        return 0;
    }

    temp_init(fd);
    temps.slave_addr = temp_addr0;
    temps.byte_addr = pos;
    temps.len = size;
    temps.buf = buf;

    temp_read(fd, iocs, &temps);
    ioctl_st_release(iocs);
    close(fd);

    return size;
}

/* 根据eeprom_st结构生成page_write时序所需的ioctl_st结构 */
static void temp_write_st_gen(ioctl_st *iocs, temp_st *temps)
{
    int size = temps->len;

    iocs->nmsgs = 1;  //page_write只需1次start信号

    iocs->msgs[0].addr = temps->slave_addr;  //填入slave_addr
    iocs->msgs[0].flags = I2C_M_WR;         //write标志
    iocs->msgs[0].len = size + 1; //信号长度: 待写入数据长度 + byte_addr长度
    iocs->msgs[0].buf[0] = temps->byte_addr;
    memcpy((iocs->msgs[0].buf + 1), temps->buf, size); //copy待写数据
}

/* 用ioctl方法向temp中写入数据 */
static int temp_write(int fd, ioctl_st *iocs, temp_st *temps)
{
    int ret;

    temp_write_st_gen(iocs, temps);
    ret = ioctl(fd, I2C_RDWR, (u32)iocs);
    if (ret == -1) {
        perror("ioctl");
        return ret;
    }
    return ret;
}

int temp_data_write(u16 pos, u8 *data, int size)
{
    int fd;
    u8 *buf = data;
    temp_st temps;
    ioctl_st *iocs = ioctl_st_init();

    fd = open(DEV_PATH, O_WRONLY);
    if (fd < 0) {
        perror("open eeprom");
        return 0;
    }
    if(size > 2)
    {
        printf("%s %d size error\n", __func__, __LINE__);
        return 0;
    }
    temp_init(fd);
    temps.slave_addr = temp_addr0;
    temps.byte_addr = pos;
    temps.len = size;
    temps.buf = buf;

    temp_write(fd, iocs, &temps);
    ioctl_st_release(iocs);
    close(fd);

    return size;
}

u8 temp_byte_read(u16 pos)
{
    u8 buf;
    temp_data_read(pos, &buf, 1);

    return buf;
}

int temp_byte_write(u16 pos, u8 data)
{
    int ret;
    u8 buf = data;
    ret = temp_data_write(pos, &buf, 1);

    return ret;
}
