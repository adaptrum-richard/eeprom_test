#include "eeprom_io.h"
#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <sys/time.h>

#define CONF_REG_ADDR 0x01
#define TEMP_REG_ADDR 0x00
#define TOS_REG_ADDR  0x03
#define THYST_REG_ADDR 0x02

//conf register
#define OS_F_QUE_1 (0 << 3)
#define OS_F_QUE_2 (1 << 3)
#define OS_F_QUE_4 (2 << 3)
#define OS_F_QUE_6 (3 << 3)

#define OS_POL_LOW (0 << 2)
#define OS_POL_HIGH (1 << 2)

#define OS_COMP ( 0 << 1)
#define OS_INT ( 1<< 1)

#define NORMAL (0 << 0)
#define SHUTDOWN ( 1 << 0)

void inline sleep_ms(long msec)
{
    struct timeval time;
    if (msec == 0)
        return;
    time.tv_sec = 0;
    time.tv_usec = msec * 1000;
    select(0, NULL, NULL, NULL, &time);
}

void init_temp(void)
{
    u8 max_temp[2] = {0x7D, 0x00};//+125度
    temp_data_write(TOS_REG_ADDR, max_temp, 2);
    sleep_ms(200);
    u8 min_temp[2] = {0xC9, 0x00};// -55度
    temp_data_write(THYST_REG_ADDR, max_temp, 2);
    sleep_ms(200);
}

void read_conf(void)
{
    u8 conf = 0;
    temp_data_read(TEMP_REG_ADDR, &conf, 1);
    printf("read conf = 0x%x\n", conf);
}
//read LM75ADP temp
int main(void)
{
    u8 conf_value = OS_F_QUE_6 | OS_POL_HIGH | OS_COMP | NORMAL;
    
    //init_temp();
    temp_byte_write(CONF_REG_ADDR, conf_value);
    sleep_ms(200);
    while(1){
        u8 temp[2] = {0};
        temp_data_read(TEMP_REG_ADDR, temp, 2);
        u16 t = (temp[1] | temp[0] << 8 ) >> 5;
        float t1 = t * 0.125;
        printf("temp = %.2f\n", t1);
        sleep_ms(1000);
    }
    return 0;
}