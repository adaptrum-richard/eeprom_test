#include "eeprom_io.h"
#include <stdio.h>
#include <time.h>
#include <sys/select.h>
#include <sys/time.h>
//cat /sys/bus/i2c/devices/i2c-1/1-0049/hwmon/hwmon0/temp1_input

void rand_fill(u8 *data, int len)
{
    int i;
    srand((unsigned)time(NULL));
    for(i = 0 ; i < len; i++)
    {
        data[i] = rand() % 0xff;
    }
}

void inline sleep_ms(long msec)
{
    struct timeval time;
    if (msec == 0)
        return;
    time.tv_sec = 0;
    time.tv_usec = msec * 1000;
    select(0, NULL, NULL, NULL, &time);
}

int main(int argc, char *argv[])
{
#define SIZE_DATA 64
    u16 pos = 0;
    int i = 0;
    int j = 0;
    int max_size = (32*1024);
    for(j = 0; j < ((max_size/SIZE_DATA) - 1); j++){
        printf("j = %d\n", j);
        u8 data[SIZE_DATA] ={0xff};
        pos+=SIZE_DATA;
        u32 read_byte;
        u8 read_data[SIZE_DATA] ={0, };

        //eeprom_byte_write(pos, data[0]);  //单字节写操作

        //read_byte = eeprom_byte_read(pos);  //单字节读操作
        //printf("write byte:0x%x, read byte:0x%x\n", data[0], read_byte);
        rand_fill(data, SIZE_DATA);
        eeprom_page_write(pos, data, SIZE_DATA);  //多字节写操作
#if 1
        //sleep_ms(2);
        eeprom_page_read(pos, read_data, SIZE_DATA);   //多字节读操作
  
        for(i = 0; i < SIZE_DATA; i++)
        {
            if(data[i] != read_data[i]){
                printf("pos: 0x%x, write data:0x%x, read data:0x%x\n", pos+i, data[i], read_data[i]);
                goto out;
            }
        }
#endif
        //sleep_ms(2);
    }
out:
    return 0;
}