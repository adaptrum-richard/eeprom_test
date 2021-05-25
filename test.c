#include "eeprom_io.h"

int main(int argc, char *argv[])
{
    u16 pos = 4;
    u8 data[5] = {0x32, 0x15, 0x99, 0x25, 0x6};
    u8 read_byte;
    u8 read_data[5] ={0, };

    eeprom_byte_write(pos, data[0]);  //单字节写操作

    read_byte = eeprom_byte_read(pos);  //单字节读操作
    printf("write byte:0x%x, read byte:0x%x\n", data[0], read_byte);

    eeprom_page_write(pos, data, 5);  //多字节写操作
    eeprom_page_read(pos, read_data, 5);   //多字节读操作

    printf("write address 0x%x, data: 0x%x 0x%x 0x%x 0x%x 0x%x \n", pos, 
        data[0], data[1], data[2], data[3], data[4]);
    printf("read address 0x%x, data: 0x%x 0x%x 0x%x 0x%x 0x%x \n", pos, 
        read_data[0], read_data[1], read_data[2], read_data[3], read_data[4]);
    return 0;
}