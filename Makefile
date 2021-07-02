TOOLCHAIN = /workspace1/richard/adaptrum/anarion/openwrt_4.4/staging_dir/toolchain-arc_arc700_gcc-arc-2016.03_uClibc-1.0.14/bin/arc-openwrt-linux-
CC = /workspace1/richard/adaptrum/anarion/openwrt_4.4/staging_dir/toolchain-arc_arc700_gcc-arc-2016.03_uClibc-1.0.14/bin/arc-openwrt-linux-gcc
KERN_DIR = /workspace1/richard/adaptrum/anarion/openwrt_4.4/build_dir/target-arc_arc700_uClibc-1.0.14/linux-arc770_generic/linux-4.4.70
export STAGING_DIR=/workspace1/richard/adaptrum/anarion/openwrt_4.4/staging_dir/toolchain-arc_arc700_gcc-arc-2016.03_uClibc-1.0.14/bin
APP_NAME = eeprom_test
i2c_anarion-objs := i2c-anarion.o
obj-m	:= i2c-anarion.o
all : 
	$(CC) -o $(APP_NAME).app eeprom_io.c read_eep.c
	$(CC) -o read_temp.app temp_io.c read_temp.c
	$(MAKE) -C $(KERN_DIR) ARCH=arc CROSS_COMPILE=$(TOOLCHAIN) M=$(PWD) modules;
clean:
	$(MAKE) -C $(KERN_DIR) M=$(PWD) clean;
	rm -f $(APP_NAME).app *.ko;
