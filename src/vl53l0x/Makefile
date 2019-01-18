vl53l0x_drv-objs :=  lib_vl53l0x/vl53l0x_i2c_comms.o lib_vl53l0x/vl53l0x_platform.o lib_vl53l0x/vl53l0x_api.o \
						lib_vl53l0x/vl53l0x_api_calibration.o lib_vl53l0x/vl53l0x_api_core.o lib_vl53l0x/vl53l0x_api_ranging.o  \
						lib_vl53l0x/vl53l0x_api_strings.o vl53l0x.o
obj-m+=vl53l0x_drv.o
all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
