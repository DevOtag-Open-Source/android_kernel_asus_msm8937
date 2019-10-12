# Makefile for the Himax touchscreen drivers.

# wangbing@wind-mobi.com 2018316 begin >>> [4/4] add the hx83102 device info
ccflags-y += -I$(srctree)/drivers/misc/wind_driver/wind_device_info/
# wangbing@wind-mobi.com 2018316 end   <<< [4/4] add the hx83102 device info

# wangbing@wind-mobi.com 20180316 begin
obj-$(CONFIG_TOUCHSCREEN_HIMAX_I2C)		+= himax_platform.o himax_ic.o himax_common.o himax_debug.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)	+= himax_ito_test.o
# wangbing@wind-mobi.com 20180316 end