CRAZYFLIE_BASE=$(PWD)/crazyflie-firmware

#OOT_CONFIG := $(PWD)/app-config
OOT_CONFIG := $(CRAZYFLIE_BASE)/build/.config # Config file for use with make menuconfig in $(CRAZYFLIE_BASE)

include $(CRAZYFLIE_BASE)/tools/make/oot.mk