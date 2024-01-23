<<<<<<< HEAD
CRAZYFLIE_BASE := $(PWD)/crazyflie-firmware
OOT_CONFIG := $(CRAZYFLIE_BASE)/build/.config
include $(CRAZYFLIE_BASE)/tools/make/oot.mk
=======
# enable app support
APP=1
APP_STACKSIZE=300

VPATH += .
PROJ_OBJ += state_machine.o
PROJ_OBJ += wallfollowing_multiranger_onboard.o
PROJ_OBJ += wallfollowing_with_avoid.o
PROJ_OBJ += SGBA.o
PROJ_OBJ += drone_variables.o

CRAZYFLIE_BASE= crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile
>>>>>>> tamie
