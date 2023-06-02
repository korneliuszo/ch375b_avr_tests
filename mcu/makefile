#
#             LUFA Library
#     Copyright (C) Dean Camera, 2019.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega328p
ARCH         = AVR8
BOARD        = NONE
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = ch375bbridge
SRC          = $(TARGET).cpp
CC_FLAGS     = -flto -Wno-volatile -g2
CPP_STANDARD = gnu++2b
LD_FLAGS     = -Os -flto -fuse-linker-plugin -g2
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_HFUSE = 0xd8
AVRDUDE_EFUSE = 0xfc
AVRDUDE_LFUSE = 0xff
AVRDUDE_LOCK  = 0x3f

# Default target
all:

# Include common DMBS build system modules
DMBS_PATH      ?= ../dmbs/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/doxygen.mk
include $(DMBS_PATH)/dfu.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/hid.mk
include $(DMBS_PATH)/avrdude.mk
include $(DMBS_PATH)/atprogram.mk
