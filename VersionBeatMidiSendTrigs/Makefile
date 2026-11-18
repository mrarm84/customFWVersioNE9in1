# Project Name
TARGET = VersionBeatMidiSendTrigs

# ALT = 1: TRIG DRUMS hold button + turn = momentary tweak, snaps back on
# release. Separate binary and build dir:  make ALT=1
#   -> build_alt/VersionBeatMidiSendTrigsAlt.bin
ALT ?= 0
ifeq ($(ALT), 1)
TARGET = VersionBeatMidiSendTrigsAlt
override BUILD_DIR = build_alt
C_DEFS += -DHOLD_TWEAK=1
endif

# Sources (DSP is header-only in ../common and ../StarLabCommon)
CPP_SOURCES = VersionBeatMidiSendTrigs.cpp

# Library Locations
LIBDAISY_DIR = ../libDaisy
DAISYSP_DIR = ../DaisySP

# USB_MIDI = 1 (default): the USB port sends the 7 jacks as MIDI CC and the
# gate as a note. The USB stack does not fit next to the firmware in the
# 128 kB internal flash, so this build runs from SRAM through the Daisy
# bootloader: flash the bootloader once (make program-boot), then the app
# through it (make program-dfu). See README section 9.
#
# USB_MIDI = 0: no MIDI, internal flash (0x08000000), no bootloader, exactly
# like the Noise Engineering firmwares. -Os keeps it under 128 kB.
#   make clean && make USB_MIDI=0
USB_MIDI ?= 1

ifeq ($(USB_MIDI), 1)
APP_TYPE = BOOT_SRAM
OPT = -O2
else
APP_TYPE = BOOT_NONE
OPT = -Os
endif

# shared DSP engines (header-only) + C++17
C_INCLUDES = -I../common
C_DEFS += -DUSB_MIDI=$(USB_MIDI)
CPP_STANDARD = -std=gnu++17

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
