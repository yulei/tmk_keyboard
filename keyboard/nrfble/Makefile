# Project name
PROJECT = nrfble

# Project dir
PROJ_DIR = .
BLE_DIR = $(PROJ_DIR)/ble
PORTING_DIR = $(PROJ_DIR)/porting
TOOL_DIR = $(PROJ_DIR)/armgcc
MMD_DIR = $(PROJ_DIR)/mmd

# Directory common source files exist
TMK_DIR = ../../tmk_core

# project specific files
SRC_FILES += matrix.c nrfble.c main.c ble_keyboard.c

INC_FOLDERS += $(PROJ_DIR)

ifdef KEYMAP
    SRC_FILES := keymap_$(KEYMAP).c $(SRC_FILES)
else
    SRC_FILES := keymap_plain.c $(SRC_FILES)
endif

CONFIG_H = config.h

# linker script to use
MCU_LDSCRIPT = $(TOOL_DIR)/nrf52.ld

# Build Options
#   comment out to disable the options.
#
BOOTMAGIC_ENABLE = yes	# Virtual DIP switch configuration
#MOUSEKEY_ENABLE = yes	# Mouse keys
#EXTRAKEY_ENABLE = yes	# Audio control and System control
#CONSOLE_ENABLE = yes	# Console for debug
#COMMAND_ENABLE = yes    # Commands for debug and configuration
#SLEEP_LED_ENABLE = yes  # Breathing sleep LED during USB suspend
#NKRO_ENABLE = yes	    # USB Nkey Rollover

JLINK_MONITOR_ENABLE = yes

include $(TOOL_DIR)/common.mk
include $(TOOL_DIR)/ble.mk
include $(TOOL_DIR)/porting.mk
include $(TOOL_DIR)/nrf52.mk
