COMMON_DIR = $(TMK_DIR)/common
SRC_FILES += \
	$(COMMON_DIR)/host.c \
	$(COMMON_DIR)/keyboard.c \
	$(COMMON_DIR)/action.c \
	$(COMMON_DIR)/action_tapping.c \
	$(COMMON_DIR)/action_macro.c \
	$(COMMON_DIR)/action_layer.c \
	$(COMMON_DIR)/action_util.c \
	$(COMMON_DIR)/keymap.c \
	$(COMMON_DIR)/print.c \
	$(COMMON_DIR)/debug.c \
	$(COMMON_DIR)/util.c \
	$(COMMON_DIR)/hook.c \

INC_FOLDERS += $(COMMON_DIR)

# Option modules
ifdef BOOTMAGIC_ENABLE
    SRC_FILES += $(COMMON_DIR)/bootmagic.c
    SRC_FILES += $(PORTING_DIR)/eeprom.c
    OPT_DEFS += -DBOOTMAGIC_ENABLE
endif

ifdef MOUSEKEY_ENABLE
    SRC_FILES += $(COMMON_DIR)/mousekey.c
    OPT_DEFS += -DMOUSEKEY_ENABLE
    OPT_DEFS += -DMOUSE_ENABLE
endif

ifdef EXTRAKEY_ENABLE
    OPT_DEFS += -DEXTRAKEY_ENABLE
endif

ifdef CONSOLE_ENABLE
    OPT_DEFS += -DCONSOLE_ENABLE
else
    OPT_DEFS += -DNO_PRINT
    OPT_DEFS += -DNO_DEBUG
endif

ifdef COMMAND_ENABLE
    SRC_FILES += $(COMMON_DIR)/command.c
    OPT_DEFS += -DCOMMAND_ENABLE
endif

ifdef NKRO_ENABLE
    OPT_DEFS += -DNKRO_ENABLE
endif

ifdef USB_6KRO_ENABLE
    OPT_DEFS += -DUSB_6KRO_ENABLE
endif

ifdef SLEEP_LED_ENABLE
    SRC_FILES += $(PORTING_DIR)/sleep_led.c
    OPT_DEFS += -DSLEEP_LED_ENABLE
    OPT_DEFS += -DNO_SUSPEND_POWER_DOWN
endif

ifdef BACKLIGHT_ENABLE
    SRC_FILES += $(COMMON_DIR)/backlight.c
    OPT_DEFS += -DBACKLIGHT_ENABLE
endif

# Version string
OPT_DEFS += -DVERSION=$(shell (git describe --always --dirty || echo 'unknown') 2> /dev/null)