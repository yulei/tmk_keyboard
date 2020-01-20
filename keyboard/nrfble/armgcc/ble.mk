SRC_FILES += \
					$(BLE_DIR)/ble_adv_service.c \
					$(BLE_DIR)/ble_bat_service.c \
					$(BLE_DIR)/ble_hid_service.c \
					$(BLE_DIR)/ble_services.c \
					$(BLE_DIR)/ble_keyboard.c \
					$(BLE_DIR)/main.c \
					$(PORTING_DIR)/bootloader.c \
					$(PORTING_DIR)/eeprom.c \
					$(PORTING_DIR)/printf.c \
					$(PORTING_DIR)/suspend.c \
					$(PORTING_DIR)/timer.c

INC_FOLDERS += $(BLE_DIR) $(PORTING_DIR)

# jlink monitor mode debug
ifdef JLINK_MONITOR_ENABLE
    SRC_FILES += $(MMD_DIR)/JLINK_MONITOR_ISR_SES.s
    SRC_FILES += $(MMD_DIR)/JLINK_MONITOR.c
    OPT_DEFS += -DCONFIG_JLINK_MONITOR_ENABLED
endif