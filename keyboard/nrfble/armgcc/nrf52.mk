TARGETS          := $(PROJECT)
OUTPUT_DIRECTORY := _build

SDK_ROOT := $(TMK_DIR)/tool/nrf52/nRF5_SDK_16.0.0_98a08e2

$(OUTPUT_DIRECTORY)/$(PROJECT).out: \
  LINKER_SCRIPT  := $(MCU_LDSCRIPT)

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer2.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/timer/drv_rtc.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/sensorsim/sensorsim.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/ble/peer_manager/auth_status_tracker.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager_handler.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas/ble_bas.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids/ble_hids.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/common \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/nfc/t4t_parser/cc_file \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/external/utf_converter \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/nfc/platform \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/mpu \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/nfc/t4t_parser/apdu \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/nfc/t4t_parser/tlv \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/nfc/ndef/parser/record \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_ROOT)/components/nfc/t2t_parser \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/libraries/sensorsim \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/nfc/ndef/parser/message \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/twi_sensor \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_ROOT)/components/libraries/stack_guard \
  $(SDK_ROOT)/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3 $(OPT_DEFS)
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DPROTOCOL_NRF
#CFLAGS += -D__arm__
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums
CFLAGS += -include $(CONFIG_H)

# C++ flags common to all targets
CXXFLAGS += $(OPT)
# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS132
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

$(PROJECT): CFLAGS += -D__HEAP_SIZE=8192
$(PROJECT): CFLAGS += -D__STACK_SIZE=8192
$(PROJECT): ASMFLAGS += -D__HEAP_SIZE=8192
$(PROJECT): ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

#############################################################################
####     tool definitions
#############################################################################
VERBOSE ?= 0
PRETTY  ?= 0
ABSOLUTE_PATHS ?= 0
PASS_INCLUDE_PATHS_VIA_FILE ?= 0
PASS_LINKER_INPUT_VIA_FILE  ?= 1

.SUFFIXES: # ignore built-in rules
%.d:       # don't try to make .d files
.PRECIOUS: %.d %.o

MK := mkdir
RM := rm -rf

# echo suspend
ifeq ($(VERBOSE),1)
  NO_ECHO :=
else
  NO_ECHO := @
endif

ifneq (,$(filter clean, $(MAKECMDGOALS)))

OTHER_GOALS := $(filter-out clean, $(MAKECMDGOALS))
ifneq (, $(OTHER_GOALS))
$(info Cannot make anything in parallel with "clean".)
$(info Execute "$(MAKE) clean \
  $(foreach goal, $(OTHER_GOALS),&& $(MAKE) $(goal))" instead.)
$(error Cannot continue)
else
.PHONY: clean
clean:
	$(RM) $(OUTPUT_DIRECTORY)
endif # ifneq(, $(OTHER_GOALS))

else # ifneq (,$(filter clean, $(MAKECMDGOALS)))

ifndef PROGRESS

ifeq ($(PRETTY),1)
    X     := @
    EMPTY :=
    SPACE := $(EMPTY) $(EMPTY)
    TOTAL := $(subst $(SPACE),,$(filter $(X), \
               $(shell "$(MAKE)" $(MAKECMDGOALS) --dry-run \
                 --no-print-directory PROGRESS=$(X))))

    5   := $(X)$(X)$(X)$(X)$(X)
    25  := $(5)$(5)$(5)$(5)$(5)
    100 := $(25)$(25)$(25)$(25)

    C       :=
    COUNTER  = $(eval C := $(C)$(100))$(C)
    P       :=
    count    = $(if $(filter $1%,$2),$(eval \
                 P += 1)$(call count,$1,$(2:$1%=%)),$(eval \
                 C := $2))
    print    = [$(if $(word 99,$1),99,$(if $(word 10,$1),, )$(words $1))%]
    PROGRESS = $(call count,$(TOTAL),$(COUNTER))$(call print,$(P)) $1
else
    PROGRESS = $1
endif # ifeq ($(PRETTY),1)

# Toolchain commands
CC      := arm-none-eabi-gcc
CXX     := arm-none-eabi-c++
AS      := arm-none-eabi-as
AR      := arm-none-eabi-ar -r
LD      := arm-none-eabi-ld
NM      := arm-none-eabi-nm
OBJDUMP := arm-none-eabi-objdump
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

# Use ccache on linux if available
CCACHE := $(if $(filter Windows%,$(OS)),, \
               $(if $(wildcard /usr/bin/ccache),ccache))
CC     := $(CCACHE) $(CC)

endif # ifndef PROGRESS

# $1 type of item
# $2 items paths to check
define ensure_exists_each
$(foreach item, $(2), \
  $(if $(wildcard $(item)),, $(warning Cannot find $(1): $(item))))
endef

ifeq ($(PASS_INCLUDE_PATHS_VIA_FILE),1)
INC_PATHS = @$($@_INC)
GENERATE_INC_FILE := 1
else
INC_PATHS = $(call target_specific, INC_PATHS, $($@_TGT))
GENERATE_INC_FILE :=
endif

# $1 object file
# $2 source file
# $3 include paths container file
# $4 target name
define bind_obj_with_src
$(eval $(1)     := $(2)) \
$(eval $(1)_INC := $(3)) \
$(eval $(1)_TGT := $(4)) \
$(eval $(1): Makefile | $(dir $(1)).) \
$(if $(GENERATE_INC_FILE), $(eval $(1): $(3)))
endef

# $1 target name
# $2 source file name
# Note: this additional .o for .s files is a workaround for issues with make 4.1
#       from MinGW (it does nothing to remake .s.o files when a rule for .S.o
#       files is defined as well).
define get_object_file_name
$(OUTPUT_DIRECTORY)/$(strip $(1))/$(notdir $(2:%.s=%.s.o)).o
endef

# $1 target name
# $2 include paths container file
# $3 list of source files
define get_object_files
$(call ensure_exists_each,source file, $(3)) \
$(foreach src_file, $(3), \
  $(eval obj_file := $(call get_object_file_name, $(1), $(src_file))) \
  $(eval DEPENDENCIES += $(obj_file:.o=.d)) \
  $(call bind_obj_with_src, $(obj_file), $(src_file), $(2), $(1)) \
  $(obj_file))
endef

# $1 variable name
# $2 target name
define target_specific
$($(addsuffix _$(strip $(2)), $(1)))
endef

ifeq ($(ABSOLUTE_PATHS),1)
get_path = $(call quote,$(abspath $1))
else
get_path = $1
endif

# $1 list of include folders
define get_inc_paths
$(call ensure_exists_each,include folder,$(1)) \
$(foreach folder,$(1),-I$(call get_path,$(folder)))
endef

# $1 target name
# $2 include paths container file
# $3 build goal name
define prepare_build
$(eval DEPENDENCIES :=) \
$(eval $(3): \
  $(call get_object_files, $(1), $(2), \
    $(SRC_FILES) $(call target_specific, SRC_FILES, $(1)))) \
$(eval -include $(DEPENDENCIES)) \
$(eval INC_PATHS_$(strip $(1)) := \
  $(call get_inc_paths, \
    $(INC_FOLDERS) $(call target_specific, INC_FOLDERS, $(1))))
endef

# $1 target name
define define_target
$(eval OUTPUT_FILE := $(OUTPUT_DIRECTORY)/$(strip $(1))) \
$(eval $(1): $(OUTPUT_FILE).out $(OUTPUT_FILE).hex $(OUTPUT_FILE).bin \
           ; @echo DONE $(strip $(1))) \
$(call prepare_build, $(1), $(OUTPUT_FILE).inc, $(OUTPUT_FILE).out)
endef

# $1 target name
# $2 library file name
define define_library
$(eval OUTPUT_FILE := $(OUTPUT_DIRECTORY)/$(strip $(1))) \
$(eval $(1) := $(2)) \
$(call prepare_build, $(1), $(OUTPUT_FILE).inc, $(1))
endef

# $1 content to be dumped
# Invokes another instance of MAKE to dump the specified content to stdout,
# which may be then redirected in shell to a file and this way stored there.
# MAKE in version prior to 4.0 does not provide the $(file ...) function.
define dump
$(eval CONTENT_TO_DUMP := $(1)) \
"$(MAKE)" -s --no-print-directory \
  -f "$(TOOL_DIR)/dump.mk" VARIABLE=CONTENT_TO_DUMP
endef
export CONTENT_TO_DUMP

.PHONY: $(TARGETS) all

all: $(TARGETS)

# Create build directories
$(OUTPUT_DIRECTORY):
	$(MK) $@
$(OUTPUT_DIRECTORY)/%/.: | $(OUTPUT_DIRECTORY)
	cd $(OUTPUT_DIRECTORY) && $(MK) $*

$(OUTPUT_DIRECTORY)/%.inc: Makefile | $(OUTPUT_DIRECTORY)
	$(info Generating $@)
	$(NO_ECHO)$(call dump, $(call target_specific, INC_PATHS, $*)) > $@

# $1 command
# $2 flags
# $3 message
define run
$(info $(call PROGRESS,$(3) file: $(notdir $($@)))) \
$(NO_ECHO)$(1) -MP -MD -c -o $@ $(call get_path,$($@)) $(2) $(INC_PATHS)
endef

# Create object files from C source files
%.c.o:
	$(call run,$(CC) -std=c99,$(CFLAGS),Compiling)

# Create object files from C++ source files
%.cpp.o:
	$(call run,$(CXX),$(CFLAGS) $(CXXFLAGS),Compiling)

# Create object files from assembly source files
%.S.o %.s.o.o:
	$(call run,$(CC) -x assembler-with-cpp,$(ASMFLAGS),Assembling)

ifeq ($(PASS_LINKER_INPUT_VIA_FILE),1)
GENERATE_LD_INPUT_FILE = $(call dump, $^ $(LIB_FILES)) > $(@:.out=.in)
LD_INPUT               = @$(@:.out=.in)
else
GENERATE_LD_INPUT_FILE =
LD_INPUT               = $^ $(LIB_FILES)
endif

# Link object files
%.out:
	$(info $(call PROGRESS,Linking target: $@))
	$(NO_ECHO)$(GENERATE_LD_INPUT_FILE)
	$(NO_ECHO)$(CC) $(LDFLAGS) $(LD_INPUT) -Wl,-Map=$(@:.out=.map) -o $@
	$(NO_ECHO)$(SIZE) $@

# Create binary .bin file from the .out file
%.bin: %.out
	$(info Preparing: $@)
	$(NO_ECHO)$(OBJCOPY) -O binary $< $@

# Create binary .hex file from the .out file
%.hex: %.out
	$(info Preparing: $@)
	$(NO_ECHO)$(OBJCOPY) -O ihex $< $@

endif # ifneq (,$(filter clean, $(MAKECMDGOALS)))

######################################################################
###   make tagetes
######################################################################
.PHONY: default help

# Default target - first one defined
default: $(PROJECT)

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		$(PROJECT)
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/$(PROJECT).hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/$(PROJECT).hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_7.0.1_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ../sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
