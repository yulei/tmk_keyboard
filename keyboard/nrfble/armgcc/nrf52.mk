TARGETS          := $(PROJECT)
OUTPUT_DIRECTORY := _build

SDK_ROOT := $(TMK_DIR)/tool/nrf52/nRF5_SDK_16.0.0_98a08e2

$(OUTPUT_DIRECTORY)/$(PROJECT).out: \
  LINKER_SCRIPT  := $(MCU_LDSCRIPT)

include $(TOOL_DIR)/nrf52_src.mk

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

#############################################################################
####     tool definitions
#############################################################################
VERBOSE ?= 0
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

# $1 type of item
# $2 items paths to check
define ensure_exists_each
$(foreach item, $(2), \
  $(if $(wildcard $(item)),, $(warning Cannot find $(1): $(item))))
endef

INC_PATHS = $(call target_specific, INC_PATHS, $($@_TGT))

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

# $1 list of include folders
define get_inc_paths
$(call ensure_exists_each,include folder,$(1)) \
$(foreach folder,$(1),-I$(folder))
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

.PHONY: $(TARGETS) all

all: $(TARGETS)

# Create build directories
$(OUTPUT_DIRECTORY):
	$(MK) $@
$(OUTPUT_DIRECTORY)/%/.: | $(OUTPUT_DIRECTORY)
	cd $(OUTPUT_DIRECTORY) && $(MK) $*

$(OUTPUT_DIRECTORY)/%.inc: Makefile | $(OUTPUT_DIRECTORY)
	$(info Generating $@)
	$(NO_ECHO)$(file >$@, $(call target_specific, INC_PATHS, $*))

# $1 command
# $2 flags
# $3 message
define run
$(info $(3) file: $(notdir $($@))) \
$(NO_ECHO)$(1) -MP -MD -c -o $@ $($@) $(2) $(INC_PATHS)
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
GENERATE_LD_INPUT_FILE = $(file >$(@:.out=.in), $^ $(LIB_FILES)) 
LD_INPUT               = @$(@:.out=.in)
else
GENERATE_LD_INPUT_FILE =
LD_INPUT               = $^ $(LIB_FILES)
endif

# Link object files
%.out:
	$(info Linking target: $@)
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
