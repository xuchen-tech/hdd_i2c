################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./hdd_i2c_utils.o" \
"./i2ccontroller.o" \
"./i2ctargetApp.o" \
"./main_freertos.o" \
"./nas2300.o" \
"./pt100.o" \
"./ti_drivers_config.o" \
"./ti_msp_dl_config.o" \
"./freertos/ticlang/startup_mspm0g350x_ticlang.o" \
"./ti/display/Display.o" \
"./ti/display/DisplayUart.o" \
"./ti/segger/SEGGER_RTT.o" \
"./ti/segger/SEGGER_RTT_ASM_ARMv7M.o" \
"./ti/segger/SEGGER_RTT_printf.o" \
"../mspm0g3507.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-l"C:/ti/mspm0_sdk_2_09_00_01/source/ti/drivers/lib/ticlang/m0p/drivers_mspm0g1x0x_g3x0x.a" \
-Wl,-lfreertos_builds_LP_MSPM0G3507_release_ticlang.lib \
-Wl,-l"C:/ti/mspm0_sdk_2_09_00_01/source/ti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a" \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include freertos/ticlang/subdir_vars.mk
-include ti/display/subdir_vars.mk
-include ti/segger/subdir_vars.mk
-include subdir_rules.mk
-include freertos/ticlang/subdir_rules.mk
-include ti/display/subdir_rules.mk
-include ti/segger/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
i2c_target_LP_MSPM0G3507_freertos_ticlang.out 

EXE_OUTPUTS__QUOTED += \
"i2c_target_LP_MSPM0G3507_freertos_ticlang.out" 

BIN_OUTPUTS += \
i2c_target_LP_MSPM0G3507_freertos_ticlang.hex 

BIN_OUTPUTS__QUOTED += \
"i2c_target_LP_MSPM0G3507_freertos_ticlang.hex" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "i2c_target_LP_MSPM0G3507_freertos_ticlang.out" secondary-outputs

dependents:
	-cd C:/Users/hugh-/workspace_ccstheia/freertos_builds_LP_MSPM0G3507_release_ticlang/Debug && $(MAKE) all

# Tool invocations
i2c_target_LP_MSPM0G3507_freertos_ticlang.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) C:/Users/hugh-/workspace_ccstheia/freertos_builds_LP_MSPM0G3507_release_ticlang/Debug/freertos_builds_LP_MSPM0G3507_release_ticlang.lib
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -D__MSPM0G3507__ -gdwarf-3 -Wl,-m"i2c_target_LP_MSPM0G3507_freertos_ticlang.map" -Wl,-i"C:/Users/hugh-/workspace_ccstheia/freertos_builds_LP_MSPM0G3507_release_ticlang/Debug" -Wl,-i"C:/ti/mspm0_sdk_2_09_00_01/source" -Wl,-i"C:/Users/hugh-/workspace_ccstheia/i2c_target_LP_MSPM0G3507_freertos_ticlang" -Wl,-i"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="i2c_target_LP_MSPM0G3507_freertos_ticlang_linkInfo.xml" -Wl,--rom_model -o "i2c_target_LP_MSPM0G3507_freertos_ticlang.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

i2c_target_LP_MSPM0G3507_freertos_ticlang.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmhex.exe" --memwidth=8 --romwidth=8 --diag_wrap=off --intel -o "i2c_target_LP_MSPM0G3507_freertos_ticlang.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti\segger\SEGGER_RTT_ASM_ARMv7M.d" 
	-$(RM) "hdd_i2c_utils.o" "i2ccontroller.o" "i2ctargetApp.o" "main_freertos.o" "nas2300.o" "pt100.o" "ti_drivers_config.o" "ti_msp_dl_config.o" "freertos\ticlang\startup_mspm0g350x_ticlang.o" "ti\display\Display.o" "ti\display\DisplayUart.o" "ti\segger\SEGGER_RTT.o" "ti\segger\SEGGER_RTT_ASM_ARMv7M.o" "ti\segger\SEGGER_RTT_printf.o" 
	-$(RM) "hdd_i2c_utils.d" "i2ccontroller.d" "i2ctargetApp.d" "main_freertos.d" "nas2300.d" "pt100.d" "ti_drivers_config.d" "ti_msp_dl_config.d" "freertos\ticlang\startup_mspm0g350x_ticlang.d" "ti\display\Display.d" "ti\display\DisplayUart.d" "ti\segger\SEGGER_RTT.d" "ti\segger\SEGGER_RTT_printf.d" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:
C:/Users/hugh-/workspace_ccstheia/freertos_builds_LP_MSPM0G3507_release_ticlang/Debug/freertos_builds_LP_MSPM0G3507_release_ticlang.lib:

-include ../makefile.targets

