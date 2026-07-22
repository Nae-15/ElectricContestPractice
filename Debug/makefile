################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./boot_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./BPS/Board/board.o" \
"./BPS/lcd/lcd.o" \
"./BPS/lcd/lcd_init.o" \
"./BPS/src/BLINK.o" \
"./BPS/src/ENCODER.o" \
"./BPS/src/IMU.o" \
"./BPS/src/KEY.o" \
"./BPS/src/MOTOR.o" \
"./BPS/src/YAW_PID.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include BPS/Board/subdir_vars.mk
-include BPS/lcd/subdir_vars.mk
-include BPS/src/subdir_vars.mk
-include subdir_rules.mk
-include BPS/Board/subdir_rules.mk
-include BPS/lcd/subdir_rules.mk
-include BPS/src/subdir_rules.mk
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
ElcContestTest.out 

EXE_OUTPUTS__QUOTED += \
"ElcContestTest.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "ElcContestTest.out"

# Tool invocations
ElcContestTest.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -g -Wall -Wl,-m"ElcContestTest.map" -Wl,-i"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source" -Wl,-i"E:/Code/Project-MSPM0G3507/ElcContestTest" -Wl,-i"E:/Code/Project-MSPM0G3507/ElcContestTest/Debug/syscfg" -Wl,-i"E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="ElcContestTest_linkInfo.xml" -Wl,--rom_model -o "ElcContestTest.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty.o" "ti_msp_dl_config.o" "boot_config.o" "startup_mspm0g350x_ticlang.o" "BPS\Board\board.o" "BPS\lcd\lcd.o" "BPS\lcd\lcd_init.o" "BPS\src\BLINK.o" "BPS\src\ENCODER.o" "BPS\src\IMU.o" "BPS\src\KEY.o" "BPS\src\MOTOR.o" "BPS\src\YAW_PID.o" 
	-$(RM) "empty.d" "ti_msp_dl_config.d" "boot_config.d" "startup_mspm0g350x_ticlang.d" "BPS\Board\board.d" "BPS\lcd\lcd.d" "BPS\lcd\lcd_init.d" "BPS\src\BLINK.d" "BPS\src\ENCODER.d" "BPS\src\IMU.d" "BPS\src\KEY.d" "BPS\src\MOTOR.d" "BPS\src\YAW_PID.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

