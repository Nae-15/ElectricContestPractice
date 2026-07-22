################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/Code/Project-MSPM0G3507/ElcContestTest" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/Debug" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/inc" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/src" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/lcd" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/Board" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1138291021: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"E:/software/IDE/Code Composer Studio/sysconfig_1.26.2/sysconfig_cli.bat" -s "E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/.metadata/product.json" --script "E:/Code/Project-MSPM0G3507/ElcContestTest/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1138291021 ../empty.syscfg
device.opt: build-1138291021
device.cmd.genlibs: build-1138291021
ti_msp_dl_config.c: build-1138291021
ti_msp_dl_config.h: build-1138291021
Event.dot: build-1138291021
boot_config.c: build-1138291021
boot_config.h: build-1138291021

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/Code/Project-MSPM0G3507/ElcContestTest" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/Debug" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/inc" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/src" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/lcd" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/Board" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"E:/software/IDE/Code Composer Studio/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/Code/Project-MSPM0G3507/ElcContestTest" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/Debug" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"E:/SDK/Ti/mspm0_sdk_2_10_00_04/mspm0_sdk_2_11_00_07/source" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/inc" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/src" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/lcd" -I"E:/Code/Project-MSPM0G3507/ElcContestTest/BPS/Board" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


