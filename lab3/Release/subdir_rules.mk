################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/example/common" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/driverlib" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/inc" --define=ccs --define=NON_NETWORK --define=cc3200 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

i2c_if.obj: /Applications/ti/lib/cc3200sdk_1.5.0/cc3200-sdk/example/common/i2c_if.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/example/common" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/driverlib" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/inc" --define=ccs --define=NON_NETWORK --define=cc3200 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="i2c_if.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_ccs.obj: /Applications/ti/lib/cc3200sdk_1.5.0/cc3200-sdk/example/common/startup_ccs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="/Applications/ti/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/example/common" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/driverlib" --include_path="/Applications/TI/lib/cc3200sdk_1.5.0/cc3200-sdk/inc" --define=ccs --define=NON_NETWORK --define=cc3200 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="startup_ccs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


