################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/Users/matan/Downloads/CCS_20.4.0.00013_win/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="C:/Users/matan/workspace_ccstheia/adc_ex1_soc_epwm_f280049c" --include_path="C:/ti/C2000Ware_26_00_00_00/device_support/f28004x/common/include/" --include_path="C:/ti/C2000Ware_26_00_00_00/device_support/f28004x/headers/include/" --include_path="C:/Users/matan/Downloads/CCS_20.4.0.00013_win/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=CPU1 --define=_FLASH --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/Users/matan/Downloads/CCS_20.4.0.00013_win/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="C:/Users/matan/workspace_ccstheia/adc_ex1_soc_epwm_f280049c" --include_path="C:/ti/C2000Ware_26_00_00_00/device_support/f28004x/common/include/" --include_path="C:/ti/C2000Ware_26_00_00_00/device_support/f28004x/headers/include/" --include_path="C:/Users/matan/Downloads/CCS_20.4.0.00013_win/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=CPU1 --define=_FLASH --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


