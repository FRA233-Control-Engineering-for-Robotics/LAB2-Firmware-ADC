################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/BasicMathFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/BayesFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/CommonTables" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/ComplexMathFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/ControllerFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/DistanceFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/FastMathFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/FilteringFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/InterpolationFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/MatrixFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/QuaternionMathFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/StatisticsFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/SupportFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/SVMFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/TransformFunctions" -I"C:/Users/AAA/Downloads/LAB2-Firmware-ADC-main/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

