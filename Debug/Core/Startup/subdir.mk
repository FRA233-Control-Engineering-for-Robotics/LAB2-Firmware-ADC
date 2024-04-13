################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/BasicMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/BayesFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/CommonTables" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/ComplexMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/ControllerFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/DistanceFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/FastMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/FilteringFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/InterpolationFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/MatrixFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/QuaternionMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/StatisticsFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/SupportFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/SVMFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/TransformFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

