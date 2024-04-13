################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/BasicMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/BayesFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/CommonTables" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/ComplexMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/ControllerFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/DistanceFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/FastMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/FilteringFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/InterpolationFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/MatrixFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/QuaternionMathFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/StatisticsFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/SupportFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/SVMFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/TransformFunctions" -I"C:/Users/Vasay/Documents/GitHub/LAB2-Firmware-ADC/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ControllerFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ControllerFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/ControllerFunctions.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ControllerFunctions

