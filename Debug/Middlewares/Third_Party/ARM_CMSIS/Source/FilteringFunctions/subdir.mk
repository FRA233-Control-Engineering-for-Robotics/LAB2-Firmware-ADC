################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/BasicMathFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/BayesFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/CommonTables" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/ComplexMathFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/ControllerFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/DistanceFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/FastMathFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/FilteringFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/InterpolationFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/MatrixFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/QuaternionMathFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/StatisticsFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/SupportFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/SVMFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/TransformFunctions" -I"C:/Users/Qzider/STM32CubeIDE/workspace_1.13.2/LAB2-Firmware-ADC/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FilteringFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FilteringFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/FilteringFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FilteringFunctions

