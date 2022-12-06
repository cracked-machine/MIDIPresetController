message("\n** Configuring STM32 Environment **")
message("** Selected Kit: ${CMAKE_C_COMPILER} **\n")


set(TOOLCHAIN /opt/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-)


set(DEVICE              STM32L011xx)
set(ARCH			    armv6-m)
set(CORE			    cortex-m0plus)
set(ARM_ASM             mthumb)
set(LINKER_SCRIPT       ${CMAKE_SOURCE_DIR}/STM32L011D4PX_FLASH.ld)
set(BUILD_NAME          build.elf)
set(HEX_NAME            build.hex)
set(MAP_NAME            build.map)

# Set the device target otherwise `CMSIS/Device/ST/STM32L0xx/Include/stm32l0xx.h` complains.
add_compile_definitions(${DEVICE})

# C compiler settings
set(CMAKE_C_FLAGS "-mcpu=cortex-m0plus -std=gnu17 -g3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -mfloat-abi=soft -mthumb" CACHE INTERNAL "c compiler flags ")

# Assembler compiler settings
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF'Core/Startup/startup_stm32l011d4px.d' -MT'Core/Startup/startup_stm32l011d4px.o' --specs=nano.specs -mfloat-abi=soft -mthumb" CACHE INTERNAL "asm compiler flags")

# Linker settings
set(CMAKE_EXE_LINKER_FLAGS  "-mcpu=cortex-m0plus -T${LINKER_SCRIPT} --specs=nosys.specs -Wl,-Map=${MAP_NAME} -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group" CACHE INTERNAL "exe link flags")

# This must come after compiler/linker settings
enable_language(C)
enable_language(ASM)




