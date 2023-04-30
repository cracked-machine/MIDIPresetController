message("\n** Configuring STM32 Environment **")
message("** Selected Kit: ${CMAKE_C_COMPILER} **\n")

set(DEVICE              STM32L011xx)
set(ARCH			    armv6-m)
set(CORE			    cortex-m0plus)
set(ARM_ASM             mthumb)
set(LINKER_SCRIPT       ${CMAKE_SOURCE_DIR}/SW/src/STM32L011D4PX_FLASH.ld)

set(HEX_NAME            build.hex)
set(MAP_NAME            build.map)

# Set the device target otherwise `CMSIS/Device/ST/STM32L0xx/Include/stm32l0xx.h` complains.
add_compile_definitions(${DEVICE})

target_compile_options(${BUILD_NAME} PRIVATE
    # # C 
    $<$<COMPILE_LANGUAGE:C>:
        -mcpu=cortex-m0plus 
        -ffunction-sections 
        -fdata-sections 
        -Wall 
        -fstack-usage 
        -MMD 
        -MP 
        -mfloat-abi=soft 
        -mthumb
    >
    $<$<AND:$<COMPILE_LANGUAGE:C>,$<CONFIG:DEBUG>>:
        -g3
        -O0
    >
    
    # # C++
    $<$<COMPILE_LANGUAGE:CXX>:
        -mcpu=cortex-m0plus
        -ffunction-sections 
        -fdata-sections 
        -Wall 
        -fstack-usage 
        -MMD 
        -MP 
        -mfloat-abi=soft 
        -mthumb
        -Wno-volatile
    >
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:DEBUG>>:
        -g3
        -O0
    >
    # Asm
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=cortex-m0plus 
        -c 
        -x assembler-with-cpp 
        -MMD 
        -MP 
        -MF'Core/Startup/startup_stm32l011d4px.d' 
        -MT'Core/Startup/startup_stm32l011d4px.o' 
        # --specs=nano.specs 
        -mfloat-abi=soft 
        -mthumb
    >
    $<$<AND:$<COMPILE_LANGUAGE:ASM>,$<CONFIG:DEBUG>>:
        -g3
        -O0
        -DDEBUG
    >

)

# Linker settings
set(CMAKE_EXE_LINKER_FLAGS  "-T${LINKER_SCRIPT} -Wl,-Map=${MAP_NAME} -Wl,--gc-sections" CACHE INTERNAL "exe link flags")


# target_link_options(${BUILD_NAME} PUBLIC "-mcpu=cortex-m0plus -T${LINKER_SCRIPT} --specs=nosys.specs -Wl,-Map=${MAP_NAME} -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group")

# This must come after compiler/linker settings
enable_language(C)
enable_language(CXX)
enable_language(ASM)




