cmake_minimum_required(VERSION 3.22)

# HAL object library
add_library(hal OBJECT)
target_include_directories(hal
    PUBLIC
        Core/Inc
        #Middlewares/Third_Party/FreeRTOS/Source/include
        #Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        #Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
)
target_sources(hal
    PRIVATE
        Core/Src/adc.c
        Core/Src/gpio.c
        Core/Src/i2c.c
        Core/Src/spi.c
        Core/Src/stm32f1xx_hal_msp.c
        Core/Src/stm32f1xx_it.c
        Core/Src/syscalls.c
        Core/Src/sysmem.c
        Core/Src/system_stm32f1xx.c
        Core/Src/tim.c
        Core/Src/usart.c
        #Middlewares/Third_Party/FreeRTOS/Source/croutine.c
        #Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
        #Middlewares/Third_Party/FreeRTOS/Source/list.c
        #Middlewares/Third_Party/FreeRTOS/Source/queue.c
        #Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
        #Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        #Middlewares/Third_Party/FreeRTOS/Source/timers.c
        #Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
        #Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        #Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)
target_compile_definitions(hal
    PUBLIC
    	USE_HAL_DRIVER
	    STM32F103xB
        $<$<CONFIG:Debug>:DEBUG>
)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
