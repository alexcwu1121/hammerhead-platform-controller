#include "bsp.hpp"

extern "C"
{
    /// @brief QP assertion handler
    /// @param module
    /// @param id
    /// @return
    Q_NORETURN Q_onError(char const* const module, int_t const id)
    {
        // NOTE: this implementation of the error handler is intended only
        // for debugging and MUST be changed for deployment of the application
        // (assuming that you ship your production code with assertions enabled).
        Q_UNUSED_PAR(module);
        Q_UNUSED_PAR(id);
        QS_ASSERTION(module, id, 10000U);  // report assertion to QS

        // Reset
        // Breakpoint here
        // NVIC_SystemReset(); // TODO: not a good idea? Reset in release only
        for (;;)
        {
        }
    }

    /// @brief Hardfault handler
    __attribute__((naked)) void HardFault_Handler(void)
    {
        __asm volatile(
            "tst lr, #4        \n"  // Which stack? MSP or PSP
            "ite eq            \n"
            "mrseq r0, msp     \n"
            "mrsne r0, psp     \n"
            "b hardfault_c     \n");
    }

    /// @brief Break out registers
    /// @param stack
    void hardfault_c(uint32_t* stack)
    {
        volatile uint32_t r0  = stack[0];
        volatile uint32_t r1  = stack[1];
        volatile uint32_t r2  = stack[2];
        volatile uint32_t r3  = stack[3];
        volatile uint32_t r12 = stack[4];
        volatile uint32_t lr  = stack[5];
        volatile uint32_t pc  = stack[6];
        volatile uint32_t psr = stack[7];

        (void)r0;
        (void)r1;
        (void)r2;
        (void)r3;
        (void)r12;
        (void)lr;
        (void)pc;
        (void)psr;

        __BKPT(1);  // Stop here and inspect
        while (1)
            ;
    }

    /// @brief ADC conversion buffer
    static volatile uint16_t adcBuf[bsp::ADCChannels::NUM_ADC_CHANNELS * bsp::NUM_ADC_SAMPLES];
    // IIR adc filter accumulator
    static float adcIIR[bsp::ADCChannels::NUM_ADC_CHANNELS] = {0};
    /// @brief Conversion counter
    static uint16_t convCounter = 0U;

    /// @brief Read ADC
    static inline void ReadADC()
    {
        // Average
        uint32_t accum[bsp::ADCChannels::NUM_ADC_CHANNELS];
        memset(accum, 0, sizeof(accum));
        for (uint16_t i = 0U; i < bsp::ADCChannels::NUM_ADC_CHANNELS * bsp::NUM_ADC_SAMPLES; i++)
            accum[i % bsp::ADCChannels::NUM_ADC_CHANNELS] += adcBuf[i];

        // Pack event
        float vin = static_cast<float>(accum[bsp::ADCChannels::VIN]) / bsp::NUM_ADC_SAMPLES
                        * bsp::ADC_TO_VIN_GAIN
                    + bsp::ADC_TO_VIN_OFFSET;
        float vmout1 = static_cast<float>(accum[bsp::ADCChannels::VMOUT1]) / bsp::NUM_ADC_SAMPLES
                           * bsp::ADC_TO_VMIN_GAIN
                       + bsp::ADC_TO_VMIN_OFFSET;
        float vmout2 = static_cast<float>(accum[bsp::ADCChannels::VMOUT2]) / bsp::NUM_ADC_SAMPLES
                           * bsp::ADC_TO_VMIN_GAIN
                       + bsp::ADC_TO_VMIN_OFFSET;

        // IIR filter
        auto iir = [](float& prior, float obs) { prior += bsp::ADC_IIR_ALPHA * (obs - prior); };
        iir(adcIIR[bsp::ADCChannels::VIN], vin);
        iir(adcIIR[bsp::ADCChannels::VMOUT1], vmout1);
        iir(adcIIR[bsp::ADCChannels::VMOUT2], vmout2);
    }

    /// @brief ADC full-conversion interrupt handler
    /// @param hadc
    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
    {
        if (hadc->Instance == ADC1)
        {
            // Read and filter ADC measurements
            ReadADC();
            // Increment conversion counter
            convCounter++;

            // Publish event if conversion counter exceeds period
            if (convCounter >= bsp::ADC_PUBLISH_PERIOD)
            {
                // Publish ADC event
                bsp::ADCEvt* evt = Q_NEW(bsp::ADCEvt, bsp::PublicSignals::ADC_SIG);
                evt->adcVoltages[bsp::ADCChannels::VIN]    = adcIIR[bsp::ADCChannels::VIN];
                evt->adcVoltages[bsp::ADCChannels::VMOUT1] = adcIIR[bsp::ADCChannels::VMOUT1];
                evt->adcVoltages[bsp::ADCChannels::VMOUT2] = adcIIR[bsp::ADCChannels::VMOUT2];
                QP::QF::PUBLISH(evt, nullptr);

                // Reset conversion counter
                convCounter = 0U;
            }
        }
    }

    /// @brief SysTick callback
    void SysTick_Handler(void);
    void SysTick_Handler(void)
    {
        QK_ISR_ENTRY();                     // Inform QK about entering an ISR
        HAL_IncTick();                      // Increment global timebase
        QP::QTimeEvt::TICK_X(0U, nullptr);  // Process QP time events at rate 0
        // ReadADC();
        QK_ISR_EXIT();  // Inform QK about exiting an ISR
    }

    /// @brief QP assertion callback
    /// @param module source file/module of the assert
    /// @param id assert code
    void assert_failed(char const* const module, int_t const id);
    void assert_failed(char const* const module, int_t const id)
    {
        Q_onError(module, id);
    }
}

namespace QP
{
/// @brief QF startup callback
void QF::onStartup()
{
    // Set up the SysTick timer to fire at bsp::TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / bsp::TICKS_PER_SEC);

    // Assign all priority bits for preemption-prio. And none to sub-prio.
    NVIC_SetPriorityGrouping(0U);

    // UART RX interrupt
    HAL_NVIC_SetPriority(USART1_IRQn, 4U, 4U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // GPIO interrupts
    // These are fault signals and thus are kernel unaware
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    // DMA interrupts
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4U, 4U);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // Start ADC DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, bsp::NUM_ADC_CHANNELS * bsp::NUM_ADC_SAMPLES);
}

/// @brief QF idle callback
void QK::onIdle() {}
}  // namespace QP

/// @brief USART1 interrupt handler
extern "C" void USART1_IRQHandler(void)
{
    QK_ISR_ENTRY();
    HAL_UART_IRQHandler(&huart1);
    QK_ISR_EXIT();
}

/// @brief EXT1 GPIO interrupt handler
/// @param
extern "C" void EXTI1_IRQHandler(void)
{
    QK_ISR_ENTRY();
    HAL_GPIO_EXTI_IRQHandler(P_M2_FAULT_Pin);
    QK_ISR_EXIT();
}

/// @brief EXT3 GPIO interrupt handler
/// @param
extern "C" void EXTI3_IRQHandler(void)
{
    QK_ISR_ENTRY();
    HAL_GPIO_EXTI_IRQHandler(P_M1_FAULT_Pin);
    QK_ISR_EXIT();
}

/// @brief ADC1 Channel 1 DMA interrupt handler
/// @param
extern "C" void DMA1_Channel1_IRQHandler(void)
{
    QK_ISR_ENTRY();
    HAL_DMA_IRQHandler(&hdma_adc1);
    QK_ISR_EXIT();
}

/// @brief System clock configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV4;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/// @brief Application error handler callback
// TODO: Get rid of this
void Error_Handler(void)
{
    __disable_irq();
    for (;;)
    {
    }
}
