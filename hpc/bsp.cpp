#include "bsp.hpp"

extern "C"
{
Q_NORETURN Q_onError(char const * const module, int_t const id)
{
    // NOTE: this implementation of the error handler is intended only
    // for debugging and MUST be changed for deployment of the application
    // (assuming that you ship your production code with assertions enabled).
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);
    QS_ASSERTION(module, id, 10000U); // report assertion to QS

    // Reset
    // Breakpoint here
    NVIC_SystemReset();
    for (;;) { }
}

/// @brief SysTick callback
void SysTick_Handler(void);
void SysTick_Handler(void)
{
    QK_ISR_ENTRY(); // Inform QK about entering an ISR
    HAL_IncTick(); // Increment global timebase
    QP::QTimeEvt::TICK_X(0U, nullptr); // Process QP time events at rate 0
    QK_ISR_EXIT();  // Inform QK about exiting an ISR
}

/// @brief QP assertion callback
/// @param module source file/module of the assert
/// @param id assert code
void assert_failed(char const * const module, int_t const id);
void assert_failed(char const * const module, int_t const id) { Q_onError(module, id); }
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

    // Set up interrupt priorities
    HAL_NVIC_SetPriority(USART1_IRQn, 4U, 4U);

    // Enable interrupts
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/// @brief QF idle callback
void QK::onIdle() { }
}

/// @brief USART1 interrupt handler
extern "C" void USART1_IRQHandler(void)
{
  QK_ISR_ENTRY();
  HAL_UART_IRQHandler(&huart1);
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
void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}
