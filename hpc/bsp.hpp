#ifndef BSP_HPP_
#define BSP_HPP_

#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "qpcpp.hpp"

namespace bsp
{
/// @brief Number of ticks per second
constexpr std::uint32_t TICKS_PER_SEC {1000U};

/// @brief Public QP signals
enum PublicSignals : QP::QSignal
{
    DUMMY_SIG = QP::Q_USER_SIG,
    MAX_PUB_SIG
};

/// @brief Public QP events

}

/// @brief System clock configuration
void SystemClock_Config(void);

#endif // BSP_HPP_
