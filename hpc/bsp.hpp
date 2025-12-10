#ifndef BSP_HPP_
#define BSP_HPP_

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "parameter.hpp"
#include "qpcpp.hpp"
#include "spi.h"
#include "tim.h"
#include "usart.h"

namespace bsp
{
/// @brief Number of ticks per second
constexpr std::uint32_t TICKS_PER_SEC{1000U};

/// @brief Public QP signals
enum PublicSignals : QP::QSignal
{
    PARAMETER_UPDATE_EVT = QP::Q_USER_SIG,
    MAX_PUB_SIG
};

/// @brief Parameter update event
class ParameterUpdateEvt : public QP::QEvt
{
   public:
    param::ParameterID id;
    param::Type        value;
};
}  // namespace bsp

/// @brief System clock configuration
void SystemClock_Config(void);

#endif  // BSP_HPP_
