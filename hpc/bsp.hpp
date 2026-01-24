#ifndef BSP_HPP_
#define BSP_HPP_

#include "adc.h"
#include "dma.h"
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
/// @brief Number of samples per channel
constexpr uint16_t NUM_ADC_SAMPLES = 8U;
/// @brief Period at which ADC publishes QP events in # conversions
constexpr uint16_t ADC_PUBLISH_PERIOD = 200U;
/// @brief ADC IIR filter learning rate
constexpr float ADC_IIR_ALPHA = 0.1f;

/// @brief ADC Channels
enum ADCChannels : uint8_t
{
    VIN,
    VMOUT1,
    VMOUT2,
    NUM_ADC_CHANNELS
};

/// @brief VIN voltage divider R1
constexpr float VIN_DIV_R1 = 100000.0f;
/// @brief VIN voltage divider R2
constexpr float VIN_DIV_R2 = 10000.0f;
/// @brief Motor driver output voltage divider R1
constexpr float VMIN_DIV_R1 = 68000.0f;
/// @brief Motor driver output voltage divider R2
constexpr float VMIN_DIV_R2 = 10000.0f;
/// @brief ADC resolution
constexpr float ADC_RES = 4096.0f;
/// @brief ADC reference voltage
constexpr float ADC_VREF = 3.3f;
// TODO make the following into parameters
/// @brief ADC to pre-voltage divider VIN voltage gain
constexpr float ADC_TO_VIN_GAIN = ADC_VREF / ADC_RES * (VIN_DIV_R1 + VIN_DIV_R2) / VIN_DIV_R2;
/// @brief ADC to pre-voltage divider motor output voltage gain
constexpr float ADC_TO_VMIN_GAIN = ADC_VREF / ADC_RES * (VMIN_DIV_R1 + VMIN_DIV_R2) / VMIN_DIV_R2;
/// @brief ADC to pre-voltage divider VIN voltage offset
constexpr float ADC_TO_VIN_OFFSET = 0.2288208f;
/// @brief ADC to pre-voltage divider motor output voltage offset
constexpr float ADC_TO_VMIN_OFFSET = 0.3917236f;

/// @brief Subsystem IDs
enum SubsystemID : uint8_t
{
    PARAMETER_SUBSYSTEM = 0U,
    MC1_SUBSYSTEM,
    MC2_SUBSYSTEM,
    CLI_SUBSYSTEM,
    MISSION_SUBSYSTEM,
    NUM_SUBSYSTEMS  // Keep this last
};

/// @brief Public QP signals
enum PublicSignals : QP::QSignal
{
    PARAMETER_UPDATE_SIG = QP::Q_USER_SIG,
    ADC_SIG,
    FAULT_SIG,
    REQUEST_FAULT_SIG,
    MAX_PUB_SIG  // Keep this last
};

/// @brief Parameter update event
class ParameterUpdateEvt : public QP::QEvt
{
   public:
    /// @brief Parameter id
    param::ParameterID id;
    /// @brief Parameter value
    param::Type value;
};

/// @brief Maximum number of faults each subsystem may implement
constexpr uint8_t MAX_SUBSYSTEM_FAULTS = 16U;

/// @brief Fault event
class FaultEvt : public QP::QEvt
{
   public:
    /// @brief Originating subsystem
    SubsystemID id;
    /// @brief Fault code
    uint8_t fault;
    /// @brief Fault active/inactive
    bool active;
};

/// @brief ADC event
class ADCEvt : public QP::QEvt
{
   public:
    /// @brief ADC voltages
    float adcVoltages[ADCChannels::NUM_ADC_CHANNELS];
};
}  // namespace bsp

/// @brief System clock configuration
void SystemClock_Config(void);

#endif  // BSP_HPP_
