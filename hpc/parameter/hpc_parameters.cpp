#include "hpc_parameters.hpp"

namespace param
{
DEFINE_PARAMETER(param::ParameterID::MC_PWM_DEADBAND, param::TypeID::UINT16,
                 param::Type{._uint16 = 75U}, "MC_PWM_DEADBAND",
                 "Motor controller PWM lower-end deadband (0<=val<=1023)")

DEFINE_PARAMETER(param::ParameterID::MC_RATE_STIFFNESS, param::TypeID::FLOAT32,
                 param::Type{._float32 = 0.001f}, "MC_RATE_STIFFNESS",
                 "Motor controller rate control stiffness (0.0<=val<=inf)");

DEFINE_PARAMETER(param::ParameterID::MC_RATE_DAMPING, param::TypeID::FLOAT32,
                 param::Type{._float32 = 0.05f}, "MC_RATE_DAMPING",
                 "Motor controller rate control damping (0.0<=val<=inf)");

DEFINE_PARAMETER(param::ParameterID::MC_UNDERVOLTAGE_FAULT_THRESHOLD, param::TypeID::FLOAT32,
                 param::Type{._float32 = 10.0f}, "MC_UNDERVOLTAGE_FAULT_THRESHOLD",
                 "Motor controller undervoltage threshold (V)");

DEFINE_PARAMETER(param::ParameterID::MC_OVERVOLTAGE_FAULT_THRESHOLD, param::TypeID::FLOAT32,
                 param::Type{._float32 = 14.0f}, "MC_OVERVOLTAGE_FAULT_THRESHOLD",
                 "Motor controller overvoltage threshold (V)");

DEFINE_PARAMETER(param::ParameterID::BATTERY_DISCHARGE_CURVE_C0, param::TypeID::FLOAT32,
                 param::Type{._float32 = 1.0f}, "BATTERY_DISCHARGE_CURVE_C0",
                 "Battery voltage/SOC curve 0th order coefficient");

DEFINE_PARAMETER(param::ParameterID::BATTERY_DISCHARGE_CURVE_C1, param::TypeID::FLOAT32,
                 param::Type{._float32 = 1.0f}, "BATTERY_DISCHARGE_CURVE_C1",
                 "Battery voltage/SOC curve 1st order coefficient");

DEFINE_PARAMETER(param::ParameterID::BATTERY_DISCHARGE_CURVE_C2, param::TypeID::FLOAT32,
                 param::Type{._float32 = 1.0f}, "BATTERY_DISCHARGE_CURVE_C2",
                 "Battery voltage/SOC curve 2nd order coefficient");

DEFINE_PARAMETER(param::ParameterID::BATTERY_DISCHARGE_CURVE_C3, param::TypeID::FLOAT32,
                 param::Type{._float32 = 1.0f}, "BATTERY_DISCHARGE_CURVE_C3",
                 "Battery voltage/SOC curve 3rd order coefficient");

DEFINE_PARAMETER(param::ParameterID::CLI_UPDATE_FREQ, param::TypeID::FLOAT32,
                 param::Type{._float32 = 20.0f}, "CLI_UPDATE_FREQ", "CLI update frequency (Hz)");
}  // namespace param
