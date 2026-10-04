#include "hpc_parameters.hpp"

// clang-format off
namespace param
{
DEFINE_PARAMETER(
    param::ParameterID::MC_PWM_DEADBAND,
    param::TypeID::UINT16,
    param::Type {._uint16 = 75U},
    "MC_PWM_DEADBAND",
    "Motor controller PWM lower-end deadband (0<=val<=1023)")

DEFINE_PARAMETER(
    param::ParameterID::MC_RATE_STIFFNESS,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 0.001f},
    "MC_RATE_STIFFNESS",
    "Motor controller rate control stiffness (0.0<=val<=inf)");

DEFINE_PARAMETER(
    param::ParameterID::MC_RATE_DAMPING,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 0.05f},
    "MC_RATE_DAMPING",
    "Motor controller rate control damping (0.0<=val<=inf)");

DEFINE_PARAMETER(
    param::ParameterID::MC_UNDERVOLTAGE_FAULT_THRESHOLD,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 10.0f},
    "MC_UNDERVOLTAGE_FAULT_THRESHOLD",
    "Motor controller undervoltage threshold (V)");

DEFINE_PARAMETER(
    param::ParameterID::MC_OVERVOLTAGE_FAULT_THRESHOLD,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 14.0f},
    "MC_OVERVOLTAGE_FAULT_THRESHOLD",
    "Motor controller overvoltage threshold (V)");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_V0,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 12.4f},
    "BATTERY_DISCHARGE_CURVE_V0",
    "Battery voltage/SOC curve linear interpolant model voltage 1");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_S0,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 0.0f},
    "BATTERY_DISCHARGE_CURVE_S0",
    "Battery voltage/SOC curve linear interpolant model SOC 1");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_V1,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 14.0f},
    "BATTERY_DISCHARGE_CURVE_V1",
    "Battery voltage/SOC curve linear interpolant model voltage 2");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_S1,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 0.1f},
    "BATTERY_DISCHARGE_CURVE_S1",
    "Battery voltage/SOC curve linear interpolant model SOC 2");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_V2,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 15.2f},
    "BATTERY_DISCHARGE_CURVE_V2",
    "Battery voltage/SOC curve linear interpolant model voltage 3");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_S2,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 0.9f},
    "BATTERY_DISCHARGE_CURVE_S2",
    "Battery voltage/SOC curve linear interpolant model SOC 3");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_V3,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 16.8f},
    "BATTERY_DISCHARGE_CURVE_V3",
    "Battery voltage/SOC curve linear interpolant model voltage 4");

DEFINE_PARAMETER(
    param::ParameterID::BATTERY_DISCHARGE_CURVE_S3,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 1.0f},
    "BATTERY_DISCHARGE_CURVE_S3",
    "Battery voltage/SOC curve linear interpolant model SOC 4");

DEFINE_PARAMETER(
    param::ParameterID::CLI_UPDATE_FREQ,
    param::TypeID::FLOAT32,
    param::Type {._float32 = 20.0f},
    "CLI_UPDATE_FREQ",
    "CLI update frequency (Hz)");
}  // namespace param
// clang-format on
