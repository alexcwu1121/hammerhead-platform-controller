#include "hpc_parameters.hpp"

namespace param
{
DEFINE_PARAMETER(param::ParameterID::M1_PWM_DEADBAND, param::TypeID::UINT16,
                 param::Type{._uint16 = 200U}, "M1_PWM_DEADBAND",
                 "Motor 1 PWM lower-end deadband (1-1023)")

DEFINE_PARAMETER(param::ParameterID::M2_PWM_DEADBAND, param::TypeID::UINT16,
                 param::Type{._uint16 = 200U}, "M2_PWM_DEADBAND",
                 "Motor 2 PWM lower-end deadband (1-1023)");
}  // namespace param
