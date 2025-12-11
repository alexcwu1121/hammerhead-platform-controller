#ifndef HPC_PARAMETERS_HPP_
#define HPC_PARAMETERS_HPP_

#include "parameter.hpp"

namespace param
{
enum class ParameterID : param::ParamIndex
{
    M1_PWM_DEADBAND = 0U,
    M2_PWM_DEADBAND,
    CLI_UPDATE_FREQ,
};
}

#endif
