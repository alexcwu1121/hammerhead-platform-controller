#include "cli_binding.hpp"

#include "cli_ao.hpp"
#include "mission_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"
#include "thirdparty/embedded_cli.h"

void cli::onClear(EmbeddedCli *cli, char *args, void *context)
{
    cli::CLIAO::Inst().Printf("\33[2J");
}

void cli::onMC(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc    = embeddedCliGetTokenCount(args);
    bool     handled = false;

    switch (argc)
    {
    case 1U:
    {
        break;
    }
    case 2U:
    {
        const char *cmd_str   = embeddedCliGetToken(args, 1U);
        const char *motor_str = embeddedCliGetToken(args, 2U);
        if (strcmp(cmd_str, "reset") == 0)
        {
            if (strcmp(motor_str, "all") == 0)
            {
                mc::MotorControlAO::MC1Inst().Reset();
                mc::MotorControlAO::MC2Inst().Reset();
                handled = true;
            }
            else
            {
                uint32_t motor = strtoulS<uint32_t>(motor_str);

                if (motor == 0U)
                {
                    mc::MotorControlAO::MC1Inst().Reset();
                    handled = true;
                }
                else if (motor == 1U)
                {
                    mc::MotorControlAO::MC2Inst().Reset();
                    handled = true;
                }
            }
        }
        break;
    }
    case 3U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "setmode") == 0)
        {
            const char *motor_str = embeddedCliGetToken(args, 2U);
            const char *mode_str  = embeddedCliGetToken(args, 3U);
            mc::Mode    mode      = static_cast<mc::Mode>(strtoulS<uint32_t>(mode_str));

            if (strcmp(motor_str, "all") == 0)
            {
                mc::MotorControlAO::MC1Inst().SetMode(mode);
                mc::MotorControlAO::MC2Inst().SetMode(mode);
                handled = true;
            }
            else
            {
                uint32_t motor = strtoulS<uint32_t>(motor_str);

                if (motor == 0U)
                {
                    mc::MotorControlAO::MC1Inst().SetMode(mode);
                    handled = true;
                }
                else if (motor == 1U)
                {
                    mc::MotorControlAO::MC2Inst().SetMode(mode);
                    handled = true;
                }
            }
        }
        else if (strcmp(cmd_str, "setduty") == 0)
        {
            const char *motor_str = embeddedCliGetToken(args, 2U);
            const char *duty_str  = embeddedCliGetToken(args, 3U);
            uint32_t    duty      = strtoulS<uint32_t>(duty_str);

            if (strcmp(motor_str, "all") == 0)
            {
                mc::MotorControlAO::MC1Inst().SetDuty(duty);
                mc::MotorControlAO::MC2Inst().SetDuty(duty);
                handled = true;
            }
            else
            {
                uint32_t motor = strtoulS<uint32_t>(motor_str);

                if (motor == 0U)
                {
                    mc::MotorControlAO::MC1Inst().SetDuty(duty);
                    handled = true;
                }
                else if (motor == 1U)
                {
                    mc::MotorControlAO::MC2Inst().SetDuty(duty);
                    handled = true;
                }
            }
        }
        else if (strcmp(cmd_str, "setdir") == 0)
        {
            const char *motor_str = embeddedCliGetToken(args, 2U);
            const char *dir_str   = embeddedCliGetToken(args, 3U);
            mc::Dir     dir       = static_cast<mc::Dir>(strtoulS<uint32_t>(dir_str));

            if (strcmp(motor_str, "all") == 0)
            {
                mc::MotorControlAO::MC1Inst().SetDir(dir);
                mc::MotorControlAO::MC2Inst().SetDir(dir);
                handled = true;
            }
            else
            {
                uint32_t motor = strtoulS<uint32_t>(motor_str);

                if (motor == 0U)
                {
                    mc::MotorControlAO::MC1Inst().SetDir(dir);
                    handled = true;
                }
                else if (motor == 1U)
                {
                    mc::MotorControlAO::MC2Inst().SetDir(dir);
                    handled = true;
                }
            }
        }
        else if (strcmp(cmd_str, "setrate") == 0)
        {
            const char *motor_str = embeddedCliGetToken(args, 2U);
            const char *rate_str  = embeddedCliGetToken(args, 3U);
            float       rate      = strtofS(rate_str);

            if (strcmp(motor_str, "all") == 0)
            {
                mc::MotorControlAO::MC1Inst().SetRate(rate);
                mc::MotorControlAO::MC2Inst().SetRate(rate);
                handled = true;
            }
            else
            {
                uint32_t motor = strtoulS<uint32_t>(motor_str);

                if (motor == 0U)
                {
                    mc::MotorControlAO::MC1Inst().SetRate(rate);
                    handled = true;
                }
                else if (motor == 1U)
                {
                    mc::MotorControlAO::MC2Inst().SetRate(rate);
                    handled = true;
                }
            }
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tmc setmode all [Mode (0|1)] --- 0==DUTY, 1==RATE\n\r"
            "\tmc setmode [Motor ID (0 |1)] [Mode (0|1)] --- 0==DUTY, 1==RATE\n\r"
            "\tmc setdir all [Dir (0|1)] --- 0==CW, 1==CCW\n\r"
            "\tmc setdir [Motor ID (0 |1)] [Dir (0|1)] --- 0==CW, 1==CCW\n\r"
            "\tmc setduty all [Duty (0<=val<=1023)]\n\r"
            "\tmc setduty [Motor ID (0|1)] [Duty (0<=val<=1023)]\n\r"
            "\tmc setrate all [Rate (-1.0<=val<=1.0)]\n\r"
            "\tmc setrate [Motor ID (0|1)] [Rate (-1.0<=val<=1.0)]\n\r"
            "\tmc reset all\n\r"
            "\tmc reset [MotorID (0|1)]\n\r");
    }
}

void cli::onParam(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc    = embeddedCliGetTokenCount(args);
    bool     handled = false;

    switch (argc)
    {
    case 1U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "list") == 0)
        {
            param::ParamAO::Inst().List();
            handled = true;
        }
        else if (strcmp(cmd_str, "commit") == 0)
        {
            param::ParamAO::Inst().Commit();
            handled = true;
        }
        else if (strcmp(cmd_str, "update") == 0)
        {
            param::ParamAO::Inst().Update();
            handled = true;
        }
        else if (strcmp(cmd_str, "reset-to-defaults") == 0)
        {
            param::ParamAO::Inst().ResetToDefaults();
            handled = true;
        }
        break;
    }
    case 2U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "get") == 0)
        {
            const char        *id_str = embeddedCliGetToken(args, 2U);
            param::ParameterID id     = static_cast<param::ParameterID>(strtoulS<uint8_t>(id_str));
            param::ParamAO::Inst().PrintParam(id);

            handled = true;
        }
        break;
    }
    case 3U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "set") == 0)
        {
            const char        *id_str    = embeddedCliGetToken(args, 2U);
            const char        *value_str = embeddedCliGetToken(args, 3U);
            param::ParameterID id = static_cast<param::ParameterID>(strtoulS<uint8_t>(id_str));
            param::TypeID      type;
            // Parameter types should never be modified at runtime
            param::Fault fault = param::ParameterList::Inst().GetType(id, type);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                break;
            }

            // Set value
            param::Type val;
            switch (type)
            {
            case param::TypeID::FLOAT32:
            {
                val._float32 = strtofS(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::UINT8:
            {
                val._uint8 = strtoulS<uint8_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::UINT16:
            {
                val._uint16 = strtoulS<uint16_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::UINT32:
            {
                val._uint32 = strtoulS<uint32_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::INT8:
            {
                val._int8 = strtolS<int8_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::INT16:
            {
                val._int16 = strtolS<int16_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            case param::TypeID::INT32:
            {
                val._int32 = strtolS<int32_t>(value_str);
                param::ParamAO::Inst().SetParam(id, val);
                break;
            }
            default:
            {
                break;
            }
            }

            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
            }

            handled = true;
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tparam list\n\r"
            "\tparam commit\n\r"
            "\tparam update\n\r"
            "\tparam reset-to-defaults\n\r"
            "\tparam set [id] [value]\n\r"
            "\tparam get [id]\n\r");
    }
}

void cli::onIMU(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc    = embeddedCliGetTokenCount(args);
    bool     handled = false;

    switch (argc)
    {
    case 1U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "run_compensation") == 0)
        {
            mission::MissionAO::Inst().RunIMUCompensation();
            handled = true;
        }
        else if (strcmp(cmd_str, "start_stream") == 0)
        {
            mission::MissionAO::Inst().StartIMUStream();
            handled = true;
        }
        else if (strcmp(cmd_str, "stop_stream") == 0)
        {
            mission::MissionAO::Inst().StopIMUStream();
            handled = true;
        }
        else if (strcmp(cmd_str, "reset") == 0)
        {
            mission::MissionAO::Inst().Reset();
            handled = true;
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\timu run_compensation\n\r"
            "\timu start_stream\n\r"
            "\timu stop_stream\n\r"
            "\timu reset\n\r");
    }
}

void cli::onMission(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc    = embeddedCliGetTokenCount(args);
    bool     handled = false;

    switch (argc)
    {
    case 1U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "print_fault") == 0)
        {
            mission::MissionAO::Inst().PrintFault();
            handled = true;
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (!handled)
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tmission print_fault\n\r");
    }
}

void cli::InitBindings(EmbeddedCli *cli)
{
    // Command binding for the clear command
    CliCommandBinding clear_binding = {.name         = "clear",
                                       .help         = "Clears the console",
                                       .tokenizeArgs = true,
                                       .context      = NULL,
                                       .binding      = onClear};
    embeddedCliAddBinding(cli, clear_binding);

    // Command binding for the motor controller command
    CliCommandBinding mc_binding = {.name         = "mc",
                                    .help         = "Control motors",
                                    .tokenizeArgs = true,
                                    .context      = NULL,
                                    .binding      = onMC};
    embeddedCliAddBinding(cli, mc_binding);

    // Command binding for the parameter system command
    CliCommandBinding param_binding = {.name         = "param",
                                       .help         = "Manage parameters",
                                       .tokenizeArgs = true,
                                       .context      = NULL,
                                       .binding      = onParam};
    embeddedCliAddBinding(cli, param_binding);

    // Command binding for the IMU system command
    CliCommandBinding imu_binding = {.name         = "imu",
                                     .help         = "Manage IMU",
                                     .tokenizeArgs = true,
                                     .context      = NULL,
                                     .binding      = onIMU};
    embeddedCliAddBinding(cli, imu_binding);

    // Command binding for the mission system command
    CliCommandBinding mission_binding = {.name         = "mission",
                                         .help         = "Manage system",
                                         .tokenizeArgs = true,
                                         .context      = NULL,
                                         .binding      = onMission};
    embeddedCliAddBinding(cli, mission_binding);
}
