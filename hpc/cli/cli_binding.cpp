#include "cli_binding.hpp"

#include <stdio.h>

#include "cli_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

void cli::onClear(EmbeddedCli *cli, char *args, void *context)
{
    cli::CLIAO::Inst().Printf("\33[2J");
}

void cli::onMC(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc = embeddedCliGetTokenCount(args);

    switch (argc)
    {
    case 1U:
    {
        // Reset
        break;
    }
    case 2U:
    {
        // Get fault
        break;
    }
    case 3U:
    {
        // Set duty
        // const char* cmd_str = embeddedCliGetToken(args, 1U);
        const char *motor_str = embeddedCliGetToken(args, 2U);
        const char *duty_str  = embeddedCliGetToken(args, 3U);
        uint32_t    motor     = strtoulS<uint32_t>(motor_str);
        uint32_t    duty      = strtoulS<uint32_t>(duty_str);

        if (motor == 0U)
        {
            mc::MotorControlAO::MC1Inst().SetDuty(duty);
        }
        else if (motor == 1U)
        {
            mc::MotorControlAO::MC2Inst().SetDuty(duty);
        }
        else
        {
            cli::CLIAO::Inst().Printf("Motor ID must be 0-1");
        }
        break;
    }
    default:
    {
        // Help dialogue
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tmc setduty [Motor ID (0-1)] [Duty (0-1023)]\n\r"
            "\tmc getfault [Motor ID (0-1)]\n\r"
            "\tmc reset\n\r");
        break;
    }
    }
}

void cli::onParam(EmbeddedCli *cli, char *args, void *context)
{
    // Get number of arguments
    uint16_t argc = embeddedCliGetTokenCount(args);

    switch (argc)
    {
    case 1U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "list") == 0)
        {
            param::ParamAO::Inst().List();
        }
        else if (strcmp(cmd_str, "commit") == 0)
        {
            param::ParamAO::Inst().Commit();
        }
        else if (strcmp(cmd_str, "reset-to-defaults") == 0)
        {
            param::ParamAO::Inst().ResetToDefaults();
        }
        break;
    }
    case 2U:
    {
        const char *cmd_str = embeddedCliGetToken(args, 1U);
        if (strcmp(cmd_str, "get") == 0)
        {
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
            param::Fault       fault = param::ParameterList::Inst().GetType(id, type);
            if (fault != param::Fault::NO_FAULT)
            {
                // TODO: handle
                break;
            }

            switch (type)
            {
            case param::TypeID::FLOAT32:
            {
                float val;
                val = strtofS(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::UINT8:
            {
                uint8_t val;
                val = strtoulS<uint8_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::UINT16:
            {
                uint16_t val;
                val = strtoulS<uint16_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::UINT32:
            {
                uint32_t val;
                val = strtoulS<uint32_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::INT8:
            {
                int8_t val;
                val = strtolS<int8_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::INT16:
            {
                int16_t val;
                val = strtolS<int16_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            case param::TypeID::INT32:
            {
                int32_t val;
                val = strtolS<int32_t>(value_str);
                // TODO: handle
                param::Fault fault = param::ParameterList::Inst().Set(id, val);
                break;
            }
            default:
                break;
            }
        }
        break;
    }
    default:
    {
        // Help dialogue
        // TODO: separate into function and run on every invalid command
        cli::CLIAO::Inst().Printf(
            "Usage:\n\r"
            "\tparam list\n\r"
            "\tparam commit\n\r"
            "\tparam reset-to-defaults\n\r"
            "\tparam set <id> <value>\n\r");
        break;
    }
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
}
