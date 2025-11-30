#include "cli_binding.hpp"

#include <stdio.h>

#include "cli_ao.hpp"

void cli::onClear(EmbeddedCli *cli, char *args, void *context)
{
    cli::CLIAO::Inst().Printf("\33[2J");
}

void cli::onLed(EmbeddedCli *cli, char *args, void *context)
{
    const char *arg1 = embeddedCliGetToken(args, 1);
    const char *arg2 = embeddedCliGetToken(args, 2);
    if (arg1 == NULL || arg2 == NULL)
    {
        cli::CLIAO::Inst().Printf("usage: get-led [arg1] [arg2]");
        return;
    }
    // Make sure to check if 'args' != NULL, printf's '%s' formatting does not like a null pointer.
    cli::CLIAO::Inst().Printf("LED with args: %s and %s", arg1, arg2);
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

    // Command binding for the led command
    CliCommandBinding led_binding = {.name         = "get-led",
                                     .help         = "Get led status",
                                     .tokenizeArgs = true,
                                     .context      = NULL,
                                     .binding      = onLed};
    embeddedCliAddBinding(cli, led_binding);
}
