/*
 * cli_binding.c
 *
 *  Created on: Jul 7, 2023
 *      Author: NeusAap
 */

#include "cli_binding.hpp"

#include "cli_ao.hpp"
#include <stdio.h>

void onClear(EmbeddedCli* cli, char *args, void *context)
{
    cli::CLIAO::Inst().Printf("\33[2J");
}

void onLed(EmbeddedCli* cli, char *args, void *context)
{
    const char *arg1 = embeddedCliGetToken(args, 1);
    const char *arg2 = embeddedCliGetToken(args, 2);
    if (arg1 == NULL || arg2 == NULL) {
        cli::CLIAO::Inst().Printf("usage: get-led [arg1] [arg2]");
        return;
    }
    // Make sure to check if 'args' != NULL, printf's '%s' formatting does not like a null pointer.
    cli::CLIAO::Inst().Printf("LED with args: %s and %s", arg1, arg2);
}
