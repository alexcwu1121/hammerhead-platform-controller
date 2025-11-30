/*
 * cli_binding.h
 *
 *  Created on: Jul 7, 2023
 *      Author: NeusAap
 */

#ifndef INC_CLI_BINDING_H_
#define INC_CLI_BINDING_H_

#include "embedded_cli.h"

/**
 * Clears the whole terminal.
 */
void onClear(EmbeddedCli *cli, char *args, void *context);

/**
 * Example callback function, that also parses 2 arguments,
 * and has an 'incorrect usage' output.
 */
void onLed(EmbeddedCli *cli, char *args, void *context);

#endif /* INC_CLI_BINDING_H_ */
