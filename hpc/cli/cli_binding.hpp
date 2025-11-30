#ifndef INC_CLI_BINDING_H_
#define INC_CLI_BINDING_H_

#include "embedded_cli.h"

namespace cli
{
/// @brief Clear binding clears cli
/// @param cli
/// @param args
/// @param context
void onClear(EmbeddedCli *cli, char *args, void *context);

/// @brief Placeholder cli binding
/// @param cli
/// @param args
/// @param context
void onLed(EmbeddedCli *cli, char *args, void *context);

/// @brief Initialize bindings for a CLI
/// @param cli ptr to CLI instance
void InitBindings(EmbeddedCli *cli);
}  // namespace cli

#endif  // INC_CLI_BINDING_H_
