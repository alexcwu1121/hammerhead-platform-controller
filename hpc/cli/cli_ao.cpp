#include "cli_ao.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "cli_binding.hpp"

// Define UART interrupt callback function to handle a received character with CLI
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    cli::CLIAO::Inst().ReceiveChar(huart);
}

cli::CLIAO::CLIAO()
    : QP::QActive(&initial),
      _processTimer(this, PrivateSignals::PROCESS_SIG, 0U),
      _retryTimer(this, PrivateSignals::RETRY_SIG, 0U),
      _fault(Fault::NO_FAULT)
{
}

void cli::CLIAO::Start(const QP::QPrioSpec priority)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage

    _isStarted = true;
}

cli::CLIAO::Fault cli::CLIAO::GetFault() const
{
    return _fault;
}

void cli::CLIAO::ClearFault()
{
    if (_isStarted)
    {
        // Clear fault
        _fault = Fault::NO_FAULT;
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

void cli::CLIAO::Printf(const char* format, ...)
{
    if (_isStarted)
    {
        PrintEvt* evt = Q_NEW(PrintEvt, PrivateSignals::PRINT_SIG);

        // Format the string using snprintf
        va_list args;
        va_start(args, format);
        int length = vsnprintf(evt->buf, sizeof(evt->buf), format, args);
        va_end(args);

        // Check if string fits in buffer
        if (length > 0)
        {
            POST(evt, this);
        }
        else
        {
            // otherwise, gc the event so it doesn't leak
            QP::QF::gc(evt);
        }
    }
}

void cli::CLIAO::ReceiveChar(UART_HandleTypeDef* huart)
{
    if (huart == _uartCliPeriph && _isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RX_CHAR_SIG);
        POST(&evt, this);
    }
}

void cli::CLIAO::InitBindings()
{
    // Command binding for the clear command
    CliCommandBinding clear_binding = {.name         = "clear",
                                       .help         = "Clears the console",
                                       .tokenizeArgs = true,
                                       .context      = NULL,
                                       .binding      = onClear};
    embeddedCliAddBinding(_cli, clear_binding);

    // Command binding for the led command
    CliCommandBinding led_binding = {.name         = "get-led",
                                     .help         = "Get led status",
                                     .tokenizeArgs = true,
                                     .context      = NULL,
                                     .binding      = onLed};
    embeddedCliAddBinding(_cli, led_binding);
}

Q_STATE_DEF(cli::CLIAO, initial)
{
    Q_UNUSED_PAR(e);
    return tran(&initializing);
}

Q_STATE_DEF(cli::CLIAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        status_ = tran(&initializing);
        break;
    }
    default:
    {
        status_ = super(&top);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(cli::CLIAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Initialize the CLI configuration settings
        EmbeddedCliConfig* config = embeddedCliDefaultConfig();
        config->cliBuffer         = _cliBuf;
        config->cliBufferSize     = _cliBufSize;
        config->rxBufferSize      = _cliRxBufSize;
        config->cmdBufferSize     = _cliCmdBufSize;
        config->historyBufferSize = _cliHistorySize;
        config->maxBindingCount   = _cliMaxBindingCount;

        // Create new CLI instance
        _cli = embeddedCliNew(config);

        // Define lambda to write char to cli
        auto write_char_to_cli = [](EmbeddedCli* embeddedCli, char c)
        {
            uint8_t char_to_send = c;
            HAL_UART_Transmit(_uartCliPeriph, &char_to_send, 1, 100);
        };

        // Assign character write function
        _cli->writeChar = write_char_to_cli;

        // CLI init failed. Is there not enough memory allocated to the CLI?
        // Please increase the 'CLI_BUFFER_SIZE' in header file.
        // Or decrease max binding / history size.
        // You can get required buffer size by calling
        // uint16_t requiredSize = embeddedCliRequiredSize(config);
        // Then check its value in debugger
        if (_cli == NULL)
        {
            static QP::QEvt evt(PrivateSignals::FAULT_SIG);
            POST(&evt, this);
        }
        else
        {
            static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
            POST(&evt, this);
        }

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::FAULT_SIG:
    {
        _fault  = Fault::INIT_FAILED;
        status_ = tran(&error);
        break;
    }
    case PrivateSignals::INITIALIZED_SIG:
    {
        // Add all the initial command bindings
        InitBindings();
        // Clear the cli
        onClear(_cli, NULL, NULL);
        status_ = tran(&active);
        break;
    }
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(cli::CLIAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Arm uart receive interrupt
        HAL_UART_Receive_IT(_uartCliPeriph, _uartRxBuf, _uartRxBufSize);
        // Arm cli process timer
        _processTimer.armX(_processInterval, _processInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disarm cli process timer
        _processTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::RX_CHAR_SIG:
    {
        HAL_UART_Receive_IT(_uartCliPeriph, _uartRxBuf, _uartRxBufSize);
        embeddedCliReceiveChar(_cli, _uartRxBuf[0]);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::PROCESS_SIG:
    {
        embeddedCliProcess(_cli);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::PRINT_SIG:
    {
        // Call embeddedCliPrint with the formatted string
        embeddedCliPrint(_cli, Q_EVT_CAST(PrintEvt)->buf);
        status_ = Q_RET_HANDLED;
        break;
    }
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(cli::CLIAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}
