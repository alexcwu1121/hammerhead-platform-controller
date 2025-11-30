#ifndef CLI_AO_HPP_
#define CLI_AO_HPP_

#include "bsp.hpp"
#include "embedded_cli.h"
#include "usart.h"
#include "qpcpp.hpp"

namespace cli
{
class CLIAO : public QP::QActive
{
public:
    CLIAO();
    static CLIAO& Inst()
    {
        static CLIAO inst;
        return inst;
    }

    /// @brief Fault codes
    enum Fault : uint8_t
    {
        NO_FAULT,
        INIT_FAILED
    };

    /// @brief Start CLIAO
    /// @param priority 
    void Start(const QP::QPrioSpec priority);

    /// @brief Get active fault
    Fault GetFault() const;

    /// @brief Attempt to clear fault and reinitialize
    void ClearFault();

    /// @brief Print formatted string to CLI
    /// @param format format string
    /// @param args
    void Printf(const char *format, ...);

    /// @brief Receive an incoming character
    void ReceiveChar(UART_HandleTypeDef* huart);

private:
    static constexpr uint16_t _queueSize = 128;
    static constexpr uint16_t _cliBufSize = 2048;
    static constexpr uint16_t _cliRxBufSize = 16;
    static constexpr uint16_t _cliCmdBufSize = 32;
    static constexpr uint16_t _cliHistorySize = 32;
    static constexpr uint16_t _cliMaxBindingCount = 32;
    static constexpr uint16_t _cliPrintBufSize = 240;
    static constexpr uint16_t _uartRxBufSize = 1;
    /// @brief CLI process interval in ticks
    static constexpr uint32_t _processInterval = bsp::TICKS_PER_SEC/100;
    /// @brief CLI recovery retry interval in ticks
    static constexpr uint32_t _retryInterval = bsp::TICKS_PER_SEC;

    /// @brief UART peripheral
    static constexpr UART_HandleTypeDef* _uartCliPeriph = &huart1;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief UART buffer
    uint8_t _uartRxBuf[_uartRxBufSize] = {0};
    /// @brief CLI buffer
    CLI_UINT _cliBuf[BYTES_TO_CLI_UINTS(_cliBufSize)];
    /// @brief Embedded CLI instance
    EmbeddedCli* _cli;
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief CLI process timed event
    QP::QTimeEvt _processTimer;
    /// @brief Retry timed event
    QP::QTimeEvt _retryTimer;
    /// @brief Last fault
    Fault _fault;

    /// @brief Initialize CLI bindings
    void InitBindings();

private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        PRINT_SIG,
        RX_CHAR_SIG,
        PROCESS_SIG,
        RETRY_SIG,
        MAX_PUB_SIG
    };

    /// @brief Print string evt
    class PrintEvt : public QP::QEvt
    {
    public:
        char buf[_cliPrintBufSize];
    };
    
    /// @brief Initial state 
    Q_STATE_DECL(initial);
    /// @brief Root state 
    Q_STATE_DECL(root);
    /// @brief Register interrupts, callbacks
    Q_STATE_DECL(initializing);
    /// @brief Process CLI events 
    Q_STATE_DECL(active);
    /// @brief Fault 
    Q_STATE_DECL(error);
};  // class CLIAO

}  // namespace cli

#endif // CLI_AO_HPP_