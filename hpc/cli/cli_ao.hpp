#ifndef CLI_AO_HPP_
#define CLI_AO_HPP_

#include "bsp.hpp"
#include "embedded_cli.h"
#include "qpcpp.hpp"
#include "usart.h"

namespace cli
{
/// @brief Fault codes
enum Fault : uint8_t
{
    NO_FAULT,
    INIT_FAILED
};

class CLIAO : public QP::QActive
{
   public:
    CLIAO();
    static CLIAO& Inst()
    {
        static CLIAO inst;
        return inst;
    }

    /// @brief Start CLIAO
    /// @param priority
    void Start(const QP::QPrioSpec priority);

    /// @brief Get active fault
    Fault GetFault() const
    {
        return _fault;
    }

    /// @brief Attempt to clear fault and reinitialize
    void ClearFault();

    /// @brief Print formatted string to CLI
    /// @param format format string
    /// @param args
    void Printf(const char* format, ...);

    /// @brief Receive an incoming character
    void ReceiveChar(UART_HandleTypeDef* huart);

   private:
    /// @brief CLI total memory size
    static constexpr uint16_t _cliBufSize = 1048U;
    /// @brief CLI receive buffer size
    static constexpr uint16_t _cliRxBufSize = 16U;
    /// @brief CLI staged command buffer size
    static constexpr uint16_t _cliCmdBufSize = 64U;
    /// @brief CLI command history size
    static constexpr uint16_t _cliHistorySize = 64U;
    /// @brief Maximum number of CLI bindings
    static constexpr uint16_t _cliMaxBindingCount = 32U;
    /// @brief Maximum size of string to print
    static constexpr uint16_t _cliPrintBufSize = 240U;
    /// @brief UART Rx buffer size (one char at a time)
    static constexpr uint16_t _uartRxBufSize = 1U;
    /// @brief CLI memory buffer
    CLI_UINT _cliBuf[BYTES_TO_CLI_UINTS(_cliBufSize)] = {0};
    /// @brief UART buffer
    uint8_t _uartRxBuf[_uartRxBufSize] = {0};
    /// @brief UART peripheral
    static constexpr UART_HandleTypeDef* _uartCliPeriph = &huart1;
    /// @brief Ptr to embedded CLI instance
    EmbeddedCli* _cli;

    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief CLI process interval in ticks
    static constexpr uint32_t _processInterval = bsp::TICKS_PER_SEC / 100;
    /// @brief CLI recovery retry interval in ticks
    static constexpr uint32_t _retryInterval = bsp::TICKS_PER_SEC;
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief CLI process timed event
    QP::QTimeEvt _processTimer;
    /// @brief Retry timed event
    QP::QTimeEvt _retryTimer;
    /// @brief Last fault
    Fault _fault;

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
        MAX_PRIV_SIG
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

#endif  // CLI_AO_HPP_
