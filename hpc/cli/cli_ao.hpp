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
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Reinitialize and attempt to clear faults
    void Reset();

    // TODO: wrap in removable macro for release-no-debug builds
    /// @brief Print formatted string to CLI
    /// @param fmt format string
    /// @param args
    void Printf(const char* fmt, ...);

    /// @brief Receive an incoming character
    void ReceiveChar(UART_HandleTypeDef* huart);

   private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
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
    static constexpr uint16_t _cliPrintBufSize = 500U;
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
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief CLI process timed event
    QP::QTimeEvt _processTimer;
    /// @brief CLI process interval in ticks
    uint32_t _processInterval = UINT32_MAX;

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        PRINT_SIG,
        RX_CHAR_SIG,
        PROCESS_SIG,
        PARAMS_UPDATED,
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
