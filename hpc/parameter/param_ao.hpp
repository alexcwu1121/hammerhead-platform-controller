#ifndef PARAM_AO_HPP_
#define PARAM_AO_HPP_

#include "EEPROM_25LC256.hpp"
#include "bsp.hpp"
#include "parameter.hpp"
#include "qpcpp.hpp"

namespace param
{
class ParamAO : public QP::QActive
{
   public:
    ParamAO();
    static ParamAO& Inst()
    {
        static ParamAO inst;
        return inst;
    }

    /// @brief Start ParamAO
    /// @param priority
    void Start(const QP::QPrioSpec priority);

    /// @brief Set the value of a parameter
    void SetParam(ParameterID param, Type value);

    /// @brief Request a parameter update event
    void RequestUpdate(ParameterID param);

    /// @brief Commit parameter values to EEPROM
    void Commit();

    /// @brief Print full parameter list to CLI
    void List();

    /// @brief Reset parameter values to default
    void ResetToDefaults();

   private:
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief SPI receive buffer size
    static constexpr uint16_t _rxBufSize = 1024U;
    /// @brief SPI receive buffer
    uint8_t _rxBuf[_rxBufSize] = {0};
    /// @brief SPI transmit buffer size
    static constexpr uint16_t _txBufSize = 1024U;
    /// @brief SPI transmit buffer
    uint8_t _txBuf[_txBufSize] = {0};
    /// @brief EEPROM driver
    eeprom::EEPROM25LC256 _eeprom;
    /// @brief Base address of parameter block
    static constexpr uint16_t _paramBlockAddr = 0x0;
    // TODO: add maximum address of parameter block

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        SET_PARAM_VALUE_SIG = bsp::MAX_PUB_SIG,
        REQUEST_UPDATE_SIG,
        COMMIT_SIG,
        LIST_SIG,
        RESET_TO_DEFAULTS_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set parameter value event
    class SetParamValueEvt : public QP::QEvt
    {
       public:
        ParameterID id;
        Type        value;
    };

    /// @brief Request parameter update event
    class RequestUpdateEvt : public QP::QEvt
    {
       public:
        ParameterID id;
    };

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Process CLI events
    Q_STATE_DECL(active);
    /// @brief Fault
    Q_STATE_DECL(error);
};  // class ParamAO

}  // namespace param

#endif  // PARAM_AO_HPP_
