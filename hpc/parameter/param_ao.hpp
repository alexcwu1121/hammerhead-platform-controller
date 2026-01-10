#ifndef PARAM_AO_HPP_
#define PARAM_AO_HPP_

#include "EEPROM_25LC256.hpp"
#include "bsp.hpp"
#include "hpc_parameters.hpp"
#include "parameter.hpp"
#include "qpcpp.hpp"

namespace param
{
// Parameter buffer size
#define PARAM_BUF_SIZE static_cast<uint16_t>(ParameterID::NUM_PARAMS) * sizeof(ParameterPayload)

/// @brief Parameter management Active Object
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
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Set the value of a parameter
    void SetParam(ParameterID id, Type value);

    /// @brief Request a parameter update event
    void RequestUpdate(ParameterID id);

    /// @brief Commit parameter values to EEPROM
    void Commit();

    /// @brief Update parameter values from EEPROM
    void Update();

    /// @brief Print full parameter list to CLI
    void List();

    /// @brief Print a parameter to CLI
    void PrintParam(ParameterID id);

    /// @brief Reset parameter values to default
    void ResetToDefaults();

   private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief SPI receive buffer size
    static constexpr uint16_t _rxBufSize = PARAM_BUF_SIZE;
    /// @brief SPI receive buffer
    uint8_t _rxBuf[_rxBufSize] = {0};
    /// @brief SPI transmit buffer size
    static constexpr uint16_t _txBufSize = PARAM_BUF_SIZE;
    /// @brief SPI transmit buffer
    uint8_t _txBuf[_txBufSize] = {0};
    /// @brief EEPROM driver
    eeprom::EEPROM25LC256 _eeprom;
    /// @brief Base address of parameter block
    static constexpr uint16_t _paramBlockAddr = 0x0;
    // TODO: add maximum address of parameter block
    /// @brief number of read retries
    static constexpr uint16_t _readRetryCount = 3U;

    /// @brief Print a parameter
    /// @param id
    void PrintParam_h(ParameterID id);

    /// @brief Publish a parameter update event
    /// @param id
    void PublishParameterUpdated(ParameterID id);

    /// @brief Read parameters from EEPROM
    void ReadParameters();

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        SET_PARAM_VALUE_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        REQUEST_UPDATE_SIG,
        COMMIT_SIG,
        LIST_SIG,
        RESET_TO_DEFAULTS_SIG,
        PRINT_PARAM_SIG,
        UPDATE_SIG,
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
    class ParamIndexEvt : public QP::QEvt
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
