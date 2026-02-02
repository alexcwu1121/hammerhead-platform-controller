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
    /// @brief Constructor
    ParamAO();
    ParamAO(const ParamAO&)            = delete;
    ParamAO& operator=(const ParamAO&) = delete;
    ParamAO(ParamAO&&)                 = delete;
    ParamAO& operator=(ParamAO&&)      = delete;

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
    inline void SetParam(ParameterID id, Type value);

    /// @brief Request a parameter update event
    inline void RequestUpdate(ParameterID id);

    /// @brief Commit parameter values to EEPROM
    inline void Commit();

    /// @brief Update parameter values from EEPROM
    inline void Update();

    /// @brief Print full parameter list to CLI
    inline void List();

    /// @brief Print a parameter to CLI
    inline void PrintParam(ParameterID id);

    /// @brief Reset parameter values to default
    inline void ResetToDefaults();

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
    /// @brief Fault states
    bool _faultStates[param::Fault::NUM_FAULTS] = {false};

    /// @brief Print a parameter
    /// @param id
    void PrintParam_h(ParameterID id);

    /// @brief Publish a parameter update event
    /// @param id
    void PublishParameterUpdated(ParameterID id);

    /// @brief Read parameters from EEPROM
    void ReadParameters();

    /// @brief Set and publish fault
    void SetFault(param::Fault fault, bool active);

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

inline void ParamAO::SetParam(ParameterID id, Type value)
{
    if (_isStarted)
    {
        SetParamValueEvt* evt = Q_NEW(SetParamValueEvt, PrivateSignals::SET_PARAM_VALUE_SIG);
        evt->id               = id;
        evt->value            = value;
        POST(evt, this);
    }
}

inline void ParamAO::RequestUpdate(ParameterID id)
{
    if (_isStarted)
    {
        ParamIndexEvt* evt = Q_NEW(ParamIndexEvt, PrivateSignals::REQUEST_UPDATE_SIG);
        evt->id            = id;
        POST(evt, this);
    }
}

inline void ParamAO::Commit()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::COMMIT_SIG);
        POST(&evt, this);
    }
}

inline void ParamAO::Update()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::UPDATE_SIG);
        POST(&evt, this);
    }
}

inline void ParamAO::List()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::LIST_SIG);
        POST(&evt, this);
    }
}

inline void ParamAO::PrintParam(ParameterID id)
{
    if (_isStarted)
    {
        ParamIndexEvt* evt = Q_NEW(ParamIndexEvt, PrivateSignals::PRINT_PARAM_SIG);
        evt->id            = id;
        POST(evt, this);
    }
}

inline void ParamAO::ResetToDefaults()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_TO_DEFAULTS_SIG);
        POST(&evt, this);
    }
}
}  // namespace param

#endif  // PARAM_AO_HPP_
