#ifndef MISSION_AO_HPP_
#define MISSION_AO_HPP_

#include "bsp.hpp"

namespace mission
{
/// @brief Fault codes
enum Fault : uint8_t
{
    MISSION_INIT_FAILED = 0U,
    BATT_LOW,
    BATT_CRITICAL,
    MISSION_CAN_TX_FAILED,
    WATCHDOG_FAULT,
    NUM_FAULTS
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(Fault fault)
{
    switch (fault)
    {
        case Fault::MISSION_INIT_FAILED:
        {
            return "MISSION_INIT_FAILED";
        }
        case Fault::BATT_LOW:
        {
            return "BATT_LOW";
        }
        case Fault::BATT_CRITICAL:
        {
            return "BATT_CRITICAL";
        }
        case Fault::MISSION_CAN_TX_FAILED:
        {
            return "MISSION_CAN_TX_FAILED";
        }
        case Fault::WATCHDOG_FAULT:
        {
            return "WATCHDOG_FAULT";
        }
        default:
        {
            return "";
        }
    }
}

/// @brief Mission AO
class MissionAO : public QP::QActive
{
public:
    /// @brief Constructor
    MissionAO();
    ~MissionAO() = default;
    MissionAO(const MissionAO&) = delete;
    MissionAO& operator=(const MissionAO&) = delete;
    MissionAO(MissionAO&&) = delete;
    MissionAO& operator=(MissionAO&&) = delete;

    /// @brief Get instance
    /// @return MissionAO&
    static MissionAO& Inst()
    {
        static MissionAO inst;
        return inst;
    }

    /// @brief Start MCAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);  // NOLINT

    /// @brief Reset mission AO
    inline void Reset();

    /// @brief Print system fault state
    inline void PrintFault();

    /// @brief Print battery state
    inline void PrintBatt();

    /// @brief Poke watchdog
    inline void PokeWatchdog();

    /// @brief Enable watchdog
    inline void EnableWatchdog();

    /// @brief Disable watchdog
    inline void DisableWatchdog();

private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;

    /// @brief Internal fault recovery timer/ reset the watchdog timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault request timer
    QP::QTimeEvt _faultRequestTimer;
    /// @brief Fault request timer period in ticks
    uint32_t _faultRequestTimerInterval = bsp::TICKS_PER_SEC / 20U;
    /// @brief Latest faults from all subsystems, including mission subsystem
    bool _faultStates[bsp::SubsystemID::NUM_SUBSYSTEMS][bsp::MAX_SUBSYSTEM_FAULTS] = {0};

    /// @brief Battery state CAN publish timer
    QP::QTimeEvt _battPubTimer;
    /// @brief Battery state CAN publish timer period
    static constexpr uint32_t _battPubTimerInterval = bsp::TICKS_PER_SEC / 5U;
    /// @brief Simple battery discharge curve linear interpolant model voltages
    std::array<float, 4> _battV {0.0f};
    /// @brief Simple battery discharge curve linear interpolant model SOCs
    std::array<float, 4> _battS {0.0f};

    /// @brief Heartbeat CAN publish timer
    QP::QTimeEvt _heartbeatTimer;
    /// @brief Heartbeat CAN publish timer period
    static constexpr uint32_t _heartbeatTimerInterval = bsp::TICKS_PER_SEC / 5U;

    /// @brief Watchdog timer
    QP::QTimeEvt _watchdogTimer;
    /// @brief Watchdog timer period
    static constexpr uint32_t _watchdogTimerInterval = bsp::TICKS_PER_SEC / 1U;
    /// @brief Whether or not watchdog is enabled
    bool _watchdogEnable = false;

    /// @brief Last Vin
    float _lastVin = 0.0f;
    /// @brief Last computed SOC
    float _lastSOC = 0.0f;

    /// @brief CAN TX header
    CAN_TxHeaderTypeDef _canTxHeader;
    /// @brief CAN TX data
    uint8_t _canTxData[8];
    /// @brief CAN TX mailbox
    uint32_t _canTxMailbox;

    /// @brief Track if this AO has successfully initialized once. Certain steps in initialization should be skipped
    /// after first time.
    bool _hasFirstTimeInit = false;

private:  // NOLINT
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
        PRINT_BATT_SIG,
        CAN_PUB_BATT_SIG,
        HEARTBEAT_SIG,
        POKE_WATCHDOG_SIG,
        WATCHDOG_EXPIRED_SIG,
        ENABLE_WATCHDOG_SIG,
        DISABLE_WATCHDOG_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set and publish fault
    void SetFault(bsp::SubsystemID subsystem, uint8_t fault, bool active);

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Initialize
    Q_STATE_DECL(initializing);
    /// @brief Active
    Q_STATE_DECL(active);
    /// @brief Fault
    Q_STATE_DECL(error);
    /// @brief Protect the platform. Entered when watchdog expires.
    Q_STATE_DECL(selfprotect);
};  // class MissionAO

inline void MissionAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::PrintFault()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::PRINT_FAULT_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::PrintBatt()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::PRINT_BATT_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::PokeWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::POKE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::EnableWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::ENABLE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::DisableWatchdog()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::DISABLE_WATCHDOG_SIG);
        POST(&evt, this);
    }
}
}  // namespace mission

#endif
