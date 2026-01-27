#ifndef MISSION_AO_HPP_
#define MISSION_AO_HPP_

/// TODO: stuff to do here:
//  - send IMU data up to MM on every read
//  - Define MM i2c protocol
//      - listen and convert MC commands from MM
//      - listen and convert parameter commands from MM

/// TODO: Nice to haves:
//  - report battery life from input voltage sense
//      - test ADC voltage readings and implement battery polynomial
//  - record motor control and imu timing stability metrics

#include "bsp.hpp"

namespace mission
{
/// @brief Fault codes
enum Fault : uint8_t
{
    MISSION_INIT_FAILED = 0U,
    BATT_LOW,
    BATT_CRITICAL,
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
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Reset mission AO
    void Reset();

    /// @brief Print system fault state
    void PrintFault();

   private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief Internal fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Internal fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Fault request timer
    QP::QTimeEvt _faultRequestTimer;
    /// @brief Fault request timer period in ticks
    uint32_t _faultRequestTimerInterval = bsp::TICKS_PER_SEC / 20U;
    /// @brief Latest faults from all subsystems, including mission subsystem
    bool _faultStates[bsp::SubsystemID::NUM_SUBSYSTEMS][bsp::MAX_SUBSYSTEM_FAULTS] = {0};
    /// @brief Mission Module I2C address
    uint8_t _mmAddr = 0x00;

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
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
};  // class MissionAO
}  // namespace mission

#endif
