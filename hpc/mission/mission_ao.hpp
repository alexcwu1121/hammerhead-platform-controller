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
#include "i2c_mailbox.hpp"
#include "imu_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

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

/// @brief Operation codes
enum opcode : uint8_t
{
    NO_OP = 0U,
    WRITE_MC1_MODE,
    WRITE_MC2_MODE,
    WRITE_MC1_RATE,
    WRITE_MC2_RATE,
    WRITE_MC1_DUTY,
    WRITE_MC2_DUTY,
    WRITE_MC1_DIR,
    WRITE_MC2_DIR,
    WRITE_MC1_RESET,
    WRITE_MC2_RESET,
    WRITE_IMU_RESET,
    WRITE_IMU_COMP,
    WRITE_PARAM_VAL,
    WRITE_PARAM_COMMIT,
    READ_PARAM_VAL,
    READ_IMU_DATA,
    NUM_OPS
};

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
    inline void Reset();

    /// @brief Reset the mailbox
    inline void ResetMailbox();

    /// @brief Print system fault state
    inline void PrintFault();

    /// @brief Initiate a slave rx transaction
    inline void StartSlaveRx();

    /// @brief Initiate a slave tx transaction
    inline void StartSlaveTx();

    /// @brief Complete a transaction
    inline void CompleteTransaction();

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
    bool _faultStates[bsp::SubsystemID::NUM_SUBSYSTEMS][bsp::MAX_SUBSYSTEM_FAULTS] = {0U};
    /// @brief Mission Module I2C address
    uint8_t _mmAddr = 0x00;
    /// @brief I2C mailbox
    I2CMailbox _i2cMailbox;

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
        RESET_MAILBOX_SIG,
        START_SLAVE_RX_SIG,
        START_SLAVE_TX_SIG,
        COMPLETE_TRANSACTION_SIG,
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

inline void MissionAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::ResetMailbox()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_MAILBOX_SIG);
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

inline void MissionAO::StartSlaveRx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_RX_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::StartSlaveTx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_TX_SIG);
        POST(&evt, this);
    }
}

inline void MissionAO::CompleteTransaction()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::COMPLETE_TRANSACTION_SIG);
        POST(&evt, this);
    }
}
}  // namespace mission

#endif
