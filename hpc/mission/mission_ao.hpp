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

#include "bmi270.hpp"
#include "bsp.hpp"

namespace mission
{
/// @brief Fault codes
enum Fault : uint8_t
{
    IMU_INIT_FAILED = 0U,
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
    case Fault::IMU_INIT_FAILED:
    {
        return "IMU_INIT_FAILED";
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

    /// @brief Run IMU compensation
    void RunIMUCompensation();

    /// @brief Start IMU stream
    void StartIMUStream();

    /// @brief Stop IMU stream
    void StopIMUStream();

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
    /// @brief IMU device
    imu::BMI270 _imu;
    /// @brief IMU service timer
    QP::QTimeEvt _imuTimer;
    /// @brief Rate control timer period in ticks
    uint32_t _imuTimerInterval = bsp::TICKS_PER_SEC / 200U;
    /// @brief IMU stream timer
    QP::QTimeEvt _imuStreamTimer;
    /// @brief IMU stream period
    uint32_t _imuStreamTimerInterval = bsp::TICKS_PER_SEC / 10U;
    /// @brief Last IMU sample
    imu::IMUData _imuData = {0};
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

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        IMU_SERVICE_SIG,
        RUN_IMU_COMPENSATION_SIG,
        START_IMU_STREAM,
        STOP_IMU_STREAM,
        IMU_STREAM_SIG,
        SUBS_FAULT_REQUEST_SIG,
        PRINT_FAULT_SIG,
        MAX_PRIV_SIG
    };

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
