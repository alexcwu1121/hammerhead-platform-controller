#ifndef MISSION_AO_HPP_
#define MISSION_AO_HPP_

// TODO: stuff to do here:
//  - collect faults
//  - send IMU data up to MM
//  - listen and convert MC commands from MM
//  - listen and convert parameter commands from MM
//  - report battery life from input voltage sense

#include "bmi270.hpp"
#include "bsp.hpp"

namespace mission
{
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
    uint32_t _imuTimerInterval = bsp::TICKS_PER_SEC / 1000U;

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        RESET_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        IMU_SERVICE_SIG,
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
