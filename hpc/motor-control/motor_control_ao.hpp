#ifndef MC_AO_HPP_
#define MC_AO_HPP_

#include <limits>

#include "bsp.hpp"
#include "gpio.h"
#include "qpcpp.hpp"
#include "tim.h"

namespace mc
{
// Add FaultAO and a global fault event system indexed by AO of origin and fault type
/// @brief Fault codes
enum Fault : uint8_t
{
    UNDERVOLTAGE_FAULT,
    OVERVOLTAGE_FAULT,
    OVERCURRENT_THERMAL_FAULT,
    NUM_FAULTS
};

/// @brief Motor direction
enum Dir : uint8_t
{
    CW,
    CCW
};

struct MotorControllerDevice;
/// @brief Motor Control AO
class MotorControlAO : public QP::QActive
{
   public:
    /// @brief Constructor
    /// @param mcDevice
    MotorControlAO(MotorControllerDevice* mcDevice);

    /// @brief Get MC1 instance
    /// @return MotorControlAO&
    static MotorControlAO& MC1Inst();

    /// @brief Get MC2 instance
    /// @return MotorControlAO&
    static MotorControlAO& MC2Inst();

    /// @brief Start MCAO
    /// @param priority
    /// @param id
    void Start(const QP::QPrioSpec priority, bsp::SubsystemID id);

    /// @brief Reinitialize and attempt to clear faults
    void Reset();

    /// @brief Set direction
    /// @param dir
    void SetDir(Dir dir);

    /// @brief Set PWM duty cycle (0 <= duty <= 1023)
    /// @param duty
    void SetDuty(uint16_t duty);

    /// @brief Set an angular rate (-1.0 <= rate <= 1.0)
    /// @param rate
    void SetRate(float rate);

    /// @brief Handle motor controller fault interrupt
    void FaultIT();

   private:
    /// @brief Subsystem ID
    bsp::SubsystemID _id;
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief Motor controller device specific properties
    MotorControllerDevice* _mcDevice;
    /// @brief PWM lower deadband
    uint16_t _pwmLowerDeadband = 0U;
    /// @brief Rate control stiffness
    float _rateStiffness = std::numeric_limits<float>::max();
    /// @brief Rate control damping
    float _rateDamping = std::numeric_limits<float>::max();
    /// @brief Motor input undervoltage threshold
    float _underVoltageThreshold = -std::numeric_limits<float>::max();
    /// @brief Motor input overvoltage threshold
    float _overVoltageThreshold = std::numeric_limits<float>::max();
    /// @brief Last measured motor input voltage
    float _vmIn = -std::numeric_limits<float>::max();
    /// @brief Fault states
    bool _faultStates[mc::Fault::NUM_FAULTS] = {false};
    /// @brief Fault recovery timer
    QP::QTimeEvt _faultRecoveryTimer;
    /// @brief Fault recovery timer period in ticks
    uint32_t _faultRecoveryTimerInterval = bsp::TICKS_PER_SEC / 100U;
    /// @brief Rate control timer
    QP::QTimeEvt _rateControlTimer;
    /// @brief Rate control timer period in ticks
    uint32_t _rateControlTimerInterval = bsp::TICKS_PER_SEC / 1000U;
    /// @brief Reference rate
    float _refRate = {0.0f};
    /// @brief Current rate
    float _currentRate = {0.0f};
    /// @brief Current rate differential
    float _currentDRate = {0.0f};
    /// @brief Current rate directional epsilon
    static constexpr float _eps = {0.0001f};
    /// @brief PWM duty cycle full scale range
    static constexpr uint16_t _fsr = {1023U};

    /// @brief Check fault conditions and update fault states
    void UpdateFaults();

    /// @brief Check if there exist active faults
    bool IsActiveFaults();

    /// @brief Check if initialized
    bool IsInitialized();

    // TODO: rate control
    // TODO: first and second order slew rates
    // TODO: print state, including driver voltage
    // TODO: start stream motor states
    // TODO: print fault states

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        FAULT_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        FAULT_IT_SIG,
        RESET_SIG,
        SET_DIR_SIG,
        SET_DUTY_SIG,
        SET_RATE_SIG,
        PARAMS_UPDATED_SIG,
        VM_UPDATED_SIG,
        FAULT_RECOVERY_SIG,
        RATE_CONTROL_UPDATE_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set duty evt
    class SetDutyEvt : public QP::QEvt
    {
       public:
        SetDutyEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        uint16_t duty;
    };

    /// @brief Set direction evt
    class SetDirEvt : public QP::QEvt
    {
       public:
        SetDirEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        Dir dir;
    };

    /// @brief Set rate evt
    class SetRateEvt : public QP::QEvt
    {
       public:
        SetRateEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        float rate;
    };

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Initialize
    Q_STATE_DECL(initializing);
    /// @brief Process CLI events
    Q_STATE_DECL(active);
    /// @brief Fault
    Q_STATE_DECL(error);
};  // class MotorControlAO

}  // namespace mc

#endif  // MC_AO_HPP_
