#ifndef MC_AO_HPP_
#define MC_AO_HPP_

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
    NO_FAULT,
    GROUND_FAULT,
    OVERCURRENT_THERMAL_FAULT
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
    void Start(const QP::QPrioSpec priority);

    /// @brief Set PWM duty cycle for a motor (0-1023)
    void SetDuty(uint16_t duty);

    /// @brief Handle motor controller fault interrupt
    void FaultIT();

   private:
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief Ground fault detection probe duty cycle, (0-1023)
    static constexpr uint16_t _groundFaultDuty = 1U;
    /// @brief Ground fault detection lower threshold, Volts
    static constexpr float _groundFaultVoltage = 5.0f;
    /// @brief Last fault
    mc::Fault _fault;
    /// @brief Motor controller device properties
    MotorControllerDevice* _mcDevice;

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::MAX_PUB_SIG,
        FAULT_SIG,
        FAULT_IT_SIG,
        RESET_SIG,
        SET_DUTY_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set duty evt
    class SetDutyEvt : public QP::QEvt
    {
       public:
        SetDutyEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        uint16_t duty;
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
