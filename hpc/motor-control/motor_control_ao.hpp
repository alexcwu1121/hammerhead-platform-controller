#ifndef MC_AO_HPP_
#define MC_AO_HPP_

#include "bsp.hpp"
#include "gpio.h"
#include "qpcpp.hpp"
#include "tim.h"

namespace mc
{
/// @brief Fault codes
enum Fault : uint8_t
{
    NO_FAULT,
    GROUND_FAULT  // Motor terminals are shorted
};

/// @brief Individual motor controller HSM
class MotorControlHSM : public QP::QHsm
{
   public:
    MotorControlHSM();

    // Motor controller pins
    enum Pin : uint8_t
    {
        M_EN,
        M_DIR,
        M_PWM,
        M_FAULT,
        M_VIN_SENS,
    };

    /// @brief Set pins
    void SetPin(Pin pin, GPIO_TypeDef* port, uint16_t pinNum);

    /// @brief Set pwm timer
    void SetTim(TIM_HandleTypeDef* htim, uint16_t htimCh);

    /// @brief Initialize motor controller
    void Initialize();

    /// @brief Reset motor controller
    void Reset();

    /// @brief Get active fault
    Fault GetFault() const
    {
        return _fault;
    }

    /// @brief Attempt to clear fault and reinitialize
    void ClearFault();

    /// @brief Set a duty cycle
    /// @param duty Duty cycle (0-1023)
    void SetDuty(uint16_t duty);

   private:
    /// @brief Ground fault detection probe duty cycle, (0-1023)
    static constexpr uint16_t _groundFaultDuty = 10U;
    /// @brief Ground fault detection lower threshold, Volts
    static constexpr float _groundFaultVoltage = 5;
    /// @brief Last fault
    Fault _fault;
    /// @brief MC enable pin port
    GPIO_TypeDef* _mEnPort;
    /// @brief MC enable pin num
    uint16_t _mEnPinNum;
    /// @brief MC dir pin port
    GPIO_TypeDef* _mDirPort;
    /// @brief MC dir pin num
    uint16_t _mDirPinNum;
    /// @brief MC pwm pin power
    GPIO_TypeDef* _mPWMPort;
    /// @brief MC pwm pin num
    uint16_t _mPWMPinNum;
    /// @brief MC fault pin port
    GPIO_TypeDef* _mFaultPort;
    /// @brief MC fault pin num
    uint16_t _mFaultPinNum;
    /// @brief MC Vin sense pin port
    GPIO_TypeDef* _mVinSensPort;
    /// @brief MC Vin sense pin num
    uint16_t _mVinSensPinNum;
    /// @brief PWM timer handle
    TIM_HandleTypeDef* _htim;
    /// @brief PWM timer channel
    uint16_t _htimCh;

    enum PrivateSignals : QP::QSignal
    {
        INITIALIZE_SIG = bsp::MAX_PUB_SIG,
        RESET_SIG,
        SET_DUTY_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Print string evt
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
};

/// @brief Maintain two motor state machines
// Take motor PWM setpoints
//
// Monitor motor voltages and currents
class MotorControlAO : public QP::QActive
{
   public:
    MotorControlAO();
    static MotorControlAO& Inst()
    {
        static MotorControlAO inst;
        return inst;
    }

    /// @brief Motors
    enum Motor : uint8_t
    {
        M1 = 0U,
        M2,
        NUM_MOTOR
    };

    /// @brief Start CLIAO
    /// @param priority
    void Start(const QP::QPrioSpec priority);

    /// @brief Get active fault
    Fault GetFault(Motor motor) const
    {
        return _motorControlHSMs[motor].GetFault();
    }

    /// @brief Set PWM duty cycle for a motor (0-1023)
    void SetDuty(Motor motor, uint16_t duty);

   private:
    /// @brief Event queue size
    static constexpr uint16_t _queueSize = 128U;
    /// @brief Event queue storage
    QP::QEvtPtr _queue[_queueSize] = {0};
    /// @brief Flag indicating if AO has executed initial transition
    bool _isStarted = false;
    /// @brief Motor controller HSMs
    MotorControlHSM _motorControlHSMs[2];

   private:
    /// @brief Private CLIAO signals
    enum PrivateSignals : QP::QSignal
    {
        INITIALIZED_SIG = bsp::MAX_PUB_SIG,
        FAULT_SIG,
        RESET_SIG,
        SET_DUTY_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Set duty evt
    class SetDutyEvt : public QP::QEvt
    {
       public:
        SetDutyEvt(QP::QSignal sig) : QP::QEvt(sig) {}
        Motor    motor;
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
