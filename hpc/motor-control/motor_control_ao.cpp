#include "motor_control_ao.hpp"

#include "adc.h"
#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"

/// Motor controller fault interrupt callback
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case P_M1_FAULT_Pin:
    {
        mc::MotorControlAO::MC1Inst().FaultIT();
        break;
    }
    case P_M2_FAULT_Pin:
    {
        mc::MotorControlAO::MC2Inst().FaultIT();
        break;
    }
    default:
    {
        break;
    }
    }
}

namespace mc
{
/// @brief Motor controller device properties
struct MotorControllerDevice
{
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
};

/// @brief MC1 device properties
static MotorControllerDevice mc1Device = {._mEnPort        = P_M1_EN_GPIO_Port,
                                          ._mEnPinNum      = P_M1_EN_Pin,
                                          ._mDirPort       = P_M1_DIR_GPIO_Port,
                                          ._mDirPinNum     = P_M1_DIR_Pin,
                                          ._mPWMPort       = P_M1_PWM_GPIO_Port,
                                          ._mPWMPinNum     = P_M1_PWM_Pin,
                                          ._mFaultPort     = P_M1_FAULT_GPIO_Port,
                                          ._mFaultPinNum   = P_M1_FAULT_Pin,
                                          ._mVinSensPort   = P_M1_12V_SENS_GPIO_Port,
                                          ._mVinSensPinNum = P_M1_12V_SENS_Pin,
                                          ._htim           = &htim2,
                                          ._htimCh         = TIM_CHANNEL_3};

/// @brief MC2 device properties
static MotorControllerDevice mc2Device = {._mEnPort        = P_M2_EN_GPIO_Port,
                                          ._mEnPinNum      = P_M2_EN_Pin,
                                          ._mDirPort       = P_M2_DIR_GPIO_Port,
                                          ._mDirPinNum     = P_M2_DIR_Pin,
                                          ._mPWMPort       = P_M2_PWM_GPIO_Port,
                                          ._mPWMPinNum     = P_M2_PWM_Pin,
                                          ._mFaultPort     = P_M2_FAULT_GPIO_Port,
                                          ._mFaultPinNum   = P_M2_FAULT_Pin,
                                          ._mVinSensPort   = P_M2_12V_SENS_GPIO_Port,
                                          ._mVinSensPinNum = P_M2_12V_SENS_Pin,
                                          ._htim           = &htim3,
                                          ._htimCh         = TIM_CHANNEL_1};

mc::MotorControlAO::MotorControlAO(MotorControllerDevice* mcDevice)
    : QP::QActive(&initial), _mcDevice(mcDevice)
{
}

mc::MotorControlAO& mc::MotorControlAO::MC1Inst()
{
    static mc::MotorControlAO mc1_inst(&mc1Device);
    return mc1_inst;
}

mc::MotorControlAO& mc::MotorControlAO::MC2Inst()
{
    static mc::MotorControlAO mc2_inst(&mc2Device);
    return mc2_inst;
}

void mc::MotorControlAO::Start(const QP::QPrioSpec priority)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
    _isStarted = true;
}

void mc::MotorControlAO::SetDuty(uint16_t duty)
{
    if (_isStarted)
    {
        SetDutyEvt* evt = Q_NEW(SetDutyEvt, PrivateSignals::SET_DUTY_SIG);
        evt->duty       = duty;
        POST(evt, this);
    }
}

void mc::MotorControlAO::FaultIT()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::FAULT_IT_SIG);
        POST(&evt, this);
    }
}

Q_STATE_DEF(mc::MotorControlAO, initial)
{
    Q_UNUSED_PAR(e);
    return tran(&initializing);
}

Q_STATE_DEF(mc::MotorControlAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        status_ = tran(&initializing);
        break;
    }
    case PrivateSignals::FAULT_IT_SIG:
    {
        status_ = tran(&error);
        break;
    }
    default:
    {
        status_ = super(&top);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mc::MotorControlAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Check for fault conditions

        // Initialize motor controller
        HAL_GPIO_WritePin(_mcDevice->_mDirPort, _mcDevice->_mDirPinNum, GPIO_PIN_RESET);
        // Initialize duty cycle to 0
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, 0U);
        // Start PWM generation
        HAL_TIM_PWM_Start(_mcDevice->_htim, _mcDevice->_htimCh);
        // Enable motor driver
        HAL_GPIO_WritePin(_mcDevice->_mEnPort, _mcDevice->_mEnPinNum, GPIO_PIN_RESET);

        // Check for fault conditions again
        static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
        POST(&evt, this);

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::FAULT_SIG:
    {
        status_ = tran(&error);
        break;
    }
    case PrivateSignals::INITIALIZED_SIG:
    {
        status_ = tran(&active);
        break;
    }
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mc::MotorControlAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::SET_DUTY_SIG:
    {
        uint16_t duty = Q_EVT_CAST(SetDutyEvt)->duty;
        // TODO: Check if configured duty cycle is within range
        // TODO: Interpolate to actual duty cycle precision
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, duty);
        status_ = Q_RET_HANDLED;
        break;
    }
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mc::MotorControlAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        cli::CLIAO::Inst().Printf("Fault");
        status_ = Q_RET_HANDLED;
        break;
    }
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}
}  // namespace mc
