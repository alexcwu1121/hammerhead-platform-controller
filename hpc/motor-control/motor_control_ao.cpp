#include "motor_control_ao.hpp"

#include "adc.h"
#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"

mc::MotorControlHSM::MotorControlHSM() : QP::QHsm(&initial), _fault(Fault::NO_FAULT) {}

void mc::MotorControlHSM::SetPin(Pin pin, GPIO_TypeDef* port, uint16_t pinNum)
{
    switch (pin)
    {
    case Pin::M_EN:
    {
        _mEnPort   = port;
        _mEnPinNum = pinNum;
        break;
    }
    case Pin::M_DIR:
    {
        _mDirPort   = port;
        _mDirPinNum = pinNum;
        break;
    }
    case Pin::M_PWM:
    {
        _mPWMPort   = port;
        _mPWMPinNum = pinNum;
        break;
    }
    case Pin::M_FAULT:
    {
        _mFaultPort   = port;
        _mFaultPinNum = pinNum;
        break;
    }
    case Pin::M_VIN_SENS:
    {
        _mVinSensPort   = port;
        _mVinSensPinNum = pinNum;
        break;
    }
    default:
        break;
    }
}

void mc::MotorControlHSM::SetTim(TIM_HandleTypeDef* htim, uint16_t htimCh)
{
    _htim   = htim;
    _htimCh = htimCh;
}

void mc::MotorControlHSM::Initialize()
{
    static QP::QEvt evt(PrivateSignals::INITIALIZE_SIG);
    dispatch(&evt, 0U);
}

void mc::MotorControlHSM::Reset()
{
    static QP::QEvt evt(PrivateSignals::RESET_SIG);
    dispatch(&evt, 0U);
}

void mc::MotorControlHSM::ClearFault()
{
    // Clear fault
    _fault = Fault::NO_FAULT;
    static QP::QEvt evt(PrivateSignals::RESET_SIG);
    dispatch(&evt, 0U);
}

void mc::MotorControlHSM::SetDuty(uint16_t duty)
{
    SetDutyEvt evt(PrivateSignals::SET_DUTY_SIG);
    evt.duty = duty;
    dispatch(&evt, 0U);
}

Q_STATE_DEF(mc::MotorControlHSM, initial)
{
    Q_UNUSED_PAR(e);
    return tran(&initializing);
}

Q_STATE_DEF(mc::MotorControlHSM, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        status_ = tran(&initializing);
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

Q_STATE_DEF(mc::MotorControlHSM, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
    case INITIALIZE_SIG:
    {
        // Start pwm at a low duty cycle
        /*
        // Check for ground fault
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t value = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        if(true)
        {
            static QP::QEvt evt(PrivateSignals::FAULT_SIG);
            dispatch(&evt, 0U);
        } else
        {
            static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
            dispatch(&evt, 0U);
        }
        */

        // Initialize direction to 0
        HAL_GPIO_WritePin(_mDirPort, _mDirPinNum, GPIO_PIN_RESET);
        // Initialize duty cycle to 0
        __HAL_TIM_SET_COMPARE(_htim, _htimCh, 0U);
        // Start PWM generation
        HAL_TIM_PWM_Start(_htim, _htimCh);
        // Enable motor driver
        HAL_GPIO_WritePin(_mEnPort, _mEnPinNum, GPIO_PIN_RESET);

        // TODO: Check mc fault pin
        // _fault  = Fault::GROUND_FAULT;
        /*
        if()
        {

        } else
        {

        }
        */

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

Q_STATE_DEF(mc::MotorControlHSM, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_DUTY_SIG:
    {
        uint16_t duty = Q_EVT_CAST(SetDutyEvt)->duty;
        // TODO: Check if configured duty cycle is within range
        // if(duty > 1023)
        //{
        //    cli::CLIAO::Inst().Printf("Duty cycle out of range (0-1023)");
        //}

        // TODO: Interpolate to actual duty cycle precision

        __HAL_TIM_SET_COMPARE(_htim, _htimCh, duty);
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

Q_STATE_DEF(mc::MotorControlHSM, error)
{
    QP::QState status_;
    switch (e->sig)
    {
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}

mc::MotorControlAO::MotorControlAO() : QP::QActive(&initial) {}

void mc::MotorControlAO::Start(const QP::QPrioSpec priority)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
    _isStarted = true;
}

void mc::MotorControlAO::SetDuty(Motor motor, uint16_t duty)
{
    if (_isStarted)
    {
        // Set duty
        SetDutyEvt* evt = Q_NEW(SetDutyEvt, PrivateSignals::SET_DUTY_SIG);
        evt->motor      = motor;
        evt->duty       = duty;
        POST(evt, this);
    }
}

Q_STATE_DEF(mc::MotorControlAO, initial)
{
    Q_UNUSED_PAR(e);

    // Set M1 pins and timers
    _motorControlHSMs[Motor::M1].SetPin(MotorControlHSM::Pin::M_EN, P_M1_EN_GPIO_Port, P_M1_EN_Pin);
    _motorControlHSMs[Motor::M1].SetPin(MotorControlHSM::Pin::M_DIR, P_M1_DIR_GPIO_Port,
                                        P_M1_DIR_Pin);
    _motorControlHSMs[Motor::M1].SetPin(MotorControlHSM::Pin::M_PWM, P_M1_PWM_GPIO_Port,
                                        P_M1_PWM_Pin);
    _motorControlHSMs[Motor::M1].SetPin(MotorControlHSM::Pin::M_FAULT, P_M1_FAULT_GPIO_Port,
                                        P_M1_FAULT_Pin);
    _motorControlHSMs[Motor::M1].SetPin(MotorControlHSM::Pin::M_VIN_SENS, P_M1_12V_SENS_GPIO_Port,
                                        P_M1_12V_SENS_Pin);
    _motorControlHSMs[Motor::M1].SetTim(&htim2, TIM_CHANNEL_3);

    // Set M2 pins and timers
    _motorControlHSMs[Motor::M2].SetPin(MotorControlHSM::Pin::M_EN, P_M2_EN_GPIO_Port, P_M2_EN_Pin);
    _motorControlHSMs[Motor::M2].SetPin(MotorControlHSM::Pin::M_DIR, P_M2_DIR_GPIO_Port,
                                        P_M2_DIR_Pin);
    _motorControlHSMs[Motor::M2].SetPin(MotorControlHSM::Pin::M_PWM, P_M2_PWM_GPIO_Port,
                                        P_M2_PWM_Pin);
    _motorControlHSMs[Motor::M2].SetPin(MotorControlHSM::Pin::M_FAULT, P_M2_FAULT_GPIO_Port,
                                        P_M2_FAULT_Pin);
    _motorControlHSMs[Motor::M2].SetPin(MotorControlHSM::Pin::M_VIN_SENS, P_M2_12V_SENS_GPIO_Port,
                                        P_M2_12V_SENS_Pin);
    _motorControlHSMs[Motor::M2].SetTim(&htim3, TIM_CHANNEL_1);

    // Initialize motor controller hsms
    _motorControlHSMs[Motor::M1].init(0U);
    _motorControlHSMs[Motor::M2].init(0U);

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
        // Attempt to clear faults
        _motorControlHSMs[Motor::M1].ClearFault();
        _motorControlHSMs[Motor::M2].ClearFault();

        // Attempt to initialize motor controllers
        _motorControlHSMs[Motor::M1].Initialize();
        _motorControlHSMs[Motor::M2].Initialize();

        // Check motor controllers for faults during initialization
        if (_motorControlHSMs[Motor::M1].GetFault() != Fault::NO_FAULT
            || _motorControlHSMs[Motor::M1].GetFault() != Fault::NO_FAULT)
        {
            static QP::QEvt evt(PrivateSignals::FAULT_SIG);
            POST(&evt, this);
        }
        else
        {
            static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
            POST(&evt, this);
        }

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
        Motor    motor = Q_EVT_CAST(SetDutyEvt)->motor;
        uint16_t duty  = Q_EVT_CAST(SetDutyEvt)->duty;
        _motorControlHSMs[motor].SetDuty(duty);
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
    default:
    {
        status_ = super(&root);
        break;
    }
    }
    return status_;
}
