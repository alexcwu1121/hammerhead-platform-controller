#include "motor_control_ao.hpp"

#include "adc.h"
#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "param_ao.hpp"

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
    /// @brief Input voltage ADC channel
    bsp::ADCChannels _adcChannel;
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
                                          ._htimCh         = TIM_CHANNEL_3,
                                          ._adcChannel     = bsp::ADCChannels::VMOUT1};

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
                                          ._htimCh         = TIM_CHANNEL_1,
                                          ._adcChannel     = bsp::ADCChannels::VMOUT2};

mc::MotorControlAO::MotorControlAO(MotorControllerDevice* mcDevice)
    : QP::QActive(&initial),
      _mcDevice(mcDevice),
      _faultRecoveryTimer(this, PrivateSignals::FAULT_RECOVERY_SIG, 0U),
      _rateControlTimer(this, PrivateSignals::RATE_CONTROL_UPDATE_SIG, 0U)
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

void mc::MotorControlAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
    _id        = id;
    _isStarted = true;
}

void mc::MotorControlAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

void mc::MotorControlAO::SetDir(Dir dir)
{
    if (_isStarted)
    {
        SetDirEvt* evt = Q_NEW(SetDirEvt, PrivateSignals::SET_DIR_SIG);
        evt->dir       = dir;
        POST(evt, this);
    }
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

void mc::MotorControlAO::SetRate(float rate)
{
    if (_isStarted)
    {
        SetRateEvt* evt = Q_NEW(SetRateEvt, PrivateSignals::SET_RATE_SIG);
        evt->rate       = rate;
        POST(evt, this);
    }
}

void mc::MotorControlAO::SetMode(Mode mode)
{
    if (_isStarted)
    {
        SetModeEvt* evt = Q_NEW(SetModeEvt, PrivateSignals::SET_MODE_SIG);
        evt->mode       = mode;
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

void mc::MotorControlAO::UpdateFaults()
{
    // Check MC fault pin state
    bool mc_fault =
        HAL_GPIO_ReadPin(_mcDevice->_mFaultPort, _mcDevice->_mFaultPinNum) == GPIO_PIN_RESET;

    // If fault state has changed, publish update and update fault state
    if (mc_fault != _faultStates[mc::Fault::OVERCURRENT_THERMAL_FAULT])
    {
        bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
        evt->id            = _id;
        evt->fault         = mc::Fault::OVERCURRENT_THERMAL_FAULT;
        evt->active        = mc_fault;
        PUBLISH(evt, this);

        _faultStates[mc::Fault::OVERCURRENT_THERMAL_FAULT] = mc_fault;
    }

    // Skip under/overvoltage faults if no ADC measurements have been received
    if (_vmIn != -std::numeric_limits<float>::max())
    {
        // Check for undervoltage condition
        bool undervoltage_fault = _vmIn < _underVoltageThreshold;

        // If fault state has changed, publish update and update fault state
        if (undervoltage_fault != _faultStates[mc::Fault::UNDERVOLTAGE_FAULT])
        {
            bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
            evt->id            = _id;
            evt->fault         = mc::Fault::UNDERVOLTAGE_FAULT;
            evt->fault         = undervoltage_fault;
            PUBLISH(evt, this);

            _faultStates[mc::Fault::UNDERVOLTAGE_FAULT] = undervoltage_fault;
        }

        // Check for overvoltage condition
        bool overvoltage_fault = _vmIn > _overVoltageThreshold;

        // If fault state has changed, publish update and update fault state
        if (overvoltage_fault != _faultStates[mc::Fault::OVERVOLTAGE_FAULT])
        {
            bsp::FaultEvt* evt = Q_NEW(bsp::FaultEvt, bsp::PublicSignals::FAULT_SIG);
            evt->id            = _id;
            evt->fault         = mc::Fault::OVERVOLTAGE_FAULT;
            evt->fault         = overvoltage_fault;
            PUBLISH(evt, this);

            _faultStates[mc::Fault::OVERVOLTAGE_FAULT] = overvoltage_fault;
        }
    }
}

bool mc::MotorControlAO::IsActiveFaults()
{
    bool active_fault = false;
    for (const auto& fault : _faultStates)
    {
        if (fault)
        {
            active_fault = true;
            break;
        }
    }
    return active_fault;
}

bool mc::MotorControlAO::IsInitialized()
{
    // Received at least one ADC measurement
    // No faults
    // Parameters initialized
    return _rateStiffness != std::numeric_limits<float>::max()
           && _rateDamping != std::numeric_limits<float>::max()
           && _underVoltageThreshold != -std::numeric_limits<float>::max()
           && _overVoltageThreshold != std::numeric_limits<float>::max() && !IsActiveFaults()
           && _vmIn != -std::numeric_limits<float>::max();
}

Q_STATE_DEF(mc::MotorControlAO, initial)
{
    Q_UNUSED_PAR(e);
    // Subscribe to signals
    subscribe(bsp::PublicSignals::PARAMETER_UPDATE_SIG);
    subscribe(bsp::PublicSignals::ADC_SIG);
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
    case PrivateSignals::SET_MODE_SIG:
    {
        _mode   = Q_EVT_CAST(SetModeEvt)->mode;
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::FAULT_SIG:
    {
        // Update fault conditions
        UpdateFaults();

        // Transition to error
        status_ = tran(&error);
        break;
    }
    case PrivateSignals::FAULT_IT_SIG:
    {
        // Update fault conditions
        UpdateFaults();

        // Transition to error
        status_ = tran(&error);
        break;
    }
    case bsp::PublicSignals::ADC_SIG:
    {
        // Update latest MC input voltage measurement
        _vmIn = Q_EVT_CAST(bsp::ADCEvt)->adcVoltages[_mcDevice->_adcChannel];

        // Update fault conditions
        UpdateFaults();

        // If any faults are active, transition to error
        if (IsActiveFaults())
        {
            status_ = tran(&error);
        }
        else
        {
            status_ = Q_RET_HANDLED;
        }

        // Self-post event indicating MC input voltage has been updated
        static QP::QEvt evt(PrivateSignals::VM_UPDATED_SIG);
        POST(&evt, this);
        break;
    }
    case bsp::PublicSignals::PARAMETER_UPDATE_SIG:
    {
        param::ParameterID id  = Q_EVT_CAST(bsp::ParameterUpdateEvt)->id;
        param::Type        val = Q_EVT_CAST(bsp::ParameterUpdateEvt)->value;

        // Update parameter value
        switch (id)
        {
        case param::ParameterID::MC_PWM_DEADBAND:
        {
            // Set lower pwm deadband
            if (val._uint16 < _fsr)
            {
                _pwmLowerDeadband = val._uint16;
            }
            break;
        }
        case param::ParameterID::MC_RATE_STIFFNESS:
        {
            // Set rate control stiffness
            if (val._float32 >= 0)
            {
                _rateStiffness = val._float32;
            }
            break;
        }
        case param::ParameterID::MC_RATE_DAMPING:
        {
            // Set rate control damping
            if (val._float32 >= 0)
            {
                _rateDamping = val._float32;
            }
            break;
        }
        case param::ParameterID::MC_UNDERVOLTAGE_FAULT_THRESHOLD:
        {
            // Set undervoltage fault threshold
            if (val._float32 > 0)
            {
                _underVoltageThreshold = val._float32;
            }
            break;
        }
        case param::ParameterID::MC_OVERVOLTAGE_FAULT_THRESHOLD:
        {
            // Set overvoltage fault threshold
            if (val._float32 > 0)
            {
                _overVoltageThreshold = val._float32;
            }
            break;
        }
        default:
        {
            break;
        }
        }

        // Self-post event indicating a parameter has been updated
        static QP::QEvt evt(PrivateSignals::PARAMS_UPDATED_SIG);
        POST(&evt, this);

        status_ = Q_RET_HANDLED;
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
        // Update faults
        UpdateFaults();
        // Fail if there are active faults
        if (IsActiveFaults())
        {
            static QP::QEvt evt(PrivateSignals::FAULT_SIG);
            POST(&evt, this);
            status_ = Q_RET_HANDLED;
            break;
        }

        // Initialize motor controller
        HAL_GPIO_WritePin(_mcDevice->_mDirPort, _mcDevice->_mDirPinNum, GPIO_PIN_RESET);
        // Initialize duty cycle to 0
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, 0U);
        // Start PWM generation
        HAL_TIM_PWM_Start(_mcDevice->_htim, _mcDevice->_htimCh);
        // Enable motor driver
        HAL_GPIO_WritePin(_mcDevice->_mEnPort, _mcDevice->_mEnPinNum, GPIO_PIN_RESET);

        // Request parameters
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MC_PWM_DEADBAND);
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MC_RATE_STIFFNESS);
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MC_RATE_DAMPING);
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MC_UNDERVOLTAGE_FAULT_THRESHOLD);
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MC_OVERVOLTAGE_FAULT_THRESHOLD);

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::PARAMS_UPDATED_SIG:
    case PrivateSignals::VM_UPDATED_SIG:
    {
        if (IsInitialized())
        {
            // Start processing
            if (_mode == Mode::DUTY)
            {
                status_ = tran(&active_duty);
            }
            else if (_mode == Mode::RATE)
            {
                status_ = tran(&active_rate);
            }
            else
            {
                status_ = tran(&error);
            }
        }
        else
        {
            // Keep waiting
            status_ = Q_RET_HANDLED;
        }
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

Q_STATE_DEF(mc::MotorControlAO, active_duty)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::SET_DIR_SIG:
    {
        Dir dir = Q_EVT_CAST(SetDirEvt)->dir;
        HAL_GPIO_WritePin(_mcDevice->_mDirPort, _mcDevice->_mDirPinNum,
                          static_cast<GPIO_PinState>(dir));
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_DUTY_SIG:
    {
        uint16_t duty = Q_EVT_CAST(SetDutyEvt)->duty;
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, duty);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_MODE_SIG:
    {
        _mode = Q_EVT_CAST(SetModeEvt)->mode;
        if (_mode == Mode::RATE)
        {
            status_ = tran(&active_rate);
        }
        else
        {
            status_ = Q_RET_HANDLED;
        }
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

Q_STATE_DEF(mc::MotorControlAO, active_rate)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Arm rate control timer
        _rateControlTimer.armX(_rateControlTimerInterval, _rateControlTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disarm rate control timer
        _rateControlTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_RATE_SIG:
    {
        // Update reference rate
        float new_rate = Q_EVT_CAST(SetRateEvt)->rate;
        if (std::abs(new_rate) <= 1.0f)
        {
            _refRate = new_rate;
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::RATE_CONTROL_UPDATE_SIG:
    {
        // Compute rate acceleration
        float rate_ddt = -_rateStiffness * (_currentRate - _refRate) - _rateDamping * _currentDRate;
        // Integrate
        _currentDRate += rate_ddt;
        _currentRate += _currentDRate;
        // Convert current rate to PWM duty cycle and direction
        uint16_t duty =
            static_cast<uint16_t>(std::abs(_currentRate) * float(_fsr - _pwmLowerDeadband))
            + _pwmLowerDeadband;
        // Set duty cycle
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, duty);
        // Set direction
        HAL_GPIO_WritePin(_mcDevice->_mDirPort, _mcDevice->_mDirPinNum,
                          static_cast<GPIO_PinState>(_currentRate > _eps));
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_MODE_SIG:
    {
        _mode = Q_EVT_CAST(SetModeEvt)->mode;
        if (_mode == Mode::DUTY)
        {
            status_ = tran(&active_duty);
        }
        else
        {
            status_ = Q_RET_HANDLED;
        }
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
        // Reset states
        _currentRate  = 0.0f;
        _currentDRate = 0.0f;
        // Disable motor driver
        HAL_GPIO_WritePin(_mcDevice->_mEnPort, _mcDevice->_mEnPinNum, GPIO_PIN_SET);
        // Set duty cycle to 0
        __HAL_TIM_SET_COMPARE(_mcDevice->_htim, _mcDevice->_htimCh, 0U);
        // Arm fault recovery timer
        _faultRecoveryTimer.armX(_faultRecoveryTimerInterval, _faultRecoveryTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disarm fault recovery timer
        _faultRecoveryTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::FAULT_RECOVERY_SIG:
    {
        // All faults are automatically recoverable as long as the fault condition clears
        // Update faults
        UpdateFaults();
        // If all fault conditions are cleared, then reset
        if (!IsActiveFaults())
        {
            status_ = tran(&initializing);
        }
        else
        {
            status_ = Q_RET_HANDLED;
        }
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
