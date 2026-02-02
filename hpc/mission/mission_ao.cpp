#include "mission_ao.hpp"

#include <stdarg.h>

#include <mutex>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "imu_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

/// @brief Slave operation codes
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
    READ_IMU_DATA,
    NUM_OPS
};

/// @brief Good ol fashioned state machine
enum mailbox_state
{
    SlaveIdle,
    SlaveWaitCmd,
    SlaveRxBusy,
    SlaveTxBusy,
};

namespace
{
/// @brief Operation buffer table
uint8_t* _opBufs[NUM_OPS] = {nullptr};
/// @brief Operation buffer max sizes
uint16_t _opBufSizes[NUM_OPS] = {0U};
/// @brief Operation callback function
using OpCallback = void (*)(void);
/// @brief Operation callback function table
OpCallback _opCallbacks[NUM_OPS] = {nullptr};
/// @brief Current state of i2c mailbox state machine
volatile mailbox_state _state = mailbox_state::SlaveIdle;
/// @brief Active operation code set by master
volatile opcode _activeOpcode = NO_OP;

/// @brief Register a mailbox operation
/// @param op
/// @param buf
/// @param size
/// @param callback
void RegisterOp(opcode op, uint8_t* buf, uint16_t size, OpCallback callback)
{
    _opBufs[(uint8_t)op]      = buf;
    _opBufSizes[(uint8_t)op]  = size;
    _opCallbacks[(uint8_t)op] = callback;
}

/// @brief MC1 Mode. Only accessed in interrupt context.
volatile mc::Mode mc1Mode = mc::Mode::RATE;
/// @brief MC2 Mode. Only accessed in interrupt context.
volatile mc::Mode mc2Mode = mc::Mode::RATE;
/// @brief MC1 Rate. Only accessed in interrupt context.
volatile float mc1Rate = 0.0f;
/// @brief MC2 Rate. Only accessed in interrupt context.
volatile float mc2Rate = 0.0f;
/// @brief MC1 Duty. Only accessed in interrupt context.
volatile uint16_t mc1Duty = 0U;
/// @brief MC2 Duty. Only accessed in interrupt context.
volatile uint16_t mc2Duty = 0U;
/// @brief MC1 Dir. Only accessed in interrupt context.
volatile mc::Dir mc1Dir = mc::Dir::CW;
/// @brief MC2 Dir. Only accessed in interrupt context.
volatile mc::Dir mc2Dir = mc::Dir::CW;

/// @brief Latest IMU data. Accessed in interrupt context and mission AO.
struct
{
    volatile float acc[3] = {0.0f};
    volatile float gyr[3] = {0.0f};
} imuData;

/// @brief MC1 mode change mailbox callback
void UpdateMC1Mode()
{
    mc::MotorControlAO::MC1Inst().SetMode(mc1Mode);
}

/// @brief MC2 mode change mailbox callback
void UpdateMC2Mode()
{
    mc::MotorControlAO::MC2Inst().SetMode(mc2Mode);
}

/// @brief MC1 rate change mailbox callback
void UpdateMC1Rate()
{
    mc::MotorControlAO::MC1Inst().SetRate(mc1Rate);
}

/// @brief MC2 rate change mailbox callback
void UpdateMC2Rate()
{
    mc::MotorControlAO::MC2Inst().SetRate(mc2Rate);
}

/// @brief MC1 duty cycle change mailbox callback
void UpdateMC1Duty()
{
    mc::MotorControlAO::MC1Inst().SetDuty(mc1Duty);
}

/// @brief MC2 duty cycle change mailbox callback
void UpdateMC2Duty()
{
    mc::MotorControlAO::MC2Inst().SetDuty(mc2Duty);
}

/// @brief MC1 dir change mailbox callback
void UpdateMC1Dir()
{
    mc::MotorControlAO::MC1Inst().SetDir(mc1Dir);
}

/// @brief MC2 dir change mailbox callback
void UpdateMC2Dir()
{
    mc::MotorControlAO::MC2Inst().SetDir(mc2Dir);
}

/// @brief MC1 reset mailbox callback
void ResetMC1()
{
    mc::MotorControlAO::MC1Inst().Reset();
}

/// @brief MC2 reset mailbox callback
void ResetMC2()
{
    mc::MotorControlAO::MC2Inst().Reset();
}

/// @brief IMU reset mailbox callback
void ResetIMU()
{
    imu::IMUAO::Inst().Reset();
}

/// @brief IMU compensation mailbox callback
void CompIMU()
{
    imu::IMUAO::Inst().RunIMUCompensation();
}
}  // namespace

/// @brief I2C slave tx callback
/// @param hi2c
extern "C" void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    }
    else if (hi2c->Instance == I2C2)
    {
        // Call tx complete callback
        if (_state == mailbox_state::SlaveTxBusy && _opCallbacks[_activeOpcode] != nullptr)
        {
            _opCallbacks[_activeOpcode]();
        }

        // No matter what, a slave tx operation finishing should result
        // in a state change back to idle
        _state = mailbox_state::SlaveIdle;
    }
}

/// @brief I2C slave rx callback
/// @param hi2c
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    }
    else if (hi2c->Instance == I2C2)
    {
        switch (_state)
        {
        case mailbox_state::SlaveIdle:
        case mailbox_state::SlaveWaitCmd:
        {
            // Operation code received
            // If no followup transaction expected, execute callback and return to idle
            if (_opBufSizes[_activeOpcode] == 0)
            {
                // execute callback
                if (_opCallbacks[_activeOpcode] != nullptr)
                {
                    _opCallbacks[_activeOpcode]();
                }
                // go back to idle
                _state = mailbox_state::SlaveIdle;
            }

            // Otherwise, stay in waiting for the followup operation
            _state = mailbox_state::SlaveWaitCmd;
            break;
        }
        case mailbox_state::SlaveRxBusy:
        {
            // Call tx complete callback
            if (_opCallbacks[_activeOpcode] != nullptr)
            {
                _opCallbacks[_activeOpcode]();
            }

            // Go back to idle
            _state = mailbox_state::SlaveIdle;
            break;
        }
        case mailbox_state::SlaveTxBusy:
        default:
        {
            // Something is obviously wrong... reset state
            _state = mailbox_state::SlaveIdle;
            break;
        }
        }
    }
}

/// @brief I2C master tx callback
/// @param hi2c
extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    }
    else if (hi2c->Instance == I2C2)
    {
    }
}

/// @brief I2C master rx callback
/// @param hi2c
extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    }
    else if (hi2c->Instance == I2C2)
    {
    }
}

/// @brief Address match callback
/// @param hi2c
/// @param TransferDirection
/// @param AddrMatchCode
extern "C" void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection,
                                     uint16_t AddrMatchCode)
{
    if (hi2c->Instance == I2C2)
    {
        if (TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            switch (_state)
            {
            case mailbox_state::SlaveIdle:
            {
                // Master should be sending an opcode. Start a receive
                auto rc = HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t*)&_activeOpcode,
                                                       sizeof(_activeOpcode), I2C_FIRST_FRAME);

                // Check if successful
                if (rc == HAL_StatusTypeDef::HAL_OK)
                {
                    // Wait for opcode to arrive
                    _state = mailbox_state::SlaveWaitCmd;
                }
                else
                {
                    _state = mailbox_state::SlaveIdle;
                }
                break;
            }
            case mailbox_state::SlaveWaitCmd:
            {
                // Master is requesting to transmit to the slave
                auto rc = HAL_I2C_Slave_Seq_Receive_IT(hi2c, _opBufs[_activeOpcode],
                                                       _opBufSizes[_activeOpcode], I2C_LAST_FRAME);

                // Check if successful
                if (rc == HAL_StatusTypeDef::HAL_OK)
                {
                    // Wait for the rx operation to finish
                    _state = mailbox_state::SlaveRxBusy;
                }
                else
                {
                    // Failed somehow, reset
                    _state = mailbox_state::SlaveIdle;
                }
                break;
            }
            case mailbox_state::SlaveRxBusy:
            case mailbox_state::SlaveTxBusy:
            default:
            {
                // Something is obviously wrong, reset state
                _state = mailbox_state::SlaveIdle;
                break;
            }
            }
        }
        else
        {
            switch (_state)
            {
            case mailbox_state::SlaveWaitCmd:
            {
                // Master is requesting to receive to the slave
                auto rc = HAL_I2C_Slave_Seq_Transmit_IT(hi2c, _opBufs[_activeOpcode],
                                                        _opBufSizes[_activeOpcode], I2C_LAST_FRAME);

                // Check if successful
                if (rc == HAL_StatusTypeDef::HAL_OK)
                {
                    // Wait for the tx operation to finish
                    _state = mailbox_state::SlaveTxBusy;
                }
                else
                {
                    // Failed somehow, reset
                    _state = mailbox_state::SlaveIdle;
                }
                break;
            }
            case mailbox_state::SlaveIdle:
            case mailbox_state::SlaveRxBusy:
            case mailbox_state::SlaveTxBusy:
            default:
            {
                // Something is obviously wrong, reset state
                _state = mailbox_state::SlaveIdle;
                break;
            }
            }
        }
    }
}

/// @brief Listen mode complete callback
/// @param hi2c
extern "C" void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        // Enable listening again
        HAL_I2C_EnableListen_IT(hi2c);
        // Idle if not already idle
        _state = mailbox_state::SlaveIdle;
    }
}

/// @brief I2C error handler callback
/// @param hi2c
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF)
        {
            // Acknowledge failure often means master stopped, good place to restart
            HAL_I2C_EnableListen_IT(hi2c);
            _state = mailbox_state::SlaveIdle;
        }
    }
}

namespace mission
{
MissionAO::MissionAO()
    : QP::QActive(&initial),
      _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
      _faultRequestTimer(this, PrivateSignals::SUBS_FAULT_REQUEST_SIG, 0U)
{
    // Register i2c mailbox callbacks
    RegisterOp(opcode::WRITE_MC1_MODE, (uint8_t*)&mc1Mode, sizeof(mc1Mode), &UpdateMC1Mode);

    RegisterOp(opcode::WRITE_MC2_MODE, (uint8_t*)&mc2Mode, sizeof(mc2Mode), &UpdateMC2Mode);

    RegisterOp(opcode::WRITE_MC1_RATE, (uint8_t*)&mc1Rate, sizeof(mc1Rate), &UpdateMC1Rate);

    RegisterOp(opcode::WRITE_MC2_RATE, (uint8_t*)&mc2Rate, sizeof(mc2Rate), &UpdateMC2Rate);

    RegisterOp(opcode::WRITE_MC1_DUTY, (uint8_t*)&mc1Duty, sizeof(mc1Duty), &UpdateMC1Duty);

    RegisterOp(opcode::WRITE_MC2_DUTY, (uint8_t*)&mc2Duty, sizeof(mc2Duty), &UpdateMC2Duty);

    RegisterOp(opcode::WRITE_MC1_DIR, (uint8_t*)&mc1Dir, sizeof(mc1Dir), &UpdateMC1Dir);

    RegisterOp(opcode::WRITE_MC2_DIR, (uint8_t*)&mc2Dir, sizeof(mc2Dir), &UpdateMC2Dir);

    RegisterOp(opcode::WRITE_MC1_DIR, (uint8_t*)&mc1Dir, sizeof(mc1Dir), &UpdateMC1Dir);

    RegisterOp(opcode::WRITE_MC2_DIR, (uint8_t*)&mc2Dir, sizeof(mc2Dir), &UpdateMC2Dir);

    RegisterOp(opcode::WRITE_MC1_RESET, nullptr, 0U, &ResetMC1);

    RegisterOp(opcode::WRITE_MC2_RESET, nullptr, 0U, &ResetMC2);

    RegisterOp(opcode::WRITE_IMU_RESET, nullptr, 0U, &ResetIMU);

    RegisterOp(opcode::WRITE_IMU_COMP, nullptr, 0U, &CompIMU);

    RegisterOp(opcode::READ_IMU_DATA, (uint8_t*)&imuData, sizeof(imuData), nullptr);

    // Start slave listen mode
    HAL_I2C_EnableListen_IT(&hi2c2);
}

void MissionAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id        = id;
    _isStarted = true;
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
}

void MissionAO::SetFault(bsp::SubsystemID id, uint8_t fault, bool active)
{
    if (fault > bsp::MAX_SUBSYSTEM_FAULTS)
    {
        // Fault code can't exceed max faults
        cli::CLIAO::Inst().Printf("ERROR: Fault code out of range");
        return;
    }

    if (_faultStates[id][fault] != active)
    {
        // Update internal fault state
        _faultStates[id][fault] = active;
        if (active)
        {
            // Enable fault LED if fault becomes active
            HAL_GPIO_WritePin(P_FAULT_LED_GPIO_Port, P_FAULT_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            // Check for presence of any fault
            bool is_fault = false;
            for (uint8_t subsystem = 0; subsystem < bsp::SubsystemID::NUM_SUBSYSTEMS; subsystem++)
            {
                for (uint8_t fault = 0; fault < bsp::MAX_SUBSYSTEM_FAULTS; fault++)
                {
                    if (_faultStates[subsystem][fault] != 0)
                    {
                        is_fault = true;
                        break;
                    }
                }
            }
            if (is_fault)
            {
                HAL_GPIO_WritePin(P_FAULT_LED_GPIO_Port, P_FAULT_LED_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(P_FAULT_LED_GPIO_Port, P_FAULT_LED_Pin, GPIO_PIN_RESET);
            }
        }
    }
}

Q_STATE_DEF(MissionAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::FAULT_SIG);
    subscribe(bsp::PublicSignals::PARAMETER_UPDATE_SIG);
    subscribe(bsp::PublicSignals::IMU_SIG);

    return tran(&initializing);
}

Q_STATE_DEF(MissionAO, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        status_ = tran(&initializing);
        break;
    }
    case PrivateSignals::FAULT_SIG:
    {
        status_ = tran(&error);
        break;
    }
    case bsp::PublicSignals::FAULT_SIG:
    {
        // Update fault status
        SetFault(Q_EVT_CAST(bsp::FaultEvt)->id, Q_EVT_CAST(bsp::FaultEvt)->fault,
                 Q_EVT_CAST(bsp::FaultEvt)->active);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::PRINT_FAULT_SIG:
    {
        static const char* fmt                              = "\t%s: %d\n\r";
        char               buf[cli::CLIAO::cliPrintBufSize] = {0};

        // Print all mission fault statuses
        static const char* mission = "Mission Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        char* ptr = (char*)memcpy(buf, mission, strlen(mission));
        ptr += strlen(mission);
        for (uint8_t fault = 0; fault < mission::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::MISSION_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            mission::FaultToStr((mission::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        // Print all motor control 1 fault statuses
        static const char* mc1 = "Motor Controller #1 Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        ptr = (char*)memcpy(buf, mc1, strlen(mc1));
        ptr += strlen(mc1);
        for (uint8_t fault = 0; fault < mc::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::MC1_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            mc::FaultToStr((mc::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        // Print all motor control 2 fault statuses
        static const char* mc2 = "Motor Controller #2 Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        ptr = (char*)memcpy(buf, mc2, strlen(mc2));
        ptr += strlen(mc2);
        for (uint8_t fault = 0; fault < mc::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::MC2_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            mc::FaultToStr((mc::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        // Print all imu fault statuses
        static const char* imu = "IMU Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        ptr = (char*)memcpy(buf, imu, strlen(imu));
        ptr += strlen(imu);
        for (uint8_t fault = 0; fault < imu::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::IMU_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            imu::FaultToStr((imu::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        // Print all parameter fault statuses
        static const char* param = "Parameter Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        ptr = (char*)memcpy(buf, param, strlen(param));
        ptr += strlen(param);
        for (uint8_t fault = 0; fault < param::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::PARAMETER_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            param::FaultToStr((param::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        // Print all cli fault statuses
        static const char* cli = "CLI Subsystem\n\r";
        memset(buf, 0U, sizeof(buf));
        ptr = (char*)memcpy(buf, cli, strlen(cli));
        ptr += strlen(cli);
        for (uint8_t fault = 0; fault < cli::Fault::NUM_FAULTS; fault++)
        {
            bool state = _faultStates[bsp::SubsystemID::CLI_SUBSYSTEM][fault];
            ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt,
                            cli::FaultToStr((cli::Fault)fault), state);
        }
        cli::CLIAO::Inst().Printf(buf);

        status_ = Q_RET_HANDLED;
        break;
    }
    case bsp::PublicSignals::PARAMETER_UPDATE_SIG:
    {
        param::ParameterID id  = Q_EVT_CAST(bsp::ParameterUpdateEvt)->id;
        param::Type        val = Q_EVT_CAST(bsp::ParameterUpdateEvt)->value;

        // Update parameter value
        switch (id)
        {
        case param::ParameterID::MM_I2C_ADDR:
        {
            _mmAddr = val._uint8;
            break;
        }
        default:
        {
            break;
        }
        }

        status_ = Q_RET_HANDLED;
        break;
    }
    case bsp::PublicSignals::IMU_SIG:
    {
        // Disable interrupts and update IMU data
        __disable_irq();
        memcpy((uint8_t*)&imuData, (uint8_t*)&(Q_EVT_CAST(imu::IMUEvt)->data), sizeof(imuData));
        __enable_irq();
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

Q_STATE_DEF(MissionAO, initializing)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Request parameters
        param::ParamAO::Inst().RequestUpdate(param::ParameterID::MM_I2C_ADDR);

        // Initialize
        static QP::QEvt evt(PrivateSignals::INITIALIZED_SIG);
        POST(&evt, this);

        status_ = Q_RET_HANDLED;
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

Q_STATE_DEF(MissionAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Arm subsystem fault heartbeat timer
        _faultRequestTimer.armX(_faultRequestTimerInterval, _faultRequestTimerInterval);

        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SUBS_FAULT_REQUEST_SIG:
    {
        QP::QEvt* evt = Q_NEW(QP::QEvt, bsp::PublicSignals::REQUEST_FAULT_SIG);
        PUBLISH(evt, this);
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

Q_STATE_DEF(MissionAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Start attempting fault recovery
        _faultRecoveryTimer.armX(_faultRecoveryTimerInterval, _faultRecoveryTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disable fault recovery
        _faultRecoveryTimer.disarm();

        // Clear all INTERNAL faults on exit
        for (uint8_t fault = 0U; fault < Fault::NUM_FAULTS; fault++)
        {
            SetFault(_id, fault, false);
        }

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
}  // namespace mission
