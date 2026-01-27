#include "mission_ao.hpp"

#include <stdarg.h>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "i2c_mailbox.hpp"
#include "imu_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

// Protocol specification
// If a valid Mission Module I2C address is defined as a parameter:
//      HPC periodically tx IMU measurements to MM (forget about this for now)
//          Master tx
// Always:
//      MM tx motor control mode to HPC
//      MM tx motor 1 rate setpoint to HPC
//      MM tx motor 1 duty setpoint to HPC
//      MM tx motor 1 dir setpoint to HPC
//      MM tx motor 2 rate setpoint to HPC
//      MM tx motor 2 duty setpoint to HPC
//      MM tx motor 2 dir setpoint to HPC
//      MM tx reset motor 1 to HPC
//      MM tx reset motor 2 to HPC
//      MM tx reset IMU to HPC
//      MM tx IMU run compensation to HPC
//      MM tx param set to HPC
//      MM tx param commit to HPC
//          Slave rx
//
//      MM tx value to read from HPC (applies to next MM master rx)
//      - Param value
//      - IMU data
//          Slave rx
//
//      MM rx data ready from HPC
//          Slave tx (lock protect)
//      MM rx data from HPC
//          Slave tx

static mission::I2CMailbox i2cMailbox(&hi2c2);

/// @brief I2C slave tx callback
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

/// @brief I2C slave tx callback
/// @param hi2c
extern "C" void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    }
    else if (hi2c->Instance == I2C2)
    {
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
        // Identify the command code
        // Pack and publish event
    }
}

/// @brief Address match callback
/// @param hi2c
/// @param TransferDirection
/// @param AddrMatchCode
extern "C" void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection,
                                     uint16_t AddrMatchCode)
{
    if (hi2c->Instance == I2C1)
    {
        if (TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            i2cMailbox.StartSlaveRx();
            // HAL_I2C_Slave_Receive_IT(&hi2c1, buf, 4);
        }
        else
        {
            i2cMailbox.StartSlaveTx();
            // HAL_I2C_Slave_Receive_IT(&hi2c1, buf, 4);
        }
    }
    else if (hi2c->Instance == I2C2)
    {
        // Identify the command code
        // Pack and publish event
    }
}

/// @brief Listen mode complete callback
/// @param hi2c
extern "C" void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c) {}

/// @brief I2C error handler callback
/// @param hi2c
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {}

namespace mission
{
MissionAO::MissionAO()
    : QP::QActive(&initial),
      _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
      _faultRequestTimer(this, PrivateSignals::SUBS_FAULT_REQUEST_SIG, 0U)
{
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

void MissionAO::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        POST(&evt, this);
    }
}

void MissionAO::PrintFault()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::PRINT_FAULT_SIG);
        POST(&evt, this);
    }
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
    i2cMailbox.init(nullptr, 0U);
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
        const char* test    = "test";
        char        buf[10] = {0};
        // volatile auto rc1 = HAL_I2C_Master_Transmit(&hi2c1, 0x08<<1, (uint8_t*)test,
        // strlen(test), 100); volatile auto rc2 = HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*)buf,
        // 10);

        // volatile auto rc1 = HAL_I2C_EnableListen_IT(&hi2c1);
        // volatile auto rc2 = HAL_I2C_Master_Transmit(&hi2c2, 0x10<<1, (uint8_t*)test,
        // strlen(test), 100); volatile auto rc1 = HAL_I2C_DisableListen_IT(&hi2c2); volatile auto
        // rc1 = HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)buf, 4U);
        volatile auto rc1 = HAL_I2C_EnableListen_IT(&hi2c1);
        volatile auto rc2 = HAL_I2C_Master_Transmit_IT(&hi2c2, 0x10 << 1, (uint8_t*)test, 4U);
        // HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)buf, 4U);

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
