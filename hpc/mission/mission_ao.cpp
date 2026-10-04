#include "mission_ao.hpp"

#include <stdarg.h>  // NOLINT

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "gpio.h"
#include "imu_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"
#include "thirdparty/printf.h"

/// @brief Published CAN ID lowest index
static constexpr uint16_t PubCANIDIdx = 0x100;

/// @brief Published CAN message IDs
enum PubCANID : uint16_t
{
    PUB_IMU_DATA_ACC_XY = PubCANIDIdx,
    PUB_IMU_DATA_ACC_Z_GYR_X,
    PUB_IMU_DATA_GYR_Y_GYR_Z,
    PUB_BATT_SOC,
    PUB_HEARTBEAT,
    PUB_FAULT_INDEX = 0x120,  // starting index for faults
    MAX_PUB_ID
};

/// @brief Susbcribed CAN ID lowest index
static constexpr uint16_t SubCANIDIdx = 0x200;

/// @brief Subscribed CAN message IDs
enum SubCANID : uint16_t
{
    SUB_WRITE_MC1_MODE = SubCANIDIdx,
    SUB_WRITE_MC2_MODE,
    SUB_WRITE_MC1_RATE,
    SUB_WRITE_MC2_RATE,
    SUB_WRITE_MC1_DUTY,
    SUB_WRITE_MC2_DUTY,
    SUB_WRITE_MC1_DIR,
    SUB_WRITE_MC2_DIR,
    SUB_WRITE_MC1_RESET,
    SUB_WRITE_MC2_RESET,
    SUB_WRITE_IMU_RESET,
    SUB_WRITE_IMU_COMP,
    SUB_WRITE_WATCHDOG,
    MAX_SUB_ID
};

extern "C"
{
    /// @brief CAN receive fifo message callback
    /// @param hcan
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];

        // Retrieve the pending message
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
        {
            /// TODO: Only standard IDs supported
            if (header.IDE == CAN_ID_STD)
            {
                // Inject events
                switch (header.StdId)
                {
                    case SubCANID::SUB_WRITE_MC1_MODE:
                    {
                        if (header.DLC == 1) { mc::MotorControlAO::MC1Inst().SetMode(static_cast<mc::Mode>(data[0])); }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC2_MODE:
                    {
                        if (header.DLC == 1) { mc::MotorControlAO::MC2Inst().SetMode(static_cast<mc::Mode>(data[0])); }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC1_RATE:
                    {
                        if (header.DLC == 4)
                        {
                            float rate = 0.0f;
                            memcpy(&rate, data, sizeof(float));
                            mc::MotorControlAO::MC1Inst().SetRate(rate);
                        }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC2_RATE:
                    {
                        if (header.DLC == 4)
                        {
                            float rate = 0.0f;
                            memcpy(&rate, data, sizeof(float));
                            mc::MotorControlAO::MC2Inst().SetRate(rate);
                        }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC1_DUTY:
                    {
                        if (header.DLC == 2)
                        {
                            uint16_t duty = 0u;
                            memcpy(&duty, data, sizeof(uint16_t));
                            mc::MotorControlAO::MC1Inst().SetDuty(duty);
                        }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC2_DUTY:
                    {
                        if (header.DLC == 2)
                        {
                            uint16_t duty = 0u;
                            memcpy(&duty, data, sizeof(uint16_t));
                            mc::MotorControlAO::MC2Inst().SetDuty(duty);
                        }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC1_DIR:
                    {
                        if (header.DLC == 1) { mc::MotorControlAO::MC1Inst().SetDir(static_cast<mc::Dir>(data[0])); }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC2_DIR:
                    {
                        if (header.DLC == 1) { mc::MotorControlAO::MC2Inst().SetDir(static_cast<mc::Dir>(data[0])); }
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC1_RESET:
                    {
                        mc::MotorControlAO::MC1Inst().Reset();
                        break;
                    }
                    case SubCANID::SUB_WRITE_MC2_RESET:
                    {
                        mc::MotorControlAO::MC2Inst().Reset();
                        break;
                    }
                    case SubCANID::SUB_WRITE_IMU_RESET:
                    {
                        imu::IMUAO::Inst().Reset();
                        break;
                    }
                    case SubCANID::SUB_WRITE_IMU_COMP:
                    {
                        imu::IMUAO::Inst().RunIMUCompensation();
                        break;
                    }
                    case SubCANID::SUB_WRITE_WATCHDOG:
                    {
                        mission::MissionAO::Inst().PokeWatchdog();
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
    }
}

namespace mission
{
MissionAO::MissionAO() :
    QP::QActive(&initial),
    _faultRecoveryTimer(this, PrivateSignals::RESET_SIG, 0U),
    _faultRequestTimer(this, PrivateSignals::SUBS_FAULT_REQUEST_SIG, 0U),
    _battPubTimer(this, PrivateSignals::CAN_PUB_BATT_SIG, 0U),
    _heartbeatTimer(this, PrivateSignals::HEARTBEAT_SIG, 0U),
    _watchdogTimer(this, PrivateSignals::WATCHDOG_EXPIRED_SIG, 0U)
{}

void MissionAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    _id = id;
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
            if (is_fault) { HAL_GPIO_WritePin(P_FAULT_LED_GPIO_Port, P_FAULT_LED_Pin, GPIO_PIN_SET); }
            else { HAL_GPIO_WritePin(P_FAULT_LED_GPIO_Port, P_FAULT_LED_Pin, GPIO_PIN_RESET); }
        }

        // Also publish update over CAN
        _canTxHeader.DLC = 1;

        // Dynamically compute can id from fault table
        // id = fault_idx + subsystem_id*BSP_MAX_FAULTS + fault_id
        _canTxHeader.StdId = PubCANID::PUB_FAULT_INDEX + id * bsp::MAX_SUBSYSTEM_FAULTS + fault;

        // make sure we haven't overflowed into the sub id region
        if (_canTxHeader.StdId >= SubCANIDIdx)
        {
            // we could trap ourselves in a recursion here...
            return;
        }

        _canTxData[0] = active;
        if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
        {
            SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
        }
    }
}

Q_STATE_DEF(MissionAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::FAULT_SIG);
    subscribe(bsp::PublicSignals::PARAMETER_UPDATE_SIG);
    subscribe(bsp::PublicSignals::IMU_SIG);
    subscribe(bsp::PublicSignals::ADC_SIG);

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
            static const char* fmt = "\t%s: %d\n\r";
            char buf[cli::CLIAO::cliPrintBufSize] = {0};

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
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, mc::FaultToStr((mc::Fault)fault),
                                state);
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
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, mc::FaultToStr((mc::Fault)fault),
                                state);
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
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, imu::FaultToStr((imu::Fault)fault),
                                state);
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
                ptr += snprintf(ptr, buf + cli::CLIAO::cliPrintBufSize - ptr, fmt, cli::FaultToStr((cli::Fault)fault),
                                state);
            }
            cli::CLIAO::Inst().Printf(buf);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::PRINT_BATT_SIG:
        {
            cli::CLIAO::Inst().Printf(
                ">>>>>>>>>>>>>>\n\r"
                "Vin: %+7.4f V\n\r"
                "SOC: %+7.4f %\n\r"
                ">>>>>>>>>>>>>>\n\r",
                _lastVin, _lastSOC);

            status_ = Q_RET_HANDLED;
            break;
        }
        case bsp::PublicSignals::PARAMETER_UPDATE_SIG:
        {
            param::ParameterID id = Q_EVT_CAST(bsp::ParameterUpdateEvt)->id;
            param::Type val = Q_EVT_CAST(bsp::ParameterUpdateEvt)->value;

            // Update parameter value
            switch (id)
            {
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_V0:
                {
                    _battV[0] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_S0:
                {
                    _battS[0] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_V1:
                {
                    _battV[1] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_S1:
                {
                    _battS[1] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_V2:
                {
                    _battV[2] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_S2:
                {
                    _battS[2] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_V3:
                {
                    _battV[3] = val._float32;
                    break;
                }
                case param::ParameterID::BATTERY_DISCHARGE_CURVE_S3:
                {
                    _battS[3] = val._float32;
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
            // sending two floats each packet
            _canTxHeader.DLC = 8;

            // Fragment IMU data into 8 byte chunks and send

            // Accelerometer x and y
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_ACC_XY;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.acc[0], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.acc[1], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
            }

            // Accelerometer z and gyro x
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_ACC_Z_GYR_X;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.acc[2], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[0], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
            }

            // Gyro y and z
            _canTxHeader.StdId = PubCANID::PUB_IMU_DATA_GYR_Y_GYR_Z;
            memcpy(_canTxData, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[1], sizeof(float));
            memcpy(_canTxData + 4, &Q_EVT_CAST(imu::IMUEvt)->data.gyr[2], sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
            }

            // Automatically clear fault state on successful transmission
            SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, false);

            status_ = Q_RET_HANDLED;
            break;
        }
        case bsp::PublicSignals::ADC_SIG:
        {
            // Linearly interpolate SOC in percent from input voltage adc measurement

            /// TODO: this platform has no true BMS, so I'm not going to spend more time on this

            if (_battV.size() < 2)
            {
                // nothing to interpolate. report nothing.
                status_ = Q_RET_HANDLED;
                break;
            }

            _lastVin = Q_EVT_CAST(bsp::ADCEvt)->adcVoltages[bsp::ADCChannels::VIN];

            // linear interpolation
            // assumes both voltage and SOC sequences are monotonically increasing
            for (uint8_t i = 1; i < _battV.size(); i++)
            {
                if (_lastVin < _battV[i] && _lastVin > _battV[i - 1])
                {
                    // interpolate
                    _lastSOC = _battS[i - 1] +
                               (_battS[i] - _battS[i - 1]) * (_lastVin - _battV[i - 1]) / (_battV[i] - _battV[i - 1]);
                }
            }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::HEARTBEAT_SIG:
        {
            _canTxHeader.DLC = 0;
            _canTxHeader.StdId = PubCANID::PUB_HEARTBEAT;
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
            }

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::POKE_WATCHDOG_SIG:
        {
            // rearm the watchdog timer
            _watchdogTimer.rearm(_watchdogTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::WATCHDOG_EXPIRED_SIG:
        {
            SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::WATCHDOG_FAULT, true);
            status_ = tran(&selfprotect);
            break;
        }
        case PrivateSignals::ENABLE_WATCHDOG_SIG:
        {
            // enable and rearm watchdog
            /// TODO: you could hack this and use this like a poke... probably not a problem?
            _watchdogEnable = true;
            _watchdogTimer.rearm(_watchdogTimerInterval);
            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::DISABLE_WATCHDOG_SIG:
        {
            // disable watchdog timer
            _watchdogEnable = false;
            _watchdogTimer.disarm();

            // clear fault if applicable (should never be)
            SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::WATCHDOG_FAULT, false);

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
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_V0);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_S0);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_V1);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_S1);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_V2);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_S2);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_V3);
            param::ParamAO::Inst().RequestUpdate(param::ParameterID::BATTERY_DISCHARGE_CURVE_S3);

            // Set up CAN filters
            CAN_FilterTypeDef can_filter;
            can_filter.FilterBank = 0;
            can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
            can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
            can_filter.FilterIdHigh = (SubCANIDIdx << 5U) & 0xFFFF;
            can_filter.FilterIdLow = 0x0000;
            can_filter.FilterMaskIdHigh = SubCANIDIdx << 5U;
            can_filter.FilterMaskIdLow = 0x0000;
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
            can_filter.FilterActivation = ENABLE;

            // Initialize can tx header
            _canTxHeader.ExtId = 0x00;
            _canTxHeader.IDE = CAN_ID_STD;
            _canTxHeader.RTR = CAN_RTR_DATA;
            _canTxHeader.TransmitGlobalTime = DISABLE;

            if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
            {
                MissionAO::SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_INIT_FAILED, true);
                status_ = tran(&error);
                break;
            }

            // Start CAN peripheral
            if (HAL_CAN_Start(&hcan) != HAL_OK)
            {
                // error code should be HAL_CAN_ERROR_NOT_READY if CAN is already started
                if (hcan.ErrorCode != HAL_CAN_ERROR_NOT_READY)
                {
                    MissionAO::SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_INIT_FAILED, true);
                    status_ = tran(&error);
                    break;
                }
            }

            // Enable recieve interrupt
            if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
            {
                MissionAO::SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_INIT_FAILED, true);
                status_ = tran(&error);
                break;
            }

            if (!_hasFirstTimeInit)
            {
                // Arm heartbeat timer asap (and never disarm)
                _heartbeatTimer.armX(_heartbeatTimerInterval, _heartbeatTimerInterval);
                // Arm fault request timer
                _faultRequestTimer.armX(_faultRequestTimerInterval, _faultRequestTimerInterval);
                // Arm battery state CAN pub timer
                _battPubTimer.armX(_battPubTimerInterval, _battPubTimerInterval);
            }

            // Arm watchdog timer
            if (_watchdogEnable) { _watchdogTimer.armX(_watchdogTimerInterval, 0U); }

            // Finish initialization
            _hasFirstTimeInit = true;
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
        case PrivateSignals::CAN_PUB_BATT_SIG:
        {
            // publish over can
            _canTxHeader.DLC = 8;
            _canTxHeader.StdId = PubCANID::PUB_BATT_SOC;
            memcpy(_canTxData, &_lastSOC, sizeof(float));
            memcpy(_canTxData + 4, &_lastVin, sizeof(float));
            if (HAL_CAN_AddTxMessage(&hcan, &_canTxHeader, _canTxData, &_canTxMailbox) != HAL_OK)
            {
                SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::MISSION_CAN_TX_FAILED, true);
            }

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
            for (uint8_t fault = 0U; fault < Fault::NUM_FAULTS; fault++) { SetFault(_id, fault, false); }

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

Q_STATE_DEF(MissionAO, selfprotect)
{
    QP::QState status_;
    switch (e->sig)
    {
        case Q_ENTRY_SIG:
        {
            // Disable motors
            mc::MotorControlAO::MC1Inst().SetWatchdogFault();
            mc::MotorControlAO::MC2Inst().SetWatchdogFault();

            status_ = Q_RET_HANDLED;
            break;
        }
        case Q_EXIT_SIG:
        {
            // Reenable motors
            mc::MotorControlAO::MC1Inst().UnsetWatchdogFault();
            mc::MotorControlAO::MC2Inst().UnsetWatchdogFault();

            // Clear watchdog fault
            SetFault(bsp::SubsystemID::MISSION_SUBSYSTEM, Fault::WATCHDOG_FAULT, false);

            status_ = Q_RET_HANDLED;
            break;
        }
        case PrivateSignals::POKE_WATCHDOG_SIG:
        {
            // watchdog will be rearmed in initializing if enabled
            status_ = tran(&initializing);
            break;
        }
        case PrivateSignals::DISABLE_WATCHDOG_SIG:
        {
            // disabling watchdog will both disable timer and exit selfprotect mode
            _watchdogEnable = false;
            _watchdogTimer.disarm();
            status_ = tran(&initializing);
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
