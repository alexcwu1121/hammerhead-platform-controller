#include "mission_ao.hpp"

#include <stdarg.h>

#include "bsp.hpp"
#include "cli_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

namespace mission
{
MissionAO::MissionAO()
    : QP::QActive(&initial),
      _imu(P_IMU_CS_GPIO_Port, P_IMU_CS_Pin, &hspi1),
      _imuTimer(this, PrivateSignals::IMU_SERVICE_SIG, 0U),
      _imuStreamTimer(this, PrivateSignals::IMU_STREAM_SIG, 0U),
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

void MissionAO::RunIMUCompensation()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RUN_IMU_COMPENSATION_SIG);
        POST(&evt, this);
    }
}

void MissionAO::StartIMUStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_IMU_STREAM);
        POST(&evt, this);
    }
}

void MissionAO::StopIMUStream()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::STOP_IMU_STREAM);
        POST(&evt, this);
    }
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

Q_STATE_DEF(MissionAO, initial)
{
    Q_UNUSED_PAR(e);
    subscribe(bsp::PublicSignals::FAULT_SIG);
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
        // Originating subsystem
        bsp::SubsystemID id = Q_EVT_CAST(bsp::FaultEvt)->id;
        // Fault code
        uint8_t fault = Q_EVT_CAST(bsp::FaultEvt)->fault;
        // Fault active/inactive
        bool active = Q_EVT_CAST(bsp::FaultEvt)->active;

        if (fault > bsp::MAX_SUBSYSTEM_FAULTS)
        {
            // Fault code can't exceed max faults
            cli::CLIAO::Inst().Printf("ERROR: Fault code out of range");
        }
        else
        {
            // If fault has cleared, check if cleared fault matches
            //  prior active fault. Log warning otherwise
            if (!active && _faultStates[id][fault])
            {
                cli::CLIAO::Inst().Printf("WARNING: Cleared fault does not match existing fault");
            }

            // Update fault status
            // System convention is 0 for nofault, but not enforced
            _faultStates[id][fault] = active;
        }

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
        // Verify IMU device ID and enable SPI peripheral
        imu::Fault fault = _imu.Initialize();

        // Set accelerometer ODR
        fault = _imu.SetAccODR(imu::BMI270::AccODR::ODR_800);

        // Set accelerometer range
        fault = _imu.SetAccRange(imu::BMI270::AccRange::G_4);

        // Set gyroscope ODR
        fault = _imu.SetGyrODR(imu::BMI270::GyrODR::ODR_1K6);

        // Set gyroscope range
        fault = _imu.SetGyrRange(imu::BMI270::GyrRange::DPS_500);

        // Initialize
        if (fault != imu::Fault::NO_FAULT)
        {
            _faultStates[_id][Fault::IMU_INIT_FAILED] = true;
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
        // Arm imu polling timer
        _imuTimer.armX(_imuTimerInterval, _imuTimerInterval);
        // Arm subsystem fault heartbeat timer
        _faultRequestTimer.armX(_faultRequestTimerInterval, _faultRequestTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disarm imu polling timer
        _imuTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::IMU_SERVICE_SIG:
    {
        imu::Fault fault = _imu.ReadData(_imuData);
        if (fault)
        {
            /// TODO: Log fault in eeprom
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::RUN_IMU_COMPENSATION_SIG:
    {
        imu::Fault fault = _imu.RunCompensation();
        if (fault != imu::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Fault during IMU compensation (%hhu)", fault);
        }
        else
        {
            cli::CLIAO::Inst().Printf("INFO: IMU compensation finished");
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::START_IMU_STREAM:
    {
        // Arm imu stream timer
        _imuStreamTimer.armX(_imuStreamTimerInterval, _imuStreamTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::STOP_IMU_STREAM:
    {
        // Disarm imu stream timer
        _imuStreamTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::IMU_STREAM_SIG:
    {
        // Print IMU data
        cli::CLIAO::Inst().Printf(
            ">>>>>>>>>>>>>>\n\r"
            "Acc X: %+7.2f g\n\r"
            "Acc Y: %+7.2f g\n\r"
            "Acc Z: %+7.2f g\n\r"
            "Gyr X: %+7.2f dps\n\r"
            "Gyr Y: %+7.2f dps\n\r"
            "Gyr Z: %+7.2f dps\n\r"
            ">>>>>>>>>>>>>>\n\r",
            _imuData.acc[0], _imuData.acc[1], _imuData.acc[2], _imuData.gyr[0], _imuData.gyr[1],
            _imuData.gyr[2]);
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
            if (_faultStates[_id][fault])
            {
                _faultStates[_id][fault] = false;
            }
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
