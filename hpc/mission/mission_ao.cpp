#include "mission_ao.hpp"

#include "bsp.hpp"
#include "cli_ao.hpp"

namespace mission
{
MissionAO::MissionAO()
    : QP::QActive(&initial),
      _imu(P_IMU_CS_GPIO_Port, P_IMU_CS_Pin, &hspi1),
      _imuTimer(this, PrivateSignals::IMU_SERVICE_SIG, 0U)
{
}

void MissionAO::Start(const QP::QPrioSpec priority, bsp::SubsystemID id)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
    _id        = id;
    _isStarted = true;
}

void MissionAO::RunIMUCompensation()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RUN_IMU_COMPENSATION_SIG);
        POST(&evt, this);
    }
}

Q_STATE_DEF(MissionAO, initial)
{
    Q_UNUSED_PAR(e);
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
        // Arm rate control timer
        _imuTimer.armX(_imuTimerInterval, _imuTimerInterval);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Disarm rate control timer
        _imuTimer.disarm();
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::IMU_SERVICE_SIG:
    {
        imu::IMUData data;
        imu::Fault   fault = _imu.ReadData(data);
        status_            = Q_RET_HANDLED;
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
