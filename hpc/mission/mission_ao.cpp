#include "mission_ao.hpp"

#include "bsp.hpp"

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

Q_STATE_DEF(MissionAO, initial)
{
    Q_UNUSED_PAR(e);
    // return tran(&initializing);
    return tran(&active);
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

Q_STATE_DEF(MissionAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Arm rate control timer
        _imuTimer.armX(_imuTimerInterval, _imuTimerInterval);

        _imu.Deselect();

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
        _imu.ReadIMUData();
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
