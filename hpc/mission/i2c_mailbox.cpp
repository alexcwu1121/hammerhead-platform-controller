#include "i2c_mailbox.hpp"

namespace mission
{
I2CMailbox::I2CMailbox(I2C_HandleTypeDef* device) : QP::QHsm(&initial), _device(device) {}

void I2CMailbox::StartMasterRx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_MASTER_RX_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::StartMasterTx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_MASTER_TX_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::StartSlaveRx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_RX_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::StartSlaveTx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_TX_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::CompleteTransaction()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::COMPLETE_TRANSACTION_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        dispatch(&evt, 0U);
    }
}

void I2CMailbox::RegisterRxCallback(RxCallback callback)
{
    if (callback != nullptr)
    {
        _rxCallback = callback;
    }
}

Q_STATE_DEF(mission::I2CMailbox, initial)
{
    Q_UNUSED_PAR(e);
    _isStarted = true;
    return tran(&idle);
}

Q_STATE_DEF(mission::I2CMailbox, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        /// TODO: Clear buffers, etc
        status_ = tran(&idle);
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

Q_STATE_DEF(mission::I2CMailbox, idle)
{
    QP::QState status_;
    switch (e->sig)
    {
    /*
    case PrivateSignals::START_MASTER_RX_SIG:
    {
        if(HAL_I2C_Master_Receive_IT(_device, _rxBuf, _rxBufSize) == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&busyRx);
        } else
        {
            status_ = Q_RET_HANDLED:
        }
        break;
    }
    case PrivateSignals::START_MASTER_TX_SIG:
    {
        if(HAL_I2C_Master_Transmit_IT(_device, _txBuf, _txBufSize) == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&busyTx);
        } else
        {
            status_ = Q_RET_HANDLED:
        }
        break;
    }
    */
    case PrivateSignals::START_SLAVE_RX_SIG:
    {
        if (HAL_I2C_Slave_Receive_IT(_device, _rxBuf, _rxBufSize) == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&busyRx);
        }
        else
        {
            status_ = Q_RET_HANDLED;
        }
        break;
    }
    case PrivateSignals::START_SLAVE_TX_SIG:
    {
        if (HAL_I2C_Slave_Transmit_IT(_device, _txBuf, _txBufSize) == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&busyTx);
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

Q_STATE_DEF(mission::I2CMailbox, busyRx)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        // Rx callback on complete transaction
        if (_rxCallback != nullptr)
        {
            _rxCallback(_rxBuf, _rxBufSize);
        }
        status_ = tran(&idle);
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

Q_STATE_DEF(mission::I2CMailbox, busyTx)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        status_ = tran(&idle);
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
