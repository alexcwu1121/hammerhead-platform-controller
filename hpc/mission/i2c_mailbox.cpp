#include "i2c_mailbox.hpp"

namespace mission
{
I2CMailbox::I2CMailbox(I2C_HandleTypeDef* device) : QP::QHsm(&initial), _device(device) {}

void I2CMailbox::RegisterOp(uint8_t opcode, uint8_t* buf, uint16_t size, OpCallback callback)
{
    if (buf != nullptr && callback != nullptr)
    {
        _opBufs[opcode]      = buf;
        _opBufSizes[opcode]  = size;
        _opCallbacks[opcode] = callback;
    }
}

Q_STATE_DEF(mission::I2CMailbox, initial)
{
    Q_UNUSED_PAR(e);
    _isStarted = true;
    return tran(&slaveIdle);
}

Q_STATE_DEF(mission::I2CMailbox, root)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::RESET_SIG:
    {
        // Clear opcode
        _activeOpcode = 0U;
        status_       = tran(&slaveIdle);
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

Q_STATE_DEF(mission::I2CMailbox, slaveIdle)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Reset active opcode
        _activeOpcode = 0U;

        // Enable listen mode
        HAL_I2C_EnableListen_IT(_device);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::START_SLAVE_RX_SIG:
    {
        // Start listening for an opcode frame
        if (HAL_I2C_Slave_Seq_Receive_IT(_device, (uint8_t*)&_activeOpcode, sizeof(_activeOpcode),
                                         I2C_FIRST_AND_NEXT_FRAME)
            == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&slaveWaitCmd);
        }
        else
        {
            status_ = Q_RET_HANDLED;
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

Q_STATE_DEF(mission::I2CMailbox, slaveBusy)
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

Q_STATE_DEF(mission::I2CMailbox, slaveWaitCmd)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        // This opcode requires no response from the slave
        // Call callback immediately then return to idle
        if (_opCallbacks[_activeOpcode] != nullptr)
        {
            _opCallbacks[_activeOpcode]();
        }
        break;
    }
    case PrivateSignals::START_SLAVE_RX_SIG:
    {
        // Start rx transaction
        if (HAL_I2C_Slave_Seq_Receive_IT(_device, _opBufs[_activeOpcode],
                                         _opBufSizes[_activeOpcode], I2C_FIRST_AND_NEXT_FRAME)
            == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&slaveRxBusy);
        }
        else
        {
            status_ = tran(&slaveIdle);
        }
        break;
    }
    case PrivateSignals::START_SLAVE_TX_SIG:
    {
        // Call tx callback before initiating tx
        if (_opCallbacks[_activeOpcode] != nullptr)
        {
            _opCallbacks[_activeOpcode]();
        }

        // Initiate Tx
        if (HAL_I2C_Slave_Seq_Transmit_IT(_device, _opBufs[_activeOpcode],
                                          _opBufSizes[_activeOpcode], I2C_FIRST_AND_NEXT_FRAME)
            == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&slaveTxBusy);
        }
        else
        {
            status_ = tran(&slaveIdle);
        }
        break;
    }
    default:
    {
        status_ = super(&slaveBusy);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mission::I2CMailbox, slaveRxBusy)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        // Rx callback on complete transaction
        if (_opCallbacks[_activeOpcode] != nullptr)
        {
            _opCallbacks[_activeOpcode]();
        }
        status_ = tran(&slaveIdle);
        break;
    }
    default:
    {
        status_ = super(&slaveBusy);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mission::I2CMailbox, slaveTxBusy)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        status_ = tran(&slaveIdle);
        break;
    }
    default:
    {
        status_ = super(&slaveBusy);
        break;
    }
    }
    return status_;
}
}  // namespace mission
