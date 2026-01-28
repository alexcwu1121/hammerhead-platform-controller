#include "i2c_mailbox.hpp"

namespace mission
{
I2CMailbox::I2CMailbox(I2C_HandleTypeDef* device) : QP::QHsm(&initial), _device(device) {}

void I2CMailbox::RegisterMasterRxCallback(RxCallback callback)
{
    if (callback != nullptr)
    {
        _masterRxCallback = callback;
    }
}

void I2CMailbox::RegisterSlaveRxCallback(RxCallback callback)
{
    if (callback != nullptr)
    {
        _slaveRxCallback = callback;
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
        /// TODO: Clear buffers, etc
        status_ = tran(&slaveIdle);
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
        _activeOpcode = opcode::NO_OP;

        // Enable listen mode
        HAL_I2C_EnableListen_IT(_device);
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::START_SLAVE_RX_SIG:
    {
        // Start listening for an opcode frame
        if (HAL_I2C_Slave_Seq_Receive_IT(_device, (uint8_t*)&_activeOpcode, sizeof(opcode),
                                         I2C_FIRST_AND_NEXT_FRAME)
            == HAL_StatusTypeDef::HAL_OK)
        {
            status_ = tran(&slaveWaitCmd);
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

Q_STATE_DEF(mission::I2CMailbox, slaveBusy)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::START_MASTER_TX_SIG:
    {
        /// TODO: Queue it up
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

Q_STATE_DEF(mission::I2CMailbox, slaveWaitCmd)
{
    QP::QState status_;
    switch (e->sig)
    {
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        // Operation frame received. These opcodes return immediately.
        switch (_activeOpcode)
        {
        case opcode::WRITE_MC1_RESET:
        {
            /// TODO: call callback immediately then return to idle
            status_ = tran(&slaveIdle);
            break;
        }
        case opcode::WRITE_MC2_RESET:
        {
            /// TODO: call callback immediately then return to idle
            status_ = tran(&slaveIdle);
            break;
        }
        case opcode::WRITE_IMU_RESET:
        {
            /// TODO: call callback immediately then return to idle
            status_ = tran(&slaveIdle);
            break;
        }
        case opcode::WRITE_IMU_COMP:
        {
            /// TODO: call callback immediately then return to idle
            status_ = tran(&slaveIdle);
            break;
        }
        case opcode::WRITE_PARAM_COMMIT:
        {
            /// TODO: call callback immediately then return to idle
            status_ = tran(&slaveIdle);
            break;
        }
        case opcode::NO_OP:
        {
            status_ = tran(&slaveIdle);
            break;
        }
        default:
        {
            status_ = Q_RET_HANDLED;
            break;
        }
        }
        break;
    }
    case PrivateSignals::START_SLAVE_RX_SIG:
    {
        auto RX = [this](const uint16_t size)
        {
            if (HAL_I2C_Slave_Seq_Receive_IT(this->_device, this->_rxBuf, size,
                                             I2C_FIRST_AND_NEXT_FRAME)
                == HAL_StatusTypeDef::HAL_OK)
            {
                return this->tran(&(this->slaveRxBusy));
            }
            else
            {
                return this->tran(&(this->slaveIdle));
            }
        };

        // Operation frame received. Wait for operation
        switch (_activeOpcode)
        {
        case opcode::WRITE_MC_MODE:
        {
            status_ = RX(1U);
            break;
        }
        case opcode::WRITE_MC1_RATE:
        case opcode::WRITE_MC2_RATE:
        {
            status_ = RX(4U);
            break;
        }
        case opcode::WRITE_MC1_DUTY:
        case opcode::WRITE_MC2_DUTY:
        {
            status_ = RX(2U);
            break;
        }
        case opcode::WRITE_MC1_DIR:
        case opcode::WRITE_MC2_DIR:
        {
            status_ = RX(1U);
            break;
        }
        case opcode::WRITE_PARAM_VAL:
        {
            status_ = RX(9U);
            break;
        }
        case opcode::NO_OP:
        default:
        {
            status_ = tran(&slaveIdle);
            break;
        }
        }
        break;
    }
    case PrivateSignals::START_SLAVE_TX_SIG:
    {
        auto TX = [this](const uint16_t size)
        {
            if (HAL_I2C_Slave_Transmit_IT(this->_device, this->_rxBuf, size)
                == HAL_StatusTypeDef::HAL_OK)
            {
                return this->tran(&(this->slaveTxBusy));
            }
            else
            {
                return this->tran(&(this->slaveIdle));
            }
        };

        // Operation frame received. Wait for operation
        switch (_activeOpcode)
        {
        case opcode::READ_PARAM_VAL:
        {
            status_ = TX(9U);
            break;
        }
        case opcode::READ_IMU_DATA:
        {
            status_ = TX(24U);
            break;
        }
        case opcode::NO_OP:
        default:
        {
            status_ = tran(&slaveIdle);
            break;
        }
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
        if (_slaveRxCallback != nullptr)
        {
            _slaveRxCallback(_rxBuf, _rxBufSize);
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

Q_STATE_DEF(mission::I2CMailbox, master)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Disable listening mode before any master mode transactions
        HAL_I2C_DisableListen_IT(_device);
        status_ = Q_RET_HANDLED;
        break;
    }
    case Q_EXIT_SIG:
    {
        // Enable listening mode after finishing master mode transactions
        HAL_I2C_EnableListen_IT(_device);
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

Q_STATE_DEF(mission::I2CMailbox, masterRxBusy)
{
    QP::QState status_;
    switch (e->sig)
    {
    /*
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        // Rx callback on complete transaction
        if (_masterRxCallback != nullptr)
        {
            _masterRxCallback(_rxBuf, _rxBufSize);
        }
        status_ = tran(&slaveIdle);
        break;
    }
    */
    default:
    {
        status_ = super(&master);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(mission::I2CMailbox, masterTxBusy)
{
    QP::QState status_;
    switch (e->sig)
    {
    /*
    case PrivateSignals::COMPLETE_TRANSACTION_SIG:
    {
        status_ = tran(&slaveIdle);
        break;
    }
    */
    default:
    {
        status_ = super(&master);
        break;
    }
    }
    return status_;
}
}  // namespace mission
