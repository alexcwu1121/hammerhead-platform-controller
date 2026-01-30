#ifndef I2C_MAILBOX_HPP_
#define I2C_MAILBOX_HPP_

#include "bsp.hpp"

namespace mission
{
/// @brief Slave operation codes
enum opcode : uint8_t
{
    NO_OP = 0U,
    WRITE_MC_MODE,
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
    WRITE_PARAM_VAL,
    WRITE_PARAM_COMMIT,
    READ_PARAM_VAL,
    READ_IMU_DATA,
    NUM_OPS
};

/// @brief I2C Mailbox class
template <uint16_t NUM_OPS>
class I2CMailbox : public QP::QHsm
{
   public:
    /// @brief Constructor
    I2CMailbox(I2C_HandleTypeDef* device);

    /// @brief Initiate a master rx transaction
    inline void StartMasterRx();

    /// @brief Initiate a master rx transaction
    inline void StartMasterTx();

    /// @brief Initiate a slave rx transaction
    inline void StartSlaveRx();

    /// @brief Initiate a slave tx transaction
    inline void StartSlaveTx();

    /// @brief Complete a transaction
    inline void CompleteTransaction();

    /// @brief Reset the mailbox
    inline void Reset();

    /// @brief Register an rx buffer for an opcode
    void RegisterRxBuf(uint8_t opcode, uint8_t* buf, uint16_t size);

    /// @brief Register tx buffer for an opcode
    void RegisterTxBuf(uint8_t opcode, const uint8_t* buf, uint16_t size);

   private:
    /// @brief I2C peripheral
    I2C_HandleTypeDef* _device;
    /// @brief Rx operation buffer table
    uint8_t* _rxBufs[NUM_OPS] = nullptr;
    /// @brief Tx operation buffer table
    uint8_t* _txBufs[NUM_OPS] = nullptr;
    /// @brief Flag indicating if this state machine has been initialized
    bool _isStarted = false;
    /// @brief Active operation code set by master
    uint8_t _activeOpcode = 0U;  // 0 is a reserved opcode

    /// @brief Private signals
    enum PrivateSignals : QP::QSignal
    {
        START_MASTER_RX_SIG = bsp::PublicSignals::MAX_PUB_SIG,
        START_MASTER_TX_SIG,
        START_SLAVE_RX_SIG,
        START_SLAVE_TX_SIG,
        COMPLETE_TRANSACTION_SIG,
        RESET_SIG,
        MAX_PRIV_SIG
    };

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);

    /// @brief Slave idle
    Q_STATE_DECL(slaveIdle);
    /// @brief Slave busy
    Q_STATE_DECL(slaveBusy);
    /// @brief Slave waiting for a command
    Q_STATE_DECL(slaveWaitCmd);
    /// @brief Slave rx busy mode
    Q_STATE_DECL(slaveRxBusy);
    /// @brief Slave tx busy mode
    Q_STATE_DECL(slaveTxBusy);

    /// @brief Master mode
    Q_STATE_DECL(master);
    /// @brief Master rx busy mode
    Q_STATE_DECL(masterRxBusy);
    /// @brief Master tx busy mode
    Q_STATE_DECL(masterTxBusy);
};

inline void I2CMailbox::StartMasterRx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_MASTER_RX_SIG);
        dispatch(&evt, 0U);
    }
}

inline void I2CMailbox::StartMasterTx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_MASTER_TX_SIG);
        dispatch(&evt, 0U);
    }
}

inline void I2CMailbox::StartSlaveRx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_RX_SIG);
        dispatch(&evt, 0U);
    }
}

inline void I2CMailbox::StartSlaveTx()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::START_SLAVE_TX_SIG);
        dispatch(&evt, 0U);
    }
}

inline void I2CMailbox::CompleteTransaction()
{
    HAL_I2C_EnableListen_IT(_device);
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::COMPLETE_TRANSACTION_SIG);
        dispatch(&evt, 0U);
    }
}

inline void I2CMailbox::Reset()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_SIG);
        dispatch(&evt, 0U);
    }
}
}  // namespace mission

#endif
