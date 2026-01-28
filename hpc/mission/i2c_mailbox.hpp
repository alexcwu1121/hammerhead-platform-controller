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
    READ_IMU_DATA
};

/// @brief I2C Master tx transaction
struct I2CMasterTxTransaction
{
    /// @brief Maximum size of a queued transaction
    static constexpr uint16_t _dataMaxSize = 64U;

    /// @brief Slave device address
    uint16_t addr;
    /// @brief Transaction data
    uint8_t data[_dataMaxSize];
    /// @brief Transaction size
    uint16_t size;
};

/// @brief I2C Mailbox class
class I2CMailbox : public QP::QHsm
{
   public:
    /// @brief Rx callback type
    using RxCallback = void (*)(uint8_t* buf, uint16_t size);

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

    /// @brief Load tx buffer
    // void LoadTxBuffer();
    /// TODO: Perfect forward a tx transaction to location in queue

    /// @brief Register a callback function for when master rx transactions succeed
    void RegisterMasterRxCallback(RxCallback callback);

    /// @brief Register a callback function for when slave rx transactions succeed
    void RegisterSlaveRxCallback(RxCallback callback);

   private:
    /// @brief Rx buffer size
    static constexpr uint16_t _rxBufSize = 32U;
    /// @brief Tx buffer size
    static constexpr uint16_t _txBufSize = 32U;
    /// @brief Rx buffer
    uint8_t _rxBuf[_rxBufSize] = {0};
    /// @brief Tx buffer
    uint8_t _txBuf[_txBufSize] = {0};
    /// @brief I2C peripheral
    I2C_HandleTypeDef* _device;
    /// @brief Master rx transaction callback
    RxCallback _masterRxCallback = nullptr;
    /// @brief Slave rx transaction callback
    RxCallback _slaveRxCallback = nullptr;
    /// @brief Flag indicating if this state machine has been initialized
    bool _isStarted = false;
    /// @brief Active operation code set by master
    opcode _activeOpcode = opcode::NO_OP;

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
