#ifndef I2C_MAILBOX_HPP_
#define I2C_MAILBOX_HPP_

#include "bsp.hpp"

namespace mission
{
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
    void StartMasterRx();

    /// @brief Initiate a master rx transaction
    void StartMasterTx();

    /// @brief Initiate a slave rx transaction
    void StartSlaveRx();

    /// @brief Initiate a slave tx transaction
    void StartSlaveTx();

    /// @brief Complete a transaction
    void CompleteTransaction();

    /// @brief Reset the mailbox
    void Reset();

    /// @brief Load tx buffer
    // void LoadTxBuffer();
    /// TODO: Perfect forward a tx transaction to location in queue

    /// @brief Register a callback function for when rx transactions succeed
    void RegisterRxCallback(RxCallback callback);

   private:
    /// @brief Rx buffer size
    static constexpr uint16_t _rxBufSize = 256U;
    /// @brief Tx buffer size
    static constexpr uint16_t _txBufSize = 256U;
    /// @brief Rx buffer
    uint8_t _rxBuf[_rxBufSize] = {0};
    /// @brief Tx buffer
    uint8_t _txBuf[_txBufSize] = {0};
    /// @brief I2C peripheral
    I2C_HandleTypeDef* _device;
    /// @brief Rx transaction callback
    RxCallback _rxCallback = nullptr;
    /// @brief Flag indicating if this state machine has been initialized
    bool _isStarted = false;

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
    /// @brief Idle bus status
    Q_STATE_DECL(idle);
    /// @brief Rx busy bus status
    Q_STATE_DECL(busyRx);
    /// @brief Tx busy bus status
    Q_STATE_DECL(busyTx);

    /// @brief Initial state
    Q_STATE_DECL(initial);
    /// @brief Root state
    Q_STATE_DECL(root);
    /// @brief Slave mode
    Q_STATE_DECL(Slave);
    /// @brief Slave idle
    Q_STATE_DECL(SlaveIdle);
    /// @brief Slave rx busy mode
    Q_STATE_DECL(SlaveRxBusy);
    /// @brief Slave tx busy mode
    Q_STATE_DECL(SlaveTxBusy);
    /// @brief Master mode
    Q_STATE_DECL(Master);
    /// @brief Master rx busy mode
    Q_STATE_DECL(MasterRxBusy);
    /// @brief Master tx busy mode
    Q_STATE_DECL(MasterTxBusy);
};
}  // namespace mission

#endif
