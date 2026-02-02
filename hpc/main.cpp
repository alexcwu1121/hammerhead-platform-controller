#include "bsp.hpp"
#include "cli_ao.hpp"
#include "imu_ao.hpp"
#include "mission_ao.hpp"
#include "motor_control_ao.hpp"
#include "param_ao.hpp"

int main(void)
{
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    // MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();

    /// TODO: Why do I need to do this to unstick the i2c peripheral
    HAL_I2C_DeInit(&hi2c2);
    HAL_I2C_Init(&hi2c2);

    // Init event pools
    static QF_MPOOL_EL(QP::QEvt) smlPoolSto[50];  // small (bare signals)
    QP::QF::poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));
    static uint8_t mdPoolSto[50][32];  // medium (average data packets)
    QP::QF::poolInit(mdPoolSto, sizeof(mdPoolSto), sizeof(mdPoolSto[0]));
    static uint8_t lgPoolSto[7][512];  // large (logs or text)
    QP::QF::poolInit(lgPoolSto, sizeof(lgPoolSto), sizeof(lgPoolSto[0]));

    // Init publish-subscribe signals
    static QP::QSubscrList subscrSto[bsp::PublicSignals::MAX_PUB_SIG];
    QP::QActive::psInit(subscrSto, Q_DIM(subscrSto));

    // Init QF scheduler
    QP::QF::init();

    // Start AOs
    // ParamAO is highest priority because it doesn't do much + must start before everything else
    param::ParamAO::Inst().Start(1U, bsp::SubsystemID::PARAMETER_SUBSYSTEM);
    cli::CLIAO::Inst().Start(2U, bsp::SubsystemID::CLI_SUBSYSTEM);
    mission::MissionAO::Inst().Start(3U, bsp::SubsystemID::MISSION_SUBSYSTEM);
    mc::MotorControlAO::MC1Inst().Start(4U, bsp::SubsystemID::MC1_SUBSYSTEM);
    mc::MotorControlAO::MC2Inst().Start(5U, bsp::SubsystemID::MC2_SUBSYSTEM);
    imu::IMUAO::Inst().Start(6U, bsp::SubsystemID::IMU_SUBSYSTEM);

    // Start QF scheduler
    return QP::QF::run();

    return 0;
}
