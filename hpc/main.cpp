#include "bsp.hpp"
#include "cli_ao.hpp"
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
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();

    // Init event pools
    static QF_MPOOL_EL(QP::QEvt) smlPoolSto[50];  // small (bare signals)
    QP::QF::poolInit(smlPoolSto, sizeof(smlPoolSto), sizeof(smlPoolSto[0]));
    static uint8_t mdPoolSto[50][16];  // medium (average data packets)
    QP::QF::poolInit(mdPoolSto, sizeof(mdPoolSto), sizeof(mdPoolSto[0]));
    static uint8_t lgPoolSto[20][256];  // large (logs or text)
    QP::QF::poolInit(lgPoolSto, sizeof(lgPoolSto), sizeof(lgPoolSto[0]));

    // Init publish-subscribe signals
    static QP::QSubscrList subscrSto[bsp::PublicSignals::MAX_PUB_SIG];
    QP::QActive::psInit(subscrSto, Q_DIM(subscrSto));

    // Init QF scheduler
    QP::QF::init();

    // Start AOs
    mc::MotorControlAO::Inst().Start(1U);
    param::ParamAO::Inst().Start(2U);
    cli::CLIAO::Inst().Start(3U);

    // Start QF scheduler
    return QP::QF::run();
}
