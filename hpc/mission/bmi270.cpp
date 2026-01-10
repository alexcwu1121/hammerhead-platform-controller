#include "bmi270.hpp"

namespace imu
{
// IMU register addresses
constexpr uint8_t FIFO_DATA = 0x26;
constexpr uint8_t AUX_X_LSB = 0x04;
constexpr uint8_t AUX_X_MSB = 0x05;
constexpr uint8_t AUX_Y_LSB = 0x06;
constexpr uint8_t AUX_Y_MSB = 0x07;
constexpr uint8_t AUX_Z_LSB = 0x08;
constexpr uint8_t AUX_Z_MSB = 0x09;
constexpr uint8_t AUX_R_LSB = 0x0A;
constexpr uint8_t AUX_R_MSB = 0x0B;
constexpr uint8_t ACC_X_LSB = 0x0C;
constexpr uint8_t ACC_X_MSB = 0x0D;
constexpr uint8_t ACC_Y_LSB = 0x0E;
constexpr uint8_t ACC_Y_MSB = 0x0F;
constexpr uint8_t ACC_Z_LSB = 0x10;
constexpr uint8_t ACC_Z_MSB = 0x11;
constexpr uint8_t GYR_X_LSB = 0x12;
constexpr uint8_t GYR_X_MSB = 0x13;
constexpr uint8_t GYR_Y_LSB = 0x14;
constexpr uint8_t GYR_Y_MSB = 0x15;
constexpr uint8_t GYR_Z_LSB = 0x16;
constexpr uint8_t GYR_Z_MSB = 0x17;

// Sequences
// SPI Startup
//  Pull CSB high
// Write
//  [0 (W) | 7-bit register addr][Data byte 1][...]
// Read
//  [1 (R) | 7-bit register addr][DUMMY][Read byte]

BMI270::BMI270(GPIO_TypeDef* csPort, uint16_t csPinNum, SPI_HandleTypeDef* spiDevice)
    : _csPort(csPort), _csPinNum(csPinNum), _spiDevice(spiDevice)
{
}

void BMI270::Select() const
{
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
}

void BMI270::Deselect() const
{
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);
}

IMUData BMI270::ReadIMUData()
{
    /*
    // Read IMU data from FIFO
    uint8_t cmd = (0x01 << 7) | (GYR_X_LSB & 0x7F);

    uint8_t buf[3U];

    Select();
    //HAL_SPI_Transmit(_spiDevice, &cmd, 1U, HAL_MAX_DELAY);
    //HAL_SPI_Receive(_spiDevice, buf, 3U, HAL_MAX_DELAY);

    HAL_SPI_TransmitReceive(_spiDevice, &cmd, buf, 3);

    Deselect();
    */
    /*
    uint16_t len = 2U;

    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    tx[0] = 0x00 | 0x80;   // set READ bit
    memset(&tx[1], 0x00, len);  // dummy bytes

    HAL_GPIO_WritePin(_csPort,
                      _csPinNum,
                      GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(_spiDevice, tx, rx, len + 1, 10);

    HAL_GPIO_WritePin(_csPort,
                      _csPinNum,
                      GPIO_PIN_SET);

    //memcpy(data, &rx[1], len);  // discard address phase
    */

    /*
    uint8_t tx[2] = { 0x80, 0x00 };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(_spiDevice, tx, rx, 2, 10);
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);
    */

    // uint8_t tx = 0x80;
    uint8_t tx = 0b10000000;
    // uint8_t tx = 0b10101010;
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_spiDevice, &tx, 1U, HAL_MAX_DELAY);
    HAL_SPI_Receive(_spiDevice, rx, 1U, HAL_MAX_DELAY);
    HAL_SPI_Receive(_spiDevice, rx + 1, 1U, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);

    return IMUData{0};
}
}  // namespace imu
