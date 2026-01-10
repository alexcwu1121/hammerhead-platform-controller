#ifndef BMI270_HPP_
#define BMI270_HPP_

#include "bsp.hpp"

namespace imu
{
/// @brief 6DOF IMU data
struct IMUData
{
    float accel_x;
    float accel_y;
    float accel_z;
    float omega_x;
    float omega_y;
    float omega_z;
};

class BMI270
{
   public:
    /// @brief BMI270 Constructor
    /// @param csPort Chip-Select pin port
    /// @param csPinNum Chip-Select pin number
    /// @param spiDevice SPI interface handle
    BMI270(GPIO_TypeDef* csPort, uint16_t csPinNum, SPI_HandleTypeDef* spiDevice);

    /// @brief Set chip select high
    void Select() const;

    /// @brief Set chip select low
    void Deselect() const;

    /// @brief Read 6DOF IMU data
    /// @return
    IMUData ReadIMUData();

    /// TODO: Enable FIFO in FIFO mode headerless for accelerometer and gyro
    /// TODO: Drain FIFO on read
    /// TODO: Set full scale range
    /// TODO: Configure for low power, normal power, performance mode

   private:
    /// @brief Chip-Select pin port
    GPIO_TypeDef* _csPort;
    /// @brief Chip-Select pin num
    uint16_t _csPinNum;
    /// @brief SPI device handle
    SPI_HandleTypeDef* _spiDevice;

    /// @brief Read a register
    uint8_t ReadRegister();
};
}  // namespace imu

#endif
