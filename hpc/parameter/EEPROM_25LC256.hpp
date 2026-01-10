#ifndef EEPROM_25LC256_HPP_
#define EEPROM_25LC256_HPP_

#include "bsp.hpp"

namespace eeprom
{
class EEPROM25LC256
{
   public:
    /// @brief EEPROM25LC256 Constructor
    /// @param csPort Chip-Select pin port
    /// @param csPinNum Chip-Select pin number
    /// @param wpPort Write-Protect pin port
    /// @param wpPinNum Write-Protect pin number
    /// @param spiDevice SPI interface handle
    EEPROM25LC256(GPIO_TypeDef* csPort, uint16_t csPinNum, GPIO_TypeDef* wpPort, uint16_t wpPinNum,
                  SPI_HandleTypeDef* spiDevice);

    /// @brief Enable write protect
    void EnableWriteProtect() const;

    /// @brief Disable write protect
    void DisableWriteProtect() const;

    /// @brief Set chip select high
    void Select() const;

    /// @brief Set chip select low
    void Deselect() const;

    /// @brief Write data to EEPROM synchronously
    /// @param inBuf
    /// @param writeSize
    /// @param address
    /// @return
    void Write(uint8_t* inBuf, uint16_t writeSize, uint16_t address);

    /// @brief Read data from EEPROM synchronously
    /// @param outBuf
    /// @param readSize
    /// @param address
    /// @return
    void Read(uint8_t* outBuf, uint16_t readSize, uint16_t address);

   private:
    /// @brief Chip-Select pin port
    GPIO_TypeDef* _csPort;
    /// @brief Chip-Select pin num
    uint16_t _csPinNum;
    /// @brief Write-Protect pin port
    GPIO_TypeDef* _wpPort;
    /// @brief Write-Protect pin num
    uint16_t _wpPinNum;
    /// @brief SPI device handle
    SPI_HandleTypeDef* _spiDevice;

    /// @brief Block until write transaction succeeds
    void WaitForWriteEnd();
};
}  // namespace eeprom

#endif
