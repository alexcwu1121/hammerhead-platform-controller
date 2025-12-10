#include "EEPROM_25LC256.hpp"

// EEPROM SPI opcodes
#define CMD_WREN 0x06
#define CMD_WRDI 0x04
#define CMD_RDSR 0x05
#define CMD_WRSR 0x01
#define CMD_READ 0x03
#define CMD_WRITE 0x02

// Transaction sizes
#define OPCODE_SIZE 1U
#define ADDR_SIZE 2U
#define PAGE_SIZE 64U

namespace eeprom
{
EEPROM25LC256::EEPROM25LC256(GPIO_TypeDef* csPort, uint16_t csPinNum, GPIO_TypeDef* wpPort,
                             uint16_t wpPinNum, SPI_HandleTypeDef* spiDevice)
    : _csPort(csPort),
      _csPinNum(csPinNum),
      _wpPort(wpPort),
      _wpPinNum(wpPinNum),
      _spiDevice(spiDevice)
{
}

void EEPROM25LC256::EnableWriteProtect() const
{
    HAL_GPIO_WritePin(_wpPort, _wpPinNum, GPIO_PIN_RESET);
}

void EEPROM25LC256::DisableWriteProtect() const
{
    HAL_GPIO_WritePin(_wpPort, _wpPinNum, GPIO_PIN_SET);
}

void EEPROM25LC256::Write(uint8_t* inBuf, uint16_t writeSize, uint16_t address)
{
    while (writeSize > 0)
    {
        // Compute chunk size
        uint16_t page_offset    = address % PAGE_SIZE;
        uint16_t page_remaining = PAGE_SIZE - page_offset;
        uint16_t chunk          = (writeSize < page_remaining) ? writeSize : page_remaining;

        // Allocate tx buffer
        uint8_t  txbuf[OPCODE_SIZE + ADDR_SIZE + PAGE_SIZE];
        uint16_t index = 0;

        // Write enable
        uint8_t wren = CMD_WREN;
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
        HAL_SPI_Transmit(_spiDevice, &wren, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);

        // Pack tx buffer
        txbuf[index++] = CMD_WRITE;
        txbuf[index++] = (address >> 8) & 0xFF;
        txbuf[index++] = (address & 0xFF);
        memcpy(&txbuf[index], inBuf, chunk);
        index += chunk;

        // Transmit chunk and wait for transaction to end
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
        HAL_SPI_Transmit(_spiDevice, txbuf, index, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);
        // Wait for write to end
        WaitForWriteEnd();

        // Advance pointers
        address += chunk;
        inBuf += chunk;
        writeSize -= chunk;
    }
}

void EEPROM25LC256::Read(uint8_t* outBuf, uint16_t readSize, uint16_t address)
{
    uint8_t cmd[3];
    cmd[0] = CMD_READ;
    cmd[1] = (address >> 8) & 0xFF;
    cmd[2] = address & 0xFF;

    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_spiDevice, cmd, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(_spiDevice, outBuf, readSize, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);
}

void EEPROM25LC256::WaitForWriteEnd(void)
{
    static uint8_t cmd = CMD_RDSR;
    uint8_t        status;

    // TODO: timeout
    do
    {
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_RESET);
        HAL_SPI_Transmit(_spiDevice, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(_spiDevice, &status, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(_csPort, _csPinNum, GPIO_PIN_SET);
    } while (status & 0x01);
}
}  // namespace eeprom
