#ifndef PC_CAN_DEFS_HPP_
#define PC_CAN_DEFS_HPP_

#include <cstdint>

/// @brief Published CAN ID lowest index
static constexpr uint16_t PubCANIDIdx = 0x100;

/// @brief Published CAN message IDs
enum PubCANID : uint16_t
{
    PUB_IMU_DATA_ACC_XY = PubCANIDIdx,
    PUB_IMU_DATA_ACC_Z_GYR_X,
    PUB_IMU_DATA_GYR_Y_GYR_Z,
    PUB_BATT_SOC,
    PUB_HEARTBEAT,
    PUB_FAULT_INDEX = 0x120,  // starting index for faults
    MAX_PUB_ID
};

/// @brief Susbcribed CAN ID lowest index
static constexpr uint16_t SubCANIDIdx = 0x200;

/// @brief Subscribed CAN message IDs
enum SubCANID : uint16_t
{
    SUB_WRITE_MC1_MODE = SubCANIDIdx,
    SUB_WRITE_MC2_MODE,
    SUB_WRITE_MC1_RATE,
    SUB_WRITE_MC2_RATE,
    SUB_WRITE_MC1_DUTY,
    SUB_WRITE_MC2_DUTY,
    SUB_WRITE_MC1_DIR,
    SUB_WRITE_MC2_DIR,
    SUB_WRITE_MC1_RESET,
    SUB_WRITE_MC2_RESET,
    SUB_WRITE_IMU_RESET,
    SUB_WRITE_IMU_COMP,
    SUB_POKE_WATCHDOG,
    SUB_ENABLE_WATCHDOG,
    SUB_DISABLE_WATCHDOG,
    MAX_SUB_ID
};

#endif
