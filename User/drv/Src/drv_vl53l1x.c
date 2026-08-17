/**
 * @file drv_vl53l1x.c
 * @brief VL53L1X ULD configuration for the gripper proximity gate.
 */

#include "drv_vl53l1x.h"

#include "VL53L1X_api.h"
#include "bsp_i2c.h"

#define VL53L1X_REG_GPIO_STATUS   (0x0031U)
#define VL53L1X_REG_RESULT_START  (0x0089U)
#define VL53L1X_RESULT_LENGTH     (17U)

static AraStatus_t map_uld_status(VL53L1X_ERROR status)
{
    return (status == 0) ? ARA_OK : ARA_ERR_IO;
}

static uint8_t map_range_status(uint8_t raw_status)
{
    switch (raw_status & 0x1FU) {
    case 9U:  return 0U;
    case 6U:  return 1U;
    case 4U:  return 2U;
    case 8U:  return 3U;
    case 5U:  return 4U;
    case 3U:  return 5U;
    case 19U: return 6U;
    case 7U:  return 7U;
    case 12U: return 9U;
    case 18U: return 10U;
    case 22U: return 11U;
    case 23U: return 12U;
    case 13U: return 13U;
    default:  return 255U;
    }
}

AraStatus_t DrvVL53L1X_Init(DrvVL53L1X_Context_t *ctx,
                            uint8_t address_7bit,
                            uint32_t timeout_ms)
{
    uint8_t booted = 0U;
    uint32_t start_ms;
    VL53L1X_ERROR uld_status;

    if ((ctx == NULL) || (address_7bit > 0x7FU)) {
        return ARA_ERR_PARAM;
    }

    ctx->dev.address_7bit = address_7bit;
    ctx->dev.timeout_ms = (timeout_ms == 0U) ?
                          DRV_VL53L1X_DEFAULT_TIMEOUT_MS : timeout_ms;
    ctx->sensor_id = 0U;
    ctx->interrupt_polarity = 0U;
    ctx->initialized = false;
    ctx->ranging = false;

    AraStatus_t status = BSP_I2C_IsReady(address_7bit, ctx->dev.timeout_ms);
    if (status != ARA_OK) {
        return status;
    }

    start_ms = BSP_I2C_GetTickMs();
    while (booted == 0U) {
        uld_status = VL53L1X_BootState(ctx->dev, &booted);
        if (uld_status != 0) {
            return map_uld_status(uld_status);
        }
        if ((uint32_t)(BSP_I2C_GetTickMs() - start_ms) >= ctx->dev.timeout_ms) {
            return ARA_TIMEOUT;
        }
        BSP_I2C_DelayMs(2U);
    }

    uld_status = VL53L1X_GetSensorId(ctx->dev, &ctx->sensor_id);
    if (uld_status != 0) {
        return map_uld_status(uld_status);
    }
    if (ctx->sensor_id != DRV_VL53L1X_EXPECTED_SENSOR_ID) {
        return ARA_ERR_DISCONNECTED;
    }

    uld_status = VL53L1X_SensorInit(ctx->dev);
    if (uld_status == 0) {
        uld_status = VL53L1X_SetDistanceMode(ctx->dev, 1U);
    }
    if (uld_status == 0) {
        uld_status = VL53L1X_SetTimingBudgetInMs(
            ctx->dev, DRV_VL53L1X_TIMING_BUDGET_MS);
    }
    if (uld_status == 0) {
        uld_status = VL53L1X_SetInterMeasurementInMs(
            ctx->dev, DRV_VL53L1X_INTERMEASUREMENT_MS);
    }
    if (uld_status == 0) {
        uld_status = VL53L1X_SetROI(
            ctx->dev, DRV_VL53L1X_ROI_WIDTH, DRV_VL53L1X_ROI_HEIGHT);
    }
    if (uld_status == 0) {
        uld_status = VL53L1X_GetInterruptPolarity(
            ctx->dev, &ctx->interrupt_polarity);
    }
    if (uld_status != 0) {
        return map_uld_status(uld_status);
    }

    ctx->initialized = true;
    return ARA_OK;
}

AraStatus_t DrvVL53L1X_StartContinuous(DrvVL53L1X_Context_t *ctx)
{
    if ((ctx == NULL) || !ctx->initialized) {
        return ARA_ERR_PARAM;
    }
    VL53L1X_ERROR status = VL53L1X_StartRanging(ctx->dev);
    if (status == 0) {
        ctx->ranging = true;
    }
    return map_uld_status(status);
}

AraStatus_t DrvVL53L1X_Stop(DrvVL53L1X_Context_t *ctx)
{
    if ((ctx == NULL) || !ctx->initialized) {
        return ARA_ERR_PARAM;
    }
    VL53L1X_ERROR status = VL53L1X_StopRanging(ctx->dev);
    if (status == 0) {
        ctx->ranging = false;
    }
    return map_uld_status(status);
}

AraStatus_t DrvVL53L1X_ReadIfReady(DrvVL53L1X_Context_t *ctx,
                                   DrvVL53L1X_Result_t *result)
{
    uint8_t ready = 0U;
    uint8_t result_block[VL53L1X_RESULT_LENGTH];
    VL53L1X_ERROR status;

    if ((ctx == NULL) || (result == NULL) || !ctx->initialized || !ctx->ranging) {
        return ARA_ERR_PARAM;
    }

    status = VL53L1_RdByte(&ctx->dev, VL53L1X_REG_GPIO_STATUS, &ready);
    if (status != 0) {
        return map_uld_status(status);
    }
    if ((ready & 0x01U) != ctx->interrupt_polarity) {
        return ARA_BUSY;
    }

    status = VL53L1_ReadMulti(&ctx->dev,
                              VL53L1X_REG_RESULT_START,
                              result_block,
                              sizeof(result_block));
    if (status == 0) {
        status = VL53L1X_ClearInterrupt(ctx->dev);
    }
    if (status != 0) {
        return map_uld_status(status);
    }

    result->range_status = map_range_status(result_block[0]);
    result->ambient_rate_kcps =
        (uint16_t)((((uint16_t)result_block[7] << 8) | result_block[8]) * 8U);
    result->distance_mm =
        ((uint16_t)result_block[13] << 8) | result_block[14];
    result->signal_rate_kcps =
        (uint16_t)((((uint16_t)result_block[15] << 8) | result_block[16]) * 8U);
    result->timestamp_ms = BSP_I2C_GetTickMs();
    return ARA_OK;
}

bool DrvVL53L1X_IsRangeValid(const DrvVL53L1X_Result_t *result)
{
    return (result != NULL) &&
           (result->range_status == DRV_VL53L1X_RANGE_STATUS_VALID);
}
