/**
 * @file drv_vl53l0x.c
 * @brief Basic VL53L0X single-shot ranging implementation.
 */

#include "drv_vl53l0x.h"

#include "bsp_i2c.h"

#define VL53L0X_REG_SYSRANGE_START             (0x00U)
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR     (0x0BU)
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS    (0x13U)
#define VL53L0X_REG_RESULT_RANGE_STATUS        (0x14U)
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID    (0xC0U)
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID (0xC2U)

#define VL53L0X_RESULT_BLOCK_LENGTH            (12U)
#define VL53L0X_INTERRUPT_NEW_SAMPLE_MASK       (0x07U)
#define VL53L0X_RESULT_READY_MASK               (0x01U)

static AraStatus_t write_u8(const DrvVL53L0X_Context_t *ctx,
                            uint8_t reg,
                            uint8_t value)
{
    return BSP_I2C_WriteReg(ctx->address_7bit,
                            reg,
                            &value,
                            1U,
                            ctx->timeout_ms);
}

static AraStatus_t read_u8(const DrvVL53L0X_Context_t *ctx,
                           uint8_t reg,
                           uint8_t *value)
{
    return BSP_I2C_ReadReg(ctx->address_7bit,
                           reg,
                           value,
                           1U,
                           ctx->timeout_ms);
}

static bool elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(BSP_I2C_GetTickMs() - start_ms) >= timeout_ms;
}

static AraStatus_t wait_for_result(DrvVL53L0X_Context_t *ctx)
{
    const uint32_t start_ms = BSP_I2C_GetTickMs();

    for (;;) {
        uint8_t interrupt_status = 0U;
        AraStatus_t status = read_u8(ctx,
                                     VL53L0X_REG_RESULT_INTERRUPT_STATUS,
                                     &interrupt_status);
        if (status != ARA_OK) {
            return status;
        }
        if ((interrupt_status & VL53L0X_INTERRUPT_NEW_SAMPLE_MASK) != 0U) {
            return ARA_OK;
        }

        /* Some early power-on examples expose readiness through bit 0 of the
         * result status register instead of the configured GPIO interrupt. */
        uint8_t range_status = 0U;
        status = read_u8(ctx, VL53L0X_REG_RESULT_RANGE_STATUS, &range_status);
        if (status != ARA_OK) {
            return status;
        }
        if ((range_status & VL53L0X_RESULT_READY_MASK) != 0U) {
            return ARA_OK;
        }
        if (elapsed(start_ms, ctx->timeout_ms)) {
            return ARA_TIMEOUT;
        }
        BSP_I2C_DelayMs(1U);
    }
}

AraStatus_t DrvVL53L0X_Init(DrvVL53L0X_Context_t *ctx,
                            uint8_t address_7bit,
                            uint32_t timeout_ms)
{
    if ((ctx == NULL) || (address_7bit > 0x7FU)) {
        return ARA_ERR_PARAM;
    }

    ctx->address_7bit = address_7bit;
    ctx->model_id = 0U;
    ctx->revision_id = 0U;
    ctx->timeout_ms = (timeout_ms == 0U) ? DRV_VL53L0X_DEFAULT_TIMEOUT_MS : timeout_ms;
    ctx->initialized = false;

    AraStatus_t status = BSP_I2C_IsReady(ctx->address_7bit, ctx->timeout_ms);
    if (status != ARA_OK) {
        return status;
    }
    status = read_u8(ctx, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &ctx->model_id);
    if (status != ARA_OK) {
        return status;
    }
    status = read_u8(ctx, VL53L0X_REG_IDENTIFICATION_REVISION_ID, &ctx->revision_id);
    if (status != ARA_OK) {
        return status;
    }
    if (ctx->model_id != DRV_VL53L0X_EXPECTED_MODEL_ID) {
        return ARA_ERR_DISCONNECTED;
    }

    ctx->initialized = true;
    return ARA_OK;
}

AraStatus_t DrvVL53L0X_StartSingle(DrvVL53L0X_Context_t *ctx)
{
    if ((ctx == NULL) || !ctx->initialized) {
        return ARA_ERR_PARAM;
    }

    AraStatus_t status = write_u8(ctx, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    if (status != ARA_OK) {
        return status;
    }
    return write_u8(ctx, VL53L0X_REG_SYSRANGE_START, 0x01U);
}

AraStatus_t DrvVL53L0X_ReadResult(DrvVL53L0X_Context_t *ctx,
                                  DrvVL53L0X_Result_t *result)
{
    if ((ctx == NULL) || (result == NULL) || !ctx->initialized) {
        return ARA_ERR_PARAM;
    }

    AraStatus_t status = wait_for_result(ctx);
    if (status != ARA_OK) {
        return status;
    }

    uint8_t block[VL53L0X_RESULT_BLOCK_LENGTH];
    status = BSP_I2C_ReadReg(ctx->address_7bit,
                             VL53L0X_REG_RESULT_RANGE_STATUS,
                             block,
                             sizeof(block),
                             ctx->timeout_ms);
    if (status != ARA_OK) {
        return status;
    }

    result->range_status = (uint8_t)((block[0] & 0x78U) >> 3);
    result->ambient_count = (uint16_t)(((uint16_t)block[6] << 8) | block[7]);
    result->signal_count = (uint16_t)(((uint16_t)block[8] << 8) | block[9]);
    result->distance_mm = (uint16_t)(((uint16_t)block[10] << 8) | block[11]);
    result->timestamp_ms = BSP_I2C_GetTickMs();

    status = write_u8(ctx, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    return status;
}

AraStatus_t DrvVL53L0X_ReadSingle(DrvVL53L0X_Context_t *ctx,
                                  DrvVL53L0X_Result_t *result)
{
    AraStatus_t status = DrvVL53L0X_StartSingle(ctx);
    if (status != ARA_OK) {
        return status;
    }
    return DrvVL53L0X_ReadResult(ctx, result);
}

bool DrvVL53L0X_IsRangeValid(const DrvVL53L0X_Result_t *result)
{
    if (result == NULL) {
        return false;
    }
    return (result->range_status == DRV_VL53L0X_RANGE_STATUS_VALID) ||
           (result->range_status == DRV_VL53L0X_RANGE_STATUS_MIN_RANGE_CLIPPED);
}
