/**
 * @file bsp_i2c.c
 * @brief I2C wrapper with bus recovery and DMA callback support.
 */

#include "bsp_i2c.h"
#include "stm32f1xx_hal.h"

/* --- Hardware Resources --- */
extern I2C_HandleTypeDef hi2c1;

static I2C_HandleTypeDef *i2c_handles[BSP_I2C_NUM] = {
    [BSP_I2C_MOTION] = &hi2c1
};

static AraCallback_t i2c_rx_cplt_cbs[BSP_I2C_NUM] = { NULL };
static volatile uint32_t last_i2c_error = HAL_I2C_ERROR_NONE;

/* --- Helper: Status Conversion --- */
static AraStatus_t HAL_To_ARA_Status(HAL_StatusTypeDef status)
{
    switch (status) {
        case HAL_OK:      return ARA_OK;
        case HAL_BUSY:    return ARA_BUSY;
        case HAL_TIMEOUT: return ARA_TIMEOUT;
        default:          return ARA_ERR_IO;
    }
}

/* --- Helper: Bus Recovery for STM32F1 --- */
static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* I2C1 is remapped to PB8/PB9 in CubeMX. */
    const uint16_t scl_pin = GPIO_PIN_8;
    const uint16_t sda_pin = GPIO_PIN_9;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();

    GPIO_InitStruct.Pin = scl_pin | sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_SET);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, sda_pin, GPIO_PIN_SET);

    hi2c->Instance->CR1 |= I2C_CR1_SWRST;
    hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_Init(hi2c);
}

/* --- API Implementation --- */
void BSP_I2C_Init(void)
{
    uint32_t hi2c_error = HAL_I2C_GetError(&hi2c1);
    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) ||
        hi2c_error != HAL_I2C_ERROR_NONE ||
        last_i2c_error != HAL_I2C_ERROR_NONE ||
        state != HAL_I2C_STATE_READY) {
        HAL_I2C_DeInit(&hi2c1);
        I2C_ClearBusyFlagErratum(&hi2c1);
    }

    last_i2c_error = HAL_I2C_ERROR_NONE;
}

uint32_t BSP_I2C_GetLastError(void)
{
    return last_i2c_error;
}

void BSP_I2C_ClearLastError(void)
{
    last_i2c_error = HAL_I2C_ERROR_NONE;
}

AraStatus_t BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr,
                            uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    uint16_t target_addr = dev_addr << 1;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        i2c_handles[dev],
        target_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        p_data,
        len,
        10
    );

    if (status != HAL_OK) {
        last_i2c_error = HAL_I2C_GetError(i2c_handles[dev]);
    }

    return HAL_To_ARA_Status(status);
}

AraStatus_t BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr,
                             uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    uint16_t target_addr = dev_addr << 1;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        i2c_handles[dev],
        target_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        p_data,
        len,
        10
    );

    if (status != HAL_OK) {
        last_i2c_error = HAL_I2C_GetError(i2c_handles[dev]);
    }

    return HAL_To_ARA_Status(status);
}

AraStatus_t BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    uint16_t target_addr = dev_addr << 1;
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c_handles[dev], target_addr, 3, 10);

    if (status != HAL_OK) {
        last_i2c_error = HAL_I2C_GetError(i2c_handles[dev]);
    }

    return (status == HAL_OK) ? ARA_OK : ARA_ERR_NACK;
}

/* === DMA Support === */
void BSP_I2C_SetRxCpltCallback(BspI2c_Dev_t dev, AraCallback_t cb)
{
    if (dev < BSP_I2C_NUM) {
        i2c_rx_cplt_cbs[dev] = cb;
    }
}

AraStatus_t BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr,
                                uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    uint16_t target_addr = dev_addr << 1;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
        i2c_handles[dev],
        target_addr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        p_data,
        len
    );

    if (status == HAL_ERROR) {
        last_i2c_error = HAL_I2C_GetError(i2c_handles[dev]);
        return ARA_ERR_DMA;
    }

    if (status != HAL_OK) {
        last_i2c_error = HAL_I2C_GetError(i2c_handles[dev]);
    }

    return HAL_To_ARA_Status(status);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    last_i2c_error = HAL_I2C_ERROR_NONE;

    for (int i = 0; i < BSP_I2C_NUM; i++) {
        if (i2c_handles[i] == hi2c) {
            if (i2c_rx_cplt_cbs[i] != NULL) {
                i2c_rx_cplt_cbs[i]();
            }
            return;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        last_i2c_error = HAL_I2C_GetError(hi2c);
    }
}
