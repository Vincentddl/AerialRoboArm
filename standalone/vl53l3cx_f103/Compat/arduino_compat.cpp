#include "Arduino.h"
#include "Wire.h"

#include "main.h"

#include <algorithm>
#include <cstring>

extern "C" void pinMode(int pin, int mode)
{
    if (pin != PIN_VL53_XSHUT) {
        return;
    }

    GPIO_InitTypeDef config{};
    config.Pin = VL53_XSHUT_Pin;
    config.Speed = GPIO_SPEED_FREQ_LOW;
    config.Mode = mode == OUTPUT ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VL53_XSHUT_GPIO_Port, &config);
}

extern "C" void digitalWrite(int pin, int value)
{
    if (pin == PIN_VL53_XSHUT) {
        HAL_GPIO_WritePin(VL53_XSHUT_GPIO_Port,
                          VL53_XSHUT_Pin,
                          value == HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

extern "C" void delay(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}

extern "C" void delayMicroseconds(uint32_t microseconds)
{
    if (microseconds == 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

extern "C" uint32_t millis(void)
{
    return HAL_GetTick();
}

TwoWire::TwoWire(I2C_HandleTypeDef *handle)
    : handle_(handle),
      address_(0U),
      tx_length_(0U),
      rx_length_(0U),
      rx_index_(0U),
      pending_register_(0U),
      pending_read_(false)
{
}

void TwoWire::attach(I2C_HandleTypeDef *handle)
{
    handle_ = handle;
}

void TwoWire::begin()
{
    if ((handle_ != nullptr) && (handle_->State == HAL_I2C_STATE_RESET)) {
        (void)HAL_I2C_Init(handle_);
    }
}

void TwoWire::end()
{
    if (handle_ != nullptr) {
        (void)HAL_I2C_DeInit(handle_);
    }
    tx_length_ = 0U;
    rx_length_ = 0U;
    rx_index_ = 0U;
    pending_read_ = false;
}

void TwoWire::beginTransmission(uint8_t address)
{
    address_ = address;
    tx_length_ = 0U;
}

size_t TwoWire::write(uint8_t value)
{
    if (tx_length_ >= BUFFER_SIZE) {
        return 0U;
    }
    tx_buffer_[tx_length_++] = value;
    return 1U;
}

size_t TwoWire::write(const uint8_t *data, size_t length)
{
    if (data == nullptr) {
        return 0U;
    }
    size_t writable = std::min(length, BUFFER_SIZE - tx_length_);
    std::memcpy(&tx_buffer_[tx_length_], data, writable);
    tx_length_ += writable;
    return writable;
}

uint8_t TwoWire::endTransmission(bool send_stop)
{
    if ((handle_ == nullptr) || (tx_length_ < 2U)) {
        return 4U;
    }

    uint16_t register_address = static_cast<uint16_t>(tx_buffer_[0]) << 8U;
    register_address |= tx_buffer_[1];

    if (!send_stop && (tx_length_ == 2U)) {
        pending_register_ = register_address;
        pending_read_ = true;
        return 0U;
    }

    if (tx_length_ == 2U) {
        pending_register_ = register_address;
        pending_read_ = true;
        return 0U;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        handle_,
        static_cast<uint16_t>(address_) << 1U,
        register_address,
        I2C_MEMADD_SIZE_16BIT,
        &tx_buffer_[2],
        static_cast<uint16_t>(tx_length_ - 2U),
        I2C_TIMEOUT_MS);
    pending_read_ = false;
    if (status != HAL_OK) {
        recover();
    }
    return status == HAL_OK ? 0U : 4U;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint32_t quantity)
{
    if ((handle_ == nullptr) || !pending_read_ || (quantity == 0U)
        || (quantity > BUFFER_SIZE)) {
        rx_length_ = 0U;
        rx_index_ = 0U;
        return 0U;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        handle_,
        static_cast<uint16_t>(address) << 1U,
        pending_register_,
        I2C_MEMADD_SIZE_16BIT,
        rx_buffer_,
        static_cast<uint16_t>(quantity),
        I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        rx_length_ = 0U;
        rx_index_ = 0U;
        pending_read_ = false;
        recover();
        return 0U;
    }

    rx_length_ = quantity;
    rx_index_ = 0U;
    pending_register_ = static_cast<uint16_t>(pending_register_ + quantity);
    return static_cast<uint8_t>(quantity);
}

int TwoWire::available() const
{
    return static_cast<int>(rx_length_ - rx_index_);
}

int TwoWire::read()
{
    if (rx_index_ >= rx_length_) {
        return -1;
    }
    return rx_buffer_[rx_index_++];
}

void TwoWire::recover()
{
    if (handle_ != nullptr) {
        (void)HAL_I2C_DeInit(handle_);
        (void)HAL_I2C_Init(handle_);
    }
    tx_length_ = 0U;
    rx_length_ = 0U;
    rx_index_ = 0U;
    pending_read_ = false;
}
