#ifndef WIRE_COMPAT_H
#define WIRE_COMPAT_H

#include "stm32f1xx_hal.h"

#include <stddef.h>
#include <stdint.h>

class TwoWire {
public:
    explicit TwoWire(I2C_HandleTypeDef *handle = nullptr);

    void attach(I2C_HandleTypeDef *handle);
    void begin();
    void end();
    void beginTransmission(uint8_t address);
    size_t write(uint8_t value);
    size_t write(const uint8_t *data, size_t length);
    uint8_t endTransmission(bool send_stop = true);
    uint8_t requestFrom(uint8_t address, uint32_t quantity);
    int available() const;
    int read();

private:
    static constexpr size_t BUFFER_SIZE = 260U;
    static constexpr uint32_t I2C_TIMEOUT_MS = 100U;

    I2C_HandleTypeDef *handle_;
    uint8_t address_;
    uint8_t tx_buffer_[BUFFER_SIZE];
    uint8_t rx_buffer_[BUFFER_SIZE];
    size_t tx_length_;
    size_t rx_length_;
    size_t rx_index_;
    uint16_t pending_register_;
    bool pending_read_;

    void recover();
};

#endif
