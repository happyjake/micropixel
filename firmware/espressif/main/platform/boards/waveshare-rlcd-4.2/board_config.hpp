#pragma once

#include "driver/gpio.h"

// Waveshare ESP32-S3-RLCD-4.2 (ESP32-S3-WROOM-1-N16R8) wiring, from the
// vendor pin table and its ESP-IDF examples.
namespace micropixel::platform::waveshare_rlcd_4_2::board {

inline constexpr int kPanelWidth = 400;
inline constexpr int kPanelHeight = 300;

inline constexpr gpio_num_t kLcdClock = GPIO_NUM_11;
inline constexpr gpio_num_t kLcdMosi = GPIO_NUM_12;
inline constexpr gpio_num_t kLcdDataCommand = GPIO_NUM_5;
inline constexpr gpio_num_t kLcdChipSelect = GPIO_NUM_40;
inline constexpr gpio_num_t kLcdReset = GPIO_NUM_41;

// Shared by the ES8311 codec, ES7210 ADC, SHTC3 and PCF85063.
inline constexpr gpio_num_t kI2cSda = GPIO_NUM_13;
inline constexpr gpio_num_t kI2cScl = GPIO_NUM_14;

inline constexpr gpio_num_t kI2sMasterClock = GPIO_NUM_16;
inline constexpr gpio_num_t kI2sBitClock = GPIO_NUM_9;
inline constexpr gpio_num_t kI2sWordSelect = GPIO_NUM_45;
inline constexpr gpio_num_t kI2sDataOut = GPIO_NUM_8;
inline constexpr gpio_num_t kAmplifierEnable = GPIO_NUM_46;

// Both side buttons are active low. PWR is a hardware power latch.
inline constexpr gpio_num_t kBootButton = GPIO_NUM_0;
inline constexpr gpio_num_t kKeyButton = GPIO_NUM_18;

}  // namespace micropixel::platform::waveshare_rlcd_4_2::board
