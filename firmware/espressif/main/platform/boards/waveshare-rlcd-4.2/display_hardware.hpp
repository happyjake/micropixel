#pragma once

#include "esp_err.h"

namespace micropixel::platform::esp32_s3_common {
struct Landscape320State;
}

namespace micropixel::platform::waveshare_rlcd_4_2 {

// Brings up the ST7305 and letterboxes the shared 320x240 landscape profile
// in the middle of its 400x300 glass.
[[nodiscard]] esp_err_t InitializeDisplayHardware(esp32_s3_common::Landscape320State& state);

}  // namespace micropixel::platform::waveshare_rlcd_4_2
