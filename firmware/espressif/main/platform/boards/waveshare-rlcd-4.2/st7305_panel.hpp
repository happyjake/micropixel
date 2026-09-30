#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_types.h"
#include "hal/gpio_types.h"

namespace micropixel::platform::waveshare_rlcd_4_2 {

struct St7305PanelConfig final {
    gpio_num_t reset_gpio{GPIO_NUM_NC};
};

// esp_lcd panel for the Sitronix ST7305 reflective LCD on the Waveshare
// ESP32-S3-RLCD-4.2: 400x300 landscape, one bit per pixel, set bit = white.
//
// LVGL keeps rendering RGB565; draw_bitmap thresholds the brightest channel of
// each big-endian (wire order) pixel into a 1bpp frame in the controller's
// native packing, offset by the set_gap origin, and streams the whole frame
// with exactly one esp_lcd_panel_io_tx_color per call. The LVGL adapter signals flush_ready
// from that transfer's on_color_trans_done, so a draw_bitmap that sends no
// color data, or more than one transfer, would stall or tear the next strip.
[[nodiscard]] esp_err_t NewSt7305Panel(esp_lcd_panel_io_handle_t io, const St7305PanelConfig& config,
                                       esp_lcd_panel_handle_t* panel);

}  // namespace micropixel::platform::waveshare_rlcd_4_2
