#include "platform/boards/waveshare-rlcd-4.2/st7305_panel.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace micropixel::platform::waveshare_rlcd_4_2 {
namespace {

constexpr char kTag[] = "st7305";

// Landscape geometry seen by callers, and the controller RAM behind it. The
// controller is addressed as 300 columns x 400 rows; one byte covers a
// 4-column x 2-row block, so a frame is 15000 bytes.
constexpr int kPanelWidth = 400;
constexpr int kPanelHeight = 300;
constexpr size_t kFrameBytes = static_cast<size_t>(kPanelWidth) * kPanelHeight / 8U;
constexpr int kBytesPerPhysicalRowPair = kPanelHeight / 4;  // 75

// A pixel is white (the reflective state) when its brightest channel reaches
// this value on 0..255. The Host UI and the bundled games draw saturated
// colours on near-black, and a luma cut makes them vanish: Snake's red food
// and green body both sit just under luma 128. The brightest channel keeps
// every coloured element white while backgrounds and card fills (<= 20) stay
// black.
constexpr uint32_t kWhiteThreshold = 96U;

struct InitCommand final {
    uint8_t command;
    std::array<uint8_t, 10> data;
    uint8_t data_bytes;
    uint16_t delay_ms;
};

// Waveshare ESP32-S3-RLCD-4.2 ESP-IDF example (09_LVGL_V9_Test,
// components/port_bsp/display_bsp.cpp, RLCD_Init), in order.
constexpr InitCommand kInitSequence[]{
    {0xD6, {0x17, 0x02}, 2, 0},                                                   // NVM load control
    {0xD1, {0x01}, 1, 0},                                                         // booster enable
    {0xC0, {0x11, 0x04}, 2, 0},                                                   // gate voltage
    {0xC1, {0x69, 0x69, 0x69, 0x69}, 4, 0},                                       // VSHP
    {0xC2, {0x19, 0x19, 0x19, 0x19}, 4, 0},                                       // VSLP
    {0xC4, {0x4B, 0x4B, 0x4B, 0x4B}, 4, 0},                                       // VSHN
    {0xC5, {0x19, 0x19, 0x19, 0x19}, 4, 0},                                       // VSLN
    {0xD8, {0x80, 0xE9}, 2, 0},                                                   // OSC
    {0xB2, {0x02}, 1, 0},                                                         // frame rate
    {0xB3, {0xE5, 0xF6, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45}, 10, 0},  // HPM gate EQ
    {0xB4, {0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45}, 8, 0},               // LPM gate EQ
    {0x62, {0x32, 0x03, 0x1F}, 3, 0},                                             // gate timing
    {0xB7, {0x13}, 1, 0},                                                         // source EQ
    {0xB0, {0x64}, 1, 0},                                                         // 400 gate lines
    {0x11, {}, 0, 200},                                                           // sleep out
    {0xC9, {0x00}, 1, 0},                                                         // source voltage select
    {0x36, {0x48}, 1, 0},                                                         // MADCTL
    {0x3A, {0x11}, 1, 0},                                                         // data format
    {0xB9, {0x20}, 1, 0},                                                         // gamma mode: mono
    {0xB8, {0x29}, 1, 0},                                                         // panel setting
    {0x21, {}, 0, 0},                                                             // inversion on
    {0x35, {0x00}, 1, 0},                                                         // tearing effect line on
    {0xD0, {0xFF}, 1, 0},                                                         // auto power down
    {0x38, {}, 0, 0},                                                             // high power mode
};

struct St7305Panel final {
    esp_lcd_panel_t base{};
    esp_lcd_panel_io_handle_t io{};
    gpio_num_t reset_gpio{GPIO_NUM_NC};
    int x_gap{};
    int y_gap{};
    // Internal DMA-capable RAM: the SPI master streams straight from it.
    uint8_t* frame{};
};

St7305Panel* FromBase(esp_lcd_panel_t* panel) {
    return reinterpret_cast<St7305Panel*>(reinterpret_cast<uint8_t*>(panel) - offsetof(St7305Panel, base));
}

// Landscape (x, y) to controller byte and mask, from the vendor's
// InitLandscapeLUT: the landscape y axis runs against the controller columns.
inline void SetPixel(uint8_t* frame, int x, int y, bool white) {
    const int inverted_y = kPanelHeight - 1 - y;
    const size_t index = static_cast<size_t>(x >> 1) * kBytesPerPhysicalRowPair + static_cast<size_t>(inverted_y >> 2);
    const uint8_t mask = static_cast<uint8_t>(1U << (7 - (((inverted_y & 3) << 1) | (x & 1))));
    if (white) {
        frame[index] = static_cast<uint8_t>(frame[index] | mask);
    } else {
        frame[index] = static_cast<uint8_t>(frame[index] & ~mask);
    }
}

esp_err_t SendCommand(St7305Panel& panel, uint8_t command, const uint8_t* data, size_t size) {
    return esp_lcd_panel_io_tx_param(panel.io, command, data, size);
}

esp_err_t SetFullWindow(St7305Panel& panel) {
    constexpr uint8_t kColumns[]{0x12, 0x2A};
    constexpr uint8_t kRows[]{0x00, 0xC7};
    ESP_RETURN_ON_ERROR(SendCommand(panel, 0x2A, kColumns, sizeof(kColumns)), kTag, "column window failed");
    return SendCommand(panel, 0x2B, kRows, sizeof(kRows));
}

// Exactly one color transfer, which is what completes an LVGL flush.
esp_err_t SendFrame(St7305Panel& panel) {
    ESP_RETURN_ON_ERROR(SetFullWindow(panel), kTag, "row window failed");
    return esp_lcd_panel_io_tx_color(panel.io, 0x2C, panel.frame, kFrameBytes);
}

esp_err_t PanelReset(esp_lcd_panel_t* base) {
    St7305Panel& panel = *FromBase(base);
    if (panel.reset_gpio == GPIO_NUM_NC) {
        return ESP_OK;
    }
    gpio_set_level(panel.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(panel.reset_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(panel.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

esp_err_t PanelInit(esp_lcd_panel_t* base) {
    St7305Panel& panel = *FromBase(base);
    for (const InitCommand& step : kInitSequence) {
        ESP_RETURN_ON_ERROR(
            SendCommand(panel, step.command, step.data_bytes == 0U ? nullptr : step.data.data(), step.data_bytes), kTag,
            "init command 0x%02x failed", step.command);
        if (step.delay_ms != 0U) {
            vTaskDelay(pdMS_TO_TICKS(step.delay_ms));
        }
    }
    std::memset(panel.frame, 0xFF, kFrameBytes);
    // Sent before the LVGL adapter registers its transfer-done callback, so
    // this frame completes no flush.
    return SendFrame(panel);
}

esp_err_t PanelDelete(esp_lcd_panel_t* base) {
    St7305Panel* panel = FromBase(base);
    if (panel->reset_gpio != GPIO_NUM_NC) {
        (void)gpio_reset_pin(panel->reset_gpio);
    }
    heap_caps_free(panel->frame);
    delete panel;
    return ESP_OK;
}

esp_err_t PanelDrawBitmap(esp_lcd_panel_t* base, int x_start, int y_start, int x_end, int y_end,
                          const void* color_data) {
    St7305Panel& panel = *FromBase(base);
    if (color_data == nullptr || x_start >= x_end || y_start >= y_end) {
        return ESP_ERR_INVALID_ARG;
    }
    const int origin_x = x_start + panel.x_gap;
    const int origin_y = y_start + panel.y_gap;
    const int width = x_end - x_start;
    const int height = y_end - y_start;
    if (origin_x < 0 || origin_y < 0 || origin_x + width > kPanelWidth || origin_y + height > kPanelHeight) {
        return ESP_ERR_INVALID_ARG;
    }
    // The adapter byte-swaps RGB565 into big-endian wire order for SPI panels
    // before it calls draw_bitmap.
    const auto* source = static_cast<const uint8_t*>(color_data);
    for (int row = 0; row < height; ++row) {
        const int y = origin_y + row;
        for (int column = 0; column < width; ++column) {
            const uint32_t pixel = (static_cast<uint32_t>(source[0]) << 8U) | source[1];
            source += 2;
            const uint32_t red = ((pixel >> 11U) & 0x1FU) << 3U;
            const uint32_t green = ((pixel >> 5U) & 0x3FU) << 2U;
            const uint32_t blue = (pixel & 0x1FU) << 3U;
            const uint32_t brightest = std::max({red, green, blue});
            SetPixel(panel.frame, origin_x + column, y, brightest >= kWhiteThreshold);
        }
    }
    return SendFrame(panel);
}

esp_err_t PanelDrawBitmap2d(esp_lcd_panel_t*, int, int, int, int, const void*, size_t, size_t, int, int, int, int) {
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PanelMirror(esp_lcd_panel_t*, bool mirror_x, bool mirror_y) {
    return mirror_x || mirror_y ? ESP_ERR_NOT_SUPPORTED : ESP_OK;
}

esp_err_t PanelSwapXy(esp_lcd_panel_t*, bool swap_axes) { return swap_axes ? ESP_ERR_NOT_SUPPORTED : ESP_OK; }

esp_err_t PanelSetGap(esp_lcd_panel_t* base, int x_gap, int y_gap) {
    St7305Panel& panel = *FromBase(base);
    panel.x_gap = x_gap;
    panel.y_gap = y_gap;
    return ESP_OK;
}

esp_err_t PanelInvertColor(esp_lcd_panel_t* base, bool invert) {
    // The panel reads correctly with inversion on (set bit = white).
    return SendCommand(*FromBase(base), invert ? 0x20 : 0x21, nullptr, 0U);
}

esp_err_t PanelDisplayOnOff(esp_lcd_panel_t* base, bool on) {
    return SendCommand(*FromBase(base), on ? 0x29 : 0x28, nullptr, 0U);
}

esp_err_t PanelSleep(esp_lcd_panel_t* base, bool sleep) {
    return SendCommand(*FromBase(base), sleep ? 0x10 : 0x11, nullptr, 0U);
}

esp_err_t PanelSetBrightness(esp_lcd_panel_t*, int) { return ESP_ERR_NOT_SUPPORTED; }

}  // namespace

esp_err_t NewSt7305Panel(esp_lcd_panel_io_handle_t io, const St7305PanelConfig& config,
                         esp_lcd_panel_handle_t* out_panel) {
    ESP_RETURN_ON_FALSE(io != nullptr && out_panel != nullptr, ESP_ERR_INVALID_ARG, kTag, "invalid argument");
    auto* panel = new (std::nothrow) St7305Panel();
    ESP_RETURN_ON_FALSE(panel != nullptr, ESP_ERR_NO_MEM, kTag, "no memory for panel");
    panel->frame = static_cast<uint8_t*>(heap_caps_malloc(kFrameBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (panel->frame == nullptr) {
        delete panel;
        return ESP_ERR_NO_MEM;
    }
    if (config.reset_gpio != GPIO_NUM_NC) {
        gpio_config_t reset_config{};
        reset_config.pin_bit_mask = 1ULL << config.reset_gpio;
        reset_config.mode = GPIO_MODE_OUTPUT;
        reset_config.pull_up_en = GPIO_PULLUP_ENABLE;
        const esp_err_t status = gpio_config(&reset_config);
        if (status != ESP_OK) {
            heap_caps_free(panel->frame);
            delete panel;
            return status;
        }
        gpio_set_level(config.reset_gpio, 1);
    }
    panel->io = io;
    panel->reset_gpio = config.reset_gpio;
    panel->base.reset = PanelReset;
    panel->base.init = PanelInit;
    panel->base.del = PanelDelete;
    panel->base.draw_bitmap = PanelDrawBitmap;
    panel->base.draw_bitmap_2d = PanelDrawBitmap2d;
    panel->base.mirror = PanelMirror;
    panel->base.swap_xy = PanelSwapXy;
    panel->base.set_gap = PanelSetGap;
    panel->base.invert_color = PanelInvertColor;
    panel->base.disp_on_off = PanelDisplayOnOff;
    panel->base.disp_sleep = PanelSleep;
    panel->base.set_brightness = PanelSetBrightness;
    *out_panel = &panel->base;
    ESP_LOGI(kTag, "ST7305 400x300 1bpp panel created, frame=%u bytes internal DMA",
             static_cast<unsigned>(kFrameBytes));
    return ESP_OK;
}

}  // namespace micropixel::platform::waveshare_rlcd_4_2
