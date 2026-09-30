#include "platform/boards/waveshare-rlcd-4.2/display_hardware.hpp"

#include <cstdint>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "platform/boards/esp32-s3-common/landscape_320_state.hpp"
#include "platform/boards/waveshare-rlcd-4.2/board_config.hpp"
#include "platform/boards/waveshare-rlcd-4.2/st7305_panel.hpp"

namespace micropixel::platform::waveshare_rlcd_4_2 {
namespace {

constexpr char kTag[] = "rlcd42_display";
constexpr spi_host_device_t kLcdSpiHost = SPI3_HOST;
// The vendor example runs this panel at 10 MHz.
constexpr uint32_t kLcdPixelClockHz = 10U * 1000U * 1000U;
constexpr size_t kPanelIoQueueDepth = 10U;
constexpr int kMaxTransferBytes = 16384;
// Centre 320x240 on 400x300.
constexpr int kGapX = (board::kPanelWidth - esp32_s3_common::kWidth) / 2;
constexpr int kGapY = (board::kPanelHeight - esp32_s3_common::kHeight) / 2;

void ReleaseDisplayHardware(esp32_s3_common::Landscape320State& state, bool bus_initialized) {
    if (state.panel != nullptr) {
        (void)esp_lcd_panel_del(state.panel);
        state.panel = nullptr;
    }
    if (state.panel_io != nullptr) {
        (void)esp_lcd_panel_io_del(state.panel_io);
        state.panel_io = nullptr;
    }
    if (bus_initialized) {
        (void)spi_bus_free(kLcdSpiHost);
    }
}

// Boot pattern drawn through the same draw_bitmap path LVGL uses: a black
// frame around the letterbox and a black square in its top-left corner, so a
// photo settles orientation and polarity even if LVGL never flushes.
esp_err_t DrawBootPattern(esp_lcd_panel_handle_t panel) {
    constexpr int kWidth = esp32_s3_common::kWidth;
    constexpr int kHeight = esp32_s3_common::kHeight;
    constexpr int kBorder = 4;
    constexpr int kMarker = 48;
    auto* pixels = static_cast<uint16_t*>(
        heap_caps_malloc(static_cast<size_t>(kWidth) * kHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM));
    if (pixels == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const bool border = x < kBorder || y < kBorder || x >= kWidth - kBorder || y >= kHeight - kBorder;
            const bool marker = x < kMarker && y < kMarker;
            // White and black are byte-order symmetric, so no wire swap is needed.
            pixels[y * kWidth + x] = border || marker ? 0x0000U : 0xFFFFU;
        }
    }
    const esp_err_t status = esp_lcd_panel_draw_bitmap(panel, 0, 0, kWidth, kHeight, pixels);
    heap_caps_free(pixels);
    return status;
}

}  // namespace

esp_err_t InitializeDisplayHardware(esp32_s3_common::Landscape320State& state) {
    spi_bus_config_t bus_config{};
    bus_config.mosi_io_num = board::kLcdMosi;
    bus_config.miso_io_num = GPIO_NUM_NC;
    bus_config.sclk_io_num = board::kLcdClock;
    bus_config.quadwp_io_num = GPIO_NUM_NC;
    bus_config.quadhd_io_num = GPIO_NUM_NC;
    bus_config.data4_io_num = GPIO_NUM_NC;
    bus_config.data5_io_num = GPIO_NUM_NC;
    bus_config.data6_io_num = GPIO_NUM_NC;
    bus_config.data7_io_num = GPIO_NUM_NC;
    bus_config.max_transfer_sz = kMaxTransferBytes;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(kLcdSpiHost, &bus_config, SPI_DMA_CH_AUTO), kTag, "SPI bus init failed");

    esp_lcd_panel_io_spi_config_t io_config{};
    io_config.cs_gpio_num = board::kLcdChipSelect;
    io_config.dc_gpio_num = board::kLcdDataCommand;
    io_config.spi_mode = 0;
    io_config.pclk_hz = kLcdPixelClockHz;
    io_config.trans_queue_depth = kPanelIoQueueDepth;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    esp_err_t status =
        esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kLcdSpiHost), &io_config, &state.panel_io);
    if (status != ESP_OK) {
        ReleaseDisplayHardware(state, true);
        return status;
    }

    status = NewSt7305Panel(state.panel_io, {.reset_gpio = board::kLcdReset}, &state.panel);
    if (status == ESP_OK) {
        status = esp_lcd_panel_reset(state.panel);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_init(state.panel);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_set_gap(state.panel, kGapX, kGapY);
    }
    if (status == ESP_OK) {
        status = esp_lcd_panel_disp_on_off(state.panel, true);
    }
    if (status == ESP_OK) {
        status = DrawBootPattern(state.panel);
    }
    if (status != ESP_OK) {
        ReleaseDisplayHardware(state, true);
        return status;
    }
    state.panel_name = "ST7305";
    ESP_LOGI(kTag, "ST7305 400x300 RLCD up: 320x240 letterboxed at (%d,%d), SPI %lu MHz", kGapX, kGapY,
             static_cast<unsigned long>(kLcdPixelClockHz / 1000000U));
    return ESP_OK;
}

}  // namespace micropixel::platform::waveshare_rlcd_4_2
