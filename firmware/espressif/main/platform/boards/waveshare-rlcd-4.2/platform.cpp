#include "platform/platform.hpp"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "host/ui/lvgl/square_common/square_system_ui.hpp"
#include "platform/adapters/graphics_adapter.hpp"
#include "platform/audio/es8311_i2s_audio_sink.hpp"
#include "platform/boards/esp32-s3-common/landscape_320_state.hpp"
#include "platform/boards/esp32-s3-common/lvgl_display.hpp"
#include "platform/boards/esp32-s3-common/presentation.hpp"
#include "platform/boards/waveshare-rlcd-4.2/board_config.hpp"
#include "platform/boards/waveshare-rlcd-4.2/display_hardware.hpp"
#include "platform/input/gpio_key_input.hpp"
#include "platform/lvgl/guest_graphics_operations.hpp"
#include "platform/memory/ext_ram_bss.hpp"
#include "platform/memory/internal_ram.hpp"
#include "platform/wifi/native_wifi_radio.hpp"
#include "platform/wifi/wifi_manager.hpp"
#include "work/task_policy.hpp"

namespace micropixel::platform {
namespace {

namespace common = esp32_s3_common;
namespace board_detail = waveshare_rlcd_4_2;
constexpr char kTag[] = "rlcd42";

// A reflective panel has no backlight; brightness requests are accepted and
// have nothing to drive.
esp_err_t IgnoreBrightness(void*, int) { return ESP_OK; }

class WaveshareRlcd42Board final : public Board {
   public:
    WaveshareRlcd42Board()
        : state_(TaskState(input_state_)),
          graphics_context_{.engine = &state_.guest_graphics, .hooks = state_.ui.GraphicsHooks()},
          graphics_(lvgl::MakeGuestGraphicsOperations(graphics_context_)),
          audio_output_(
              {
                  .name = "RLCD-4.2 ES8311/I2S",
                  .log_tag = "rlcd42_audio",
                  .i2c_port = I2C_NUM_0,
                  .i2s_port = I2S_NUM_0,
                  .master_clock = board_detail::board::kI2sMasterClock,
                  .bit_clock = board_detail::board::kI2sBitClock,
                  .word_select = board_detail::board::kI2sWordSelect,
                  .data_out = board_detail::board::kI2sDataOut,
                  .amplifier_enable = board_detail::board::kAmplifierEnable,
                  .amplifier_setter = nullptr,
                  .amplifier_context = nullptr,
                  .codec_i2c_address = static_cast<uint8_t>(ES8311_CODEC_DEFAULT_ADDR >> 1U),
                  .sample_rate = 16000U,
                  .i2c_clock_hz = 100000U,
                  .amplifier_preroll_ms = 24U,
                  .dma_descriptor_count = 6U,
                  .dma_frame_count = 240U,
                  .amplifier_active_low = false,
                  .probe_before_attach = true,
              },
              {.amplifier_voltage = 3.3F, .codec_dac_voltage = 3.3F}),
          presentation_(state_, kTag, IgnoreBrightness, nullptr),
          system_ui_(state_.ui, presentation_) {
        state_.guest_graphics.SetPresentationHooks(state_.ui.GuestFrameHooks());
    }

    [[nodiscard]] esp_err_t Initialize(BoardContext& context) override {
        ESP_RETURN_ON_FALSE(memory::IsInternalObject(*this), ESP_ERR_INVALID_STATE, kTag,
                            "Board control objects must reside in internal RAM");
        presentation_.BindAudioEngine(context.AudioEngine());
        ESP_LOGI(kTag, "initializing Waveshare ESP32-S3-RLCD-4.2");
        ESP_RETURN_ON_ERROR(InitializeI2cBus(), kTag, "create shared I2C bus failed");
        ESP_RETURN_ON_ERROR(board_detail::InitializeDisplayHardware(state_), kTag, "initialize ST7305 display failed");
        ESP_RETURN_ON_ERROR(common::RegisterLvglDisplay(state_, false), kTag, "register LVGL display failed");
        ESP_RETURN_ON_ERROR(state_.display_shadow.Initialize(state_.display), kTag,
                            "initialize PSRAM displayed shadow failed");
        ESP_RETURN_ON_ERROR(state_.i2c_executor.Initialize(), kTag, "start shared I2C executor failed");
        // No touch controller: touches arrive only through USB development
        // input and remote sequences.
        ESP_RETURN_ON_ERROR(state_.touch_input.InitializeVirtual(), kTag, "bind injected touch input failed");
        ESP_RETURN_ON_ERROR(state_.guest_graphics.Initialize(state_.display, nullptr), kTag,
                            "initialize RGB565 Guest graphics failed");

        if (esp_lv_adapter_lock(-1) != ESP_OK) {
            return ESP_FAIL;
        }
        const esp_err_t ui_status = state_.ui.InitializeLocked(state_.display);
        esp_lv_adapter_unlock();
        ESP_RETURN_ON_ERROR(ui_status, kTag, "initialize 320x240 Host UI failed");
        ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), kTag, "start LVGL adapter failed");
        ESP_RETURN_ON_ERROR(state_.touch_input.Start(state_.display), kTag, "start injected touch input failed");

        const esp_err_t audio_status = audio_output_.Configure(i2c_bus_, state_.i2c_executor);
        if (audio_status != ESP_OK) {
            ESP_LOGW(kTag, "audio unavailable for this boot: %s", esp_err_to_name(audio_status));
        }
        ESP_RETURN_ON_ERROR(boot_button_.Initialize(state_.ui.Input()), kTag, "initialize BOOT key failed");
        ESP_RETURN_ON_ERROR(key_button_.Initialize(state_.ui.Input()), kTag, "initialize KEY button failed");
        // USB development screenshots take the same path as Remote Control.
        ESP_RETURN_ON_ERROR(
            state_.development_display.Start(state_.touch_input, state_.local_control, common::kWidth, common::kHeight,
                                             transports::DevelopmentCaptureHook::For(presentation_)),
            kTag, "start USB development control failed");

        // Process-lifetime PSRAM storage keeps this aggregate off the Host task stack.
        static MICROPIXEL_EXT_RAM_BSS BoardRegistration registration{{
            .board = "Waveshare ESP32-S3-RLCD-4.2",
            .host_chip = "ESP32-S3",
            .firmware_target = "waveshare-rlcd42",
            .wifi_coprocessor = "Native ESP32-S3",
            .touch_controller = "None (injected input only)",
            .display =
                {
                    .driver = state_.panel_name,
                    .interface = "SPI 10 MHz",
                    .pixel_format = "1bpp reflective (RGB565 brightest-channel threshold)",
                    .width_pixels = common::kWidth,
                    .height_pixels = common::kHeight,
                },
            .graphics_acceleration = "CPU only; SPI DMA transport",
        }};
        registration.SetInput(state_.ui.Input());
        registration.SetGraphics(graphics_);
        if (audio_status == ESP_OK) {
            registration.SetAudioOutput(audio_output_, audio_output_.SampleRate());
        }
        registration.SetWifi(wifi_);
        registration.SetLocalControl(state_.local_control);
        registration.SetSystemUi(system_ui_);
        ESP_LOGI(kTag, "ready: ST7305 RLCD, BOOT/KEY buttons, native Wi-Fi, audio=%s",
                 audio_status == ESP_OK ? "ES8311" : "off");
        return context.Publish(registration) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    void BindBackgroundExecutor(work::BackgroundExecutor& executor) override {
        state_.ui.BindBackgroundExecutor(executor);
        state_.guest_graphics.BindBackgroundExecutor(executor);
        wifi_.BindBackgroundExecutor(executor);
    }

   private:
    static common::Landscape320State& TaskState(common::Landscape320InputState& input) {
        static MICROPIXEL_EXT_RAM_BSS common::Landscape320State state(input);
        return state;
    }

    [[nodiscard]] esp_err_t InitializeI2cBus() {
        i2c_master_bus_config_t bus_config{};
        bus_config.i2c_port = I2C_NUM_0;
        bus_config.sda_io_num = board_detail::board::kI2cSda;
        bus_config.scl_io_num = board_detail::board::kI2cScl;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7U;
        bus_config.flags.enable_internal_pullup = true;
        return i2c_new_master_bus(&bus_config, &i2c_bus_);
    }

    common::Landscape320InputState input_state_{};
    common::Landscape320State& state_;
    lvgl::GuestGraphicsOperationsContext graphics_context_{};
    adapters::GraphicsAdapter graphics_;
    i2c_master_bus_handle_t i2c_bus_{};
    audio::Es8311I2sAudioSink audio_output_;
    common::Landscape320Presentation presentation_;
    host_ui::lvgl::square_common::SquareSystemUi system_ui_;
    input::GpioKeyInput boot_button_{{.pin = board_detail::board::kBootButton,
                                      .code = device::KeyCode::kConfirm,
                                      .log_tag = "rlcd42_boot",
                                      .task_name = "rlcd42_boot",
                                      .task_core = task_policy::kSystemCore}};
    input::GpioKeyInput key_button_{{.pin = board_detail::board::kKeyButton,
                                     .code = device::KeyCode::kBack,
                                     .log_tag = "rlcd42_key",
                                     .task_name = "rlcd42_key",
                                     .task_core = task_policy::kSystemCore}};
    wifi::NativeWifiRadio wifi_radio_{};
    wifi::WifiManager wifi_{wifi_radio_};
};

}  // namespace

Board& ConfiguredBoard() {
    static WaveshareRlcd42Board board;
    return board;
}

}  // namespace micropixel::platform
