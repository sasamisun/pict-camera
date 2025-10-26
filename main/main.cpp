/*
 * AtomS3R ピクセルアートカメラ (ESP-IDF 5.4完全対応版)
 * SSD1306ディスプレイ動作テスト版
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <math.h>

// ESP-IDF 5.4コア
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_random.h"

// ドライバー (ESP-IDF 5.4対応)
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"

// カメラ (esp-camera 2.1.3)
#include "esp_camera.h"

// ファイルシステム
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// M5GFX (ディスプレイ用、オプション)
#ifdef CONFIG_ENABLE_M5GFX
#include <M5GFX.h>
#endif

// カスタムクラス - エンコーダーのみ新API使用
#include "pimoroni_encoder.h"
// #include "PixelArtProcessor.h"
#include "camera_utils.h"
#include "ssd1306_display.h"

// 定数定義など... (省略、元のファイルと同じ)
static const char *TAG = "PixelArtCamera";

#define SHUTTER_BUTTON_PIN GPIO_NUM_38
#define MENU_BUTTON_PIN GPIO_NUM_39
#define LED_PIN GPIO_NUM_39

#define I2C_SDA_PIN GPIO_NUM_1
#define I2C_SCL_PIN GPIO_NUM_2
#define I2C_FREQ_HZ 400000

#define EXTERNAL_I2C_NUM I2C_NUM_1
#define EXTERNAL_I2C_FREQ_HZ 400000

// SDカード用SPI設定
#define SPI_SCK GPIO_NUM_7
#define SPI_MISO GPIO_NUM_8
#define SPI_MOSI GPIO_NUM_6
#define SPI_CS GPIO_NUM_15
#define SPI_FREQ 10000000

#define BUTTON_LONG_PRESS_MS 1000
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_REPEAT_DELAY_MS 500

#define DEBOUNCE_DELAY_MS 300
#define MAX_PALETTE_INDEX 8
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 176
#define CAPTURE_TASK_STACK 8192
#define PROCESS_TASK_STACK 16384
#define ENCODER_TASK_STACK 4096

// エンコーダーチャタリング対策設定
#define ENCODER_DEBOUNCE_MS 200      // 値変化後のデバウンス時間(ms)
#define ENCODER_POLL_INTERVAL_MS 50  // ポーリング間隔(ms)

// 構造体定義など... (省略、元のファイルと同じ)
typedef struct
{
    gpio_num_t pin;
    bool current_state;
    bool last_state;
    uint32_t press_start_time;
    uint32_t last_change_time;
    bool long_press_triggered;
    const char *name;
} button_state_t;

typedef enum
{
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_PRESS_START,
    BUTTON_EVENT_PRESS_END
} button_event_t;

typedef struct
{
    int16_t value;
    int16_t delta;
    uint32_t timestamp;
} encoder_event_t;

typedef enum
{
    SYSTEM_STATUS_INITIALIZING,
    SYSTEM_STATUS_READY,
    SYSTEM_STATUS_CAPTURING,
    SYSTEM_STATUS_PROCESSING,
    SYSTEM_STATUS_SAVING,
    SYSTEM_STATUS_ERROR
} system_status_t;

// グローバル変数 - エンコーダーのみC構造体に変更
static SemaphoreHandle_t g_capture_semaphore = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static QueueHandle_t g_capture_queue = NULL;
static QueueHandle_t g_encoder_event_queue = NULL;

static button_state_t g_shutter_button = {
    .pin = SHUTTER_BUTTON_PIN,
    .current_state = false,
    .last_state = false,
    .press_start_time = 0,
    .last_change_time = 0,
    .long_press_triggered = false,
    .name = "Shutter"};

static button_state_t g_menu_button = {
    .pin = MENU_BUTTON_PIN,
    .current_state = false,
    .last_state = false,
    .press_start_time = 0,
    .last_change_time = 0,
    .long_press_triggered = false,
    .name = "Menu"};

static volatile int g_current_palette_index = 0;
static volatile int g_file_counter = 1;
static volatile uint32_t g_last_button_press = 0;

// ハードウェアオブジェクト - エンコーダーのみC構造体に変更
static pimoroni_encoder_t g_encoder;
static CameraUtils *g_camera_utils = nullptr;
static SSD1306Display *g_display = nullptr;

static volatile bool g_system_ready = false;
static bool g_camera_ready = false;
static bool g_encoder_ready = false;
static bool g_display_ready = false;
static bool g_sd_card_ready = false;

static volatile system_status_t g_system_status = SYSTEM_STATUS_INITIALIZING;
static volatile bool g_status_led_enabled = true;

static sdmmc_card_t *g_sd_card = NULL;
static const char *g_mount_point = "/sdcard";
static bool g_sd_card_mounted = false;

// カラーパレット定義
static const uint32_t COLOR_PALETTES[8][8] = {
    {0x0D2B45, 0x203C56, 0x544E68, 0x8D697A, 0xD08159, 0xFFAA5E, 0xFFD4A3, 0xFFECD6},
    {0x000000, 0x000B22, 0x112B43, 0x437290, 0x437290, 0xE0D8D1, 0xE0D8D1, 0xFFFFFF},
    {0x010101, 0x33669F, 0x33669F, 0x33669F, 0x498DB7, 0x498DB7, 0xFBE379, 0xFBE379},
    {0x0E0E12, 0x1A1A24, 0x333346, 0x535373, 0x8080A4, 0xA6A6BF, 0xC1C1D2, 0xE6E6EC},
    {0x1E1C32, 0x1E1C32, 0x1E1C32, 0x1E1C32, 0xC6BAAC, 0xC6BAAC, 0xC6BAAC, 0xC6BAAC},
    {0x252525, 0x252525, 0x4B564D, 0x4B564D, 0x9AA57C, 0x9AA57C, 0xE0E9C4, 0xE0E9C4},
    {0x001D2A, 0x085562, 0x009A98, 0x00BE91, 0x38D88E, 0x9AF089, 0xF2FF66, 0xF2FF66},
    {0x000000, 0x012036, 0x3A7BAA, 0x7D8FAE, 0xA1B4C1, 0xF0B9B9, 0xFFD159, 0xFFFFFF},
};

static const uint32_t PALETTE_REP_COLORS[8] = {
    0x8D697A,
    0x437290,
    0x498DB7,
    0x8080A4,
    0xC6BAAC,
    0x9AA57C,
    0x38D88E,
    0xFFD159,
};

// 関数プロトタイプ
void update_button_state(button_state_t *button);
button_event_t get_button_event(button_state_t *button);
void process_button_events(void);
void encoder_task(void *parameter);
void camera_preview_task(void *parameter);
esp_err_t init_sd_card(void);
void print_sd_card_info(void);
esp_err_t init_external_i2c(void);
esp_err_t scan_i2c_devices(i2c_port_t i2c_num, int *found_devices);
esp_err_t init_gpio(void);
void display_init_step(Terminal *terminal, const char *step_name);
void display_init_step(Terminal *terminal, bool success);
void run_display_test_patterns(void);

// ★★★ 初期化ステップ表示ヘルパー関数 ★★★
void display_init_step(Terminal *terminal, const char *step_name)
{
    if (!terminal || !step_name || !g_display)
    {
        return;
    }

    // 処理開始: スペース + メッセージ表示（改行なし）
    // uint8_t start_row = terminal->get_cursor_row();
    // g_display->terminal_print(terminal, " ");
    g_display->terminal_print(terminal, step_name);

    // 画面更新
    g_display->clear();
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 0, false);
    g_display->draw_terminal(terminal);
    g_display->display();

    // 改行して次のステップに備える
    // terminal->newline();
}

void display_init_step(Terminal *terminal, bool success)
{
    if (!terminal || !g_display)
    {
        return;
    }

    uint8_t start_row = terminal->get_cursor_row();
    // 処理完了後: 最初の文字を○または×に上書き
    terminal->set_char(start_row, 0, success ? 26 : 91);

    // 画面更新
    g_display->clear();
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 0, false);
    g_display->draw_terminal(terminal);
    g_display->display();

    // 3. 改行して次のステップに備える
    terminal->newline();

    // 4. エラー時は1秒停止 + 無限ループ
    if (!success)
    {
        ESP_LOGE(TAG, "❌ 初期化失敗");
        vTaskDelay(pdMS_TO_TICKS(1000));

        // エラー時は無限ループで停止
        ESP_LOGE(TAG, "システムエラー - 停止します");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/*
* テスト集
*/

// ★★★ SSD1306表示テスト関数 ★★★
void run_display_test_patterns(void)
{
    if (!g_display_ready || g_display == nullptr)
    {
        ESP_LOGE(TAG, "❌ ディスプレイが準備できていません");
        return;
    }

    ESP_LOGI(TAG, "🎨 スプラッシュ画面＋ターミナル");

    ESP_LOGI(TAG, "ビットマップ画像表示（通常）");
    g_display->clear();
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 0, false);
    g_display->display();

    // 画像の下にターミナル表示
    ESP_LOGI(TAG, "画像の下にターミナル表示（枠線なし）");
    Terminal terminal;
    terminal.init();
    terminal.set_position(0, 40);
    terminal.set_border(false);

    // UTF-8文字列をターミナルに表示
    g_display->terminal_println(&terminal, "PICT Camera v1.0");
    g_display->terminal_println(&terminal, "System Init...");

    g_display->draw_terminal(&terminal);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト: 日本語文字列
    ESP_LOGI(TAG, "日本語文字列テスト");
    terminal.clear();
    terminal.set_border(true);
    terminal.set_position(0, 0);

    g_display->clear();
    g_display->terminal_println(&terminal, "こんにちは!");
    g_display->terminal_println(&terminal, "カメラ準備OK");
    g_display->terminal_println(&terminal, "ABC:123");
    g_display->terminal_println(&terminal, "記号?!@#$%");

    g_display->draw_terminal(&terminal);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "✅ ディスプレイ描画テスト完了");
}

/*
* 初期化処理
*/

esp_err_t init_external_i2c(void)
{
    i2c_config_t external_i2c_config = {};
    external_i2c_config.mode = I2C_MODE_MASTER;
    external_i2c_config.sda_io_num = I2C_SDA_PIN;
    external_i2c_config.scl_io_num = I2C_SCL_PIN;
    external_i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    external_i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    external_i2c_config.master.clk_speed = EXTERNAL_I2C_FREQ_HZ;
    external_i2c_config.clk_flags = 0;

    esp_err_t ret = i2c_param_config(EXTERNAL_I2C_NUM, &external_i2c_config);
    if (ret != ESP_OK)
        return ret;

    ret = i2c_driver_install(EXTERNAL_I2C_NUM, external_i2c_config.mode, 0, 0, 0);
    return ret;
}

esp_err_t init_gpio(void)
{
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << SHUTTER_BUTTON_PIN) | (1ULL << MENU_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK)
        return ret;

    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    ret = gpio_config(&led_config);
    if (ret != ESP_OK)
        return ret;

    gpio_set_level(LED_PIN, 0);
    return ESP_OK;
}

esp_err_t init_sd_card(void)
{
    ESP_LOGI(TAG, "SDカード初期化開始");

    // 1. SDカードマウント設定
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // 2. SPIバス設定
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // 3. SPIバス初期化
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIバス初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ SPIバス初期化完了");

    // 4. SDカードホスト設定
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SPI_FREQ / 1000;

    // 5. SDカードスロット設定
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS;
    slot_config.host_id = SPI2_HOST;

    // 6. SDカードマウント
    ret = esp_vfs_fat_sdspi_mount(g_mount_point, &host, &slot_config,
                                   &mount_config, &g_sd_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "SDカードマウント失敗（カード未挿入？）");
        } else {
            ESP_LOGE(TAG, "SDカード初期化失敗: %s", esp_err_to_name(ret));
        }
        spi_bus_free(SPI2_HOST);
        return ret;
    }

    g_sd_card_mounted = true;
    ESP_LOGI(TAG, "✅ SDカードマウント成功");
    return ESP_OK;
}

void print_sd_card_info(void)
{
    if (!g_sd_card_mounted || g_sd_card == NULL)
    {
        ESP_LOGW(TAG, "SDカードがマウントされていません");
        return;
    }

    ESP_LOGI(TAG, "=== SDカード情報 ===");
    ESP_LOGI(TAG, "名前: %s", g_sd_card->cid.name);
    ESP_LOGI(TAG, "速度: %" PRIu32 " kHz", g_sd_card->max_freq_khz);
    ESP_LOGI(TAG, "容量: %llu MB",
             ((uint64_t)g_sd_card->csd.capacity) * g_sd_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "セクタサイズ: %d", g_sd_card->csd.sector_size);
}


// ボタン処理関数群（省略、元のファイルと同じ）
void update_button_state(button_state_t *button)
{
    bool current_gpio_state = gpio_get_level(button->pin) == 0;
    uint32_t current_time = esp_timer_get_time() / 1000;

    if (abs((int)(current_time - button->last_change_time)) < BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    if (current_gpio_state != button->current_state)
    {
        button->last_state = button->current_state;
        button->current_state = current_gpio_state;
        button->last_change_time = current_time;

        if (button->current_state)
        {
            button->press_start_time = current_time;
            button->long_press_triggered = false;
            ESP_LOGD(TAG, "%s ボタン押下開始", button->name);
        }
        else
        {
            ESP_LOGD(TAG, "%s ボタン押下終了", button->name);
        }
    }

    if (button->current_state &&
        !button->long_press_triggered &&
        (current_time - button->press_start_time) >= BUTTON_LONG_PRESS_MS)
    {
        button->long_press_triggered = true;
        ESP_LOGD(TAG, "%s ボタン長押し検出", button->name);
    }
}

button_event_t get_button_event(button_state_t *button)
{
    if (button->current_state && button->long_press_triggered)
    {
        button->long_press_triggered = false;
        return BUTTON_EVENT_LONG_PRESS;
    }

    if (!button->current_state && button->last_state)
    {
        uint32_t press_duration = button->last_change_time - button->press_start_time;
        if (press_duration < BUTTON_LONG_PRESS_MS)
        {
            return BUTTON_EVENT_SHORT_PRESS;
        }
    }

    return BUTTON_EVENT_NONE;
}

void process_button_events(void)
{
    button_event_t shutter_event = get_button_event(&g_shutter_button);
    switch (shutter_event)
    {
    case BUTTON_EVENT_SHORT_PRESS:
        ESP_LOGI(TAG, "📸 シャッター短押し: ディスプレイテスト再実行");

        break;
    case BUTTON_EVENT_LONG_PRESS:
        ESP_LOGI(TAG, "📸 シャッター長押し: 全パレット撮影");
        break;
    default:
        break;
    }

    button_event_t menu_event = get_button_event(&g_menu_button);
    switch (menu_event)
    {
    case BUTTON_EVENT_SHORT_PRESS:
        ESP_LOGI(TAG, "🎨 メニュー短押し: LEDテスト実行");
        if (g_encoder_ready)
        {
            ESP_LOGI(TAG, "🌈 RGB LEDテスト開始");
            for (int i = 0; i < 8; i++)
            {
                uint32_t color = PALETTE_REP_COLORS[i];
                uint8_t r = (color >> 16) & 0xFF;
                uint8_t g = (color >> 8) & 0xFF;
                uint8_t b = color & 0xFF;

                pimoroni_encoder_set_led(&g_encoder, r, g, b);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            uint32_t current_color = PALETTE_REP_COLORS[g_current_palette_index];
            uint8_t r = (current_color >> 16) & 0xFF;
            uint8_t g = (current_color >> 8) & 0xFF;
            uint8_t b = current_color & 0xFF;
            pimoroni_encoder_set_led(&g_encoder, r, g, b);
        }
        break;
    case BUTTON_EVENT_LONG_PRESS:
        ESP_LOGI(TAG, "ℹ️ メニュー長押し: システム情報表示");
        ESP_LOGI(TAG, "=== システム情報 ===");
        ESP_LOGI(TAG, "カメラ: %s", g_camera_ready ? "OK" : "NG");
        ESP_LOGI(TAG, "エンコーダー: %s", g_encoder_ready ? "OK" : "NG");
        ESP_LOGI(TAG, "ディスプレイ: %s", g_display_ready ? "OK" : "NG");
        ESP_LOGI(TAG, "SDカード: %s", g_sd_card_ready ? "OK" : "NG");
        ESP_LOGI(TAG, "現在のパレット: %d", g_current_palette_index);
        break;
    default:
        break;
    }
}

// ★★★ チャタリング対策を追加したエンコーダータスク ★★★
void encoder_task(void *parameter)
{
    ESP_LOGI(TAG, "🔄 エンコーダータスク開始（デバウンス: %dms, ポーリング: %dms）",
             ENCODER_DEBOUNCE_MS, ENCODER_POLL_INTERVAL_MS);

    int16_t last_encoder_value = 0;
    uint32_t last_update_time = 0;
    uint32_t last_change_time = 0;  // チャタリング対策用: 最後に値が変化した時刻

    while (1)
    {
        if (g_encoder_ready)
        {
            // エンコーダー値を読み取り（新APIを使用）
            int16_t current_value = pimoroni_encoder_read(&g_encoder);
            uint32_t current_time = esp_timer_get_time() / 1000;

            // 値をパレットインデックス範囲に制限
            current_value = current_value % MAX_PALETTE_INDEX;
            if (current_value < 0)
                current_value += MAX_PALETTE_INDEX;

            // チャタリング対策: デバウンス時間をチェック
            uint32_t time_since_last_change = current_time - last_change_time;

            // 値が変化した場合の処理
            if (current_value != last_encoder_value)
            {
                // デバウンス時間が経過している場合のみ処理
                if (time_since_last_change >= ENCODER_DEBOUNCE_MS)
                {
                    int16_t delta = current_value - last_encoder_value;

                    // 最後の変化時刻を更新
                    last_change_time = current_time;

                    ESP_LOGI(TAG, "🔄 エンコーダー値変更: %d → %d (Δ%d)",
                             last_encoder_value, current_value, delta);

                    // パレットインデックスを更新
                    g_current_palette_index = current_value;

                    // LEDの色を変更（新APIを使用）
                    uint32_t color = PALETTE_REP_COLORS[current_value];
                    uint8_t r = (color >> 16) & 0xFF;
                    uint8_t g = (color >> 8) & 0xFF;
                    uint8_t b = color & 0xFF;

                    esp_err_t led_result = pimoroni_encoder_set_led(&g_encoder, r, g, b);

                    if (led_result == ESP_OK)
                    {
                        ESP_LOGI(TAG, "🎨 LED色変更成功: パレット%d → RGB(%d,%d,%d)",
                                 current_value, r, g, b);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "LED色変更失敗: %s", esp_err_to_name(led_result));
                    }

                    // ディスプレイを更新（簡単なステータス表示）
                    if (g_display_ready && (current_time - last_update_time) > 100)
                    {
                        g_display->clear();

                        // パレット番号を表示（簡単なパターンで）
                        int palette_dots = current_value + 1;
                        for (int i = 0; i < palette_dots && i < 8; i++)
                        {
                            g_display->set_pixel(10 + i * 8, 10, true);
                        }

                        // RGB値を簡単なバーで表示
                        for (int i = 0; i < r / 8 && i < 32; i++)
                        {
                            g_display->set_pixel(10 + i, 25, true); // R
                        }
                        for (int i = 0; i < g / 8 && i < 32; i++)
                        {
                            g_display->set_pixel(10 + i, 35, true); // G
                        }
                        for (int i = 0; i < b / 8 && i < 32; i++)
                        {
                            g_display->set_pixel(10 + i, 45, true); // B
                        }

                        g_display->display();
                        last_update_time = current_time;
                    }

                    // エンコーダーイベントをキューに送信
                    if (g_encoder_event_queue != NULL)
                    {
                        encoder_event_t event = {
                            .value = current_value,
                            .delta = delta,
                            .timestamp = current_time};

                        if (xQueueSend(g_encoder_event_queue, &event, 0) != pdTRUE)
                        {
                            ESP_LOGW(TAG, "エンコーダーイベントキュー満杯");
                        }
                    }

                    last_encoder_value = current_value;
                }
                else
                {
                    // デバウンス時間内の変化は無視（デバッグログ出力）
                    ESP_LOGD(TAG, "エンコーダー値変化を無視（デバウンス中: %ldms経過）",
                             time_since_last_change);
                }
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // ポーリング間隔を延長してノイズを低減
        vTaskDelay(pdMS_TO_TICKS(ENCODER_POLL_INTERVAL_MS));
    }
}

// ★★★ カメラプレビュータスク ★★★
void camera_preview_task(void *parameter)
{
    ESP_LOGI(TAG, "📷 カメラプレビュータスク開始");

    const int PREVIEW_SIZE = 64;  // プレビューサイズ 64x64
    const int UPDATE_INTERVAL_MS = 100;  // 更新間隔

    // プレビュー用バッファ (64x64の2値データ、1ビット/ピクセル)
    uint8_t *preview_buffer = (uint8_t *)heap_caps_malloc(PREVIEW_SIZE * PREVIEW_SIZE, MALLOC_CAP_8BIT);
    if (preview_buffer == NULL)
    {
        ESP_LOGE(TAG, "❌ プレビューバッファ確保失敗");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        if (g_camera_ready && g_display_ready)
        {
            // カメラからフレーム取得
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb == NULL)
            {
                ESP_LOGW(TAG, "カメラフレーム取得失敗");
                vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
                continue;
            }

            // 元画像のサイズを取得
            int src_width = fb->width;
            int src_height = fb->height;

            // スケーリング計算（短い辺を64pxにする）
            float scale_w = (float)PREVIEW_SIZE / src_width;
            float scale_h = (float)PREVIEW_SIZE / src_height;
            float scale = (scale_w > scale_h) ? scale_w : scale_h;  // 大きい方を採用（短い辺を64pxに）

            int scaled_width = (int)(src_width * scale);
            int scaled_height = (int)(src_height * scale);

            // トリミングオフセット（センタートリミング）
            int offset_x = (scaled_width - PREVIEW_SIZE) / 2;
            int offset_y = (scaled_height - PREVIEW_SIZE) / 2;

            // プレビュー画像生成（スケーリング + トリミング + 2値化）
            for (int y = 0; y < PREVIEW_SIZE; y++)
            {
                for (int x = 0; x < PREVIEW_SIZE; x++)
                {
                    // スケールバック（プレビュー座標→元画像座標）
                    int src_x = (int)((x + offset_x) / scale);
                    int src_y = (int)((y + offset_y) / scale);

                    // 範囲チェック
                    if (src_x < 0) src_x = 0;
                    if (src_x >= src_width) src_x = src_width - 1;
                    if (src_y < 0) src_y = 0;
                    if (src_y >= src_height) src_y = src_height - 1;

                    // ピクセル値取得（RGB565フォーマットを想定）
                    int pixel_index = src_y * src_width + src_x;
                    uint8_t r, g, b;

                    if (fb->format == PIXFORMAT_RGB565)
                    {
                        uint16_t pixel = ((uint16_t *)fb->buf)[pixel_index];
                        r = ((pixel >> 11) & 0x1F) << 3;
                        g = ((pixel >> 5) & 0x3F) << 2;
                        b = (pixel & 0x1F) << 3;
                    }
                    else if (fb->format == PIXFORMAT_GRAYSCALE)
                    {
                        r = g = b = fb->buf[pixel_index];
                    }
                    else
                    {
                        // その他のフォーマットは未対応（黒にする）
                        r = g = b = 0;
                    }

                    // 輝度計算（ITU-R BT.601）
                    uint8_t luminance = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);

                    // 2値化（閾値128）
                    preview_buffer[y * PREVIEW_SIZE + x] = (luminance >= 128) ? 1 : 0;
                }
            }

            // フレームバッファ解放
            esp_camera_fb_return(fb);

            // ディスプレイに描画
            g_display->clear();

            for (int y = 0; y < PREVIEW_SIZE; y++)
            {
                for (int x = 0; x < PREVIEW_SIZE; x++)
                {
                    bool pixel = preview_buffer[y * PREVIEW_SIZE + x];
                    g_display->set_pixel(x, y, pixel);
                }
            }

            g_display->display();
        }
        else
        {
            // カメラまたはディスプレイが準備できていない
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }

    // クリーンアップ（到達しない）
    if (preview_buffer != NULL)
    {
        heap_caps_free(preview_buffer);
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "\n🎮 ===== AtomS3R ピクセルアートカメラ起動 =====");
    ESP_LOGI(TAG, "ESP-IDF v%s", esp_get_idf_version());

    // ===== 基本システム初期化 =====

    // NVS初期化
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    ESP_LOGI(TAG, "✅ NVS初期化成功");

    // I2C初期化（最優先）
    esp_err_t i2c_result = init_external_i2c();
    if (i2c_result != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ I2C初期化失敗 - システム停止");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "✅ I2C初期化成功");

    // ディスプレイ初期化（2番目）
    ESP_LOGI(TAG, "📺 SSD1306ディスプレイ初期化中...");
    g_display = new SSD1306Display(EXTERNAL_I2C_NUM, SSD1306_DEFAULT_ADDR);
    if (g_display == nullptr)
    {
        ESP_LOGE(TAG, "❌ ディスプレイオブジェクト作成失敗 - システム停止");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_err_t display_result = g_display->init();
    if (display_result != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ ディスプレイ初期化失敗 - システム停止");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "✅ ディスプレイ初期化成功");
    g_display_ready = true;

    // ===== スプラッシュ画面表示 =====
    Terminal terminal(64, 3);
    terminal.init();
    terminal.set_position(0, 40); // ロゴの下（Y=40）
    terminal.set_border(false);

    g_display->clear();
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 0, false);
    g_display->terminal_println(&terminal, "PICT Camera v1.0");
    g_display->draw_terminal(&terminal);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(500));

    // ===== 初期化シーケンス開始 =====

    // GPIO初期化

    display_init_step(&terminal, " GPIO init");
    esp_err_t gpio_result = init_gpio();
    display_init_step(&terminal, gpio_result == ESP_OK);

    // カメラ初期化
    display_init_step(&terminal, " Camera init");
    g_camera_utils = new CameraUtils();
    bool camera_ok = false;
    if (g_camera_utils != nullptr)
    {
        esp_err_t camera_result = g_camera_utils->init();
        camera_ok = (camera_result == ESP_OK);
        g_camera_ready = camera_ok;
    }
    display_init_step(&terminal, camera_ok);

    // SDカード初期化
    display_init_step(&terminal, " SD Card init");
    esp_err_t sd_result = init_sd_card();
    g_sd_card_ready = (sd_result == ESP_OK);
    display_init_step(&terminal, g_sd_card_ready);

    if (g_sd_card_ready)
    {
        print_sd_card_info();
    }

    // エンコーダー初期化
    display_init_step(&terminal, " Encoder init");
    pimoroni_encoder_config_t encoder_config = pimoroni_encoder_get_default_config(EXTERNAL_I2C_NUM);
    encoder_config.i2c_address = PIMORONI_ENCODER_I2C_ADDR;
    encoder_config.direction = PIMORONI_ENCODER_CW;
    encoder_config.brightness = 1.0f;
    encoder_config.interrupt_pin = GPIO_NUM_NC;
    encoder_config.skip_chip_id_check = false;

    esp_err_t encoder_result = pimoroni_encoder_init(&g_encoder, &encoder_config);
    g_encoder_ready = (encoder_result == ESP_OK);
    display_init_step(&terminal, g_encoder_ready);

    // エンコーダーLED初期設定
    if (g_encoder_ready)
    {
        uint32_t initial_color = PALETTE_REP_COLORS[0];
        uint8_t r = (initial_color >> 16) & 0xFF;
        uint8_t g = (initial_color >> 8) & 0xFF;
        uint8_t b = initial_color & 0xFF;
        pimoroni_encoder_set_led(&g_encoder, r, g, b);
    }

    // キュー作成
    display_init_step(&terminal, " Queue create");
    g_encoder_event_queue = xQueueCreate(10, sizeof(encoder_event_t));
    display_init_step(&terminal, g_encoder_event_queue != NULL);

    // エンコーダータスク開始
    display_init_step(&terminal, " Task start");
    BaseType_t task_result = pdFAIL;
    if (g_encoder_ready)
    {
        task_result = xTaskCreate(encoder_task, "encoder_task",
                                  ENCODER_TASK_STACK, NULL,
                                  tskIDLE_PRIORITY + 2, NULL);
    }
    display_init_step(&terminal, task_result == pdPASS);

    // LCDクリア
    if (g_display_ready)
    {
        g_display->clear();
        g_display->display();
    }

    // カメラプレビュータスク開始
    if (g_camera_ready && g_display_ready)
    {
        BaseType_t camera_task_result = xTaskCreate(
            camera_preview_task,
            "camera_preview",
            8192,  // スタックサイズ
            NULL,
            tskIDLE_PRIORITY + 1,  // エンコーダーより低い優先度
            NULL
        );

        if (camera_task_result == pdPASS)
        {
            ESP_LOGI(TAG, "✅ カメラプレビュータスク起動成功");
        }
        else
        {
            ESP_LOGE(TAG, "❌ カメラプレビュータスク起動失敗");
        }
    }

    // ===== 初期化完了 =====
    g_system_ready = true;
    ESP_LOGI(TAG, "🎉 システム初期化完了！");

    // 完了メッセージ表示（2秒）
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "🔘 メインループ開始");

    // メインループ
    while (1)
    {
        update_button_state(&g_shutter_button);
        update_button_state(&g_menu_button);
        process_button_events();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}