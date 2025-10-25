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
//#include "PixelArtProcessor.h"
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

// 構造体定義など... (省略、元のファイルと同じ)
typedef struct {
    gpio_num_t pin;
    bool current_state;
    bool last_state;
    uint32_t press_start_time;
    uint32_t last_change_time;
    bool long_press_triggered;
    const char* name;
} button_state_t;

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_PRESS_START,
    BUTTON_EVENT_PRESS_END
} button_event_t;

typedef struct {
    int16_t value;
    int16_t delta;
    uint32_t timestamp;
} encoder_event_t;

typedef enum {
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
    .name = "Shutter"
};

static button_state_t g_menu_button = {
    .pin = MENU_BUTTON_PIN,
    .current_state = false,
    .last_state = false,
    .press_start_time = 0,
    .last_change_time = 0,
    .long_press_triggered = false,
    .name = "Menu"
};

static volatile int g_current_palette_index = 0;
static volatile int g_file_counter = 1;
static volatile uint32_t g_last_button_press = 0;

// ハードウェアオブジェクト - エンコーダーのみC構造体に変更
static pimoroni_encoder_t g_encoder;
static CameraUtils* g_camera_utils = nullptr;
static SSD1306Display* g_display = nullptr;

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
    0x8D697A, 0x437290, 0x498DB7, 0x8080A4,
    0xC6BAAC, 0x9AA57C, 0x38D88E, 0xFFD159,
};

// 関数プロトタイプ
void update_button_state(button_state_t* button);
button_event_t get_button_event(button_state_t* button);
void process_button_events(void);
void encoder_task(void* parameter);
esp_err_t init_sd_card(void);
void print_sd_card_info(void);
esp_err_t init_external_i2c(void);
esp_err_t scan_i2c_devices(i2c_port_t i2c_num, int *found_devices);
esp_err_t init_gpio(void);
void run_display_test_patterns(void);

// ★★★ 新規追加: SSD1306表示テスト関数 ★★★
void run_display_test_patterns(void)
{
    if (!g_display_ready || g_display == nullptr) {
        ESP_LOGE(TAG, "❌ ディスプレイが準備できていません");
        return;
    }

    ESP_LOGI(TAG, "🎨 ディスプレイ描画テスト開始");

    // テスト1: ビットマップ画像表示（通常）
    ESP_LOGI(TAG, "テスト1: ビットマップ画像表示（通常）");
    g_display->clear();
    g_display->draw_rect(0, 0, 128, 64, true, false);
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 0, false);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト2: ビットマップ画像表示（反転）
    ESP_LOGI(TAG, "テスト2: ビットマップ画像表示（反転）");
    g_display->clear();
    g_display->draw_rect(0, 0, 128, 64, true, false);
    g_display->draw_bitmap(image_logo, IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT, 0, 12, true);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト3: 美咲フォント文字描画
    ESP_LOGI(TAG, "テスト3: 美咲フォント文字描画");
    g_display->clear();
    g_display->draw_rect(0, 0, 128, 64, true, false);

    // ランダムな位置にランダムな文字を表示（10文字）
    for (int i = 0; i < 10; i++) {
        uint16_t char_index = esp_random() % MISAKI_TOTAL_CHARS;
        int16_t x = 1 + (esp_random() % 119);
        int16_t y = 1 + (esp_random() % 55);
        g_display->draw_char(char_index, x, y, true);
    }

    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト4: ターミナル表示（枠線なし）
    ESP_LOGI(TAG, "テスト4: ターミナル表示（枠線なし）");
    Terminal terminal;
    terminal.init();
    terminal.set_position(0, 0);
    terminal.set_border(false);

    // いくつかの文字を配置
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 16; col++) {
            // チェッカーパターン
            uint16_t char_index = ((row + col) % 2) ? 100 : 200;
            terminal.set_char(row, col, char_index);
        }
    }

    g_display->clear();
    g_display->draw_terminal(&terminal);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト5: ターミナル表示（枠線あり）
    ESP_LOGI(TAG, "テスト5: ターミナル表示（枠線あり）");
    terminal.clear();
    terminal.set_border(true);

    // print_char()を使って文字を出力
    for (int i = 0; i < 50; i++) {
        uint16_t char_index = 100 + (i % 200);
        terminal.print_char(char_index);
    }

    g_display->clear();
    g_display->draw_terminal(&terminal);
    g_display->display();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // テスト6: ターミナルスクロールテスト
    ESP_LOGI(TAG, "テスト6: ターミナルスクロールテスト");
    terminal.clear();
    terminal.set_border(true);

    // 1行ずつ追加してスクロールを確認
    for (int line = 0; line < 12; line++) {
        for (int col = 0; col < 18; col++) {
            terminal.print_char(300 + (line * 10 + col) % 400);
        }
        terminal.newline();

        g_display->clear();
        g_display->draw_terminal(&terminal);
        g_display->display();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "✅ ディスプレイ描画テスト完了");
}

// ボタン処理関数群（省略、元のファイルと同じ）
void update_button_state(button_state_t* button)
{
    bool current_gpio_state = gpio_get_level(button->pin) == 0;
    uint32_t current_time = esp_timer_get_time() / 1000;

    if (abs((int)(current_time - button->last_change_time)) < BUTTON_DEBOUNCE_MS) {
        return;
    }

    if (current_gpio_state != button->current_state) {
        button->last_state = button->current_state;
        button->current_state = current_gpio_state;
        button->last_change_time = current_time;

        if (button->current_state) {
            button->press_start_time = current_time;
            button->long_press_triggered = false;
            ESP_LOGD(TAG, "%s ボタン押下開始", button->name);
        } else {
            ESP_LOGD(TAG, "%s ボタン押下終了", button->name);
        }
    }

    if (button->current_state && 
        !button->long_press_triggered && 
        (current_time - button->press_start_time) >= BUTTON_LONG_PRESS_MS) {
        button->long_press_triggered = true;
        ESP_LOGD(TAG, "%s ボタン長押し検出", button->name);
    }
}

button_event_t get_button_event(button_state_t* button)
{
    if (button->current_state && button->long_press_triggered) {
        button->long_press_triggered = false;
        return BUTTON_EVENT_LONG_PRESS;
    }

    if (!button->current_state && button->last_state) {
        uint32_t press_duration = button->last_change_time - button->press_start_time;
        if (press_duration < BUTTON_LONG_PRESS_MS) {
            return BUTTON_EVENT_SHORT_PRESS;
        }
    }

    return BUTTON_EVENT_NONE;
}

void process_button_events(void)
{
    button_event_t shutter_event = get_button_event(&g_shutter_button);
    switch (shutter_event) {
        case BUTTON_EVENT_SHORT_PRESS:
            ESP_LOGI(TAG, "📸 シャッター短押し: ディスプレイテスト再実行");
            if (g_display_ready) {
                run_display_test_patterns();
            }
            break;
        case BUTTON_EVENT_LONG_PRESS:
            ESP_LOGI(TAG, "📸 シャッター長押し: 全パレット撮影");
            break;
        default:
            break;
    }

    button_event_t menu_event = get_button_event(&g_menu_button);
    switch (menu_event) {
        case BUTTON_EVENT_SHORT_PRESS:
            ESP_LOGI(TAG, "🎨 メニュー短押し: LEDテスト実行");
            if (g_encoder_ready) {
                ESP_LOGI(TAG, "🌈 RGB LEDテスト開始");
                for (int i = 0; i < 8; i++) {
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

// ★★★ 修正されたエンコーダータスク ★★★
void encoder_task(void* parameter)
{
    ESP_LOGI(TAG, "🔄 エンコーダータスク開始");
    
    int16_t last_encoder_value = 0;
    uint32_t last_update_time = 0;
    
    while (1) {
        if (g_encoder_ready) {
            // エンコーダー値を読み取り（新APIを使用）
            int16_t current_value = pimoroni_encoder_read(&g_encoder);
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            // 値をパレットインデックス範囲に制限
            current_value = current_value % MAX_PALETTE_INDEX;
            if (current_value < 0) current_value += MAX_PALETTE_INDEX;
            
            // 値が変化した場合の処理
            if (current_value != last_encoder_value) {
                int16_t delta = current_value - last_encoder_value;
                
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
                
                if (led_result == ESP_OK) {
                    ESP_LOGI(TAG, "🎨 LED色変更成功: パレット%d → RGB(%d,%d,%d)", 
                            current_value, r, g, b);
                } else {
                    ESP_LOGW(TAG, "LED色変更失敗: %s", esp_err_to_name(led_result));
                }
                
                // ディスプレイを更新（簡単なステータス表示）
                if (g_display_ready && (current_time - last_update_time) > 100) {
                    g_display->clear();
                    
                    // パレット番号を表示（簡単なパターンで）
                    int palette_dots = current_value + 1;
                    for (int i = 0; i < palette_dots && i < 8; i++) {
                        g_display->set_pixel(10 + i * 8, 10, true);
                    }
                    
                    // RGB値を簡単なバーで表示
                    for (int i = 0; i < r/8 && i < 32; i++) {
                        g_display->set_pixel(10 + i, 25, true); // R
                    }
                    for (int i = 0; i < g/8 && i < 32; i++) {
                        g_display->set_pixel(10 + i, 35, true); // G
                    }
                    for (int i = 0; i < b/8 && i < 32; i++) {
                        g_display->set_pixel(10 + i, 45, true); // B
                    }
                    
                    g_display->display();
                    last_update_time = current_time;
                }
                
                // エンコーダーイベントをキューに送信
                if (g_encoder_event_queue != NULL) {
                    encoder_event_t event = {
                        .value = current_value,
                        .delta = delta,
                        .timestamp = current_time
                    };
                    
                    if (xQueueSend(g_encoder_event_queue, &event, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "エンコーダーイベントキュー満杯");
                    }
                }
                
                last_encoder_value = current_value;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 他の関数は省略（元のファイルと同じ）
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
    if (ret != ESP_OK) return ret;

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
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) return ret;

    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&led_config);
    if (ret != ESP_OK) return ret;

    gpio_set_level(LED_PIN, 0);
    return ESP_OK;
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "\n🎮 ===== AtomS3R ピクセルアートカメラ起動 =====");
    ESP_LOGI(TAG, "ESP-IDF v%s", esp_get_idf_version());

    // 基本システム初期化
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);

    esp_err_t gpio_result = init_gpio();
    if (gpio_result == ESP_OK) {
        ESP_LOGI(TAG, "✅ GPIO初期化成功");
    }

    // カメラ初期化（省略）
    g_camera_utils = new CameraUtils();
    if (g_camera_utils != nullptr) {
        esp_err_t camera_result = g_camera_utils->init();
        g_camera_ready = (camera_result == ESP_OK);
    }

    // 同期オブジェクト作成
    g_encoder_event_queue = xQueueCreate(10, sizeof(encoder_event_t));

    // I2C初期化
    esp_err_t external_i2c_result = init_external_i2c();
    if (external_i2c_result == ESP_OK) {
        ESP_LOGI(TAG, "✅ 外部装置用I2C初期化成功");
    }

    // ★★★ 修正されたエンコーダー初期化 ★★★
    ESP_LOGI(TAG, "🔄 Pimoroniエンコーダー初期化中...");
    ESP_LOGI(TAG, "   I2Cアドレス: 0x%02X", PIMORONI_ENCODER_I2C_ADDR);

    // エンコーダー設定構造体作成
    pimoroni_encoder_config_t encoder_config = pimoroni_encoder_get_default_config(EXTERNAL_I2C_NUM);
    encoder_config.i2c_address = PIMORONI_ENCODER_I2C_ADDR;
    encoder_config.direction = PIMORONI_ENCODER_CW;
    encoder_config.brightness = 1.0f;
    encoder_config.interrupt_pin = GPIO_NUM_NC;
    encoder_config.skip_chip_id_check = false;

    esp_err_t encoder_result = pimoroni_encoder_init(&g_encoder, &encoder_config);
    if (encoder_result == ESP_OK) {
        ESP_LOGI(TAG, "✅ エンコーダー初期化成功");
        g_encoder_ready = true;

        // 初期LED色設定
        pimoroni_encoder_set_led(&g_encoder, 0, 64, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // LEDテスト
        ESP_LOGI(TAG, "🎨 エンコーダーLEDテスト開始");
        for (int i = 0; i < 8; i++) {
            uint32_t color = PALETTE_REP_COLORS[i];
            uint8_t r = (color >> 16) & 0xFF;
            uint8_t g = (color >> 8) & 0xFF;
            uint8_t b = color & 0xFF;
            
            pimoroni_encoder_set_led(&g_encoder, r, g, b);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        // 初期パレット色に設定
        uint32_t initial_color = PALETTE_REP_COLORS[0];
        uint8_t r = (initial_color >> 16) & 0xFF;
        uint8_t g = (initial_color >> 8) & 0xFF;
        uint8_t b = initial_color & 0xFF;
        pimoroni_encoder_set_led(&g_encoder, r, g, b);
    } else {
        ESP_LOGE(TAG, "❌ エンコーダー初期化失敗: %s", esp_err_to_name(encoder_result));
        g_encoder_ready = false;
    }

    // ★★★ SSD1306ディスプレイ初期化とテストパターン表示 ★★★
    ESP_LOGI(TAG, "📺 SSD1306ディスプレイ初期化中...");
    g_display = new SSD1306Display(EXTERNAL_I2C_NUM, SSD1306_DEFAULT_ADDR);
    if (g_display != nullptr) {
        esp_err_t display_result = g_display->init();
        if (display_result == ESP_OK) {
            ESP_LOGI(TAG, "✅ ディスプレイ初期化成功");
            g_display_ready = true;
            
            // ディスプレイ動作確認用テストパターン表示
            ESP_LOGI(TAG, "🎨 ディスプレイテストパターン実行中...");
            run_display_test_patterns();
            
        } else {
            ESP_LOGE(TAG, "❌ ディスプレイ初期化失敗: %s", esp_err_to_name(display_result));
        }
    } else {
        ESP_LOGE(TAG, "❌ ディスプレイオブジェクト作成失敗");
    }

    // システム準備完了
    g_system_ready = true;
    ESP_LOGI(TAG, "🎉 システム初期化完了！");

    // エンコーダータスク開始
    if (g_encoder_ready) {
        xTaskCreate(encoder_task, "encoder_task", ENCODER_TASK_STACK, NULL, tskIDLE_PRIORITY + 2, NULL);
        ESP_LOGI(TAG, "✅ エンコーダータスク作成成功");
    }

    ESP_LOGI(TAG, "🔘 ボタンテストモード開始");
    ESP_LOGI(TAG, "   シャッター短押し: ディスプレイテスト再実行");
    ESP_LOGI(TAG, "   メニュー短押し: LEDテスト実行");
    ESP_LOGI(TAG, "   メニュー長押し: システム情報表示");

    // メインループ
    while (1) {
        update_button_state(&g_shutter_button);
        update_button_state(&g_menu_button);
        process_button_events();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}