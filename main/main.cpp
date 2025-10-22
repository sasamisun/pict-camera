/*
 * AtomS3R ピクセルアートカメラ (ESP-IDF 5.4完全対応版)
 *
 * ToDo:
 * G38 G39プルアップボタン入力（シャッターボタン・メニューボタン）→完了
 * encoder 入力、LEDテスト→実装中
 * LCD表示テスト フォント準備 M5GFX
 * カメラからLCD表示
 * カメラ画像をカラーパレットをもとに変換（短押し１パレット、長押し全パレット）
 * カメラ画像をSDカードに保存
 * エンコーダで露出補正、明度マップ表示
 * メニュー作成　メニューボタンで移行　シャッターボタンで決定　エンコーダで選択　メニューボタンで戻る
 * 　メニュー内容（Photos,Pallets,Resolution,
 * 
 * 以下、ふわふわ要件
 * RFIDで画像送信機能
 * 吹き出し機能
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>

// ESP-IDF 5.4コア
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
// ESP-IDF 5.4では esp_psram.h の代わりに esp_heap_caps.h を使う
#include "esp_heap_caps.h"
#include "esp_chip_info.h" // esp_chip_info用

// ドライバー (ESP-IDF 5.4対応)
#include "driver/gpio.h"
#include "driver/spi_master.h"
// #include "driver/i2c_master.h"
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

// カスタムクラス
#include "PimoroniEncoder.h"
//#include "PixelArtProcessor.h"
#include "camera_utils.h"
#include "ssd1306_display.h"

// ========================================
// 定数定義
// ========================================
static const char *TAG = "PixelArtCamera";

// ピン設定（AtomS3R専用）
#define SPI_SCK_PIN GPIO_NUM_7 // SDカード用SPI
#define SPI_MISO_PIN GPIO_NUM_8
#define SPI_MOSI_PIN GPIO_NUM_6
#define SPI_CS_PIN GPIO_NUM_15

#define SHUTTER_BUTTON_PIN GPIO_NUM_38   // 撮影ボタン（プルアップ、押下でLOW）
#define MENU_BUTTON_PIN GPIO_NUM_39      // メニューボタン（プルアップ、押下でLOW）
#define LED_PIN GPIO_NUM_39              // 状態LED

// I2C設定
#define I2C_SDA_PIN GPIO_NUM_1 // 外部I2C SDA
#define I2C_SCL_PIN GPIO_NUM_2 // 外部I2C SCL
#define I2C_FREQ_HZ 400000     // 400kHz

// 外部装置I2C設定
#define EXTERNAL_I2C_NUM I2C_NUM_1  // 外部装置専用I2C
#define EXTERNAL_I2C_FREQ_HZ 400000 // 外部装置は400kHzで高速動作

// ボタン設定
#define BUTTON_LONG_PRESS_MS 1000    // 長押し判定時間
#define BUTTON_DEBOUNCE_MS 50        // チャタリング防止時間
#define BUTTON_REPEAT_DELAY_MS 500   // 連続押し間隔

// その他設定
#define DEBOUNCE_DELAY_MS 300    // ボタンチャタリング防止
#define MAX_PALETTE_INDEX 8      // パレット数
#define IMAGE_WIDTH 240          // 画像幅
#define IMAGE_HEIGHT 176         // 画像高
#define CAPTURE_TASK_STACK 8192  // タスクスタックサイズ
#define PROCESS_TASK_STACK 16384 // 画像処理用大きめスタック
#define ENCODER_TASK_STACK 4096  // エンコーダータスクスタック

// ========================================
// ボタン状態管理構造体
// ========================================
typedef struct {
    gpio_num_t pin;                    // GPIOピン番号
    bool current_state;                // 現在の状態（true=押下、false=離す）
    bool last_state;                   // 前回の状態
    uint32_t press_start_time;         // 押下開始時刻
    uint32_t last_change_time;         // 最後の状態変化時刻
    bool long_press_triggered;         // 長押しイベント発生済みフラグ
    const char* name;                  // ボタン名（デバッグ用）
} button_state_t;

// ========================================
// ボタンイベント定義
// ========================================
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,    // 短押し
    BUTTON_EVENT_LONG_PRESS,     // 長押し
    BUTTON_EVENT_PRESS_START,    // 押下開始
    BUTTON_EVENT_PRESS_END       // 押下終了
} button_event_t;

// ========================================
// エンコーダーイベント定義
// ========================================
typedef struct {
    int16_t value;              // エンコーダー値
    int16_t delta;              // 前回からの変化量
    uint32_t timestamp;         // イベント発生時刻
} encoder_event_t;

// ========================================
// ステータスLED制御定義
// ========================================
typedef enum
{
    SYSTEM_STATUS_INITIALIZING, // 初期化中（赤点滅）
    SYSTEM_STATUS_READY,        // 待機中（青点灯）
    SYSTEM_STATUS_CAPTURING,    // 撮影中（赤点滅）
    SYSTEM_STATUS_PROCESSING,   // 処理中（赤点滅）
    SYSTEM_STATUS_SAVING,       // 保存中（赤点滅）
    SYSTEM_STATUS_ERROR         // エラー（赤高速点滅）
} system_status_t;

// ========================================
// グローバル変数
// ========================================

// 同期制御
static SemaphoreHandle_t g_capture_semaphore = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static QueueHandle_t g_capture_queue = NULL;
static QueueHandle_t g_encoder_event_queue = NULL;

// ボタン状態
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

// 状態管理
static volatile int g_current_palette_index = 0;
static volatile int g_file_counter = 1;
static volatile uint32_t g_last_button_press = 0;

// ハードウェアオブジェクト
static PimoroniEncoder* g_encoder = nullptr;
static CameraUtils* g_camera_utils = nullptr;
static SSD1306Display* g_display = nullptr;

// デバイス状態
static volatile bool g_system_ready = false;
static bool g_camera_ready = false;
static bool g_encoder_ready = false;
static bool g_display_ready = false;
static bool g_sd_card_ready = false;

// ステータスLED制御
static volatile system_status_t g_system_status = SYSTEM_STATUS_INITIALIZING;
static volatile bool g_status_led_enabled = true;

// SDカード
static sdmmc_card_t *g_sd_card = NULL;
static const char *g_mount_point = "/sdcard";
static bool g_sd_card_mounted = false;

// ========================================
// カラーパレット定義（8種類×8色）
// ========================================
static const uint32_t COLOR_PALETTES[8][8] = {
    {// パレット0: slso8 (暖色系)
     0x0D2B45, 0x203C56, 0x544E68, 0x8D697A,
     0xD08159, 0xFFAA5E, 0xFFD4A3, 0xFFECD6},
    {// パレット1: 都市伝説解体センター風 (モノクロ青)
     0x000000, 0x000B22, 0x112B43, 0x437290,
     0x437290, 0xE0D8D1, 0xE0D8D1, 0xFFFFFF},
    {// パレット2: ファミレス風 (青黄)
     0x010101, 0x33669F, 0x33669F, 0x33669F,
     0x498DB7, 0x498DB7, 0xFBE379, 0xFBE379},
    {// パレット3: ゴシックビット (グレー)
     0x0E0E12, 0x1A1A24, 0x333346, 0x535373,
     0x8080A4, 0xA6A6BF, 0xC1C1D2, 0xE6E6EC},
    {// パレット4: ノワール (コントラスト)
     0x1E1C32, 0x1E1C32, 0x1E1C32, 0x1E1C32,
     0xC6BAAC, 0xC6BAAC, 0xC6BAAC, 0xC6BAAC},
    {// パレット5: デミボーイ (オリーブ)
     0x252525, 0x252525, 0x4B564D, 0x4B564D,
     0x9AA57C, 0x9AA57C, 0xE0E9C4, 0xE0E9C4},
    {// パレット6: ディープメイズ (緑)
     0x001D2A, 0x085562, 0x009A98, 0x00BE91,
     0x38D88E, 0x9AF089, 0xF2FF66, 0xF2FF66},
    {// パレット7: ナイトレイン (レインボー)
     0x000000, 0x012036, 0x3A7BAA, 0x7D8FAE,
     0xA1B4C1, 0xF0B9B9, 0xFFD159, 0xFFFFFF},
};

// エンコーダLED用代表色（パレット選択時のみ使用）
static const uint32_t PALETTE_REP_COLORS[8] = {
    0x8D697A, // パレット0: ピンク系
    0x437290, // パレット1: 青系
    0x498DB7, // パレット2: 水色系
    0x8080A4, // パレット3: グレー系
    0xC6BAAC, // パレット4: ベージュ系
    0x9AA57C, // パレット5: オリーブ系
    0x38D88E, // パレット6: 緑系
    0xFFD159, // パレット7: 黄色系
};

// ========================================
// ボタン処理関数群
// ========================================

/**
 * ボタンの状態を更新する関数
 */
void update_button_state(button_state_t* button)
{
    // GPIO状態を読み取り（プルアップなので反転）
    bool pressed = !gpio_get_level(button->pin);
    uint32_t now = esp_timer_get_time() / 1000; // ミリ秒に変換
    
    // チャタリング防止
    if (now - button->last_change_time < BUTTON_DEBOUNCE_MS) {
        return;
    }
    
    // 状態変化検出
    if (pressed != button->current_state) {
        button->last_state = button->current_state;
        button->current_state = pressed;
        button->last_change_time = now;
        
        if (pressed) {
            // 押下開始
            button->press_start_time = now;
            button->long_press_triggered = false;
            ESP_LOGI(TAG, "🔘 %sボタン押下開始", button->name);
        } else {
            // 押下終了
            uint32_t press_duration = now - button->press_start_time;
            ESP_LOGI(TAG, "🔘 %sボタン押下終了 (持続時間: %lu ms)", button->name, (unsigned long)press_duration);
        }
    }
}

/**
 * ボタンイベントを処理する関数
 */
void process_button_events()
{
    uint32_t now = esp_timer_get_time() / 1000;
    
    // シャッターボタンの処理
    if (g_shutter_button.current_state) {
        uint32_t press_duration = now - g_shutter_button.press_start_time;
        
        // 長押し判定（一度だけ発火）
        if (press_duration >= BUTTON_LONG_PRESS_MS && !g_shutter_button.long_press_triggered) {
            g_shutter_button.long_press_triggered = true;
            ESP_LOGI(TAG, "📸 シャッターボタン長押し → 全パレット撮影");
            
            // エンコーダーLEDを黄色に設定（全パレット撮影モード）
            if (g_encoder_ready) {
                g_encoder->set_color(0xFFFF00);
            }
            
            // ディスプレイに撮影モード表示
            if (g_display_ready) {
                g_display->clear();
                g_display->draw_string(0, 0, "Capture Mode", true);
                g_display->draw_string(0, 16, "All Palettes", true);
                g_display->draw_string(0, 32, "Processing...", true);
                g_display->display();
            }
        }
    } else if (g_shutter_button.last_state && !g_shutter_button.current_state) {
        // ボタンが離された時の処理
        uint32_t press_duration = now - g_shutter_button.press_start_time;
        
        if (press_duration < BUTTON_LONG_PRESS_MS && !g_shutter_button.long_press_triggered) {
            // 短押し
            ESP_LOGI(TAG, "📸 シャッターボタン短押し → 単一パレット撮影 (パレット%d)", g_current_palette_index);
            
            // エンコーダーLEDを白色に設定（単一パレット撮影）
            if (g_encoder_ready) {
                g_encoder->set_color(0xFFFFFF);
                vTaskDelay(pdMS_TO_TICKS(200));
                g_encoder->set_color(PALETTE_REP_COLORS[g_current_palette_index]);
            }
            
            // ディスプレイに撮影情報表示
            if (g_display_ready) {
                g_display->clear();
                g_display->draw_string(0, 0, "Capture Mode", true);
                char palette_str[32];
                snprintf(palette_str, sizeof(palette_str), "Palette: %d", g_current_palette_index);
                g_display->draw_string(0, 16, palette_str, true);
                g_display->draw_string(0, 32, "Processing...", true);
                g_display->display();
            }
        }
        
        g_shutter_button.last_state = false;
    }
    
    // メニューボタンの処理
    if (g_menu_button.current_state) {
        uint32_t press_duration = now - g_menu_button.press_start_time;
        
        // 長押し判定
        if (press_duration >= BUTTON_LONG_PRESS_MS && !g_menu_button.long_press_triggered) {
            g_menu_button.long_press_triggered = true;
            ESP_LOGI(TAG, "⚙️ メニューボタン長押し → システム情報表示");
            
            // システム情報をディスプレイに表示
            if (g_display_ready) {
                g_display->clear();
                g_display->draw_string(0, 0, "System Info", true);
                
                char info_str[32];
                snprintf(info_str, sizeof(info_str), "Cam: %s", g_camera_ready ? "OK" : "NG");
                g_display->draw_string(0, 16, info_str, true);
                
                snprintf(info_str, sizeof(info_str), "Enc: %s", g_encoder_ready ? "OK" : "NG");
                g_display->draw_string(0, 32, info_str, true);
                
                snprintf(info_str, sizeof(info_str), "SD: %s", g_sd_card_ready ? "OK" : "NG");
                g_display->draw_string(0, 48, info_str, true);
                
                g_display->display();
            }
        }
    } else if (g_menu_button.last_state && !g_menu_button.current_state) {
        // ボタンが離された時の処理
        uint32_t press_duration = now - g_menu_button.press_start_time;
        
        if (press_duration < BUTTON_LONG_PRESS_MS && !g_menu_button.long_press_triggered) {
            // 短押し
            ESP_LOGI(TAG, "⚙️ メニューボタン短押し → 設定メニュー");
            
            // エンコーダーLEDテストを実行
            if (g_encoder_ready) {
                ESP_LOGI(TAG, "🎨 LEDテスト開始");
                g_encoder->test_led_colors();
                g_encoder->set_color(PALETTE_REP_COLORS[g_current_palette_index]);
            }
            
            // ディスプレイにメニュー表示
            if (g_display_ready) {
                g_display->clear();
                g_display->draw_string(0, 0, "Settings Menu", true);
                g_display->draw_string(0, 16, "LED Test Done", true);
                g_display->draw_string(0, 32, "Use encoder", true);
                g_display->draw_string(0, 48, "to select", true);
                g_display->display();
            }
        }
        
        g_menu_button.last_state = false;
    }
}

/**
 * GPIO初期化関数
 */
esp_err_t init_gpio()
{
    // ボタン用GPIO設定（プルアップ）
    gpio_config_t button_config = {};
    button_config.pin_bit_mask = (1ULL << SHUTTER_BUTTON_PIN) | (1ULL << MENU_BUTTON_PIN);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ボタンGPIO設定失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ ボタンGPIO初期化完了 (GPIO%d, GPIO%d)", SHUTTER_BUTTON_PIN, MENU_BUTTON_PIN);
    return ESP_OK;
}

// ========================================
// エンコーダータスク
// ========================================

/**
 * エンコーダー専用タスク関数
 * エンコーダーの値変化を監視し、LEDの色を更新する
 */
void encoder_task(void* parameter)
{
    ESP_LOGI(TAG, "🔄 エンコーダータスク開始");
    
    int16_t last_encoder_value = 0;
    uint32_t last_update_time = 0;
    
    // エンコーダー接続確認
    if (g_encoder_ready) {
        bool connection_ok = g_encoder->check_encoder_connection();
        ESP_LOGI(TAG, "エンコーダー接続状態: %s", connection_ok ? "OK" : "NG");
    }
    
    while (1) {
        if (g_encoder_ready) {
            // エンコーダー値を更新
            int16_t current_value = g_encoder->update();
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            // 値が変化した場合の処理
            if (current_value != last_encoder_value) {
                int16_t delta = current_value - last_encoder_value;
                
                ESP_LOGI(TAG, "🔄 エンコーダー値変更: %d → %d (Δ%d)", 
                        last_encoder_value, current_value, delta);
                
                // パレットインデックスを更新
                g_current_palette_index = current_value;
                
                // LEDの色を変更
                if (current_value < MAX_PALETTE_INDEX) {
                    uint32_t color = PALETTE_REP_COLORS[current_value];
                    esp_err_t led_result = g_encoder->set_color(color);
                    
                    if (led_result == ESP_OK) {
                        ESP_LOGI(TAG, "🎨 LED色変更成功: パレット%d → 0x%06lX", 
                                current_value, (unsigned long)color);
                    } else {
                        ESP_LOGW(TAG, "LED色変更失敗: %s", esp_err_to_name(led_result));
                    }
                }
                
                // ディスプレイを更新（頻繁な更新を防ぐため100ms間隔制限）
                if (g_display_ready && (current_time - last_update_time) > 100) {
                    g_display->clear();
                    g_display->draw_string(0, 0, "Palette Mode", true);
                    
                    char palette_str[32];
                    snprintf(palette_str, sizeof(palette_str), "Current: %d", current_value);
                    g_display->draw_string(0, 16, palette_str, true);
                    
                    char color_str[32];
                    snprintf(color_str, sizeof(color_str), "Color:0x%06lX", 
                            (unsigned long)PALETTE_REP_COLORS[current_value]);
                    g_display->draw_string(0, 32, color_str, true);
                    
                    g_display->draw_string(0, 48, "Press buttons", true);
                    g_display->display();
                    
                    last_update_time = current_time;
                }
                
                // エンコーダーイベントをキューに送信（他のタスクで使用可能）
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
            // エンコーダーが初期化されていない場合は長めに待機
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // エンコーダータスクは高い応答性のため10ms周期で実行
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========================================
// SDカード初期化関数
// ========================================

/**
 * SDカードを初期化する関数
 * @return ESP_OK: 成功, その他: エラーコード
 */
esp_err_t init_sd_card(void)
{
    esp_err_t ret;

    // SDカード用SPI設定
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SPI_MOSI_PIN;
    bus_cfg.miso_io_num = SPI_MISO_PIN;
    bus_cfg.sclk_io_num = SPI_SCK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    // SPIバス初期化
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // SDカードホスト設定
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = SPI2_HOST;

    // マウントオプション設定
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    // SDカードをマウント
    ret = esp_vfs_fat_sdspi_mount(g_mount_point, &host, &slot_config, &mount_config, &g_sd_card);

    return ret;
}

/**
 * SDカード情報を表示する関数
 */
void print_sd_card_info(void)
{
    if (g_sd_card == NULL)
    {
        return;
    }

    ESP_LOGI(TAG, "💾 SDカード情報:");
    ESP_LOGI(TAG, "   カード名: %s", g_sd_card->cid.name);
    ESP_LOGI(TAG, "   容量: %llu MB", ((uint64_t)g_sd_card->csd.capacity) * g_sd_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "   最大周波数: %lu kHz", (unsigned long)g_sd_card->max_freq_khz);
}

// ========================================
// I2C初期化関数群
// ========================================

/**
 * 外部装置用I2C (I2C_NUM_1) を初期化する関数
 * @return ESP_OK: 成功, その他: エラーコード
 */
esp_err_t init_external_i2c(void)
{
    // 外部装置I2C設定構造体を初期化
    i2c_config_t external_i2c_config = {};
    external_i2c_config.mode = I2C_MODE_MASTER;
    external_i2c_config.sda_io_num = I2C_SDA_PIN;
    external_i2c_config.scl_io_num = I2C_SCL_PIN;
    external_i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    external_i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    external_i2c_config.master.clk_speed = EXTERNAL_I2C_FREQ_HZ;
    external_i2c_config.clk_flags = 0; // ESP-IDF 5.4では明示的に0を設定

    // I2Cパラメータを設定
    esp_err_t ret = i2c_param_config(EXTERNAL_I2C_NUM, &external_i2c_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // I2Cドライバーをインストール
    ret = i2c_driver_install(EXTERNAL_I2C_NUM, external_i2c_config.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

/**
 * I2Cデバイススキャン関数（デバッグ用）
 * @param i2c_num I2Cポート番号
 * @param found_devices 見つかったデバイス数を格納するポインタ
 * @return ESP_OK: 成功, その他: エラーコード
 */
esp_err_t scan_i2c_devices(i2c_port_t i2c_num, int *found_devices)
{
    int device_count = 0;
    esp_err_t ret;

    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            device_count++;
        }
    }

    if (found_devices != NULL)
    {
        *found_devices = device_count;
    }

    return ESP_OK;
}

// ========================================
// アプリケーションメイン
// ========================================
extern "C" void app_main()
{
    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  📸 ピクセルアートカメラ ESP-IDF 5.4版 📸  ║");
    ESP_LOGI(TAG, "║    🎨 エンコーダーとLEDテスト対応版 🎨     ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "🔧 ESP-IDF Version: %s", esp_get_idf_version());

    // NVS初期化
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    ESP_LOGI(TAG, "✅ NVS初期化完了");

    // チップ情報を表示
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "💾 ESP32 チップ: %s rev %d, CPU コア数: %d",
             CONFIG_IDF_TARGET, chip_info.revision, chip_info.cores);

    // PSRAMチェック
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0)
    {
        ESP_LOGI(TAG, "✅ PSRAM検出: %lu KB", (unsigned long)(psram_size / 1024));
    }
    else
    {
        ESP_LOGW(TAG, "⚠️  PSRAM未検出: 内蔵RAMのみで動作");
    }

    // ========================================
    // GPIO初期化（ボタン）
    // ========================================
    ESP_LOGI(TAG, "🔘 GPIO初期化中...");
    esp_err_t gpio_result = init_gpio();
    if (gpio_result != ESP_OK) {
        ESP_LOGE(TAG, "❌ GPIO初期化失敗: %s", esp_err_to_name(gpio_result));
    } else {
        ESP_LOGI(TAG, "✅ GPIO初期化成功");
    }

    // ========================================
    // カメラ初期化
    // ========================================
    ESP_LOGI(TAG, "📸 カメラ初期化中...");
    g_camera_utils = new CameraUtils();
    if (g_camera_utils != nullptr)
    {
        esp_err_t camera_result = g_camera_utils->init(CAMERA_NIGHT_MODE_OFF);
        if (camera_result == ESP_OK)
        {
            ESP_LOGI(TAG, "✅ カメラ初期化成功");
            g_camera_ready = true;

            // カメラ情報表示
            int width, height;
            if (g_camera_utils->get_frame_size(&width, &height) == ESP_OK)
            {
                ESP_LOGI(TAG, "   解像度: %dx%d", width, height);
            }
            ESP_LOGI(TAG, "   PSRAM使用: %lu KB", (unsigned long)(g_camera_utils->get_psram_size() / 1024));
        }
        else
        {
            ESP_LOGE(TAG, "❌ カメラ初期化失敗: %s", esp_err_to_name(camera_result));
            g_camera_ready = false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "❌ カメラオブジェクト作成失敗");
        g_camera_ready = false;
    }

    // ========================================
    // 同期オブジェクト作成
    // ========================================
    ESP_LOGI(TAG, "🔄 同期オブジェクト作成中...");
    
    // エンコーダーイベントキュー作成
    g_encoder_event_queue = xQueueCreate(10, sizeof(encoder_event_t));
    if (g_encoder_event_queue == NULL) {
        ESP_LOGE(TAG, "❌ エンコーダーイベントキュー作成失敗");
    } else {
        ESP_LOGI(TAG, "✅ エンコーダーイベントキュー作成成功");
    }

    ESP_LOGI(TAG, "\n🔧 === I2Cバス初期化開始 ===");

    // ========================================
    // 外部装置用I2C初期化
    // ========================================
    ESP_LOGI(TAG, "🔧 外部装置用I2C (I2C_NUM_1) 初期化中...");
    ESP_LOGI(TAG, "   SDA: GPIO%d, SCL: GPIO%d, 周波数: %d Hz",
             I2C_SDA_PIN, I2C_SCL_PIN, EXTERNAL_I2C_FREQ_HZ);

    esp_err_t external_i2c_result = init_external_i2c();
    if (external_i2c_result == ESP_OK)
    {
        ESP_LOGI(TAG, "✅ 外部装置用I2C初期化成功");

        // 外部I2Cバスでデバイススキャン
        int external_devices = 0;
        scan_i2c_devices(EXTERNAL_I2C_NUM, &external_devices);
        ESP_LOGI(TAG, "   検出デバイス数: %d", external_devices);
    }
    else
    {
        ESP_LOGE(TAG, "❌ 外部装置用I2C初期化失敗: %s", esp_err_to_name(external_i2c_result));
    }

    ESP_LOGI(TAG, "\n📦 === デバイス初期化開始 ===");

    // ========================================
    // Pimoroniエンコーダー初期化
    // ========================================
    ESP_LOGI(TAG, "🔄 Pimoroniエンコーダー初期化中...");
    ESP_LOGI(TAG, "   I2Cアドレス: 0x%02X", PIMORONI_ENCODER_DEFAULT_ADDR);

    g_encoder = new PimoroniEncoder(EXTERNAL_I2C_NUM, PIMORONI_ENCODER_DEFAULT_ADDR);
    if (g_encoder != nullptr)
    {
        esp_err_t encoder_result = g_encoder->init();
        if (encoder_result == ESP_OK)
        {
            ESP_LOGI(TAG, "✅ エンコーダー初期化成功");
            g_encoder_ready = true;

            // エンコーダー設定
            g_encoder->set_value_range(0, MAX_PALETTE_INDEX - 1);
            g_encoder->set_color(0x004000); // 緑色で初期化完了を表示
            
            // LEDテストを実行
            ESP_LOGI(TAG, "🎨 エンコーダーLEDテスト開始");
            g_encoder->test_led_colors();
            
            // 初期パレット色に設定
            g_encoder->set_color(PALETTE_REP_COLORS[0]);
        }
        else
        {
            ESP_LOGE(TAG, "❌ エンコーダー初期化失敗: %s", esp_err_to_name(encoder_result));
            g_encoder_ready = false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "❌ エンコーダーオブジェクト作成失敗");
        g_encoder_ready = false;
    }

    // ========================================
    // SSD1306 OLED ディスプレイ初期化
    // ========================================
    ESP_LOGI(TAG, "📺 SSD1306 OLED ディスプレイ初期化中...");
    ESP_LOGI(TAG, "   I2Cアドレス: 0x%02X", SSD1306_DEFAULT_ADDR);

    g_display = new SSD1306Display(EXTERNAL_I2C_NUM, SSD1306_DEFAULT_ADDR);
    if (g_display != nullptr)
    {
        esp_err_t display_result = g_display->init();
        if (display_result == ESP_OK)
        {
            ESP_LOGI(TAG, "✅ ディスプレイ初期化成功");
            g_display_ready = true;

            // 初期画面表示
            g_display->clear();
            g_display->draw_string(0, 0, "PixelArt Camera", true);
            g_display->draw_string(0, 16, "ESP-IDF 5.4", true);
            g_display->draw_string(0, 32, "Encoder Test", true);
            g_display->draw_string(0, 48, "Initializing...", true);
            g_display->display();
        }
        else
        {
            ESP_LOGE(TAG, "❌ ディスプレイ初期化失敗: %s", esp_err_to_name(display_result));
            g_display_ready = false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "❌ ディスプレイオブジェクト作成失敗");
        g_display_ready = false;
    }

    // ========================================
    // SDカード初期化
    // ========================================
    ESP_LOGI(TAG, "💾 SDカード初期化中...");
    ESP_LOGI(TAG, "   マウントポイント: %s", g_mount_point);
    ESP_LOGI(TAG, "   SPI - SCK:GPIO%d, MISO:GPIO%d, MOSI:GPIO%d, CS:GPIO%d",
             SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);

    esp_err_t sd_result = init_sd_card();
    if (sd_result == ESP_OK)
    {
        ESP_LOGI(TAG, "✅ SDカード初期化成功");
        g_sd_card_ready = true;
        print_sd_card_info();
    }
    else
    {
        ESP_LOGE(TAG, "❌ SDカード初期化失敗: %s", esp_err_to_name(sd_result));
        g_sd_card_ready = false;
    }

    // ========================================
    // 初期化結果サマリー
    // ========================================
    ESP_LOGI(TAG, "\n📋 === 初期化結果サマリー ===");
    ESP_LOGI(TAG, "GPIO:                     %s",
             gpio_result == ESP_OK ? "✅ 成功" : "❌ 失敗");
    ESP_LOGI(TAG, "外部装置I2C (I2C_NUM_1):   %s",
             external_i2c_result == ESP_OK ? "✅ 成功" : "❌ 失敗");
    ESP_LOGI(TAG, "カメラ:                   %s",
             g_camera_ready ? "✅ 成功" : "❌ 失敗");
    ESP_LOGI(TAG, "エンコーダー:             %s",
             g_encoder_ready ? "✅ 成功" : "❌ 失敗");
    ESP_LOGI(TAG, "OLEDディスプレイ:         %s",
             g_display_ready ? "✅ 成功" : "❌ 失敗");
    ESP_LOGI(TAG, "SDカード:                 %s",
             g_sd_card_ready ? "✅ 成功" : "❌ 失敗");

    // 全デバイス成功チェック
    int success_count = 0;
    if (gpio_result == ESP_OK) success_count++;
    if (external_i2c_result == ESP_OK) success_count++;
    if (g_camera_ready) success_count++;
    if (g_encoder_ready) success_count++;
    if (g_display_ready) success_count++;
    if (g_sd_card_ready) success_count++;

    ESP_LOGI(TAG, "\n🎯 初期化完了: %d/7 成功", success_count);

    if (success_count >= 5)
    { // 最低限の機能が動作
        ESP_LOGI(TAG, "🎉 システム初期化完了！エンコーダーとLEDテスト準備OK");
        g_system_ready = true;

        // ディスプレイに成功メッセージ表示
        if (g_display_ready)
        {
            g_display->clear();
            g_display->draw_string(0, 0, "System Ready!", true);
            g_display->draw_string(0, 16, "Encoder Test", true);
            g_display->draw_string(0, 32, "Mode Active", true);
            g_display->draw_string(0, 48, "Turn encoder!", true);
            g_display->display();
        }

        // エンコーダーLEDを青色に設定（待機状態）
        if (g_encoder_ready)
        {
            g_encoder->set_color(0x0000FF);
        }
    }
    else
    {
        ESP_LOGE(TAG, "💥 重要なデバイスの初期化に失敗。ハードウェア接続を確認してください");

        // エラー状態をディスプレイに表示
        if (g_display_ready)
        {
            g_display->clear();
            g_display->draw_string(0, 0, "Init Error!", true);
            g_display->draw_string(0, 16, "Check Hardware", true);
            g_display->display();
        }

        // エンコーダーLEDを赤色に設定（エラー状態）
        if (g_encoder_ready)
        {
            g_encoder->set_color(0xFF0000);
        }
    }

    // ========================================
    // エンコーダータスク開始
    // ========================================
    if (g_encoder_ready) {
        ESP_LOGI(TAG, "🔄 エンコーダータスク作成中...");
        BaseType_t task_result = xTaskCreate(
            encoder_task,           // タスク関数
            "encoder_task",         // タスク名
            ENCODER_TASK_STACK,     // スタックサイズ
            NULL,                   // パラメータ
            tskIDLE_PRIORITY + 2,   // 優先度（高め）
            NULL                    // タスクハンドル
        );
        
        if (task_result == pdPASS) {
            ESP_LOGI(TAG, "✅ エンコーダータスク作成成功");
        } else {
            ESP_LOGE(TAG, "❌ エンコーダータスク作成失敗");
        }
    }

    ESP_LOGI(TAG, "\n🔘 ボタンテストモード開始");
    ESP_LOGI(TAG, "   GPIO%d (シャッター): 短押し=単一パレット撮影, 長押し=全パレット撮影", SHUTTER_BUTTON_PIN);
    ESP_LOGI(TAG, "   GPIO%d (メニュー): 短押し=LEDテスト, 長押し=システム情報", MENU_BUTTON_PIN);
    ESP_LOGI(TAG, "   エンコーダー: 回転でパレット選択とLED色変更");

    // メインループ（ボタン処理メイン）
    while (1)
    {
        // ボタン状態を更新
        update_button_state(&g_shutter_button);
        update_button_state(&g_menu_button);
        
        // ボタンイベントを処理
        process_button_events();
        
        // メインループは20ms周期（ボタン応答性重視）
        // エンコーダーは専用タスクで10ms周期で処理
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}