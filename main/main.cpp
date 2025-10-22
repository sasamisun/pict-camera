/*
 * AtomS3R ピクセルアートカメラ (ESP-IDF 5.4完全対応版)
 *
 * ToDo:
 * G38 G39プルアップボタン入力（シャッターボタン・メニューボタン）
 * encoder 入力、LEDテスト
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

#define BUTTON_PIN GPIO_NUM_38       // 撮影ボタン
#define LED_PIN GPIO_NUM_39          // 状態LED

// I2C設定
#define I2C_SDA_PIN GPIO_NUM_1 // 外部I2C SDA
#define I2C_SCL_PIN GPIO_NUM_2 // 外部I2C SCL
#define I2C_FREQ_HZ 400000     // 400kHz

// 外部装置I2C設定
#define EXTERNAL_I2C_NUM I2C_NUM_1  // 外部装置専用I2C
#define EXTERNAL_I2C_FREQ_HZ 400000 // 外部装置は400kHzで高速動作



// その他設定
#define DEBOUNCE_DELAY_MS 300    // ボタンチャタリング防止
#define MAX_PALETTE_INDEX 8      // パレット数
#define IMAGE_WIDTH 240          // 画像幅
#define IMAGE_HEIGHT 176         // 画像高
#define CAPTURE_TASK_STACK 8192  // タスクスタックサイズ
#define PROCESS_TASK_STACK 16384 // 画像処理用大きめスタック

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

// 点滅設定
#define BLINK_NORMAL_INTERVAL_MS 500 // 通常点滅間隔
#define BLINK_FAST_INTERVAL_MS 200   // 高速点滅間隔

// ========================================
// グローバル変数
// ========================================

// 同期制御
static SemaphoreHandle_t g_capture_semaphore = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static QueueHandle_t g_capture_queue = NULL;

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

// 撮影データ構造体
typedef struct
{
    uint32_t timestamp;
    int palette_index;
    bool save_all_palettes; // 全パレット保存フラグ
} capture_request_t;

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
 * カメラ用I2C (I2C_NUM_0) を初期化する関数
 * @return ESP_OK: 成功, その他: エラーコード
 */
/*
esp_err_t init_camera_i2c(void)
{
    // カメラI2C設定構造体を初期化
    i2c_config_t camera_i2c_config = {};
    camera_i2c_config.mode = I2C_MODE_MASTER;
    camera_i2c_config.sda_io_num = CAMERA_I2C_SDA;
    camera_i2c_config.scl_io_num = CAMERA_I2C_SCL;
    camera_i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    camera_i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    camera_i2c_config.master.clk_speed = CAMERA_I2C_FREQ_HZ;
    camera_i2c_config.clk_flags = 0; // ESP-IDF 5.4では明示的に0を設定

    // I2Cパラメータを設定
    esp_err_t ret = i2c_param_config(CAMERA_I2C_NUM, &camera_i2c_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // I2Cドライバーをインストール
    // (ポート番号, モード, スレーブ受信バッファサイズ, スレーブ送信バッファサイズ, 割り込みフラグ)
    ret = i2c_driver_install(CAMERA_I2C_NUM, camera_i2c_config.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}
*/

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
    // (ポート番号, モード, スレーブ受信バッファサイズ, スレーブ送信バッファサイズ, 割り込みフラグ)
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
    ESP_LOGI(TAG, "║       🎨 8色パレット + ステータスLED 🎨      ║");
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
            g_display->draw_string(0, 32, "Initializing...", true);
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
    if (external_i2c_result == ESP_OK)
        success_count++;
    if (g_camera_ready)
        success_count++;
    if (g_encoder_ready)
        success_count++;
    if (g_display_ready)
        success_count++;
    if (g_sd_card_ready)
        success_count++;

    ESP_LOGI(TAG, "\n🎯 初期化完了: %d/6 成功", success_count);

    if (success_count >= 4)
    { // 最低限の機能が動作
        ESP_LOGI(TAG, "🎉 システム初期化完了！次のステップに進む準備OK");
        g_system_ready = true;

        // ディスプレイに成功メッセージ表示
        if (g_display_ready)
        {
            g_display->clear();
            g_display->draw_string(0, 0, "System Ready!", true);
            g_display->draw_string(0, 16, "Camera: ", true);
            g_display->draw_string(56, 16, g_camera_ready ? "OK" : "NG", g_camera_ready);
            g_display->draw_string(0, 24, "Encoder:", true);
            g_display->draw_string(56, 24, g_encoder_ready ? "OK" : "NG", g_encoder_ready);
            g_display->draw_string(0, 32, "SD Card:", true);
            g_display->draw_string(56, 32, g_sd_card_ready ? "OK" : "NG", g_sd_card_ready);
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

    ESP_LOGI(TAG, "\nシステム初期化完了。次の実装を待機中...");

    // メインループ（現在は何もしない）
    while (1)
    {
        // エンコーダー値更新（テスト用）
        if (g_encoder_ready)
        {
            int current_value = g_encoder->update();
            static int last_value = -1;
            if (current_value != last_value)
            {
                ESP_LOGI(TAG, "🔄 エンコーダー値変更: %d", current_value);
                last_value = current_value;

                // パレット表示色を変更
                if (current_value < MAX_PALETTE_INDEX)
                {
                    g_encoder->set_color(PALETTE_REP_COLORS[current_value]);
                    g_current_palette_index = current_value;
                }

                // ディスプレイに現在のパレット番号を表示
                if (g_display_ready)
                {
                    g_display->clear();
                    g_display->draw_string(0, 0, "Palette Mode", true);
                    char palette_str[32];
                    snprintf(palette_str, sizeof(palette_str), "Current: %d", current_value);
                    g_display->draw_string(0, 16, palette_str, true);
                    g_display->display();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms周期で更新
    }
}
