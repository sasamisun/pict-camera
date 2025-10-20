/*
 * AtomS3R ピクセルアートカメラ (ESP-IDF 5.4完全対応版)
 *
 * 新機能追加:
 * - ステータス表示用LEDエンコーダ制御にゃ
 * - 赤点滅: 操作禁止時（起動・撮影・書き込み中）
 * - 青点灯: 操作可能時（待機中）
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
// ESP-IDF 5.4では esp_psram.h の代わりに esp_heap_caps.h を使うにゃ
#include "esp_heap_caps.h"
#include "esp_chip_info.h" // esp_chip_info用にゃ

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
#include "PixelArtProcessor.h"
#include "camera_utils.h"
#include "ssd1306_display.h"

// ========================================
// 定数定義にゃ
// ========================================
static const char *TAG = "PixelArtCamera";

// ピン設定（AtomS3R専用）
#define SPI_SCK_PIN GPIO_NUM_7 // SDカード用SPI
#define SPI_MISO_PIN GPIO_NUM_8
#define SPI_MOSI_PIN GPIO_NUM_6
#define SPI_CS_PIN GPIO_NUM_15

#define CAMERA_POWER_PIN GPIO_NUM_18 // カメラ電源制御
#define BUTTON_PIN GPIO_NUM_38       // 撮影ボタン
#define LED_PIN GPIO_NUM_39          // 状態LED

// I2C設定（分離して競合回避にゃ）
#define I2C_MASTER_NUM I2C_NUM_1 // エンコーダ用I2C
#define I2C_SDA_PIN GPIO_NUM_1   // エンコーダI2C SDA
#define I2C_SCL_PIN GPIO_NUM_2   // エンコーダI2C SCL
#define I2C_FREQ_HZ 400000       // 400kHz

// カメラI2C設定（ESP32-S3内蔵I2C使用）
#define CAMERA_I2C_NUM I2C_NUM_0   // カメラ専用I2C
#define CAMERA_I2C_SDA GPIO_NUM_12 // カメラI2C SDA
#define CAMERA_I2C_SCL GPIO_NUM_9  // カメラI2C SCL

// カメラピン設定
#define CAM_PIN_XCLK GPIO_NUM_21
#define CAM_PIN_VSYNC GPIO_NUM_10
#define CAM_PIN_HREF GPIO_NUM_14
#define CAM_PIN_PCLK GPIO_NUM_40
#define CAM_PIN_D0 GPIO_NUM_3
#define CAM_PIN_D1 GPIO_NUM_42
#define CAM_PIN_D2 GPIO_NUM_46
#define CAM_PIN_D3 GPIO_NUM_48
#define CAM_PIN_D4 GPIO_NUM_4
#define CAM_PIN_D5 GPIO_NUM_17
#define CAM_PIN_D6 GPIO_NUM_11
#define CAM_PIN_D7 GPIO_NUM_13

// その他設定
#define DEBOUNCE_DELAY_MS 300    // ボタンチャタリング防止
#define MAX_PALETTE_INDEX 8      // パレット数
#define IMAGE_WIDTH 240          // 画像幅
#define IMAGE_HEIGHT 176         // 画像高
#define CAPTURE_TASK_STACK 8192  // タスクスタックサイズ
#define PROCESS_TASK_STACK 16384 // 画像処理用大きめスタック

// ========================================
// ステータスLED制御定義にゃ
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

// ステータス色定義
#define STATUS_COLOR_RED_BLINK 0xFF0000   // 赤（操作禁止）
#define STATUS_COLOR_BLUE_STEADY 0x0000FF // 青（操作可能）
#define STATUS_COLOR_OFF 0x000000         // 消灯

// 点滅設定
#define BLINK_NORMAL_INTERVAL_MS 500 // 通常点滅間隔
#define BLINK_FAST_INTERVAL_MS 200   // 高速点滅間隔

// ========================================
// グローバル変数にゃ
// ========================================

// 同期制御
static SemaphoreHandle_t g_capture_semaphore = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static QueueHandle_t g_capture_queue = NULL;

// ハードウェア
// static i2c_master_bus_handle_t g_i2c_bus_handle = NULL; // 新しいI2Cバスハンドル
static PimoroniEncoder *g_encoder = nullptr;
static PixelArtProcessor *g_processor = nullptr;
static CameraUtils *g_camera_utils = nullptr;
static bool g_camera_ready = false;
// SSD1306ディスプレイオブジェクト
static SSD1306Display *g_display = nullptr;
static bool g_display_enabled = false;

// 状態管理
static volatile bool g_system_ready = false;
static volatile int g_current_palette_index = 0;
static volatile int g_file_counter = 1;
static volatile uint32_t g_last_button_press = 0;

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
// カラーパレット定義（8種類×8色）にゃ
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

// 関数プロトタイプ宣言
static void update_display_status(const char *status, int palette_index);
static void show_camera_preview(camera_fb_t *fb);

// ========================================
// ステータスLED制御関数にゃ
// ========================================

// システムステータス設定
static void set_system_status(system_status_t status)
{
    system_status_t old_status = g_system_status;
    g_system_status = status;

    const char *status_names[] = {
        "INITIALIZING", "READY", "CAPTURING", "PROCESSING", "SAVING", "ERROR" // PROCESSINGを追加
    };

    if (old_status != status)
    {
        ESP_LOGI(TAG, "🎨 システムステータス変更: %s -> %s",
                 status_names[old_status], status_names[status]);
    }

    // ディスプレイステータス更新
    switch (status)
    {
    case SYSTEM_STATUS_READY:
        update_display_status("Ready", g_current_palette_index);
        break;
    case SYSTEM_STATUS_CAPTURING:
        update_display_status("Capturing...", g_current_palette_index);
        break;
    case SYSTEM_STATUS_PROCESSING: // これで認識されるようになります
        update_display_status("Processing...", g_current_palette_index);
        break;
    case SYSTEM_STATUS_SAVING:
        update_display_status("Saving...", g_current_palette_index);
        break;
    case SYSTEM_STATUS_ERROR:
        update_display_status("Error!", g_current_palette_index);
        break;
    default:
        break;
    }
}

// エンコーダLED更新（ステータス優先）
static void update_encoder()
{
    if (!g_encoder || !g_encoder->is_initialized() || !g_status_led_enabled)
    {
        return;
    }

    static uint32_t last_blink_time = 0;
    static bool blink_state = false;
    uint32_t current_time = esp_timer_get_time() / 1000;

    switch (g_system_status)
    {
    case SYSTEM_STATUS_READY:
        // 待機中: 青点灯 + パレット色も表示
        if (g_current_palette_index >= 0 && g_current_palette_index < 8)
        {
            // パレット代表色を薄めの青とミックス
            uint32_t palette_color = PALETTE_REP_COLORS[g_current_palette_index];
            uint8_t r = ((palette_color >> 16) & 0xFF) / 4; // 25%の明度
            uint8_t g = ((palette_color >> 8) & 0xFF) / 4;
            uint8_t b = ((palette_color & 0xFF) / 4) + 64; // 青成分を強化
            g_encoder->set_led(r, g, b);
        }
        else
        {
            g_encoder->set_led_color(STATUS_COLOR_BLUE_STEADY);
        }
        break;

    case SYSTEM_STATUS_INITIALIZING:
    case SYSTEM_STATUS_CAPTURING:
    case SYSTEM_STATUS_PROCESSING:
    case SYSTEM_STATUS_SAVING:
        // 操作禁止: 赤点滅
        if (current_time - last_blink_time >= BLINK_NORMAL_INTERVAL_MS)
        {
            blink_state = !blink_state;
            last_blink_time = current_time;
            g_encoder->set_led_color(blink_state ? STATUS_COLOR_RED_BLINK : STATUS_COLOR_OFF);
        }
        break;

    case SYSTEM_STATUS_ERROR:
        // エラー: 赤高速点滅
        if (current_time - last_blink_time >= BLINK_FAST_INTERVAL_MS)
        {
            blink_state = !blink_state;
            last_blink_time = current_time;
            g_encoder->set_led_color(blink_state ? STATUS_COLOR_RED_BLINK : STATUS_COLOR_OFF);
        }
        break;
    }
}

// ステータスLED有効/無効切り替え
static void enable_status_led(bool enable)
{
    g_status_led_enabled = enable;
    if (!enable && g_encoder && g_encoder->is_initialized())
    {
        g_encoder->led_off();
    }
}

// ========================================
// GPIO制御関数にゃ
// ========================================

static void status_led_on()
{
    gpio_set_level(LED_PIN, 1);
}

static void status_led_off()
{
    gpio_set_level(LED_PIN, 0);
}

static void status_led_blink(int count, int on_ms, int off_ms)
{
    for (int i = 0; i < count; i++)
    {
        status_led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        status_led_off();
        if (i < count - 1)
        { // 最後以外は待機
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

static bool is_button_pressed()
{
    // プルアップなので、押されたときLOW
    if (gpio_get_level(BUTTON_PIN) == 0)
    {
        // チャタリング防止
        uint32_t now = esp_timer_get_time() / 1000; // ミリ秒取得
        if (now - g_last_button_press > DEBOUNCE_DELAY_MS)
        {
            g_last_button_press = now;
            return true;
        }
    }
    return false;
}

// ========================================
// GPIO初期化 (ESP-IDF 5.4対応)にゃ
// ========================================
static esp_err_t init_gpio()
{
    ESP_LOGI(TAG, "🔧 GPIO初期化開始にゃ");
    set_system_status(SYSTEM_STATUS_INITIALIZING);

    // ボタンピン設定（プルアップ）
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ボタンGPIO設定失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // LEDピン設定（出力）
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ret = gpio_config(&led_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LED GPIO設定失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // カメラ電源ピン設定
    gpio_config_t power_config = {
        .pin_bit_mask = (1ULL << CAMERA_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ret = gpio_config(&power_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "電源GPIO設定失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // 初期状態設定
    status_led_off();
    gpio_set_level(CAMERA_POWER_PIN, 0); // カメラ電源ON（LOWで有効）

    ESP_LOGI(TAG, "✅ GPIO初期化完了にゃ");
    return ESP_OK;
}

// ========================================
// I2C初期化 (ESP-IDF 5.4新しいドライバー)にゃ
// ========================================
static esp_err_t init_i2c_master()
{
    ESP_LOGI(TAG, "🔧 I2Cマスター初期化開始（レガシードライバー使用）");

    // I2C設定構造体
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz（安定性重視）

    // I2Cパラメータ設定
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C設定失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // I2Cドライバーインストール
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2Cドライバーインストール失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    ESP_LOGI(TAG, "✅ I2Cマスター初期化完了（レガシー版）");
    ESP_LOGI(TAG, "   ポート: %d, SDA: GPIO%d, SCL: GPIO%d, 速度: 100kHz",
             I2C_MASTER_NUM, I2C_SDA_PIN, I2C_SCL_PIN);

    return ESP_OK;
}

// ========================================
// SDカード初期化 (ESP-IDF 5.4対応)にゃ
// ========================================
static esp_err_t init_sdcard()
{
    ESP_LOGI(TAG, "🔧 SDカード初期化開始にゃ");

    // SDカード用SPI設定
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT; // デフォルト周波数使用

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = (spi_host_device_t)host.slot;
    slot_config.gpio_cd = GPIO_NUM_NC;
    slot_config.gpio_wp = GPIO_NUM_NC;

    // SPIバス設定（ESP-IDF 5.4で追加されたフィールドを初期化）
    spi_bus_config_t bus_cfg = {}; // 全フィールドを0で初期化
    bus_cfg.mosi_io_num = SPI_MOSI_PIN;
    bus_cfg.miso_io_num = SPI_MISO_PIN;
    bus_cfg.sclk_io_num = SPI_SCK_PIN;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = 4092;
    bus_cfg.flags = 0;
    bus_cfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    bus_cfg.intr_flags = 0;

    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIバス初期化失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // VFSマウント設定
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    ret = esp_vfs_fat_sdspi_mount(
        g_mount_point, &host, &slot_config, &mount_config, &g_sd_card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SDカードマウント失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        return ret;
    }

    // カード情報表示（フォーマット修正）
    ESP_LOGI(TAG, "📱 SDカード情報:");
    ESP_LOGI(TAG, "  名前: %s", g_sd_card->cid.name);
    ESP_LOGI(TAG, "  容量: %lluMB",
             ((uint64_t)g_sd_card->csd.capacity) * g_sd_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "  最大周波数: %lukHz", (unsigned long)g_sd_card->max_freq_khz);

    ESP_LOGI(TAG, "✅ SDカード初期化完了にゃ");
    return ESP_OK;
}

// ========================================
// カメラ初期化 (esp-camera 2.1.3対応)にゃ
// ========================================
static esp_err_t init_camera()
{
    ESP_LOGI(TAG, "🔧 カメラ初期化開始にゃ");

    // PSRAM確認（ESP-IDF 5.4対応）
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size == 0)
    {
        ESP_LOGW(TAG, "⚠️ PSRAMが検出されません。内蔵RAMを使用します");
    }
    else
    {
        ESP_LOGI(TAG, "✅ PSRAM検出: %zu bytes", psram_size);
    }

    // カメラ電源を有効化
    gpio_set_level(CAMERA_POWER_PIN, 0); // LOWで電源ON
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "🔌 カメラ電源ON");

    // カメラ設定構造体（esp-camera 2.1.3対応）
    camera_config_t camera_config = {}; // 全フィールドを0で初期化
    camera_config.pin_pwdn = GPIO_NUM_NC;
    camera_config.pin_reset = GPIO_NUM_NC;
    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_sccb_sda = CAMERA_I2C_SDA;
    camera_config.pin_sccb_scl = CAMERA_I2C_SCL;
    camera_config.pin_d7 = CAM_PIN_D7;
    camera_config.pin_d6 = CAM_PIN_D6;
    camera_config.pin_d5 = CAM_PIN_D5;
    camera_config.pin_d4 = CAM_PIN_D4;
    camera_config.pin_d3 = CAM_PIN_D3;
    camera_config.pin_d2 = CAM_PIN_D2;
    camera_config.pin_d1 = CAM_PIN_D1;
    camera_config.pin_d0 = CAM_PIN_D0;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_pclk = CAM_PIN_PCLK;
    camera_config.xclk_freq_hz = 20000000;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.pixel_format = PIXFORMAT_RGB565;
    camera_config.frame_size = FRAMESIZE_HQVGA;
    camera_config.jpeg_quality = 0;
    camera_config.fb_count = 2;
    camera_config.fb_location = (psram_size > 0) ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    camera_config.grab_mode = CAMERA_GRAB_LATEST;
    camera_config.sccb_i2c_port = CAMERA_I2C_NUM;

    // カメラドライバ初期化
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ カメラ初期化失敗: 0x%x (%s)", err, esp_err_to_name(err));
        set_system_status(SYSTEM_STATUS_ERROR);
        return err;
    }

    // センサー設定（AtomS3R Cam用）
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor)
    {
        sensor->set_hmirror(sensor, 1); // 左右反転
        sensor->set_vflip(sensor, 1);   // 上下反転

        // 追加設定
        sensor->set_brightness(sensor, 0); // 明度: -2～2
        sensor->set_contrast(sensor, 0);   // コントラスト: -2～2
        sensor->set_saturation(sensor, 0); // 彩度: -2～2
        sensor->set_sharpness(sensor, 0);  // シャープネス: -2～2

        ESP_LOGI(TAG, "📷 センサー設定完了");
    }

    ESP_LOGI(TAG, "✅ カメラ初期化完了にゃ");
    return ESP_OK;
}

/**
 * @brief SSD1306ディスプレイの初期化
 * @note エンコーダ初期化後に呼び出す
 * @return ESP_OK 成功, その他 エラーコード
 */
static esp_err_t init_display()
{
    ESP_LOGI(TAG, "🔧 ディスプレイ初期化開始（暫定的にスキップ）");

    // TODO: SSD1306Displayクラスが実装されたら有効化
    /*
    // レガシードライバーでは I2C ポート番号を使用
    g_display = new SSD1306Display(I2C_MASTER_NUM, SSD1306_DEFAULT_ADDR);
    if (!g_display)
    {
        ESP_LOGE(TAG, "❌ ディスプレイオブジェクト作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = g_display->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ ディスプレイ初期化失敗、ディスプレイなしで続行");
        delete g_display;
        g_display = nullptr;
        return ESP_OK; // ディスプレイなしでも動作可能
    }

    // 初期表示
    g_display->clear();
    g_display->set_text_size(1);
    g_display->set_cursor(0, 0);
    g_display->print("Pixel Art Camera");
    g_display->set_cursor(0, 16);
    g_display->print("Initializing...");
    g_display->display();
    */

    ESP_LOGI(TAG, "✅ ディスプレイ初期化完了（スキップ）");
    return ESP_OK;
}

// ========================================
// 修正版I2Cスキャン機能にゃ
// ========================================
static void scan_i2c_devices()
{
    ESP_LOGI(TAG, "🔍 I2Cデバイススキャン開始（レガシー版）");

    int device_count = 0;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:          ");

    for (int i = 3; i < 0x78; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (i % 16 == 0)
        {
            printf("\n%.2x:", i);
        }

        if (ret == ESP_OK)
        {
            printf(" %.2x", i);
            device_count++;
        }
        else
        {
            printf(" --");
        }
    }
    printf("\n");

    ESP_LOGI(TAG, "🎯 検出されたデバイス数: %d", device_count);

    if (device_count == 0)
    {
        ESP_LOGW(TAG, "⚠️ I2Cデバイスが見つかりません");
        ESP_LOGI(TAG, "💡 確認項目:");
        ESP_LOGI(TAG, "  1. エンコーダの電源（3.3V/5V）");
        ESP_LOGI(TAG, "  2. SDA配線 (GPIO%d)", I2C_SDA_PIN);
        ESP_LOGI(TAG, "  3. SCL配線 (GPIO%d)", I2C_SCL_PIN);
        ESP_LOGI(TAG, "  4. GND接続");
        ESP_LOGI(TAG, "  5. プルアップ抵抗（内蔵プルアップ有効済み）");
        ESP_LOGI(TAG, "  6. エンコーダモジュールの動作確認");

        // GPIO状態確認
        ESP_LOGI(TAG, "🔍 GPIO状態確認:");
        ESP_LOGI(TAG, "  SDA (GPIO%d): %s", I2C_SDA_PIN, gpio_get_level((gpio_num_t)I2C_SDA_PIN) ? "HIGH" : "LOW");
        ESP_LOGI(TAG, "  SCL (GPIO%d): %s", I2C_SCL_PIN, gpio_get_level((gpio_num_t)I2C_SCL_PIN) ? "HIGH" : "LOW");
    }
    else
    {
        // Pimoroniエンコーダの一般的なアドレスをチェック
        const uint8_t pimoroni_addresses[] = {0x0F, 0x12, 0x20, 0x21, 0x22, 0x23};
        bool found_pimoroni = false;

        for (int i = 0; i < sizeof(pimoroni_addresses); i++)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (pimoroni_addresses[i] << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);

            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "🎯 Pimoroni候補発見: 0x%02X", pimoroni_addresses[i]);
                found_pimoroni = true;
            }
        }

        if (!found_pimoroni)
        {
            ESP_LOGW(TAG, "⚠️ Pimoroniエンコーダらしきデバイスが見つかりません");
            ESP_LOGI(TAG, "💡 検出されたデバイスが別のアドレスにある可能性があります");
        }
    }
}

// ========================================
// エンコーダ初期化の修正
// ========================================
static esp_err_t init_encoder()
{
    scan_i2c_devices();
    ESP_LOGI(TAG, "🔧 エンコーダ初期化開始");

    // レガシードライバー版では、バスハンドルは不要
    g_encoder = new PimoroniEncoder(I2C_MASTER_NUM, 0x0F);  // バスハンドルではなくI2Cポート番号を渡す
    if (!g_encoder)
    {
        ESP_LOGE(TAG, "❌ エンコーダオブジェクト作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = g_encoder->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ エンコーダ初期化失敗、エンコーダなしで続行");
        delete g_encoder;
        g_encoder = nullptr;
        return ESP_OK; // エンコーダなしでも動作可能
    }

    // エンコーダ設定
    g_encoder->set_value_range(0, 7);                // パレット0-7
    g_encoder->set_value(0);                         // 初期値0
    g_encoder->set_led_brightness(0.3f);             // 輝度30%
    
    // 初期ステータス表示（初期化中なので赤点滅）
    enable_status_led(true);

    ESP_LOGI(TAG, "✅ エンコーダ初期化完了");
    return ESP_OK;
}

// ========================================
// 画像処理クラス初期化にゃ
// ========================================
static esp_err_t init_processor()
{
    ESP_LOGI(TAG, "🔧 画像処理エンジン初期化開始にゃ");

    g_processor = new PixelArtProcessor(COLOR_PALETTES, MAX_PALETTE_INDEX);
    if (!g_processor)
    {
        ESP_LOGE(TAG, "❌ 画像処理オブジェクト作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "✅ 画像処理エンジン初期化完了にゃ");
    return ESP_OK;
}

// ========================================
// カメラユーティリティ初期化にゃ
// ========================================
static esp_err_t init_camera_utils()
{
    ESP_LOGI(TAG, "🔧 カメラユーティリティ初期化開始にゃ");

    g_camera_utils = new CameraUtils(g_mount_point);
    if (!g_camera_utils)
    {
        ESP_LOGE(TAG, "❌ カメラユーティリティオブジェクト作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "✅ カメラユーティリティ初期化完了にゃ");
    return ESP_OK;
}

// ========================================
// 同期オブジェクト初期化にゃ
// ========================================
static esp_err_t init_sync_objects()
{
    ESP_LOGI(TAG, "🔧 同期オブジェクト初期化開始にゃ");

    // セマフォ作成
    g_capture_semaphore = xSemaphoreCreateBinary();
    if (!g_capture_semaphore)
    {
        ESP_LOGE(TAG, "❌ 撮影セマフォ作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    g_i2c_mutex = xSemaphoreCreateMutex();
    if (!g_i2c_mutex)
    {
        ESP_LOGE(TAG, "❌ I2Cミューテックス作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    // キュー作成
    g_capture_queue = xQueueCreate(5, sizeof(capture_request_t));
    if (!g_capture_queue)
    {
        ESP_LOGE(TAG, "❌ 撮影キュー作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "✅ 同期オブジェクト初期化完了にゃ");
    return ESP_OK;
}

// ========================================
// エンコーダ値更新とLED表示にゃ
// ========================================
/*
static void update_encoder() {
    if (!g_encoder || !g_encoder->is_initialized()) return;

    // ミューテックスでI2C保護
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int new_value = g_encoder->update();

        // 値に変化があった場合のみLED更新（重要！）
        if (new_value != g_current_palette_index) {
            ESP_LOGI(TAG, "🎨 パレット変更: %d → %d", g_current_palette_index, new_value);
            g_current_palette_index = new_value;

            // パレット代表色をLEDに表示
            uint32_t rep_color = PALETTE_REP_COLORS[g_current_palette_index];
            ESP_LOGI(TAG, "💡 LED色設定: パレット%d = 0x%06lX",
                     g_current_palette_index, (unsigned long)rep_color);

            g_encoder->set_led_color(rep_color);
        }

        xSemaphoreGive(g_i2c_mutex);
    }
}
    */

// ========================================
// 撮影要求処理にゃ
// ========================================
static bool request_capture(bool save_all_palettes)
{
    capture_request_t request = {
        .timestamp = (uint32_t)(esp_timer_get_time() / 1000), // キャスト追加
        .palette_index = g_current_palette_index,
        .save_all_palettes = save_all_palettes};

    BaseType_t result = xQueueSend(g_capture_queue, &request, pdMS_TO_TICKS(100));
    if (result != pdTRUE)
    {
        ESP_LOGW(TAG, "⚠️ 撮影キューが満杯です");
        return false;
    }

    // セマフォで撮影タスクに通知
    xSemaphoreGive(g_capture_semaphore);
    return true;
}

// ========================================
// カメラプレビュー表示
// ========================================
static void show_camera_preview(camera_fb_t *fb)
{
    if (!g_display_enabled || !g_display || !fb)
    {
        return; // ディスプレイが無効、または無効なフレームバッファ
    }

    // プレビュー表示（エラーでも続行）
    esp_err_t ret = g_display->show_camera_preview(fb);
    if (ret != ESP_OK)
    {
        ESP_LOGD(TAG, "プレビュー表示スキップ: %s", esp_err_to_name(ret));
    }
}

// ========================================
// ディスプレイステータス表示にゃ
// ========================================
static void update_display_status(const char *status, int palette_index)
{
    if (!g_display_enabled || !g_display)
    {
        return;
    }

    g_display->clear();
    g_display->draw_text(0, 0, "PixelArt Camera", 1);

    if (status)
    {
        g_display->draw_text(0, 16, status, 1);
    }

    if (palette_index >= 0)
    {
        char palette_str[32];
        snprintf(palette_str, sizeof(palette_str), "Palette: %d", palette_index);
        g_display->draw_text(0, 32, palette_str, 1);
    }

    // システムステータスを表示
    const char *sys_status = "";
    switch (g_system_status)
    {
    case SYSTEM_STATUS_INITIALIZING:
        sys_status = "Initializing...";
        break;
    case SYSTEM_STATUS_READY:
        sys_status = "Ready!";
        break;
    case SYSTEM_STATUS_CAPTURING:
        sys_status = "Capturing...";
        break;
    case SYSTEM_STATUS_PROCESSING:
        sys_status = "Processing...";
        break;
    case SYSTEM_STATUS_SAVING:
        sys_status = "Saving...";
        break;
    case SYSTEM_STATUS_ERROR:
        sys_status = "Error!";
        break;
    default:
        sys_status = "Unknown";
        break;
    }
    g_display->draw_text(0, 48, sys_status, 1);

    g_display->update();
}

// ========================================
// 撮影処理タスク（コア1で実行）
// ========================================
static void capture_task(void *pvParameters)
{
    ESP_LOGI(TAG, "📸 撮影タスク開始 (Core %d)", xPortGetCoreID());

    capture_request_t request;

    while (1)
    {
        // 撮影要求を待機
        if (xSemaphoreTake(g_capture_semaphore, portMAX_DELAY) == pdTRUE)
        {

            // キューから撮影要求を取得
            if (xQueueReceive(g_capture_queue, &request, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // ステータス: 撮影中
                set_system_status(SYSTEM_STATUS_CAPTURING);

                ESP_LOGI(TAG, "📸 撮影開始: ファイル番号 %d", g_file_counter);
                status_led_on();

                // フレーム取得
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb)
                {
                    ESP_LOGE(TAG, "❌ フレーム取得失敗");
                    set_system_status(SYSTEM_STATUS_ERROR);
                    status_led_blink(3, 200, 200);          // エラー表示
                    vTaskDelay(pdMS_TO_TICKS(2000));        // 2秒待機
                    set_system_status(SYSTEM_STATUS_READY); // 復帰
                    continue;
                }

                ESP_LOGI(TAG, "📷 フレーム取得完了 (%dx%d, %zu bytes)",
                         fb->width, fb->height, fb->len);

                // カメラプレビューを表示（撮影前）
                show_camera_preview(fb);

                // ステータス: 保存中
                set_system_status(SYSTEM_STATUS_SAVING);

                // 画像保存処理
                bool success = true;

                // オリジナル画像保存
                if (g_camera_utils->save_original_bmp(fb, request.timestamp, g_file_counter))
                {
                    ESP_LOGI(TAG, "💾 オリジナル画像保存完了");
                }
                else
                {
                    ESP_LOGW(TAG, "⚠️ オリジナル画像保存失敗");
                    success = false;
                }

                // 輝度データ計算
                uint8_t *gray_data = g_processor->calculate_luminance(fb);
                if (!gray_data)
                {
                    ESP_LOGE(TAG, "❌ 輝度計算失敗");
                    esp_camera_fb_return(fb);
                    set_system_status(SYSTEM_STATUS_ERROR);
                    status_led_blink(3, 200, 200);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    set_system_status(SYSTEM_STATUS_READY);
                    continue;
                }

                // パレット変換画像保存
                if (request.save_all_palettes)
                {
                    // 全パレット保存
                    for (int i = 0; i < MAX_PALETTE_INDEX; i++)
                    {
                        if (g_processor->save_palette_bmp(gray_data, fb->width, fb->height,
                                                          i, request.timestamp, g_file_counter, g_mount_point))
                        {
                            ESP_LOGI(TAG, "💾 パレット%d画像保存完了", i);
                        }
                        else
                        {
                            ESP_LOGW(TAG, "⚠️ パレット%d画像保存失敗", i);
                            success = false;
                        }
                    }
                }
                else
                {
                    // 選択パレットのみ保存
                    if (g_processor->save_palette_bmp(gray_data, fb->width, fb->height,
                                                      request.palette_index, request.timestamp,
                                                      g_file_counter, g_mount_point))
                    {
                        ESP_LOGI(TAG, "💾 パレット%d画像保存完了", request.palette_index);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "⚠️ パレット%d画像保存失敗", request.palette_index);
                        success = false;
                    }
                }

                // クリーンアップ
                free(gray_data);
                esp_camera_fb_return(fb);

                // 結果表示
                if (success)
                {
                    ESP_LOGI(TAG, "✅ 撮影完了: ファイル番号 %d", g_file_counter);
                    // volatile変数のインクリメントを修正
                    int temp_counter = g_file_counter;
                    temp_counter++;
                    g_file_counter = temp_counter;
                    status_led_blink(2, 100, 100);          // 成功表示
                    set_system_status(SYSTEM_STATUS_READY); // 待機状態に戻る
                }
                else
                {
                    ESP_LOGE(TAG, "❌ 撮影失敗: ファイル番号 %d", g_file_counter);
                    set_system_status(SYSTEM_STATUS_ERROR);
                    status_led_blink(3, 200, 200);          // エラー表示
                    vTaskDelay(pdMS_TO_TICKS(2000));        // 2秒待機
                    set_system_status(SYSTEM_STATUS_READY); // 復帰
                }
            }
        }
    }
}

// ========================================
// エンコーダ監視タスク（コア0で実行）にゃ
// ========================================
static void encoder_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🎛️ エンコーダタスク開始 (Core %d)", xPortGetCoreID());

    while (!g_system_ready)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // システム準備完了まで待機
    }

    // 初期LED設定
    if (g_encoder && g_encoder->is_initialized())
    {
        ESP_LOGI(TAG, "🎨 初期パレット設定: %d", g_current_palette_index);
        g_encoder->set_led_color(PALETTE_REP_COLORS[g_current_palette_index]);
    }

    uint32_t update_counter = 0;

    while (1)
    {
        update_encoder();
        update_counter++;

        // 10秒ごとに統計表示
        if (update_counter % 500 == 0)
        { // 50Hz × 500 = 10秒
            ESP_LOGI(TAG, "🎛️ エンコーダ統計: 更新回数=%lu, 現在値=%d",
                     (unsigned long)update_counter, g_current_palette_index);
        }

        // ディスプレイのパレット表示を更新
        if (g_display_enabled && g_system_status == SYSTEM_STATUS_READY)
        {
            update_display_status("Ready", g_current_palette_index);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz更新
    }
}

// ========================================
// ボタン監視タスク（コア0で実行）にゃ
// ========================================
static void button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🔘 ボタンタスク開始 (Core %d)", xPortGetCoreID());

    while (!g_system_ready)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // システム準備完了まで待機
    }

    while (1)
    {
        if (is_button_pressed())
        {
            // 操作禁止中は撮影を受け付けない
            if (g_system_status != SYSTEM_STATUS_READY)
            {
                ESP_LOGW(TAG, "⚠️ システムビジー中のため撮影を無視");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            ESP_LOGI(TAG, "🔘 ボタン押下検出！");
            ESP_LOGI(TAG, "🎨 現在のパレット: %d", g_current_palette_index);

            // 長押し判定（1秒以上で全パレット保存）
            uint32_t press_start = esp_timer_get_time() / 1000;
            bool is_long_press = false;

            // ボタンが離されるまで待機（最大2秒）
            while (gpio_get_level(BUTTON_PIN) == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                uint32_t press_duration = (esp_timer_get_time() / 1000) - press_start;

                if (press_duration > 1000 && !is_long_press)
                {
                    is_long_press = true;
                    ESP_LOGI(TAG, "🔘 長押し検出: 全パレット保存モード");
                    // LED点滅で長押し通知
                    status_led_blink(5, 50, 50);
                }

                if (press_duration > 2000)
                    break; // 2秒でタイムアウト
            }

            // 撮影要求
            if (request_capture(is_long_press))
            {
                if (is_long_press)
                {
                    ESP_LOGI(TAG, "📸 全パレット撮影要求送信");
                }
                else
                {
                    ESP_LOGI(TAG, "📸 単一パレット撮影要求送信");
                }
            }
            else
            {
                ESP_LOGW(TAG, "⚠️ 撮影要求送信失敗");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz監視
    }
}

// ========================================
// 統計情報表示タスクにゃ
// ========================================
static void stats_task(void *pvParameters)
{
    ESP_LOGI(TAG, "📊 統計タスク開始 (Core %d)", xPortGetCoreID());

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(30000)); // 30秒間隔

        ESP_LOGI(TAG, "\n📊 === システム統計情報 ===");
        ESP_LOGI(TAG, "🔢 撮影回数: %d", g_file_counter - 1);
        ESP_LOGI(TAG, "🎨 現在のパレット: %d", g_current_palette_index);
        ESP_LOGI(TAG, "🎯 システムステータス: %d", g_system_status);
        ESP_LOGI(TAG, "💾 フリーヒープ: %lu bytes", (unsigned long)esp_get_free_heap_size());
        ESP_LOGI(TAG, "🧠 最小フリーヒープ: %lu bytes", (unsigned long)esp_get_minimum_free_heap_size());

        // PSRAM情報表示（ESP-IDF 5.4対応）
        size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        if (free_psram > 0)
        {
            ESP_LOGI(TAG, "💽 フリーPSRAM: %zu bytes", free_psram);
        }

        // タスク情報表示
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        ESP_LOGI(TAG, "🔄 実行中タスク数: %d", task_count);
        ESP_LOGI(TAG, "============================\n");
    }
}

// ========================================
// ディスプレイプレビュータスクにゃ（オプション）
// ========================================
static void display_preview_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🖥️ ディスプレイプレビュータスク開始 (Core %d)", xPortGetCoreID());

    while (1)
    {
        // システムが準備完了でディスプレイが有効な場合のみプレビュー更新
        if (g_display_enabled && g_system_status == SYSTEM_STATUS_READY)
        {
            // カメラフレームを取得してプレビュー表示
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb)
            {
                show_camera_preview(fb);
                esp_camera_fb_return(fb);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10FPS更新
    }
}

// ========================================
// 全システム初期化にゃ
// ========================================
static esp_err_t init_all_systems()
{
    ESP_LOGI(TAG, "\n🚀 === システム初期化開始 ===");
    set_system_status(SYSTEM_STATUS_INITIALIZING);

    esp_err_t ret;

    // 基本システム初期化
    ret = init_gpio();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_sync_objects();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "同期オブジェクト初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_i2c_master();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // ハードウェア初期化
    ret = init_camera();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "カメラ初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_sdcard();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SDカード初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // アプリケーション初期化
    ret = init_processor();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "画像処理初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = init_camera_utils();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "カメラユーティリティ初期化失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // エンコーダ初期化（オプション）
    init_encoder(); // 失敗してもシステム続行

    // LCD初期化
    init_display(); // 失敗してもシステム続行

    ESP_LOGI(TAG, "✅ === システム初期化完了 ===\n");
    return ESP_OK;
}

// ========================================
// タスク作成にゃ
// ========================================
static esp_err_t create_tasks()
{
    ESP_LOGI(TAG, "🔧 タスク作成開始にゃ");

    // 撮影タスク（コア1、高優先度）
    BaseType_t result = xTaskCreatePinnedToCore(
        capture_task,
        "capture_task",
        CAPTURE_TASK_STACK,
        NULL,
        configMAX_PRIORITIES - 1, // 最高優先度
        NULL,
        1 // コア1
    );
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "❌ 撮影タスク作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_FAIL;
    }

    // エンコーダタスク（コア0、中優先度）
    result = xTaskCreatePinnedToCore(
        encoder_task,
        "encoder_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 3,
        NULL,
        0 // コア0
    );
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "❌ エンコーダタスク作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_FAIL;
    }

    // ボタンタスク（コア0、中優先度）
    result = xTaskCreatePinnedToCore(
        button_task,
        "button_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 3,
        NULL,
        0 // コア0
    );
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "❌ ボタンタスク作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_FAIL;
    }

    // 統計タスク（コア0、低優先度）
    result = xTaskCreatePinnedToCore(
        stats_task,
        "stats_task",
        3072,
        NULL,
        1, // 低優先度
        NULL,
        0 // コア0
    );
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "❌ 統計タスク作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        return ESP_FAIL;
    }

    // ディスプレイプレビュータスク（オプション）
    if (g_display_enabled)
    {
        result = xTaskCreatePinnedToCore(
            display_preview_task,
            "display_preview_task",
            4096,
            NULL,
            configMAX_PRIORITIES - 4, // 低優先度
            NULL,
            0 // コア0
        );
        if (result != pdPASS)
        {
            ESP_LOGW(TAG, "⚠️ ディスプレイプレビュータスク作成失敗");
        }
    }

    ESP_LOGI(TAG, "✅ 全タスク作成完了にゃ");
    return ESP_OK;
}

// ========================================
// システム終了処理にゃ
// ========================================
static void cleanup_system()
{
    ESP_LOGI(TAG, "🧹 システムクリーンアップ開始");

    // エンコーダクリーンアップ
    if (g_encoder)
    {
        delete g_encoder;
        g_encoder = nullptr;
    }

    // ディスプレイクリーンアップ（コメントアウト中）
    /*
    if (g_display)
    {
        delete g_display;
        g_display = nullptr;
    }
    */

    // 画像処理エンジンクリーンアップ
    if (g_processor)
    {
        delete g_processor;
        g_processor = nullptr;
    }

    // カメラユーティリティクリーンアップ
    if (g_camera_utils)
    {
        delete g_camera_utils;
        g_camera_utils = nullptr;
    }

    // カメラクリーンアップ（変数名修正）
    if (g_camera_ready)
    {
        esp_camera_deinit();
        g_camera_ready = false;
    }

    // I2Cドライバー削除（レガシー版）
    i2c_driver_delete(I2C_MASTER_NUM);

    // SDカードクリーンアップ（変数名修正）
    if (g_sd_card_mounted)
    {
        esp_vfs_fat_sdcard_unmount(g_mount_point, g_sd_card);
        g_sd_card_mounted = false;
    }

    // SPIバス削除
    spi_bus_free(SPI2_HOST);

    ESP_LOGI(TAG, "✅ システムクリーンアップ完了");
}

// ========================================
// アプリケーションメインにゃ
// ========================================
extern "C" void app_main()
{
    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  📸 ピクセルアートカメラ ESP-IDF 5.4版 📸  ║");
    ESP_LOGI(TAG, "║       🎨 8色パレット + ステータスLED 🎨      ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "🔧 ESP-IDF Version: %s", esp_get_idf_version());

    // チップ情報取得（ESP-IDF 5.4対応）
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "💾 Chip: %s Rev.%d", CONFIG_IDF_TARGET, chip_info.revision);
    ESP_LOGI(TAG, "🧠 CPU Cores: %d", chip_info.cores);

    // NVS初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "⚠️ NVSを消去して再初期化します");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS初期化完了");

    // システム初期化
    ret = init_all_systems();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ システム初期化失敗: %s", esp_err_to_name(ret));
        set_system_status(SYSTEM_STATUS_ERROR);
        cleanup_system();
        return;
    }

    // タスク作成
    ret = create_tasks();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ タスク作成失敗");
        set_system_status(SYSTEM_STATUS_ERROR);
        cleanup_system();
        return;
    }

    // システム準備完了
    g_system_ready = true;
    set_system_status(SYSTEM_STATUS_READY); // 待機状態に移行

    // 準備完了通知（LED 3回点滅）
    ESP_LOGI(TAG, "\n🎉 ========================================");
    ESP_LOGI(TAG, "✅ システム準備完了にゃ！");
    ESP_LOGI(TAG, "🎨 ステータスLED:");
    ESP_LOGI(TAG, "  🔴 赤点滅: 操作禁止中（起動・撮影・保存）");
    ESP_LOGI(TAG, "  🔵 青点灯: 操作可能（待機中）");
    ESP_LOGI(TAG, "🔘 ショートプレス: 選択パレットで撮影");
    ESP_LOGI(TAG, "🔘 ロングプレス(1秒): 全パレットで撮影");
    if (g_encoder && g_encoder->is_initialized())
    {
        ESP_LOGI(TAG, "🎛️ エンコーダ: パレット選択(0-7)");
    }
    else
    {
        ESP_LOGI(TAG, "⚠️ エンコーダ: 検出されませんでした");
    }
    ESP_LOGI(TAG, "========================================\n");

    status_led_blink(3, 100, 100);

    // メインタスクは終了（他のタスクが動作継続）
    ESP_LOGI(TAG, "🔚 メインタスク終了、他のタスクが動作を継続します");
}