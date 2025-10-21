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
/*
#include "PimoroniEncoder.h"
#include "PixelArtProcessor.h"
#include "camera_utils.h"
#include "ssd1306_display.h"
*/

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

// I2C設定
#define I2C_SDA_PIN GPIO_NUM_1   // 外部I2C SDA
#define I2C_SCL_PIN GPIO_NUM_2   // 外部I2C SCL
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

// ハードウェア
// static i2c_master_bus_handle_t g_i2c_bus_handle = NULL; // 新しいI2Cバスハンドル
//static PimoroniEncoder *g_encoder = nullptr;
//static PixelArtProcessor *g_processor = nullptr;
//static CameraUtils *g_camera_utils = nullptr;
static bool g_camera_ready = false;

// SSD1306ディスプレイオブジェクト
//static SSD1306Display *g_display = nullptr;
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


}