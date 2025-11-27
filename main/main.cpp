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
#include <dirent.h>

// ESP-IDF 5.4コア
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"
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

// USB MSC
#include "tinyusb.h"
#include "tusb_msc_storage.h"

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

// BMPヘッダー構造体
#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;           // ファイルタイプ (0x4D42 = "BM")
    uint32_t bfSize;           // ファイルサイズ
    uint16_t bfReserved1;      // 予約領域1
    uint16_t bfReserved2;      // 予約領域2
    uint32_t bfOffBits;        // 画像データまでのオフセット
    uint32_t biSize;           // 情報ヘッダサイズ
    int32_t  biWidth;          // 画像の幅
    int32_t  biHeight;         // 画像の高さ
    uint16_t biPlanes;         // プレーン数
    uint16_t biBitCount;       // 1ピクセルあたりのビット数
    uint32_t biCompression;    // 圧縮形式
    uint32_t biSizeImage;      // 画像データサイズ
    int32_t  biXPelsPerMeter;  // 水平解像度
    int32_t  biYPelsPerMeter;  // 垂直解像度
    uint32_t biClrUsed;        // 使用する色数
    uint32_t biClrImportant;   // 重要な色数
} bitmap_header_t;
#pragma pack(pop)

#define SHUTTER_BUTTON_PIN GPIO_NUM_38
#define MENU_BUTTON_PIN GPIO_NUM_39

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
#define MAX_RESOLUTION_INDEX 9
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 176
#define CAPTURE_TASK_STACK 8192
#define PROCESS_TASK_STACK 16384
#define ENCODER_TASK_STACK 4096

// NVS設定
#define NVS_NAMESPACE "pict_camera"
#define NVS_KEY_PALETTE "palette_idx"
#define NVS_KEY_RESOLUTION "resolution"

// 注: エンコーダーのフィルタリング・デバウンス設定はドライバ内部に移行しました
// (pimoroni_encoder.cpp参照)

// 構造体定義
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
    SYSTEM_STATUS_SAVING,
    SYSTEM_STATUS_USB_MSC,
    SYSTEM_STATUS_ERROR,
} system_status_t;

// メニュー項目定義（エンコーダの動作が変わる）
typedef enum
{
    MENU_ITEM_CAPTURE = 0, //撮影モード(シャッターで撮影、メニュー非表示)
    MENU_ITEM_PALETTE, //カラーパレット変更
    MENU_ITEM_RESOLUTION, //解像度変更(シャッターで決定)
    MENU_ITEM_PHOTO_LIST, //撮影画像見る
    MENU_ITEM_PALETTE_READ, //シャッターでSDカードからパレット読み込み
    MENU_ITEM_USB, //シャッターでUSB MSCモード
} menu_list_t;

// 注: MENU_ITEM_CAPTUREはメニューに表示されない特殊なモード
// 解像度項目定義
typedef enum
{
    RESOLUTION_128x128 = 0,    //FRAMESIZE_128X128,2
    RESOLUTION_176x144,    //FRAMESIZE_QCIF,   3
    RESOLUTION_240x176,    //FRAMESIZE_HQVGA,  4
    RESOLUTION_240x240,    //FRAMESIZE_240X240,5
    RESOLUTION_320x240,    //FRAMESIZE_QVGA,   6
    RESOLUTION_320x320,    //FRAMESIZE_320X320,7
    RESOLUTION_400x296,    //FRAMESIZE_CIF,    8
    RESOLUTION_480x320,    //FRAMESIZE_HVGA,   9
    RESOLUTION_640x480,    //FRAMESIZE_VGA,    10
} resolution_t;

// 解像度→framesize_t変換テーブル
static const framesize_t RESOLUTION_TO_FRAMESIZE[9] = {
    FRAMESIZE_128X128,   // RESOLUTION_128x128
    FRAMESIZE_QCIF,      // RESOLUTION_176x144
    FRAMESIZE_HQVGA,     // RESOLUTION_240x176
    FRAMESIZE_240X240,   // RESOLUTION_240x240
    FRAMESIZE_QVGA,      // RESOLUTION_320x240
    FRAMESIZE_320X320,   // RESOLUTION_320x320
    FRAMESIZE_CIF,       // RESOLUTION_400x296
    FRAMESIZE_HVGA,      // RESOLUTION_480x320
    FRAMESIZE_VGA,       // RESOLUTION_640x480
};

//メニュー用グローバル変数　最初は撮影モード
static menu_list_t g_current_menu = MENU_ITEM_CAPTURE;
// 現在選択中のパレット
static volatile uint8_t g_current_palette_index = 0;
// 現在選択中の解像度
static volatile resolution_t g_current_resolution = RESOLUTION_240x240;

// グローバル変数 - エンコーダーのみC構造体に変更
static SemaphoreHandle_t g_capture_semaphore = NULL;
static SemaphoreHandle_t g_i2c_mutex = NULL;
static SemaphoreHandle_t g_display_mutex = NULL;
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

// 撮影画像ファイル連番
static volatile uint16_t g_file_counter = 0;

// ハードウェアオブジェクト - エンコーダーのみC構造体に変更
static pimoroni_encoder_t g_encoder;
static CameraUtils *g_camera_utils = nullptr;
static SSD1306Display *g_display = nullptr;

static volatile bool g_system_ready = false;
static bool g_camera_ready = false;
static bool g_encoder_ready = false;
static bool g_display_ready = false;
static bool g_sd_card_ready = false;

// システム状態管理
static volatile system_status_t g_system_status = SYSTEM_STATUS_INITIALIZING;

static sdmmc_card_t *g_sd_card = NULL;
static const char *g_mount_point = "/sdcard";
static bool g_sd_card_mounted = false;

// USB MSC状態管理
static bool g_usb_msc_active = false;

// ========================================
// カラーパレット定義（8種類×8色）
// ========================================
static const uint32_t COLOR_PALETTES_8[8][8] = {
  { // パレット0 slso8
    0x0D2B45, 0x203C56, 0x544E68, 0x8D697A, 0xD08159, 0xFFAA5E, 0xFFD4A3, 0xFFECD6 },
  { // パレット1 都市伝説解体センター風
    0x000000, 0x000B22, 0x112B43, 0x437290, 0x437290, 0xE0D8D1, 0xE0D8D1, 0xFFFFFF },
  { // パレット2 ファミレスを享受せよ風
    0x010101, 0x33669F, 0x33669F, 0x33669F, 0x498DB7, 0x498DB7, 0xFBE379, 0xFBE379 },
  { // パレット3 gothic-bit
    0x0E0E12, 0x1A1A24, 0x333346, 0x535373, 0x8080A4, 0xA6A6BF, 0xC1C1D2, 0xE6E6EC },
  { // パレット4 noire-truth
    0x1E1C32, 0x1E1C32, 0x1E1C32, 0x1E1C32, 0xC6BAAC, 0xC6BAAC, 0xC6BAAC, 0xC6BAAC },
  { // パレット5 2BIT DEMIBOY
    0x252525, 0x252525, 0x4B564D, 0x4B564D, 0x9AA57C, 0x9AA57C, 0xE0E9C4, 0xE0E9C4 },
  { // パレット6 deep-maze
    0x001D2A, 0x085562, 0x009A98, 0x00BE91, 0x38D88E, 0x9AF089, 0xF2FF66, 0xF2FF66 },
  { // パレット7 night-rain
    0x000000, 0x012036, 0x3A7BAA, 0x7D8FAE, 0xA1B4C1, 0xF0B9B9, 0xFFD159, 0xFFFFFF },
};

// ========================================
// カラーパレット定義（8種類×16色）
// 8色パレットの各テーマを16色に拡張
// ========================================
static const uint32_t COLOR_PALETTES_16[8][16] = {
  { // パレット0 slso8 (16色版)
    0x071622, 0x0D2B45, 0x17334F, 0x203C56, 0x3C4560, 0x544E68, 0x6E5C71, 0x8D697A,
    0xAB7969, 0xD08159, 0xE7954D, 0xFFAA5E, 0xFFBE80, 0xFFD4A3, 0xFFE3C0, 0xFFECD6 },
  { // パレット1 都市伝説解体センター風 (16色版)
    0x000000, 0x000611, 0x000B22, 0x091B33, 0x112B43, 0x295168, 0x437290, 0x5A8BA9,
    0x84A9BD, 0xA1BFD0, 0xB8D0DC, 0xD0DBDF, 0xE0D8D1, 0xE8E0DA, 0xF4ECE8, 0xFFFFFF },
  { // パレット2 ファミレスを享受せよ風 (16色版)
    0x010101, 0x1A3350, 0x264D7F, 0x33669F, 0x3D78AD, 0x498DB7, 0x5C9EC4, 0x71AFD0,
    0x8BC6E1, 0xA8D7EC, 0xC5E5F4, 0xD9EEFB, 0xE5D35E, 0xF0DC6B, 0xFBE379, 0xFFEE9E },
  { // パレット3 gothic-bit (16色版)
    0x07070A, 0x0E0E12, 0x14141B, 0x1A1A24, 0x26262F, 0x333346, 0x434359, 0x535373,
    0x696989, 0x8080A4, 0x9393B2, 0xA6A6BF, 0xB4B4C9, 0xC1C1D2, 0xD4D4E0, 0xE6E6EC },
  { // パレット4 noire-truth (16色版)
    0x0F0D19, 0x1E1C32, 0x2D2A4B, 0x3C3964, 0x4E4877, 0x60588A, 0x766F9D, 0x8D86B0,
    0x9F99BB, 0xB3ADC4, 0xC6BAAC, 0xCEC3B5, 0xD6CCBE, 0xDED5C7, 0xE6DDD0, 0xEEE6D9 },
  { // パレット5 2BIT DEMIBOY (16色版)
    0x121212, 0x252525, 0x383837, 0x4B4B49, 0x3D4239, 0x4B564D, 0x6D7961, 0x8A9470,
    0x9AA57C, 0xABB58E, 0xBCC5A0, 0xCDD5B2, 0xD4DEB8, 0xDDE7C0, 0xE0E9C4, 0xEDF4DC },
  { // パレット6 deep-maze (16色版)
    0x000E15, 0x001D2A, 0x043B46, 0x085562, 0x00777D, 0x009A98, 0x00AAAA, 0x00BE91,
    0x1ECB8F, 0x38D88E, 0x69E18D, 0x9AF089, 0xC8F67A, 0xDDFA6E, 0xF2FF66, 0xF8FF99 },
  { // パレット7 night-rain (16色版)
    0x000000, 0x01101B, 0x012036, 0x1D3A5A, 0x2E5478, 0x3A7BAA, 0x5988B5, 0x7D8FAE,
    0x8F9FB9, 0xA1B4C1, 0xB8C4CE, 0xD4CFD0, 0xE2D4D4, 0xF0B9B9, 0xFFD159, 0xFFFFFF },
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

static const char* PALETTE_NAMES[8] = {
    "slso",
    "lege",
    "fami",
    "goth",
    "noir",
    "demi",
    "maze",
    "nigh"
};

// パレット説明文（カメラプレビュー領域に表示）
static const char* PALETTE_DESCRIPTIONS[8] = {
    "slso8\nあたたかみの\nあるいろ",
    "としでんせつ\nかいたい\nセンター",
    "ファミレスを\nきょうじゅ\nせよ",
    "gothic-bit\nモノクロ\nちっく",
    "noire-truth\nしろくろ\n2しょく",
    "2BIT DEMIBOY\nしぜんな\nみどり",
    "deep-maze\nあかるい\nグラデ",
    "night-rain\nよぞらと\nゆうひ"
};

// メニュー項目名（8文字以内、日本語）
// 注: MENU_ITEM_CAPTURE(0)はメニューに表示されないため、インデックス1から開始
static const char* MENU_ITEM_NAMES[5] = {
    "パレット",
    "かいぞうど",
    "しゃしん",
    "SDよみこみ",
    "USB MSC"
};

// 解像度項目名（8文字以内、日本語）
static const char* RESOLUTION_NAMES[9] = {
    "128SQ",
    "QCIF",
    "HQVGA",
    "240SQ",
    "QVGA",
    "320SQ",
    "CIF",
    "HVGA",
    "VGA",
};

// 解像度の実際の寸法（幅、高さ）
static const int RESOLUTION_DIMENSIONS[9][2] = {
    {128, 128},  // RESOLUTION_128x128
    {176, 144},  // RESOLUTION_176x144 (QCIF)
    {240, 176},  // RESOLUTION_240x176 (HQVGA)
    {240, 240},  // RESOLUTION_240x240
    {320, 240},  // RESOLUTION_320x240 (QVGA)
    {320, 320},  // RESOLUTION_320x320
    {400, 296},  // RESOLUTION_400x296 (CIF)
    {480, 320},  // RESOLUTION_480x320 (HVGA)
    {640, 480},  // RESOLUTION_640x480 (VGA)
};

// 解像度説明文（カメラプレビュー領域に表示）
static const char* RESOLUTION_DESCRIPTIONS[9] = {
    "128x128\nせいほうけい\nちいさめ",
    "176x144\nQCIF\nよこなが",
    "240x176\nHQVGA\nよこなが",
    "240x240\nせいほうけい\nちゅうがた",
    "320x240\nQVGA\nよこなが",
    "320x320\nせいほうけい\nおおがた",
    "400x296\nCIF\nよこなが",
    "480x320\nHVGA\nよこなが",
    "640x480\nVGA\nひょうじゅん"
};

// 関数プロトタイプ
void update_button_state(button_state_t *button);
button_event_t get_button_event(button_state_t *button);
void process_button_events(void);
void encoder_task(void *parameter);
void camera_preview_task(void *parameter);
void histogram_task(void *parameter);
void menu_display_task(void *parameter);
void capture_task(void *parameter);
esp_err_t init_sd_card(void);
void print_sd_card_info(void);
esp_err_t init_external_i2c(void);
esp_err_t scan_i2c_devices(i2c_port_t i2c_num, int *found_devices);
esp_err_t init_gpio(void);
void display_init_step(Terminal *terminal, const char *step_name);
void display_init_step(Terminal *terminal, bool success);
void run_display_test_patterns(void);
void init_file_counter_from_sd(void);
void generate_random_string(char* buf, int len);
void apply_palette_reduction(uint8_t* rgb_data, int width, int height, int palette_idx, int color_count);
esp_err_t save_rgb_as_bmp(uint8_t* rgb_data, int width, int height, const char* filepath);
void draw_progress_bar(float progress, const char* status_text = nullptr);
void start_capture(bool all_palettes);
static void load_settings_from_nvs(void);
static void save_settings_to_nvs(void);

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

esp_err_t deinit_sd_card(void)
{
    ESP_LOGI(TAG, "SDカードアンマウント開始");

    if (!g_sd_card_mounted) {
        ESP_LOGW(TAG, "SDカードは既にアンマウント済み");
        return ESP_OK;
    }

    // SDカードをアンマウント
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(g_mount_point, g_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDカードアンマウント失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // SPIバスを解放
    ret = spi_bus_free(SPI2_HOST);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIバス解放失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    g_sd_card_mounted = false;
    g_sd_card = NULL;
    ESP_LOGI(TAG, "✅ SDカードアンマウント完了");
    return ESP_OK;
}

// ========================================
// USB MSC関連関数
// ========================================

// TinyUSBドライバインストール済みフラグ
static bool g_tinyusb_driver_installed = false;
// USB MSCストレージ初期化済みフラグ
static bool g_usb_msc_initialized = false;

esp_err_t init_tinyusb_driver(void)
{
    ESP_LOGI(TAG, "📦 TinyUSBドライバ初期化開始");
    ESP_LOGI(TAG, "💾 空きヒープ（開始時）: %u bytes", (unsigned int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "💾 空き内部RAM: %u bytes", (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "💾 空きDMA RAM: %u bytes", (unsigned int)heap_caps_get_free_size(MALLOC_CAP_DMA));

    if (g_tinyusb_driver_installed) {
        ESP_LOGW(TAG, "⚠️ TinyUSBドライバは既にインストール済みです");
        return ESP_OK;
    }

    // ウォッチドッグをリセット（長時間処理のため）
    ESP_LOGI(TAG, "🐕 ウォッチドッグリセット");
    esp_task_wdt_reset();

    ESP_LOGI(TAG, "🔧 TinyUSB設定準備中...");
    // TinyUSBドライバをインストール（MSC専用デバイス）
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,          // デフォルト記述子を使用
        .string_descriptor = NULL,          // デフォルト文字列を使用
        .string_descriptor_count = 0,       // デフォルト文字列数
        .external_phy = false,              // 内蔵PHYを使用
        .configuration_descriptor = NULL,   // デフォルト設定を使用（MSC専用）
        .self_powered = false,              // バスパワーデバイス
        .vbus_monitor_io = -1               // VBUS検出GPIO無効化（重要: USB接続時のクラッシュ回避）
    };

    ESP_LOGI(TAG, "📞 tinyusb_driver_install()呼び出し直前...");
    ESP_LOGI(TAG, "⏱️  タスク優先度: %d", uxTaskPriorityGet(NULL));

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);

    ESP_LOGI(TAG, "📞 tinyusb_driver_install()完了: %s", esp_err_to_name(ret));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ TinyUSBドライバインストール失敗: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "💾 空きヒープ（失敗時）: %u bytes", (unsigned int)esp_get_free_heap_size());
        return ret;
    }

    ESP_LOGI(TAG, "💾 空きヒープ（インストール後）: %u bytes", (unsigned int)esp_get_free_heap_size());
    ESP_LOGI(TAG, "💾 空き内部RAM（インストール後）: %u bytes", (unsigned int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    g_tinyusb_driver_installed = true;

    // ウォッチドッグをリセット（USBバス安定化待機前）
    esp_task_wdt_reset();

    ESP_LOGI(TAG, "⏳ USBバス安定化待機中（500ms）...");
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "✅ TinyUSBドライバ初期化完了（MSC専用デバイス）");
    return ESP_OK;
}

esp_err_t start_usb_msc(void)
{
    ESP_LOGI(TAG, "🔌 USB MSCモード開始");

    if (g_usb_msc_active) {
        ESP_LOGW(TAG, "⚠️ USB MSCモードは既に有効です");
        return ESP_OK;
    }

    if (!g_tinyusb_driver_installed) {
        ESP_LOGE(TAG, "❌ TinyUSBドライバが初期化されていません");
        ESP_LOGE(TAG, "   起動時の初期化に失敗した可能性があります");
        return ESP_FAIL;
    }

    if (!g_sd_card || !g_sd_card_mounted) {
        ESP_LOGE(TAG, "❌ SDカードがマウントされていません");
        return ESP_FAIL;
    }

    // 通常のSDカードをアンマウント
    ESP_LOGI(TAG, "📂 SDカードをアンマウント中...");
    esp_err_t ret = deinit_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SDカードアンマウント失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ SDカードアンマウント完了");

    // TinyUSB MSCストレージとしてSDカードを初期化
    ESP_LOGI(TAG, "💾 USB MSCストレージ初期化中...");
    const tinyusb_msc_sdmmc_config_t msc_cfg = {
        .card = g_sd_card,
        .callback_mount_changed = NULL,
        .callback_premount_changed = NULL,
        .mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        }
    };

    ret = tinyusb_msc_storage_init_sdmmc(&msc_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ USB MSCストレージ初期化失敗: %s", esp_err_to_name(ret));
        // エラー時は通常のSDカードを再マウント
        ESP_LOGW(TAG, "🔄 SDカードを再マウント中...");
        init_sd_card();
        return ret;
    }

    g_usb_msc_initialized = true;
    g_usb_msc_active = true;
    g_sd_card_mounted = false;

    ESP_LOGI(TAG, "✅ USB MSCモード開始完了");
    ESP_LOGI(TAG, "🖥️  PCからマスストレージデバイスとして見えます");
    return ESP_OK;
}

esp_err_t stop_usb_msc(void)
{
    ESP_LOGI(TAG, "🔌 USB MSCモード停止");

    if (!g_usb_msc_active) {
        ESP_LOGW(TAG, "⚠️ USB MSCモードは既に停止しています");
        return ESP_OK;
    }

    if (!g_usb_msc_initialized) {
        ESP_LOGE(TAG, "❌ USB MSCストレージが初期化されていません");
        g_usb_msc_active = false;
        return ESP_FAIL;
    }

    // TinyUSB MSCストレージを解放
    ESP_LOGI(TAG, "🗑️  USB MSCストレージ解放中...");
    tinyusb_msc_storage_deinit();
    g_usb_msc_initialized = false;
    ESP_LOGI(TAG, "✅ USB MSCストレージ解放完了");

    // 少し待機してから再マウント
    vTaskDelay(pdMS_TO_TICKS(200));

    // 通常のSDカードを再マウント
    ESP_LOGI(TAG, "📂 SDカード再マウント中...");
    esp_err_t ret = init_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SDカード再マウント失敗: %s", esp_err_to_name(ret));
        g_usb_msc_active = false;
        return ret;
    }

    g_usb_msc_active = false;
    ESP_LOGI(TAG, "✅ USB MSCモード停止完了");
    ESP_LOGI(TAG, "📷 通常の撮影モードに戻りました");
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


// ボタン処理関数群
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
        button->last_state = false;  // ★ イベント処理後に状態をクリア

        if (press_duration < BUTTON_LONG_PRESS_MS)
        {
            return BUTTON_EVENT_SHORT_PRESS;
        }
        else
        {
            // 長押し後のリリースもクリア
            return BUTTON_EVENT_NONE;
        }
    }

    return BUTTON_EVENT_NONE;
}

void process_button_events(void)
{
    // 撮影中、保存中、またはUSB MSCモード中は入力を無視（USB MSCモードを除く）
    if (g_system_status == SYSTEM_STATUS_CAPTURING || g_system_status == SYSTEM_STATUS_SAVING)
    {
        return;
    }

    // USB MSCモード中はメニューボタン以外の操作を制限
    if (g_system_status == SYSTEM_STATUS_USB_MSC && g_current_menu != MENU_ITEM_USB)
    {
        return;
    }

    button_event_t shutter_event = get_button_event(&g_shutter_button);

    if (shutter_event != BUTTON_EVENT_NONE)
    {
        ESP_LOGI(TAG, "🔔 シャッターボタンイベント検出: %d", shutter_event);
    }

    switch (shutter_event)
    {
    case BUTTON_EVENT_SHORT_PRESS:
        if (g_current_menu == MENU_ITEM_CAPTURE)
        {
            // 撮影モード: 単一パレット撮影
            ESP_LOGI(TAG, "📸 シャッター短押し: 選択中のパレットで撮影");
            start_capture(false);
        }
        else if (g_current_menu == MENU_ITEM_PALETTE)
        {
            // パレットモード: 次のパレットに変更（0-7の循環）
            g_current_palette_index = (g_current_palette_index + 1) % MAX_PALETTE_INDEX;
            ESP_LOGI(TAG, "🎨 パレット変更: %s (index: %d)",
                     PALETTE_NAMES[g_current_palette_index], g_current_palette_index);

            // エンコーダーLEDの色も変更
            if (g_encoder_ready)
            {
                uint32_t color = PALETTE_REP_COLORS[g_current_palette_index];
                uint8_t r = (color >> 16) & 0xFF;
                uint8_t g = (color >> 8) & 0xFF;
                uint8_t b = color & 0xFF;
                pimoroni_encoder_set_led(&g_encoder, r, g, b);
            }
        }
        else if (g_current_menu == MENU_ITEM_RESOLUTION)
        {
            // 解像度モード: 次の解像度に変更（0-8の循環）
            g_current_resolution = (resolution_t)((g_current_resolution + 1) % MAX_RESOLUTION_INDEX);
            ESP_LOGI(TAG, "📐 解像度変更: %s (index: %d)",
                     RESOLUTION_NAMES[g_current_resolution], g_current_resolution);
        }
        else if (g_current_menu == MENU_ITEM_USB)
        {
            // USB MSCモード: オン/オフ切り替え
            if (g_usb_msc_active)
            {
                // USB MSCモードを停止して通常モードに戻る
                if (stop_usb_msc() == ESP_OK)
                {
                    g_system_status = SYSTEM_STATUS_READY;
                }
            }
            else
            {
                // USB MSCモードを開始
                if (start_usb_msc() == ESP_OK)
                {
                    g_system_status = SYSTEM_STATUS_USB_MSC;
                }
            }
        }
        else
        {
            ESP_LOGD(TAG, "⚠️ シャッター短押し: このモードでは機能なし");
        }
        break;
    case BUTTON_EVENT_LONG_PRESS:
        if (g_current_menu == MENU_ITEM_CAPTURE)
        {
            // 撮影モード: 全パレット撮影
            ESP_LOGI(TAG, "📸 シャッター長押し: 全パレット撮影");
            start_capture(true);
        }
        else
        {
            ESP_LOGD(TAG, "⚠️ シャッター長押し: このモードでは機能なし");
        }
        break;
    default:
        break;
    }

    button_event_t menu_event = get_button_event(&g_menu_button);
    switch (menu_event)
    {
    case BUTTON_EVENT_SHORT_PRESS:
        if (g_current_menu == MENU_ITEM_CAPTURE)
        {
            // 撮影モードの場合: メニューモード（パレット）に切り替え
            ESP_LOGI(TAG, "📋 撮影モード→メニューモード（パレット）");
            g_current_menu = MENU_ITEM_PALETTE;
        }
        else if (g_current_menu == MENU_ITEM_USB)
        {
            // USBモードの場合: 撮影モードに戻る
            ESP_LOGI(TAG, "📋 USBモードから撮影モードへ戻る");
            g_current_menu = MENU_ITEM_CAPTURE;
            save_settings_to_nvs();
        }
        else
        {
            // メニューモード内での移動: 次のメニュー項目へ（1→2→3→4→5→1の循環）
            g_current_menu = (menu_list_t)((g_current_menu == MENU_ITEM_USB) ? MENU_ITEM_PALETTE : (g_current_menu + 1));
            ESP_LOGI(TAG, "📋 メニュー切り替え: %s", MENU_ITEM_NAMES[g_current_menu - 1]);
        }

        // エンコーダ値をクリアして負の値の累積を防止
        if (g_encoder_ready)
        {
            pimoroni_encoder_clear(&g_encoder);
            ESP_LOGD(TAG, "エンコーダークリア実行");
        }
        break;
    case BUTTON_EVENT_LONG_PRESS:
        // メニューボタン長押し: 常に撮影モードに戻る
        ESP_LOGI(TAG, "📋 メニュー長押し: 撮影モードへ戻る");
        g_current_menu = MENU_ITEM_CAPTURE;
        save_settings_to_nvs();

        // エンコーダ値をクリア
        if (g_encoder_ready)
        {
            pimoroni_encoder_clear(&g_encoder);
            ESP_LOGD(TAG, "エンコーダークリア実行");
        }
        break;
    default:
        break;
    }
}

/**
 * @brief 安全なmodulo演算（常に正の結果を返す）
 *
 * 負の値でも正しく0～modulus-1の範囲に収める
 * 例: safe_modulo(-23, 8) = 1
 *     safe_modulo(-1, 8) = 7
 *     safe_modulo(9, 8) = 1
 */
static inline int16_t safe_modulo(int16_t value, int16_t modulus)
{
    return ((value % modulus) + modulus) % modulus;
}

// ★★★ チャタリング対策を追加したエンコーダータスク ★★★
void encoder_task(void *parameter)
{
    ESP_LOGI(TAG, "🔄 エンコーダータスク開始（ドライバ内部でフィルタリング済み）");

    int16_t last_encoder_value = 0;

    while (1)
    {
        if (g_encoder_ready && g_system_status == SYSTEM_STATUS_READY)
        {
            // ドライバから安定した値を取得（フィルタリング・デバウンス済み）
            int16_t current_value = pimoroni_encoder_get_value(&g_encoder);
            uint32_t current_time = esp_timer_get_time() / 1000;

            // エンコーダーは現在使用していない（シャッターボタンで設定変更）
            // 将来的に別の機能に使用する可能性があるため、タスクは残す
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // アプリケーション側のポーリング間隔（ドライバは独自にポーリング）
        vTaskDelay(pdMS_TO_TICKS(50));
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
        // 撮影中または保存中、または撮影モード以外はプレビューを停止
        if (g_system_status != SYSTEM_STATUS_READY || g_current_menu != MENU_ITEM_CAPTURE)
        {
            vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
            continue;
        }

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

            // ディスプレイに描画（ミューテックス保護）
            if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // clear()を呼ばない - プレビュー領域のみを更新して他の領域を保持

                for (int y = 0; y < PREVIEW_SIZE; y++)
                {
                    for (int x = 0; x < PREVIEW_SIZE; x++)
                    {
                        bool pixel = preview_buffer[y * PREVIEW_SIZE + x];
                        g_display->set_pixel(x, y, pixel);
                    }
                }

                // display()はメニュータスクに任せるため、ここでは呼ばない

                xSemaphoreGive(g_display_mutex);
            }
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

// ★★★ ヒストグラムタスク ★★★
void histogram_task(void *parameter)
{
    ESP_LOGI(TAG, "📊 ヒストグラムタスク開始");

    const int HISTOGRAM_BINS = 16;  // 明るさを16段階に分割
    const int UPDATE_INTERVAL_MS = 200;  // 更新間隔
    const int GRAPH_X = 64;  // グラフの開始X座標
    const int GRAPH_Y = 48;  // グラフの開始Y座標
    const int GRAPH_WIDTH = 16;  // グラフの幅（16本の棒）
    const int GRAPH_HEIGHT = 16;  // グラフの高さ
    const float MAX_PERCENTAGE = 50.0f;  // グラフ最大値 = 50%
    const int WARNING_X = 80;  // 警告表示のX座標
    const int WARNING_Y = 56;  // 警告表示のY座標
    const float OVEREXPOSURE_PERCENTAGE = 10.0f;  // 白飛び検出閾値（総ピクセル数の10%）

    uint32_t histogram[HISTOGRAM_BINS] = {0};

    while (1)
    {
        // 撮影中または保存中、または撮影モード以外はヒストグラムを停止
        if (g_system_status != SYSTEM_STATUS_READY || g_current_menu != MENU_ITEM_CAPTURE)
        {
            vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
            continue;
        }

        if (g_camera_ready && g_display_ready)
        {
            // カメラからフレーム取得
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb == NULL)
            {
                ESP_LOGW(TAG, "ヒストグラム: カメラフレーム取得失敗");
                vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
                continue;
            }

            // ヒストグラムをリセット
            memset(histogram, 0, sizeof(histogram));

            // 元画像のサイズを取得
            int src_width = fb->width;
            int src_height = fb->height;
            int total_pixels = src_width * src_height;

            // 全ピクセルの明るさを計算してヒストグラムに集計
            for (int y = 0; y < src_height; y++)
            {
                for (int x = 0; x < src_width; x++)
                {
                    int pixel_index = y * src_width + x;
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
                        r = g = b = 0;
                    }

                    // 輝度計算（ITU-R BT.601）
                    uint8_t luminance = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);

                    // ヒストグラムのビン番号を計算（0-255を16段階に分割）
                    int bin = luminance / 16;
                    if (bin >= HISTOGRAM_BINS) bin = HISTOGRAM_BINS - 1;

                    histogram[bin]++;
                }
            }

            // フレームバッファ解放
            esp_camera_fb_return(fb);

            // 白飛び検出閾値を計算（総ピクセル数の割合ベース）
            int overexposure_threshold = (int)(total_pixels * OVEREXPOSURE_PERCENTAGE / 100.0f);
            bool overexposed = (histogram[HISTOGRAM_BINS - 1] >= overexposure_threshold);

            // ディスプレイに描画（ミューテックス保護）
            if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // ヒストグラムグラフを描画（縦棒グラフ、割合ベース）
                for (int i = 0; i < HISTOGRAM_BINS; i++)
                {
                    // 割合を計算
                    float percentage = (histogram[i] * 100.0f) / total_pixels;

                    // 棒の高さを計算（50%が最大値）
                    int bar_height = (int)((percentage * GRAPH_HEIGHT) / MAX_PERCENTAGE);
                    if (bar_height > GRAPH_HEIGHT) bar_height = GRAPH_HEIGHT;

                    // 棒を下から上に描画
                    int bar_x = GRAPH_X + i;
                    for (int h = 0; h < bar_height; h++)
                    {
                        int bar_y = GRAPH_Y + GRAPH_HEIGHT - 1 - h;  // 下から上へ
                        g_display->set_pixel(bar_x, bar_y, true);
                    }
                    for (int h = bar_height; h < GRAPH_HEIGHT; h++)
                    {
                        int bar_y = GRAPH_Y + GRAPH_HEIGHT - 1 - h;
                        g_display->set_pixel(bar_x, bar_y, false);
                    }
                }

                // 白飛び警告表示
                if (overexposed)
                {
                    // 「！」マークを表示（簡易的に縦線と点で表現）
                    // 縦線（3ピクセル）
                    g_display->set_pixel(WARNING_X, WARNING_Y, true);
                    g_display->set_pixel(WARNING_X, WARNING_Y + 1, true);
                    g_display->set_pixel(WARNING_X, WARNING_Y + 2, true);
                    // 点（1ピクセル、1ピクセル空けて）
                    g_display->set_pixel(WARNING_X, WARNING_Y + 4, true);

                    // 割合も計算してログ出力
                    float overexposed_percentage = (histogram[HISTOGRAM_BINS - 1] * 100.0f) / total_pixels;
                    ESP_LOGW(TAG, "⚠️ 白飛び検出: 最大明るさ段階に%ld個のピクセル (%.1f%%)",
                             histogram[HISTOGRAM_BINS - 1], overexposed_percentage);
                }

                // 画面更新はメニュータスクに任せる（display()を呼ばない）

                xSemaphoreGive(g_display_mutex);
            }
        }
        else
        {
            // カメラまたはディスプレイが準備できていない
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

// メニュー表示タスク
void menu_display_task(void *parameter)
{
    ESP_LOGI(TAG, "📋 メニュー表示タスク開始");

    const int UPDATE_INTERVAL_MS = 200;  // 更新間隔

    // メニュー用ターミナル作成（8文字×5行）
    Terminal menu_terminal(8, 5);
    menu_terminal.init();
    menu_terminal.set_position(64, 0);  // 表示位置 (64, 0)
    menu_terminal.set_border(false);     // 枠線なし
    menu_terminal.set_auto_wrap(false);
    menu_terminal.set_auto_scroll(false);

    // 撮影モード表示用ターミナル作成
    Terminal capture_terminal(8, 2);
    capture_terminal.init();
    capture_terminal.set_position(64, 0);
    capture_terminal.set_border(false);
    capture_terminal.set_auto_wrap(false);
    capture_terminal.set_auto_scroll(false);

    while (1)
    {
        // 撮影中または保存中はメニュー表示を抑止
        if (g_system_status == SYSTEM_STATUS_CAPTURING || g_system_status == SYSTEM_STATUS_SAVING)
        {
            vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
            continue;
        }

        // 常にメニューを表示（全モードで動作）
        if (g_display_ready && g_display_mutex != NULL &&
            xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (g_current_menu == MENU_ITEM_CAPTURE)
            {
                // 撮影モード: カメラプレビューを優先し、メニューは表示しない

                // メニュー表示領域（右側）のみをクリア
                // プレビュー領域(0-63, 0-63)は残し、右側(64-127, 0-63)をクリア
                for (int y = 0; y < 64; y++)
                {
                    for (int x = 64; x < 128; x++)
                    {
                        g_display->set_pixel(x, y, false);
                    }
                }

                // 「さつえいモード」とだけ表示
                g_display->draw_string(64, 0, "さつえい", true);
                g_display->draw_string(64, 8, "モード", true);

                // 選択中のパレット名を表示
                g_display->draw_string(80, 40, PALETTE_NAMES[g_current_palette_index], true);
                // 選択中の解像度名を表示
                g_display->draw_string(80, 48, RESOLUTION_NAMES[g_current_resolution], true);
                // SDカードの状態を表示
                if (g_sd_card_mounted)
                {
                    g_display->draw_string(80, 56, "SD: OK", true);
                }
                else
                {
                    g_display->draw_string(80, 56, "SD: ERR", true);
                }

                // 画面更新
                g_display->display();
            }
            else
            {
                // メニューモード: 画面をクリアしてメニューを表示
                g_display->clear();

                // パレットモード、解像度モード、USBモードの場合、カメラプレビュー領域に説明文を表示
                if (g_current_menu == MENU_ITEM_PALETTE)
                {
                    // パレット説明文を表示（左側領域、0, 0から）
                    g_display->draw_string(0, 0, PALETTE_DESCRIPTIONS[g_current_palette_index], false);
                }
                else if (g_current_menu == MENU_ITEM_RESOLUTION)
                {
                    // 解像度説明文を表示（左側領域、0, 0から）
                    g_display->draw_string(0, 0, RESOLUTION_DESCRIPTIONS[g_current_resolution], false);
                }
                else if (g_current_menu == MENU_ITEM_USB)
                {
                    // USBモード説明文を表示
                    if (g_usb_msc_active)
                    {
                        g_display->draw_string(0, 0, "USB MSC", false);
                        g_display->draw_string(0, 8, "MODE: ON", false);
                        g_display->draw_string(0, 16, "Connected", false);
                        g_display->draw_string(0, 24, "to PC", false);
                    }
                    else
                    {
                        g_display->draw_string(0, 0, "USB MSC", false);
                        g_display->draw_string(0, 8, "MODE: OFF", false);
                        g_display->draw_string(0, 16, "Press", false);
                        g_display->draw_string(0, 24, "Shutter", false);
                    }
                }

                // ターミナルをクリア
                menu_terminal.clear();

                // メニュー項目を表示（MENU_ITEM_PALETTE以降、インデックス1から）
                for (int i = 1; i < 6; i++)  // i=1から開始（MENU_ITEM_PALETTEから）
                {
                    // カーソル位置を設定（表示上は0行目から）
                    menu_terminal.set_cursor(i - 1, 0);

                    // 選択中の項目には先頭に「>」を表示
                    if (i == g_current_menu)
                    {
                        g_display->terminal_print(&menu_terminal, ">");
                    }
                    else
                    {
                        g_display->terminal_print(&menu_terminal, " ");
                    }

                    // メニュー項目名を表示（MENU_ITEM_NAMES[i-1]でアクセス）
                    g_display->terminal_print(&menu_terminal, MENU_ITEM_NAMES[i - 1]);
                }

                // 選択中のパレット名を表示
                g_display->draw_string(80, 40, PALETTE_NAMES[g_current_palette_index], true);
                // 選択中の解像度名を表示
                g_display->draw_string(80, 48, RESOLUTION_NAMES[g_current_resolution], true);
                // SDカードの状態を表示
                if (g_sd_card_mounted)
                {
                    g_display->draw_string(80, 56, "SD: OK", true);
                }
                else
                {
                    g_display->draw_string(80, 56, "SD: ERR", true);
                }
                // メニューを描画して画面を更新
                g_display->draw_terminal(&menu_terminal);
                g_display->display();
            }

            xSemaphoreGive(g_display_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    }
}

// ★★★ 撮影機能ヘルパー関数 ★★★

// SDカードから最大ファイル番号を検索してg_file_counterを初期化
void init_file_counter_from_sd(void)
{
    if (!g_sd_card_mounted)
    {
        ESP_LOGW(TAG, "SDカード未マウント: ファイルカウンター初期化スキップ");
        return;
    }

    DIR* dir = opendir(g_mount_point);
    if (dir == NULL)
    {
        ESP_LOGE(TAG, "ディレクトリ開けません: %s", g_mount_point);
        return;
    }

    int max_number = 0;
    struct dirent* entry;

    while ((entry = readdir(dir)) != NULL)
    {
        // ファイル名が数字4桁で始まるものを検索
        if (entry->d_type == DT_REG && strlen(entry->d_name) >= 4)
        {
            int num = 0;
            if (sscanf(entry->d_name, "%04d_", &num) == 1)
            {
                if (num > max_number)
                {
                    max_number = num;
                }
            }
        }
    }

    closedir(dir);

    g_file_counter = max_number + 1;
    ESP_LOGI(TAG, "ファイルカウンター初期化: %d", g_file_counter);
}

// ランダムな英数字列を生成（大文字小文字含む）
void generate_random_string(char* buf, int len)
{
    const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const int charset_size = sizeof(charset) - 1;

    for (int i = 0; i < len; i++)
    {
        uint32_t rand_val = esp_random();
        buf[i] = charset[rand_val % charset_size];
    }
    buf[len] = '\0';
}

/**
 * @brief 画像のリサイズとクロップを行う
 *
 * 640x480の画像を目標サイズにリサイズします。
 * アスペクト比を維持しながら、目標サイズに収まる最大サイズを計算し、
 * 必要に応じて中央トリミングを行います。
 *
 * @param src_rgb888 入力RGB888データ（640x480固定）
 * @param dst_rgb888 出力RGB888データバッファ（dst_width * dst_height * 3バイト必要）
 * @param dst_width 目標幅
 * @param dst_height 目標高さ
 */
void resize_and_crop_image(uint8_t* src_rgb888, uint8_t* dst_rgb888, int dst_width, int dst_height)
{
    const int src_width = 640;
    const int src_height = 480;

    // 目標サイズがソースと同じ場合はコピーのみ
    if (dst_width == src_width && dst_height == src_height)
    {
        memcpy(dst_rgb888, src_rgb888, src_width * src_height * 3);
        return;
    }

    ESP_LOGI(TAG, "リサイズ開始: %dx%d -> %dx%d", src_width, src_height, dst_width, dst_height);

    // アスペクト比を計算
    float src_aspect = (float)src_width / (float)src_height;  // 640/480 = 1.333
    float dst_aspect = (float)dst_width / (float)dst_height;

    // クロップ領域を計算（ソース画像上での座標）
    int crop_x = 0, crop_y = 0;
    int crop_width = src_width;
    int crop_height = src_height;

    if (dst_aspect > src_aspect)
    {
        // 目標が横長: 縦をクロップ
        crop_height = (int)(src_width / dst_aspect);
        crop_y = (src_height - crop_height) / 2;
    }
    else if (dst_aspect < src_aspect)
    {
        // 目標が縦長: 横をクロップ
        crop_width = (int)(src_height * dst_aspect);
        crop_x = (src_width - crop_width) / 2;
    }

    ESP_LOGD(TAG, "クロップ領域: x=%d, y=%d, w=%d, h=%d", crop_x, crop_y, crop_width, crop_height);

    // バイリニア補間でリサイズ
    for (int dst_y = 0; dst_y < dst_height; dst_y++)
    {
        for (int dst_x = 0; dst_x < dst_width; dst_x++)
        {
            // 目標座標をソース座標にマッピング
            float src_x_f = crop_x + (dst_x + 0.5f) * crop_width / dst_width - 0.5f;
            float src_y_f = crop_y + (dst_y + 0.5f) * crop_height / dst_height - 0.5f;

            // 座標をクリップ
            if (src_x_f < crop_x) src_x_f = crop_x;
            if (src_y_f < crop_y) src_y_f = crop_y;
            if (src_x_f > crop_x + crop_width - 1) src_x_f = crop_x + crop_width - 1;
            if (src_y_f > crop_y + crop_height - 1) src_y_f = crop_y + crop_height - 1;

            int x0 = (int)src_x_f;
            int y0 = (int)src_y_f;
            int x1 = (x0 + 1 < src_width) ? (x0 + 1) : x0;
            int y1 = (y0 + 1 < src_height) ? (y0 + 1) : y0;

            float dx = src_x_f - x0;
            float dy = src_y_f - y0;

            // 4点の画素を取得
            int idx00 = (y0 * src_width + x0) * 3;
            int idx01 = (y0 * src_width + x1) * 3;
            int idx10 = (y1 * src_width + x0) * 3;
            int idx11 = (y1 * src_width + x1) * 3;

            // RGB各チャンネルでバイリニア補間
            int dst_idx = (dst_y * dst_width + dst_x) * 3;
            for (int c = 0; c < 3; c++)
            {
                float val00 = src_rgb888[idx00 + c];
                float val01 = src_rgb888[idx01 + c];
                float val10 = src_rgb888[idx10 + c];
                float val11 = src_rgb888[idx11 + c];

                float val0 = val00 * (1.0f - dx) + val01 * dx;
                float val1 = val10 * (1.0f - dx) + val11 * dx;
                float val = val0 * (1.0f - dy) + val1 * dy;

                dst_rgb888[dst_idx + c] = (uint8_t)(val + 0.5f);
            }
        }
    }

    ESP_LOGI(TAG, "リサイズ完了");
}

// パレット減色アルゴリズム（最近傍色探索）
// color_count: 8 または 16
void apply_palette_reduction(uint8_t* rgb_data, int width, int height, int palette_idx, int color_count)
{
    const uint32_t* palette;

    // 色数に応じてパレットを選択
    if (color_count == 16)
    {
        palette = COLOR_PALETTES_16[palette_idx];
    }
    else
    {
        palette = COLOR_PALETTES_8[palette_idx];
        color_count = 8;  // 念のため8に固定
    }

    for (int i = 0; i < width * height; i++)
    {
        int pixel_offset = i * 3;
        uint8_t r = rgb_data[pixel_offset];
        uint8_t g = rgb_data[pixel_offset + 1];
        uint8_t b = rgb_data[pixel_offset + 2];

        // 最近傍色を探索
        int min_distance = INT32_MAX;
        uint32_t closest_color = palette[0];

        for (int j = 0; j < color_count; j++)
        {
            uint32_t pal_color = palette[j];
            int pal_r = (pal_color >> 16) & 0xFF;
            int pal_g = (pal_color >> 8) & 0xFF;
            int pal_b = pal_color & 0xFF;

            int dr = r - pal_r;
            int dg = g - pal_g;
            int db = b - pal_b;
            int distance = dr * dr + dg * dg + db * db;

            if (distance < min_distance)
            {
                min_distance = distance;
                closest_color = pal_color;
            }
        }

        // 最近傍色で置き換え
        rgb_data[pixel_offset] = (closest_color >> 16) & 0xFF;
        rgb_data[pixel_offset + 1] = (closest_color >> 8) & 0xFF;
        rgb_data[pixel_offset + 2] = closest_color & 0xFF;
    }
}

// RGB画像をBMP形式でSDカードに保存
esp_err_t save_rgb_as_bmp(uint8_t* rgb_data, int width, int height, const char* filepath)
{
    // 行サイズを計算（4バイトアライメント）
    int row_size = (3 * width + 3) & ~3;

    // BMPヘッダー作成
    bitmap_header_t header;
    header.bfType = 0x4D42;  // "BM"
    header.bfSize = row_size * height + sizeof(bitmap_header_t);
    header.bfReserved1 = 0;
    header.bfReserved2 = 0;
    header.bfOffBits = sizeof(bitmap_header_t);
    header.biSize = 40;
    header.biWidth = width;
    header.biHeight = height;
    header.biPlanes = 1;
    header.biBitCount = 24;
    header.biCompression = 0;
    header.biSizeImage = 0;
    header.biXPelsPerMeter = 2835;
    header.biYPelsPerMeter = 2835;
    header.biClrUsed = 0;
    header.biClrImportant = 0;

    // ファイルオープン
    FILE* file = fopen(filepath, "wb");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "ファイル開けません: %s", filepath);
        return ESP_FAIL;
    }

    // ヘッダー書き込み
    size_t written = fwrite(&header, sizeof(bitmap_header_t), 1, file);
    if (written != 1)
    {
        ESP_LOGE(TAG, "ヘッダー書き込み失敗: %s", filepath);
        fclose(file);
        return ESP_FAIL;
    }

    // 行バッファ確保
    uint8_t* row_buffer = (uint8_t*)malloc(row_size);
    if (row_buffer == NULL)
    {
        ESP_LOGE(TAG, "行バッファ確保失敗");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    // ピクセルデータ書き込み（下から上へ、BGRの順）
    memset(row_buffer, 0, row_size);  // パディング部分をゼロクリア

    for (int y = height - 1; y >= 0; y--)
    {
        for (int x = 0; x < width; x++)
        {
            int src_offset = (y * width + x) * 3;
            int dst_offset = x * 3;

            // RGB → BGR変換
            row_buffer[dst_offset] = rgb_data[src_offset + 2];      // B
            row_buffer[dst_offset + 1] = rgb_data[src_offset + 1];  // G
            row_buffer[dst_offset + 2] = rgb_data[src_offset];      // R
        }

        written = fwrite(row_buffer, row_size, 1, file);
        if (written != 1)
        {
            ESP_LOGE(TAG, "ピクセルデータ書き込み失敗: %s (行: %d)", filepath, y);
            free(row_buffer);
            fclose(file);
            return ESP_FAIL;
        }
    }

    // クリーンアップ
    free(row_buffer);
    fclose(file);

    ESP_LOGI(TAG, "BMP保存成功: %s (%ld bytes)", filepath, header.bfSize);
    return ESP_OK;
}

// プログレスバー描画（画面中央、横幅128px、高さ3px、枠線1px）
void draw_progress_bar(float progress, const char* status_text)
{
    if (!g_display_ready || g_display == nullptr)
    {
        return;
    }

    const int BAR_WIDTH = 128;
    const int BAR_HEIGHT = 3;
    const int BAR_X = 0;
    const int BAR_Y = (64 - BAR_HEIGHT) / 2;  // 画面中央
    const int TEXT_Y = 18;  // 状態テキストのY座標（プログレスバーの上）
    
    // 画面クリア
    g_display->clear();

    // 状態テキストを表示（指定されている場合）
    if (status_text != nullptr)
    {
        // テキストを中央揃えで表示
        g_display->draw_string(0, TEXT_Y, status_text, true);
    }

    // 枠線描画（白）
    for (int x = BAR_X; x < BAR_X + BAR_WIDTH; x++)
    {
        g_display->set_pixel(x, BAR_Y, true);  // 上
        g_display->set_pixel(x, BAR_Y + BAR_HEIGHT - 1, true);  // 下
    }
    for (int y = BAR_Y; y < BAR_Y + BAR_HEIGHT; y++)
    {
        g_display->set_pixel(BAR_X, y, true);  // 左
        g_display->set_pixel(BAR_X + BAR_WIDTH - 1, y, true);  // 右
    }

    // プログレス部分を描画（白、内側）
    int progress_width = (int)((BAR_WIDTH - 2) * progress);  // 枠線分を引く
    for (int y = BAR_Y + 1; y < BAR_Y + BAR_HEIGHT - 1; y++)
    {
        for (int x = BAR_X + 1; x < BAR_X + 1 + progress_width; x++)
        {
            g_display->set_pixel(x, y, true);
        }
    }

    g_display->display();
}

// ★★★ 撮影タスク ★★★
void capture_task(void *parameter)
{
    bool all_palettes = *((bool *)parameter);
    free(parameter);  // パラメータメモリ解放

    ESP_LOGI(TAG, "📸 撮影タスク開始 (全パレット: %s)", all_palettes ? "Yes" : "No");
    ESP_LOGI(TAG, "   現在の状態: %d → CAPTURING に変更", g_system_status);

    // システム状態を撮影中に変更
    g_system_status = SYSTEM_STATUS_CAPTURING;
    ESP_LOGI(TAG, "   状態変更完了: %d", g_system_status);

    // ランダム文字列生成
    char random_str[5];
    generate_random_string(random_str, 4);
    ESP_LOGI(TAG, "   ランダム文字列: %s", random_str);
    ESP_LOGI(TAG, "   ファイルカウンター: %d", g_file_counter);

    // 元画像 + パレット数×2（8色と16色）
    int total_steps = all_palettes ? 17 : 3;  // 全パレット: 1 + 8*2, 単一: 1 + 1*2
    int current_step = 0;
    ESP_LOGI(TAG, "   総ステップ数: %d", total_steps);

    // カメラからフレーム取得
    ESP_LOGI(TAG, "   フレーム取得開始...");
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL)
    {
        ESP_LOGE(TAG, "❌ カメラフレーム取得失敗");

        // エラー表示
        if (g_display_ready && g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            g_display->clear();
            // 簡易的なエラー表示（Xマーク）
            for (int i = 0; i < 16; i++)
            {
                g_display->set_pixel(56 + i, 24 + i, true);  // 左上から右下
                g_display->set_pixel(56 + i, 40 - i, true);  // 左下から右上
            }
            g_display->display();
            xSemaphoreGive(g_display_mutex);
        }

        // エラー表示後、待機してシステム状態を戻す
        vTaskDelay(pdMS_TO_TICKS(2000));
        g_system_status = SYSTEM_STATUS_READY;
        ESP_LOGI(TAG, "システム状態をREADYに戻しました");
        vTaskDelete(NULL);
        return;
    }

    int src_width = fb->width;
    int src_height = fb->height;
    size_t rgb_buffer_size = src_width * src_height * 3;

    // フレームバッファ情報をログ出力
    ESP_LOGI(TAG, "   フレームバッファ情報:");
    ESP_LOGI(TAG, "     - サイズ: %dx%d", src_width, src_height);
    ESP_LOGI(TAG, "     - フォーマット: %d (0=JPEG, 2=RGB565, 4=GRAYSCALE)", fb->format);
    ESP_LOGI(TAG, "     - バッファサイズ: %zu bytes", fb->len);
    ESP_LOGI(TAG, "     - 期待サイズ: %zu bytes (RGB565)", (size_t)(src_width * src_height * 2));

    // RGB888バッファ確保
    uint8_t *rgb_buffer = (uint8_t *)heap_caps_malloc(rgb_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rgb_buffer == NULL)
    {
        ESP_LOGE(TAG, "❌ RGBバッファ確保失敗");
        esp_camera_fb_return(fb);
        g_system_status = SYSTEM_STATUS_READY;
        vTaskDelete(NULL);
        return;
    }

    // フォーマットに応じた変換処理
    if (fb->format == PIXFORMAT_RGB565)
    {
        ESP_LOGI(TAG, "   RGB565 → RGB888変換開始");
        uint8_t* fb_data = fb->buf;

        for (int i = 0; i < src_width * src_height; i++)
        {
            // RGB565をビッグエンディアンで読み取り（GC0308カメラの出力形式）
            uint16_t rgb565Color = (fb_data[i * 2] << 8) | fb_data[i * 2 + 1];

            // RGB565からRGB888へ変換
            uint8_t r = ((rgb565Color >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565Color >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565Color & 0x1F) * 255 / 31;

            rgb_buffer[i * 3] = r;
            rgb_buffer[i * 3 + 1] = g;
            rgb_buffer[i * 3 + 2] = b;
        }
        ESP_LOGI(TAG, "   RGB565→RGB888変換完了");
    }
    else
    {
        ESP_LOGE(TAG, "❌ 未対応のピクセルフォーマット: %d", fb->format);
        heap_caps_free(rgb_buffer);
        esp_camera_fb_return(fb);
        g_system_status = SYSTEM_STATUS_READY;
        vTaskDelete(NULL);
        return;
    }

    // フレームバッファ解放
    esp_camera_fb_return(fb);

    // 選択された解像度を取得
    int target_width = RESOLUTION_DIMENSIONS[g_current_resolution][0];
    int target_height = RESOLUTION_DIMENSIONS[g_current_resolution][1];
    ESP_LOGI(TAG, "   目標解像度: %dx%d (%s)", target_width, target_height, RESOLUTION_NAMES[g_current_resolution]);

    // リサイズが必要かチェック（640x480以外の場合）
    uint8_t *final_buffer = rgb_buffer;
    size_t final_buffer_size = rgb_buffer_size;
    int final_width = src_width;
    int final_height = src_height;

    if (target_width != 640 || target_height != 480)
    {
        // リサイズ用バッファ確保
        final_buffer_size = target_width * target_height * 3;
        final_buffer = (uint8_t *)heap_caps_malloc(final_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (final_buffer == NULL)
        {
            ESP_LOGE(TAG, "❌ リサイズバッファ確保失敗");
            heap_caps_free(rgb_buffer);
            g_system_status = SYSTEM_STATUS_READY;
            vTaskDelete(NULL);
            return;
        }

        // リサイズ実行
        resize_and_crop_image(rgb_buffer, final_buffer, target_width, target_height);

        final_width = target_width;
        final_height = target_height;

        // 元のバッファは不要になったので解放
        heap_caps_free(rgb_buffer);
    }
    else
    {
        ESP_LOGI(TAG, "   リサイズ不要（640x480のまま）");
    }

    // 保存状態に変更
    g_system_status = SYSTEM_STATUS_SAVING;

    // 元画像保存
    char filepath[128];
    snprintf(filepath, sizeof(filepath), "%s/%04d_%s_Original.bmp", g_mount_point, g_file_counter, random_str);

    if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        draw_progress_bar((float)current_step / total_steps, "ほぞんちゅう origin");
        xSemaphoreGive(g_display_mutex);
    }

    esp_err_t result = save_rgb_as_bmp(final_buffer, final_width, final_height, filepath);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ 元画像保存失敗");
        heap_caps_free(final_buffer);
        g_system_status = SYSTEM_STATUS_READY;
        vTaskDelete(NULL);
        return;
    }

    current_step++;
    ESP_LOGI(TAG, "✅ 元画像保存完了 (%d/%d)", current_step, total_steps);

    // パレット減色画像保存（8色版と16色版の両方）
    int palette_count = all_palettes ? 8 : 1;
    int start_palette = all_palettes ? 0 : g_current_palette_index;

    for (int i = 0; i < palette_count; i++)
    {
        int palette_idx = all_palettes ? i : start_palette;

        // 8色版の保存
        {
            // RGB888バッファをコピー（減色処理は破壊的なため）
            uint8_t *temp_buffer = (uint8_t *)heap_caps_malloc(final_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (temp_buffer == NULL)
            {
                ESP_LOGE(TAG, "❌ 一時バッファ確保失敗");
                heap_caps_free(final_buffer);
                g_system_status = SYSTEM_STATUS_READY;
                vTaskDelete(NULL);
                return;
            }

            memcpy(temp_buffer, final_buffer, final_buffer_size);

            // プログレスバー更新
            if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                draw_progress_bar((float)current_step / total_steps, "ほぞんちゅう 8col");
                xSemaphoreGive(g_display_mutex);
            }

            // 8色パレット減色
            apply_palette_reduction(temp_buffer, final_width, final_height, palette_idx, 8);

            // ファイル名生成（8色版）
            snprintf(filepath, sizeof(filepath), "%s/%04d_%s_%s_8c.bmp",
                     g_mount_point, g_file_counter, random_str, PALETTE_NAMES[palette_idx]);

            // BMP保存
            result = save_rgb_as_bmp(temp_buffer, final_width, final_height, filepath);
            heap_caps_free(temp_buffer);

            if (result != ESP_OK)
            {
                ESP_LOGE(TAG, "❌ 8色パレット画像保存失敗: %s", PALETTE_NAMES[palette_idx]);
                heap_caps_free(final_buffer);
                g_system_status = SYSTEM_STATUS_READY;
                vTaskDelete(NULL);
                return;
            }

            current_step++;
            ESP_LOGI(TAG, "✅ 8色パレット画像保存完了: %s (%d/%d)", PALETTE_NAMES[palette_idx], current_step, total_steps);
        }

        // 16色版の保存
        {
            // RGB888バッファをコピー
            uint8_t *temp_buffer = (uint8_t *)heap_caps_malloc(final_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (temp_buffer == NULL)
            {
                ESP_LOGE(TAG, "❌ 一時バッファ確保失敗");
                heap_caps_free(final_buffer);
                g_system_status = SYSTEM_STATUS_READY;
                vTaskDelete(NULL);
                return;
            }

            memcpy(temp_buffer, final_buffer, final_buffer_size);

            // プログレスバー更新
            if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                draw_progress_bar((float)current_step / total_steps, "ほぞんちゅう 16col");
                xSemaphoreGive(g_display_mutex);
            }

            // 16色パレット減色
            apply_palette_reduction(temp_buffer, final_width, final_height, palette_idx, 16);

            // ファイル名生成（16色版）
            snprintf(filepath, sizeof(filepath), "%s/%04d_%s_%s_16c.bmp",
                     g_mount_point, g_file_counter, random_str, PALETTE_NAMES[palette_idx]);

            // BMP保存
            result = save_rgb_as_bmp(temp_buffer, final_width, final_height, filepath);
            heap_caps_free(temp_buffer);

            if (result != ESP_OK)
            {
                ESP_LOGE(TAG, "❌ 16色パレット画像保存失敗: %s", PALETTE_NAMES[palette_idx]);
                heap_caps_free(final_buffer);
                g_system_status = SYSTEM_STATUS_READY;
                vTaskDelete(NULL);
                return;
            }

            current_step++;
            ESP_LOGI(TAG, "✅ 16色パレット画像保存完了: %s (%d/%d)", PALETTE_NAMES[palette_idx], current_step, total_steps);
        }
    }

    // メモリ解放
    heap_caps_free(final_buffer);

    // ファイルカウンター更新
    g_file_counter++;

    // 最終プログレスバー表示
    if (g_display_mutex != NULL && xSemaphoreTake(g_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        draw_progress_bar(1.0f);
        xSemaphoreGive(g_display_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // 完了表示

    // システム状態を元に戻す
    ESP_LOGI(TAG, "   状態を READY に戻します (現在: %d)", g_system_status);
    g_system_status = SYSTEM_STATUS_READY;
    ESP_LOGI(TAG, "   状態変更完了: %d", g_system_status);

    ESP_LOGI(TAG, "🎉 撮影完了！タスクを削除します");

    // タスク自己削除
    vTaskDelete(NULL);
    ESP_LOGI(TAG, "❌ この行は実行されないはず");  // デバッグ用
}

// 撮影開始（タスク生成）
void start_capture(bool all_palettes)
{
    // USB MSCモード中は撮影を防止
    if (g_system_status == SYSTEM_STATUS_USB_MSC)
    {
        ESP_LOGW(TAG, "⚠️ USB MSCモード中のため撮影できません");
        return;
    }

    g_system_status = SYSTEM_STATUS_CAPTURING;

    ESP_LOGI(TAG, "🚀 start_capture() 呼び出し (all_palettes: %s)", all_palettes ? "Yes" : "No");
    ESP_LOGI(TAG, "   現在の状態: %d", g_system_status);

    // パラメータをヒープに確保
    bool *param = (bool *)malloc(sizeof(bool));
    if (param == NULL)
    {
        ESP_LOGE(TAG, "❌ パラメータメモリ確保失敗");
        return;
    }
    *param = all_palettes;

    ESP_LOGI(TAG, "   撮影タスクを作成します...");

    BaseType_t result = xTaskCreate(
        capture_task,
        "capture_task",
        16384,  // スタックサイズ
        (void *)param,
        tskIDLE_PRIORITY + 2,
        NULL);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "❌ 撮影タスク作成失敗");
        free(param);
    }
    else
    {
        ESP_LOGI(TAG, "✅ 撮影タスク作成成功");
    }
}


/**
 * @brief NVSから設定を読み込む
 *
 * パレット番号と解像度設定をNVSから読み込み、グローバル変数に反映します。
 * エラー時はデフォルト値を維持します。
 */
static void load_settings_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "⚠️ NVS名前空間オープン失敗: %s - デフォルト設定を使用", esp_err_to_name(err));
        return;
    }

    // パレット番号の読み込み
    uint8_t palette_idx;
    err = nvs_get_u8(nvs_handle, NVS_KEY_PALETTE, &palette_idx);
    if (err == ESP_OK)
    {
        if (palette_idx < MAX_PALETTE_INDEX)
        {
            g_current_palette_index = palette_idx;
            ESP_LOGI(TAG, "📥 パレット読み込み: %d (%s)", palette_idx, PALETTE_NAMES[palette_idx]);
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ 無効なパレット番号: %d - デフォルト値を使用", palette_idx);
        }
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "📥 パレット設定が見つかりません - デフォルト値を使用");
    }
    else
    {
        ESP_LOGE(TAG, "❌ パレット読み込みエラー: %s", esp_err_to_name(err));
    }

    // 解像度の読み込み
    uint8_t resolution;
    err = nvs_get_u8(nvs_handle, NVS_KEY_RESOLUTION, &resolution);
    if (err == ESP_OK)
    {
        if (resolution < MAX_RESOLUTION_INDEX)
        {
            g_current_resolution = (resolution_t)resolution;
            ESP_LOGI(TAG, "📥 解像度読み込み: %d (%s)", resolution, RESOLUTION_NAMES[resolution]);
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ 無効な解像度: %d - デフォルト値を使用", resolution);
        }
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "📥 解像度設定が見つかりません - デフォルト値を使用");
    }
    else
    {
        ESP_LOGE(TAG, "❌ 解像度読み込みエラー: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "✅ 設定読み込み完了: パレット=%d, 解像度=%d",
             g_current_palette_index, g_current_resolution);
}

/**
 * @brief NVSに設定を保存する
 *
 * 現在のパレット番号と解像度設定をNVSに保存します。
 * エラー時はログ出力のみで、カメラ動作は継続します。
 */
static void save_settings_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ NVS名前空間オープン失敗: %s", esp_err_to_name(err));
        return;
    }

    bool save_failed = false;

    // パレット番号の保存
    err = nvs_set_u8(nvs_handle, NVS_KEY_PALETTE, g_current_palette_index);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ パレット保存失敗: %s", esp_err_to_name(err));
        save_failed = true;
    }

    // 解像度の保存
    err = nvs_set_u8(nvs_handle, NVS_KEY_RESOLUTION, (uint8_t)g_current_resolution);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ 解像度保存失敗: %s", esp_err_to_name(err));
        save_failed = true;
    }

    // 変更をコミット
    err = nvs_commit(nvs_handle);
    if (err == ESP_OK && !save_failed)
    {
        ESP_LOGI(TAG, "💾 設定保存完了: パレット=%d (%s), 解像度=%d (%s)",
                 g_current_palette_index, PALETTE_NAMES[g_current_palette_index],
                 g_current_resolution, RESOLUTION_NAMES[g_current_resolution]);
    }
    else
    {
        ESP_LOGE(TAG, "❌ NVSコミット失敗: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
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

    // NVSから設定を読み込み
    load_settings_from_nvs();

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

    // GPIO初期化（最初に実行）
    display_init_step(&terminal, " GPIO init");
    esp_err_t gpio_result = init_gpio();
    display_init_step(&terminal, gpio_result == ESP_OK);

    // TinyUSBドライバ初期化（カメラより前に実行し、リソース競合を回避）
    display_init_step(&terminal, " TinyUSB init");
    esp_err_t usb_result = init_tinyusb_driver();
    bool usb_ok = (usb_result == ESP_OK);
    display_init_step(&terminal, usb_ok);

    if (!usb_ok) {
        ESP_LOGW(TAG, "⚠️ TinyUSB初期化失敗 - USB MSC機能は使用できません");
    }

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
        // ファイルカウンター初期化
        init_file_counter_from_sd();
    }

    // エンコーダー初期化
    display_init_step(&terminal, " Encoder init");
    pimoroni_encoder_config_t encoder_config = pimoroni_encoder_get_default_config(EXTERNAL_I2C_NUM);
    encoder_config.i2c_address = PIMORONI_ENCODER_I2C_ADDR;
    encoder_config.direction = PIMORONI_ENCODER_CW; // 時計回り（正の方向）
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

    // キュー・ミューテックス作成
    display_init_step(&terminal, " Queue create");
    g_encoder_event_queue = xQueueCreate(10, sizeof(encoder_event_t));
    g_display_mutex = xSemaphoreCreateMutex();
    display_init_step(&terminal, g_encoder_event_queue != NULL && g_display_mutex != NULL);

    // エンコーダータスク開始
    display_init_step(&terminal, " Task start");
    BaseType_t task_result = pdFAIL;
    if (g_encoder_ready)
    {
        task_result = xTaskCreate(encoder_task, "encoder_task",
                                  ENCODER_TASK_STACK, NULL,
                                  tskIDLE_PRIORITY + 4, NULL); // 優先度向上: 取りこぼし防止
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

    // ヒストグラムタスク開始
    if (g_camera_ready && g_display_ready && g_display_mutex != NULL)
    {
        BaseType_t histogram_task_result = xTaskCreate(
            histogram_task,
            "histogram",
            8192,  // スタックサイズ
            NULL,
            tskIDLE_PRIORITY + 1,  // プレビューと同じ優先度
            NULL
        );

        if (histogram_task_result == pdPASS)
        {
            ESP_LOGI(TAG, "✅ ヒストグラムタスク起動成功");
        }
        else
        {
            ESP_LOGE(TAG, "❌ ヒストグラムタスク起動失敗");
        }
    }

    // メニュー表示タスク開始
    if (g_display_ready && g_display_mutex != NULL)
    {
        BaseType_t menu_task_result = xTaskCreate(
            menu_display_task,
            "menu_display",
            4096,  // スタックサイズ
            NULL,
            tskIDLE_PRIORITY + 1,  // プレビューと同じ優先度
            NULL
        );

        if (menu_task_result == pdPASS)
        {
            ESP_LOGI(TAG, "✅ メニュー表示タスク起動成功");
        }
        else
        {
            ESP_LOGE(TAG, "❌ メニュー表示タスク起動失敗");
        }
    }

    // ===== 初期化完了 =====
    g_system_ready = true;
    g_system_status = SYSTEM_STATUS_READY;  // システム状態をREADYに設定
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