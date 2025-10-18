/*
 * AtomS3R Cam ピクセルアートカメラ (ESP-IDF版)
 * 
 * 機能:
 * - ボタン(GPIO38)を押すと撮影してSDカードに保存
 * - ロータリーエンコーダでパレット選択（0-7）
 * - エンコーダのRGB LEDで選択中のパレットを表示
 * - 状態LED(GPIO39)で保存状況を通知
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>

// ESP-IDFコア
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

// ドライバー
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_camera.h"

// ファイルシステム
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

//M5gfx
#include <M5GFX.h>

// カスタムクラス
#include "PimoroniEncoder.h"

// ========================================
// 定数定義
// ========================================
static const char* TAG = "PixelArtCamera";

// ピン設定
#define SPI_SCK_PIN     (gpio_num_t)7
#define SPI_MISO_PIN    (gpio_num_t)8
#define SPI_MOSI_PIN    (gpio_num_t)6
#define SPI_CS_PIN      (gpio_num_t)15

#define CAMERA_POWER_PIN (gpio_num_t)18
#define BUTTON_PIN       (gpio_num_t)38
#define LED_PIN          (gpio_num_t)39

#define I2C_SDA_PIN      (gpio_num_t)1
#define I2C_SCL_PIN      (gpio_num_t)2

// カメラ設定
#define CAMERA_I2C_SDA   (gpio_num_t)12
#define CAMERA_I2C_SCL   (gpio_num_t)9

// その他
#define DEBOUNCE_DELAY_MS     300
#define MAX_PALETTE_INDEX     8
#define IMAGE_WIDTH           240
#define IMAGE_HEIGHT          176

// ========================================
// グローバル変数
// ========================================
static camera_fb_t* s_fb = NULL;
static int s_file_counter = 1;
static uint32_t s_last_button_press = 0;
static uint8_t s_gray_data[IMAGE_WIDTH * IMAGE_HEIGHT];

// エンコーダ
static PimoroniEncoder* s_encoder = nullptr;
static int s_current_palette_index = 0;
static int s_last_palette_index = -1;

// SDカード
static sdmmc_card_t* s_card = NULL;
static const char* s_mount_point = "/sdcard";

// ========================================
// カラーパレット定義（8種類×8色）
// ========================================
static const uint32_t COLOR_PALETTES[8][8] = {
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

// パレット代表色（エンコーダLED表示用）
static const uint32_t PALETTE_REP_COLORS[8] = {
    0x8D697A,  // パレット0: ピンク系
    0x437290,  // パレット1: 青系
    0x498DB7,  // パレット2: 水色系
    0x8080A4,  // パレット3: グレー系
    0xC6BAAC,  // パレット4: ベージュ系
    0x9AA57C,  // パレット5: オリーブ系
    0x38D88E,  // パレット6: 緑系
    0xFFD159,  // パレット7: 黄色系
};

// ========================================
// カメラ設定構造体
// ========================================
static camera_config_t camera_config = {
    .pin_pwdn     = GPIO_NUM_NC,
    .pin_reset    = GPIO_NUM_NC,
    .pin_xclk     = GPIO_NUM_21,
    .pin_sccb_sda = CAMERA_I2C_SDA,
    .pin_sccb_scl = CAMERA_I2C_SCL,
    
    .pin_d7       = GPIO_NUM_13,
    .pin_d6       = GPIO_NUM_11,
    .pin_d5       = GPIO_NUM_17,
    .pin_d4       = GPIO_NUM_4,
    .pin_d3       = GPIO_NUM_48,
    .pin_d2       = GPIO_NUM_46,
    .pin_d1       = GPIO_NUM_42,
    .pin_d0       = GPIO_NUM_3,
    
    .pin_vsync    = GPIO_NUM_10,
    .pin_href     = GPIO_NUM_14,
    .pin_pclk     = GPIO_NUM_40,
    
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_HQVGA,    // 240x176
    
    .jpeg_quality = 0,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
    .sccb_i2c_port = 0,
};

// ========================================
// GPIO制御関数
// ========================================
static void led_on() {
    gpio_set_level(LED_PIN, 1);
}

static void led_off() {
    gpio_set_level(LED_PIN, 0);
}

static bool is_button_pressed() {
    // プルアップなので、押されたときLOW
    if (gpio_get_level(BUTTON_PIN) == 0) {
        // チャタリング防止
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - s_last_button_press > DEBOUNCE_DELAY_MS) {
            s_last_button_press = now;
            return true;
        }
    }
    return false;
}

// ========================================
// GPIO初期化
// ========================================
static esp_err_t init_gpio() {
    ESP_LOGI(TAG, "Initializing GPIO...");
    
    // ボタンピン設定（プルアップ）
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) return ret;
    
    // LEDピン設定（出力）
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&led_config);
    if (ret != ESP_OK) return ret;
    
    // カメラ電源ピン設定
    gpio_config_t power_config = {
        .pin_bit_mask = (1ULL << CAMERA_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&power_config);
    if (ret != ESP_OK) return ret;
    
    // 初期状態設定
    led_off();
    gpio_set_level(CAMERA_POWER_PIN, 0);  // カメラ電源ON（LOWで有効）
    
    ESP_LOGI(TAG, "GPIO initialization completed");
    return ESP_OK;
}

// ========================================
// SDカード初期化
// ========================================
static esp_err_t init_sdcard() {
    ESP_LOGI(TAG, "Initializing SD card...");
    
    esp_err_t ret;
    
    // SDカード用のSPI設定
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS_PIN;
    slot_config.host_id = (spi_host_device_t)host.slot;
    slot_config.gpio_cd = GPIO_NUM_NC;
    slot_config.gpio_wp = GPIO_NUM_NC;
    
    // SPIバス設定
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // VFSマウント設定
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };
    
    ret = esp_vfs_fat_sdspi_mount(s_mount_point, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount filesystem: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // カード情報を表示
    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "SD card initialized successfully");
    
    return ESP_OK;
}

// ========================================
// カメラ初期化
// ========================================
static esp_err_t init_camera() {
    ESP_LOGI(TAG, "Initializing camera...");
    
    // カメラ電源を有効化
    gpio_set_level(CAMERA_POWER_PIN, 0);  // LOWで電源ON
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Camera power ON");
    
    // カメラドライバの初期化
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: 0x%x", err);
        return err;
    }
    
    // センサー設定（左右反転・上下反転）
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, 1);  // 左右反転
        s->set_vflip(s, 1);    // 上下反転
        ESP_LOGI(TAG, "Camera sensor configured");
    }
    
    ESP_LOGI(TAG, "Camera initialization completed");
    return ESP_OK;
}

// ========================================
// エンコーダ初期化
// ========================================
static esp_err_t init_encoder() {
    ESP_LOGI(TAG, "Initializing encoder...");
    
    s_encoder = new PimoroniEncoder(I2C_NUM_1, 0x0F);
    if (!s_encoder) {
        ESP_LOGE(TAG, "Failed to create encoder object");
        return ESP_FAIL;
    }
    
    esp_err_t ret = s_encoder->begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Encoder initialization failed");
        delete s_encoder;
        s_encoder = nullptr;
        return ret;
    }
    
    // 値の範囲を設定（0-7: パレットインデックス）
    s_encoder->set_value_range(0, 7);
    s_encoder->set_value(0);  // 初期値0
    
    // LED輝度設定（0.0-1.0）
    s_encoder->set_led_brightness(0.3);  // 30%の輝度
    
    // 初期パレット色を表示
    s_encoder->set_led_color(PALETTE_REP_COLORS[0]);
    
    ESP_LOGI(TAG, "Encoder initialization completed");
    return ESP_OK;
}

// ========================================
// 輝度情報を保存する関数（フレームバッファから）
// ========================================
static void save_graylevel_fb() {
    if (!s_fb) return;
    
    uint8_t* fb_data = s_fb->buf;
    int width = s_fb->width;
    int height = s_fb->height;
    int i = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < (width * 2); x = x + 2) {
            // RGB565の色を取得（ビッグエンディアン）
            uint32_t rgb565_color = (fb_data[y * width * 2 + x] << 8) | fb_data[y * width * 2 + x + 1];

            // RGB565からRGB888へ変換
            uint8_t r = ((rgb565_color >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565_color >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565_color & 0x1F) * 255 / 31;

            // 輝度の計算（BT.709の係数を使用）
            uint16_t luminance = (uint16_t)(0.2126f * r + 0.7152f * g + 0.0722f * b);

            // 輝度を8階調のグレースケールに変換（0-7）
            uint8_t gray_level = luminance / 32;  // 256/32 = 8
            if (gray_level > 7) gray_level = 7;    // 念のため範囲制限

            // 輝度情報を保存
            s_gray_data[i] = gray_level;
            i++;
        }
    }
}

// ========================================
// BMPヘッダー構造体
// ========================================
typedef struct {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
    uint32_t biSize;
    int32_t biWidth;
    int32_t biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t biXPelsPerMeter;
    int32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} __attribute__((packed)) bitmap_header_t;

// ========================================
// RGB565からBMP変換
// ========================================
/*
static bool frame2bmp(camera_fb_t *fb, uint8_t **out_buf, size_t *out_len) {
    if (!fb || !out_buf || !out_len) return false;
    
    int width = fb->width;
    int height = fb->height;
    int row_size = (3 * width + 3) & ~3;  // 4バイトアラインメント
    
    // BMPヘッダー
    bitmap_header_t header = {
        .bfType = 0x4D42,  // "BM"
        .bfSize = row_size * height + sizeof(header),
        .bfReserved1 = 0,
        .bfReserved2 = 0,
        .bfOffBits = sizeof(header),
        .biSize = 40,
        .biWidth = width,
        .biHeight = height,
        .biPlanes = 1,
        .biBitCount = 24,
        .biCompression = 0,
        .biSizeImage = 0,
        .biXPelsPerMeter = 2835,
        .biYPelsPerMeter = 2835,
        .biClrUsed = 0,
        .biClrImportant = 0
    };
    
    // バッファ確保
    size_t total_size = sizeof(header) + row_size * height;
    uint8_t *buffer = (uint8_t*)malloc(total_size);
    if (!buffer) return false;
    
    // ヘッダーコピー
    memcpy(buffer, &header, sizeof(header));
    
    // ピクセルデータ変換（下から上へ）
    uint8_t *pixel_data = buffer + sizeof(header);
    uint16_t *fb_data = (uint16_t*)fb->buf;
    
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            uint16_t rgb565 = fb_data[y * width + x];
            
            // RGB565からRGB888への変換
            uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565 & 0x1F) * 255 / 31;
            
            // BGRの順で格納
            int index = (height - 1 - y) * row_size + x * 3;
            pixel_data[index] = b;
            pixel_data[index + 1] = g;
            pixel_data[index + 2] = r;
        }
    }
    
    *out_buf = buffer;
    *out_len = total_size;
    return true;
}
*/
// ========================================
// オリジナル画像をBMP形式でSDカードに保存
// ========================================
static bool save_original_bmp(uint32_t timestamp) {
    char filename[64];
    snprintf(filename, sizeof(filename), "%s/%010lu_%04d_Original.bmp", 
             s_mount_point, timestamp, s_file_counter);
    
    ESP_LOGI(TAG, "Saving original: %s", filename);
    
    FILE *file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }
    
    // RGB565からBMPに変換
    uint8_t *bmp_buffer = NULL;
    size_t bmp_size = 0;
    bool success = frame2bmp(s_fb, &bmp_buffer, &bmp_size);
    
    if (success && bmp_buffer) {
        size_t written = fwrite(bmp_buffer, 1, bmp_size, file);
        fclose(file);
        free(bmp_buffer);
        
        if (written == bmp_size) {
            ESP_LOGI(TAG, "Original saved successfully (%zu bytes)", bmp_size);
            return true;
        } else {
            ESP_LOGE(TAG, "Write error: %zu/%zu bytes", written, bmp_size);
            return false;
        }
    } else {
        ESP_LOGE(TAG, "BMP conversion failed");
        fclose(file);
        return false;
    }
}

// ========================================
// パレット変換画像をBMP形式でSDカードに保存
// ========================================
static bool save_palette_bmp(uint32_t timestamp, int palette_index) {
    if (!s_fb) return false;
    
    char filename[64];
    snprintf(filename, sizeof(filename), "%s/%010lu_%04d_palette%01d.bmp", 
             s_mount_point, timestamp, s_file_counter, palette_index);
    
    ESP_LOGI(TAG, "Saving palette %d: %s", palette_index, filename);
    
    FILE *file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }
    
    int width = s_fb->width;
    int height = s_fb->height;
    int row_size = (3 * width + 3) & ~3;  // 4バイトアラインメント
    
    // BMPヘッダー作成
    bitmap_header_t header = {
        .bfType = 0x4D42,  // "BM"
        .bfSize = row_size * height + sizeof(header),
        .bfReserved1 = 0,
        .bfReserved2 = 0,
        .bfOffBits = sizeof(header),
        .biSize = 40,
        .biWidth = width,
        .biHeight = height,
        .biPlanes = 1,
        .biBitCount = 24,
        .biCompression = 0,
        .biSizeImage = 0,
        .biXPelsPerMeter = 2835,
        .biYPelsPerMeter = 2835,
        .biClrUsed = 0,
        .biClrImportant = 0
    };
    
    // ヘッダー書き込み
    fwrite(&header, sizeof(header), 1, file);
    
    // ピクセルデータ書き込み（下から上へ）
    uint8_t *buffer = (uint8_t*)malloc(row_size);
    if (!buffer) {
        fclose(file);
        return false;
    }
    
    for (int y = height - 1; y >= 0; y--) {
        memset(buffer, 0, row_size);  // パディング部分をゼロクリア
        
        for (int x = 0; x < width; x++) {
            // グレイデータを読み出す
            int i_gray = y * width + x;
            uint8_t gray = s_gray_data[i_gray];
            
            // カラーパレットから色を取得
            uint32_t new_color = COLOR_PALETTES[palette_index][gray];
            uint8_t r = (new_color >> 16) & 0xFF;
            uint8_t g = (new_color >> 8) & 0xFF;
            uint8_t b = new_color & 0xFF;
            
            // バッファに書き込み（BGRの順）
            int i_buffer = x * 3;
            buffer[i_buffer] = b;
            buffer[i_buffer + 1] = g;
            buffer[i_buffer + 2] = r;
        }
        fwrite(buffer, row_size, 1, file);
    }
    
    free(buffer);
    fclose(file);
    ESP_LOGI(TAG, "Palette %d saved successfully", palette_index);
    return true;
}

// ========================================
// 画像をSDカードに保存（オリジナル＋全パレット）
// ========================================
static bool save_images() {
    uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    ESP_LOGI(TAG, "Starting capture... (File number: %04d)", s_file_counter);
    
    // ステップ1: フレーム取得
    s_fb = esp_camera_fb_get();
    if (!s_fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return false;
    }
    
    ESP_LOGI(TAG, "Frame captured (%dx%d)", s_fb->width, s_fb->height);
    
    // ステップ2: オリジナル画像を保存
    bool success = save_original_bmp(timestamp);
    if (!success) {
        ESP_LOGE(TAG, "Failed to save original image");
        esp_camera_fb_return(s_fb);
        s_fb = NULL;
        return false;
    }
    
    // ステップ3: 輝度情報を計算
    ESP_LOGI(TAG, "Calculating luminance data...");
    save_graylevel_fb();
    ESP_LOGI(TAG, "Luminance calculation completed");
    
    // ステップ4: 全パレット（0-7）で変換して保存
    ESP_LOGI(TAG, "Converting color palettes...");
    for (int i = 0; i < MAX_PALETTE_INDEX; i++) {
        bool palette_success = save_palette_bmp(timestamp, i);
        if (!palette_success) {
            ESP_LOGW(TAG, "Failed to save palette %d", i);
        }
    }
    
    // ステップ5: クリーンアップ
    esp_camera_fb_return(s_fb);
    s_fb = NULL;
    
    ESP_LOGI(TAG, "All save operations completed");
    ESP_LOGI(TAG, "Total: Original(1) + Palettes(%d) = %d files", 
             MAX_PALETTE_INDEX, MAX_PALETTE_INDEX + 1);
    
    s_file_counter++;  // 連番を更新
    
    return true;
}

// ========================================
// エンコーダ値の更新とLED表示
// ========================================
static void update_encoder() {
    // エンコーダが初期化されていない場合は何もしない
    if (!s_encoder || !s_encoder->is_initialized()) return;
    
    // エンコーダ値を更新
    s_current_palette_index = s_encoder->update();
    
    // 値が変化した場合のみLEDを更新
    if (s_current_palette_index != s_last_palette_index) {
        // パレット代表色をLEDに表示
        s_encoder->set_led_color(PALETTE_REP_COLORS[s_current_palette_index]);
        
        // ログ出力
        ESP_LOGI(TAG, "Palette selected: %d", s_current_palette_index);
        
        s_last_palette_index = s_current_palette_index;
    }
}

// ========================================
// メインタスク
// ========================================
static void main_task(void* pvParameters) {
    ESP_LOGI(TAG, "Main task started");
    
    while (1) {
        // エンコーダの状態を更新
        update_encoder();
        
        // ボタンが押されたかチェック
        if (is_button_pressed()) {
            ESP_LOGI(TAG, "Button pressed!");
            ESP_LOGI(TAG, "Current palette: %d", s_current_palette_index);
            
            // LED点灯（保存中を示す）
            led_on();
            
            // 画像を撮影＆保存
            bool success = save_images();
            
            if (success) {
                ESP_LOGI(TAG, "Capture & save completed!");
                ESP_LOGI(TAG, "(Original: 1 + Color palettes: 8 = Total: 9 files)");
                
                // 成功を知らせる（短く2回点滅）
                led_off();
                vTaskDelay(pdMS_TO_TICKS(100));
                led_on();
                vTaskDelay(pdMS_TO_TICKS(100));
                led_off();
                vTaskDelay(pdMS_TO_TICKS(100));
                led_on();
                vTaskDelay(pdMS_TO_TICKS(100));
                led_off();
            } else {
                ESP_LOGE(TAG, "Save failed...");
                
                // 失敗を知らせる（長く3回点滅）
                for (int i = 0; i < 3; i++) {
                    led_off();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    led_on();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                led_off();
            }
            
            ESP_LOGI(TAG, "Next file number: %04d", s_file_counter);
            
            // エンコーダLEDを現在のパレット色に戻す
            if (s_encoder && s_encoder->is_initialized()) {
                s_encoder->set_led_color(PALETTE_REP_COLORS[s_current_palette_index]);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // ループ負荷軽減
    }
}

// ========================================
// アプリケーションメイン
// ========================================
extern "C" void app_main() {
    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  📸 ピクセルアートカメラ 起動にゃ！ 📸 ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    // NVS初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // GPIO初期化
    ESP_ERROR_CHECK(init_gpio());
    ESP_LOGI(TAG, "GPIO (Button:GPIO38, LED:GPIO39) initialized");
    
    // エンコーダ初期化
    ret = init_encoder();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Encoder initialization failed, continuing without encoder...");
    }
    
    // カメラ初期化
    ESP_ERROR_CHECK(init_camera());
    
    // SDカード初期化
    ESP_ERROR_CHECK(init_sdcard());
    
    // 準備完了を知らせる（LED 3回点滅）
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ システム準備完了にゃ！");
    ESP_LOGI(TAG, "   🔘 ボタン: 撮影＆保存");
    if (s_encoder && s_encoder->is_initialized()) {
        ESP_LOGI(TAG, "   🎨 エンコーダ: パレット選択(0-7)");
    }
    ESP_LOGI(TAG, "========================================");
    
    for (int i = 0; i < 3; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(100));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // メインタスクを作成
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}