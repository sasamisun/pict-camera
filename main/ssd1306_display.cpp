/*
 * SSD1306 OLED ディスプレイ制御クラス (ESP-IDF 5.4対応)
 * ピクセルアートカメラ プレビュー表示用にゃ
 * 
 * 機能:
 * - 128x64 SSD1306 OLEDディスプレイ制御
 * - カメラプレビュー画像の表示
 * - 自動サイズ変更と減色処理
 * - I2C共有（エンコーダと同じバス使用）
 * 
 * 作成者: にゃんにゃんプログラマー
 */

#include "ssd1306_display.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SSD1306";

// ========================================
// SSD1306コマンド定義にゃ
// ========================================
#define SSD1306_CONTROL_CMD_SINGLE    0x80
#define SSD1306_CONTROL_CMD_STREAM    0x00
#define SSD1306_CONTROL_DATA_STREAM   0x40

// 基本コマンド
#define SSD1306_CMD_SET_CHARGE_PUMP   0x8D
#define SSD1306_CMD_SET_SEGMENT_REMAP 0xA1
#define SSD1306_CMD_SET_MUX_RATIO     0xA8
#define SSD1306_CMD_SET_COM_SCAN_MODE 0xC8
#define SSD1306_CMD_SET_DISP_OFFSET   0xD3
#define SSD1306_CMD_SET_COM_PIN_MAP   0xDA
#define SSD1306_CMD_SET_DISP_CLK_DIV  0xD5
#define SSD1306_CMD_SET_PRECHARGE     0xD9
#define SSD1306_CMD_SET_VCOMH_DESELCT 0xDB
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE 0x20
#define SSD1306_CMD_SET_HORI_ADDR_RANGE 0x21
#define SSD1306_CMD_SET_VERT_ADDR_RANGE 0x22
#define SSD1306_CMD_SET_DISP_ON       0xAF
#define SSD1306_CMD_SET_DISP_OFF      0xAE
#define SSD1306_CMD_SET_ENTIRE_DISP_ON 0xA5
#define SSD1306_CMD_SET_NORM_DISP     0xA6
#define SSD1306_CMD_SET_INVERT_DISP   0xA7
#define SSD1306_CMD_SET_CONTRAST      0x81

// 初期化シーケンス用のコマンド配列にゃ
static const uint8_t ssd1306_init_sequence[] = {
    SSD1306_CMD_SET_DISP_OFF,        // ディスプレイOFF
    SSD1306_CMD_SET_MUX_RATIO, 63,   // 高さ64px設定
    SSD1306_CMD_SET_DISP_OFFSET, 0,  // オフセットなし
    0x40,                            // スタートライン = 0
    SSD1306_CMD_SET_SEGMENT_REMAP,   // セグメントリマップ
    SSD1306_CMD_SET_COM_SCAN_MODE,   // COM出力スキャン方向
    SSD1306_CMD_SET_COM_PIN_MAP, 0x12, // COMピン設定
    SSD1306_CMD_SET_CONTRAST, 0x7F,  // コントラスト中程度
    SSD1306_CMD_SET_ENTIRE_DISP_ON,  // 全画面ON無効
    SSD1306_CMD_SET_NORM_DISP,       // 通常表示（反転なし）
    SSD1306_CMD_SET_DISP_CLK_DIV, 0x80, // クロック分周比
    SSD1306_CMD_SET_CHARGE_PUMP, 0x14,  // チャージポンプON
    SSD1306_CMD_SET_MEMORY_ADDR_MODE, 0x00, // 水平アドレッシングモード
    SSD1306_CMD_SET_HORI_ADDR_RANGE, 0, 127, // 水平範囲: 0-127
    SSD1306_CMD_SET_VERT_ADDR_RANGE, 0, 7,   // 垂直範囲: 0-7 (8ページ)
    SSD1306_CMD_SET_DISP_ON          // ディスプレイON
};

// ========================================
// コンストラクタ/デストラクタにゃ
// ========================================
SSD1306Display::SSD1306Display(i2c_master_bus_handle_t i2c_bus, uint8_t address)
    : _i2c_bus_handle(i2c_bus)
    , _i2c_dev_handle(nullptr)
    , _i2c_address(address)
    , _initialized(false)
    , _display_mutex(nullptr)
    , _buffer(nullptr)
{
    ESP_LOGI(TAG, "🖥️ SSD1306ディスプレイオブジェクト作成: アドレス 0x%02X", address);
}

SSD1306Display::~SSD1306Display() {
    end();
}

// ========================================
// 初期化処理にゃ
// ========================================
esp_err_t SSD1306Display::begin() {
    ESP_LOGI(TAG, "🔧 SSD1306初期化開始にゃ");
    
    if (_initialized) {
        ESP_LOGW(TAG, "⚠️ 既に初期化済みです");
        return ESP_OK;
    }
    
    if (!_i2c_bus_handle) {
        ESP_LOGE(TAG, "❌ I2Cバスハンドルが無効です");
        return ESP_ERR_INVALID_ARG;
    }
    
    // ミューテックス作成
    _display_mutex = xSemaphoreCreateMutex();
    if (!_display_mutex) {
        ESP_LOGE(TAG, "❌ ミューテックス作成失敗");
        return ESP_ERR_NO_MEM;
    }
    
    // フレームバッファ確保（128x64 = 1024バイト）
    _buffer = (uint8_t*)malloc(SSD1306_WIDTH * SSD1306_HEIGHT / 8);
    if (!_buffer) {
        ESP_LOGE(TAG, "❌ フレームバッファ確保失敗");
        vSemaphoreDelete(_display_mutex);
        _display_mutex = nullptr;
        return ESP_ERR_NO_MEM;
    }
    
    // バッファクリア
    memset(_buffer, 0, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
    
    // I2Cデバイス設定
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = _i2c_address;
    dev_cfg.scl_speed_hz = 100000; // 100kHzに下げる（SSD1306は高速通信で問題が出ることがある）
    dev_cfg.scl_wait_us = 2000;    // 待機時間を増加
    dev_cfg.flags.disable_ack_check = false;
    
    esp_err_t ret = i2c_master_bus_add_device(_i2c_bus_handle, &dev_cfg, &_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ I2Cデバイス追加失敗: %s", esp_err_to_name(ret));
        free(_buffer);
        _buffer = nullptr;
        vSemaphoreDelete(_display_mutex);
        _display_mutex = nullptr;
        return ret;
    }
    
    // デバイス存在確認
    ESP_LOGI(TAG, "🔍 SSD1306デバイス存在確認...");
    ret = i2c_master_probe(_i2c_bus_handle, _i2c_address, 2000); // タイムアウト延長
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SSD1306デバイスが見つかりません (0x%02X): %s", 
                 _i2c_address, esp_err_to_name(ret));
        end();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "✅ SSD1306デバイス検出成功!");
    
    // 少し待機してからコマンド送信
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 初期化シーケンス送信
    ESP_LOGI(TAG, "📡 初期化コマンド送信中...");
    ret = send_init_sequence();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ 初期化シーケンス送信失敗");
        end();
        return ret;
    }
    
    // 初期化完了
    _initialized = true;
    ESP_LOGI(TAG, "🎉 SSD1306初期化完了にゃ!");
    
    // test - ディスプレイ動作確認用テストパターン
    ESP_LOGI(TAG, "📺 テストパターン表示開始");
    
    // test - パターン1: 全画面クリア（黒）
    clear();
    update();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // test - パターン2: 全画面白
    if (take_mutex()) {
        memset(_buffer, 0xFF, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
        give_mutex();
    }
    update();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // test - パターン3: チェッカーボードパターン
    clear();
    for (int y = 0; y < SSD1306_HEIGHT; y += 8) {
        for (int x = 0; x < SSD1306_WIDTH; x += 8) {
            if (((x / 8) + (y / 8)) % 2 == 0) {
                draw_rect(x, y, 8, 8, true);
            }
        }
    }
    update();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // test - パターン4: 縦線パターン
    clear();
    for (int x = 0; x < SSD1306_WIDTH; x += 8) {
        for (int y = 0; y < SSD1306_HEIGHT; y++) {
            draw_pixel(x, y, true);
        }
    }
    update();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // test - パターン5: 境界テスト（四角形の枠）
    clear();
    // 外枠
    for (int x = 0; x < SSD1306_WIDTH; x++) {
        draw_pixel(x, 0, true);                    // 上端
        draw_pixel(x, SSD1306_HEIGHT-1, true);    // 下端
    }
    for (int y = 0; y < SSD1306_HEIGHT; y++) {
        draw_pixel(0, y, true);                    // 左端
        draw_pixel(SSD1306_WIDTH-1, y, true);     // 右端
    }
    // 中央の十字
    int center_x = SSD1306_WIDTH / 2;
    int center_y = SSD1306_HEIGHT / 2;
    for (int x = 0; x < SSD1306_WIDTH; x++) {
        draw_pixel(x, center_y, true);             // 水平線
    }
    for (int y = 0; y < SSD1306_HEIGHT; y++) {
        draw_pixel(center_x, y, true);             // 垂直線
    }
    update();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // test - 最終パターン: 通常の初期化メッセージ
    clear();
    draw_text(0, 0, "PixelArt Camera", 1);
    draw_text(0, 16, "SSD1306 Ready!", 1);
    draw_text(0, 32, "Waiting for", 1);
    draw_text(0, 48, "camera preview...", 1);
    update();
    
    ESP_LOGI(TAG, "✅ テストパターン表示完了");
    
    return ESP_OK;
}

// ========================================
// 終了処理にゃ
// ========================================
void SSD1306Display::end() {
    if (!_initialized) return;
    
    ESP_LOGI(TAG, "🔚 SSD1306終了処理開始");
    
    _initialized = false;
    
    // ディスプレイOFF
    if (_i2c_dev_handle) {
        send_command(SSD1306_CMD_SET_DISP_OFF);
    }
    
    // I2Cデバイス削除
    if (_i2c_dev_handle) {
        i2c_master_bus_rm_device(_i2c_dev_handle);
        _i2c_dev_handle = nullptr;
    }
    
    // バッファ解放
    if (_buffer) {
        free(_buffer);
        _buffer = nullptr;
    }
    
    // ミューテックス削除
    if (_display_mutex) {
        vSemaphoreDelete(_display_mutex);
        _display_mutex = nullptr;
    }
    
    ESP_LOGI(TAG, "✅ SSD1306終了処理完了");
}

// ========================================
// I2C通信：コマンド送信にゃ
// ========================================
esp_err_t SSD1306Display::send_command(uint8_t cmd) {
    if (!_i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[2] = {SSD1306_CONTROL_CMD_SINGLE, cmd};
    esp_err_t ret = i2c_master_transmit(_i2c_dev_handle, data, 2, 1000);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ コマンド送信失敗: 0x%02X, エラー: %s", cmd, esp_err_to_name(ret));
    }
    
    return ret;
}

// ========================================
// I2C通信：データ送信にゃ
// ========================================
esp_err_t SSD1306Display::send_data(const uint8_t* data, size_t len) {
    if (!_initialized || !_i2c_dev_handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // コントロールバイト + データの形式で送信
    uint8_t* buffer = (uint8_t*)malloc(len + 1);
    if (!buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    buffer[0] = SSD1306_CONTROL_DATA_STREAM;
    memcpy(&buffer[1], data, len);
    
    esp_err_t ret = i2c_master_transmit(_i2c_dev_handle, buffer, len + 1, 1000);
    free(buffer);
    
    return ret;
}

// ========================================
// 初期化シーケンス送信にゃ
// ========================================
esp_err_t SSD1306Display::send_init_sequence() {
    ESP_LOGI(TAG, "📡 SSD1306初期化シーケンス開始");
    
    // より簡単な初期化シーケンスを使用
    const uint8_t simple_init[] = {
        SSD1306_CMD_SET_DISP_OFF,        // ディスプレイOFF
        SSD1306_CMD_SET_MUX_RATIO, 63,   // 高さ64px設定
        SSD1306_CMD_SET_DISP_OFFSET, 0,  // オフセットなし
        0x40,                            // スタートライン = 0
        SSD1306_CMD_SET_SEGMENT_REMAP,   // セグメントリマップ
        SSD1306_CMD_SET_COM_SCAN_MODE,   // COM出力スキャン方向
        SSD1306_CMD_SET_COM_PIN_MAP, 0x12, // COMピン設定
        SSD1306_CMD_SET_CONTRAST, 0x7F,  // コントラスト中程度
        SSD1306_CMD_SET_ENTIRE_DISP_ON,  // 全画面ON無効
        SSD1306_CMD_SET_NORM_DISP,       // 通常表示
        SSD1306_CMD_SET_DISP_CLK_DIV, 0x80, // クロック分周比
        SSD1306_CMD_SET_CHARGE_PUMP, 0x14,  // チャージポンプON
        SSD1306_CMD_SET_MEMORY_ADDR_MODE, 0x00, // 水平アドレッシングモード
        SSD1306_CMD_SET_DISP_ON          // ディスプレイON
    };
    
    for (size_t i = 0; i < sizeof(simple_init); i++) {
        esp_err_t ret = send_command(simple_init[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ 初期化コマンド送信失敗 [%zu]: 0x%02X", i, simple_init[i]);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(2)); // 少し長めの待機
        
        // デバッグ用：成功したコマンドをログ出力
        ESP_LOGD(TAG, "✅ コマンド送信成功 [%zu]: 0x%02X", i, simple_init[i]);
    }
    
    ESP_LOGI(TAG, "✅ 初期化シーケンス完了");
    return ESP_OK;
}

// ========================================
// フレームバッファクリアにゃ
// ========================================
void SSD1306Display::clear() {
    if (!_buffer) return;
    
    if (take_mutex()) {
        memset(_buffer, 0, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
        give_mutex();
    }
}

// ========================================
// ピクセル描画にゃ
// ========================================
void SSD1306Display::draw_pixel(int16_t x, int16_t y, bool color) {
    if (!_buffer || x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (take_mutex()) {
        uint16_t byte_index = x + (y / 8) * SSD1306_WIDTH;
        uint8_t bit_mask = 1 << (y % 8);
        
        if (color) {
            _buffer[byte_index] |= bit_mask;  // ピクセルON
        } else {
            _buffer[byte_index] &= ~bit_mask; // ピクセルOFF
        }
        
        give_mutex();
    }
}

// ========================================
// 矩形描画にゃ
// ========================================
void SSD1306Display::draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color) {
    for (int16_t i = 0; i < w; i++) {
        for (int16_t j = 0; j < h; j++) {
            draw_pixel(x + i, y + j, color);
        }
    }
}

// ========================================
// 簡易フォント（8x8ピクセル）にゃ
// ========================================
static const uint8_t font_8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // (space)
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00, 0x00}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00}, // F
    // ... 他の文字も必要に応じて追加
};

// ========================================
// テキスト描画にゃ
// ========================================
void SSD1306Display::draw_text(int16_t x, int16_t y, const char* text, uint8_t size) {
    if (!text) return;
    
    int16_t cursor_x = x;
    int16_t cursor_y = y;
    
    while (*text) {
        char c = *text++;
        
        // 基本的な文字のみサポート（簡易実装）
        if (c >= 'A' && c <= 'F') {
            uint8_t char_index = c - 'A' + 1; // font_8x8配列のインデックス
            draw_char(cursor_x, cursor_y, char_index, size);
        } else if (c == ' ') {
            draw_char(cursor_x, cursor_y, 0, size); // スペース
        } else {
            // サポートしない文字は・で表示
            draw_rect(cursor_x + 3, cursor_y + 3, 2, 2, true);
        }
        
        cursor_x += 8 * size; // 次の文字位置
        
        // 行末で折り返し
        if (cursor_x + 8 * size > SSD1306_WIDTH) {
            cursor_x = x;
            cursor_y += 8 * size;
        }
    }
}

// ========================================
// 文字描画（8x8フォント）にゃ
// ========================================
void SSD1306Display::draw_char(int16_t x, int16_t y, uint8_t char_index, uint8_t size) {
    if (char_index >= sizeof(font_8x8) / sizeof(font_8x8[0])) {
        return;
    }
    
    const uint8_t* char_data = font_8x8[char_index];
    
    for (int i = 0; i < 8; i++) {
        uint8_t line = char_data[i];
        for (int j = 0; j < 8; j++) {
            if (line & (0x80 >> j)) {
                // サイズに応じて拡大描画
                for (int sx = 0; sx < size; sx++) {
                    for (int sy = 0; sy < size; sy++) {
                        draw_pixel(x + j * size + sx, y + i * size + sy, true);
                    }
                }
            }
        }
    }
}

// ========================================
// フレームバッファをディスプレイに転送にゃ
// ========================================
esp_err_t SSD1306Display::update() {
    if (!_initialized || !_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    
    if (take_mutex()) {
        // アドレス範囲設定
        ret = send_command(SSD1306_CMD_SET_HORI_ADDR_RANGE);
        if (ret == ESP_OK) ret = send_command(0); // 開始列
        if (ret == ESP_OK) ret = send_command(127); // 終了列
        
        ret = send_command(SSD1306_CMD_SET_VERT_ADDR_RANGE);
        if (ret == ESP_OK) ret = send_command(0); // 開始ページ
        if (ret == ESP_OK) ret = send_command(7); // 終了ページ
        
        // フレームバッファ全体を送信
        if (ret == ESP_OK) {
            ret = send_data(_buffer, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
        }
        
        give_mutex();
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ ディスプレイ更新失敗: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// ========================================
// カメラプレビュー表示にゃ
// ========================================
esp_err_t SSD1306Display::show_camera_preview(camera_fb_t* fb) {
    if (!_initialized || !fb || !fb->buf) {
        // test - エラー原因の詳細ログ
        if (!_initialized) {
            ESP_LOGE(TAG, "❌ ディスプレイが初期化されていません");
        }
        if (!fb) {
            ESP_LOGE(TAG, "❌ フレームバッファがnullです");
        }
        if (fb && !fb->buf) {
            ESP_LOGE(TAG, "❌ フレームバッファのデータがnullです");
        }
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "📷 カメラプレビュー表示開始");
    ESP_LOGI(TAG, "  原画像: %dx%d, format: %d", fb->width, fb->height, fb->format);
    
    // 現在はRGB565フォーマットのみサポート
    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "❌ サポートしていない画像フォーマット: %d", fb->format);
        ESP_LOGI(TAG, "💡 PIXFORMAT_RGB565 (%d) が必要です", PIXFORMAT_RGB565);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // test - フレームバッファの状態確認
    ESP_LOGI(TAG, "📊 フレームバッファ詳細:");
    ESP_LOGI(TAG, "  データ長: %zu bytes", fb->len);
    ESP_LOGI(TAG, "  期待サイズ: %d bytes", fb->width * fb->height * 2); // RGB565は2バイト/ピクセル
    ESP_LOGI(TAG, "  タイムスタンプ: %llu", fb->timestamp.tv_sec * 1000000ULL + fb->timestamp.tv_usec);
    
    // test - データ破損チェック
    if (fb->len != fb->width * fb->height * 2) {
        ESP_LOGW(TAG, "⚠️ フレームバッファサイズが不正です");
        ESP_LOGW(TAG, "  実際: %zu bytes, 期待: %d bytes", fb->len, fb->width * fb->height * 2);
    }
    
    clear(); // 画面クリア
    
    // RGB565をモノクロに変換してSSD1306に適したサイズにリサイズ
    esp_err_t ret = convert_and_resize_rgb565(fb);
    
    if (ret == ESP_OK) {
        // test - 更新前のフレームバッファ状態をサンプリング
        ESP_LOGI(TAG, "🖥️ ディスプレイ更新前の状態:");
        if (take_mutex()) {
            int sample_pixels = 0;
            int white_count = 0;
            // 画面の数箇所をサンプリング
            for (int y = 0; y < SSD1306_HEIGHT; y += 16) {
                for (int x = 0; x < SSD1306_WIDTH; x += 16) {
                    uint16_t byte_index = x + (y / 8) * SSD1306_WIDTH;
                    uint8_t bit_mask = 1 << (y % 8);
                    bool pixel_on = (_buffer[byte_index] & bit_mask) != 0;
                    if (pixel_on) white_count++;
                    sample_pixels++;
                }
            }
            give_mutex();
            ESP_LOGI(TAG, "  サンプル: %d/%d ピクセルが白", white_count, sample_pixels);
            
            // test - 異常パターンの検出
            if (white_count == sample_pixels) {
                ESP_LOGW(TAG, "⚠️ サンプルした全てのピクセルが白です");
            } else if (white_count == 0) {
                ESP_LOGW(TAG, "⚠️ サンプルした全てのピクセルが黒です");
            } else {
                ESP_LOGI(TAG, "✅ 適切な白黒分布です");
            }
        }
        
        ret = update(); // ディスプレイに反映
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ プレビュー表示完了");
        } else {
            ESP_LOGE(TAG, "❌ ディスプレイ更新失敗: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "❌ 画像変換失敗: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// ========================================
// RGB565画像をモノクロでリサイズにゃ
// ========================================
esp_err_t SSD1306Display::convert_and_resize_rgb565(camera_fb_t* fb) {
    if (!fb || !fb->buf) return ESP_ERR_INVALID_ARG;
    
    const uint16_t* rgb565_data = (const uint16_t*)fb->buf;
    
    // test - カメラデータの基本情報をログ出力
    ESP_LOGI(TAG, "📷 カメラデータ解析開始");
    ESP_LOGI(TAG, "  画像サイズ: %dx%d", fb->width, fb->height);
    ESP_LOGI(TAG, "  データ長: %zu bytes", fb->len);
    ESP_LOGI(TAG, "  フォーマット: %d", fb->format);
    
    // test - 最初の数ピクセルのRGB565データをサンプリング
    ESP_LOGI(TAG, "  RGB565サンプル:");
    for (int i = 0; i < 8 && i < (fb->width * fb->height); i++) {
        uint16_t pixel = rgb565_data[i];
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        ESP_LOGI(TAG, "    [%d]: 0x%04X (R:%d G:%d B:%d)", i, pixel, r, g, b);
    }
    
    // スケーリング計算（アスペクト比維持）
    float scale_x = (float)SSD1306_WIDTH / fb->width;
    float scale_y = (float)SSD1306_HEIGHT / fb->height;
    float scale = (scale_x < scale_y) ? scale_x : scale_y; // 小さい方を選択
    
    int display_width = (int)(fb->width * scale);
    int display_height = (int)(fb->height * scale);
    
    // 中央配置用のオフセット
    int offset_x = (SSD1306_WIDTH - display_width) / 2;
    int offset_y = (SSD1306_HEIGHT - display_height) / 2;
    
    ESP_LOGI(TAG, "🔄 画像変換: %dx%d → %dx%d (scale: %.2f)", 
             fb->width, fb->height, display_width, display_height, scale);
    ESP_LOGI(TAG, "📐 オフセット: x=%d, y=%d", offset_x, offset_y);
    
    // test - 輝度統計用カウンタ
    int white_pixels = 0;  // 白ピクセル数
    int black_pixels = 0;  // 黒ピクセル数
    int total_gray = 0;    // 輝度の総和
    int pixel_count = 0;   // 処理したピクセル数
    
    // リサイズ＆モノクロ変換
    for (int y = 0; y < display_height; y++) {
        for (int x = 0; x < display_width; x++) {
            // 元画像の対応ピクセル位置を計算
            int src_x = (int)(x / scale);
            int src_y = (int)(y / scale);
            
            // 境界チェック
            if (src_x >= fb->width || src_y >= fb->height) continue;
            
            // RGB565ピクセルを取得
            uint16_t rgb565 = rgb565_data[src_y * fb->width + src_x];
            
            // RGB565をRGBに分解
            uint8_t r = (rgb565 >> 11) & 0x1F;
            uint8_t g = (rgb565 >> 5) & 0x3F;
            uint8_t b = rgb565 & 0x1F;
            
            // 8ビットRGBに正規化
            r = (r * 255) / 31;
            g = (g * 255) / 63;
            b = (b * 255) / 31;
            
            // 輝度計算（ITU-R BT.601）
            uint8_t gray = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
            
            // test - 統計情報更新
            total_gray += gray;
            pixel_count++;
            
            // 閾値処理（ディザリングなしの単純な白黒変換）
            bool pixel_on = gray > 128;
            
            // test - 白黒ピクセル数カウント
            if (pixel_on) {
                white_pixels++;
            } else {
                black_pixels++;
            }
            
            // test - 最初の数ピクセルの変換過程をログ出力
            if (pixel_count <= 8) {
                ESP_LOGI(TAG, "    変換[%d,%d]: RGB(%d,%d,%d) → Gray=%d → %s", 
                         x, y, r, g, b, gray, pixel_on ? "白" : "黒");
            }
            
            // ディスプレイにピクセル描画
            draw_pixel(offset_x + x, offset_y + y, pixel_on);
        }
    }
    
    // test - 変換統計情報をログ出力
    if (pixel_count > 0) {
        int avg_gray = total_gray / pixel_count;
        ESP_LOGI(TAG, "📊 変換統計:");
        ESP_LOGI(TAG, "  処理ピクセル数: %d", pixel_count);
        ESP_LOGI(TAG, "  平均輝度: %d/255", avg_gray);
        ESP_LOGI(TAG, "  白ピクセル: %d (%.1f%%)", white_pixels, (float)white_pixels * 100 / pixel_count);
        ESP_LOGI(TAG, "  黒ピクセル: %d (%.1f%%)", black_pixels, (float)black_pixels * 100 / pixel_count);
        
        // test - 異常パターンの検出
        if (white_pixels == pixel_count) {
            ESP_LOGW(TAG, "⚠️ 全ピクセルが白です - カメラ露出過多の可能性");
        } else if (black_pixels == pixel_count) {
            ESP_LOGW(TAG, "⚠️ 全ピクセルが黒です - カメラ露出不足の可能性");
        } else if (avg_gray < 64) {
            ESP_LOGW(TAG, "⚠️ 全体的に暗い画像です (平均輝度: %d)", avg_gray);
        } else if (avg_gray > 192) {
            ESP_LOGW(TAG, "⚠️ 全体的に明るい画像です (平均輝度: %d)", avg_gray);
        } else {
            ESP_LOGI(TAG, "✅ 適切な輝度分布です");
        }
    }
    
    return ESP_OK;
}

// ========================================
// コントラスト設定にゃ
// ========================================
esp_err_t SSD1306Display::set_contrast(uint8_t contrast) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = send_command(SSD1306_CMD_SET_CONTRAST);
    if (ret == ESP_OK) {
        ret = send_command(contrast);
    }
    
    return ret;
}

// ========================================
// 表示ON/OFF にゃ
// ========================================
esp_err_t SSD1306Display::set_display_on(bool on) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return send_command(on ? SSD1306_CMD_SET_DISP_ON : SSD1306_CMD_SET_DISP_OFF);
}

// ========================================
// 反転表示にゃ
// ========================================
esp_err_t SSD1306Display::set_invert_display(bool invert) {
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return send_command(invert ? SSD1306_CMD_SET_INVERT_DISP : SSD1306_CMD_SET_NORM_DISP);
}

// ========================================
// ステータス確認にゃ
// ========================================
bool SSD1306Display::is_initialized() const {
    return _initialized;
}

// ========================================
// I2Cアドレス取得にゃ
// ========================================
uint8_t SSD1306Display::get_i2c_address() const {
    return _i2c_address;
}

// ========================================
// ミューテックス処理にゃ
// ========================================
bool SSD1306Display::take_mutex() {
    if (!_display_mutex) return false;
    return xSemaphoreTake(_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

void SSD1306Display::give_mutex() {
    if (_display_mutex) {
        xSemaphoreGive(_display_mutex);
    }
}