/*
 * SSD1306 OLED ディスプレイ制御クラス 実装ファイル (レガシーI2C統一版)
 * ESP-IDF 5.4対応 + ピクセルアートカメラ プレビュー表示用
 */

#include "ssd1306_display.h"
#include <string.h>
#include <stdio.h>
#include "esp_timer.h"

static const char* TAG = "SSD1306Display";

// 8x8 簡易フォント（英数字のみ）
static const uint8_t font_8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 空白
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00, 0x00, 0x00}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, 0x00, 0x00}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00}, // 9
    // A-Z（簡略版）
    {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, 0x00, 0x00}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00}, // C
    // ... 他の文字は必要に応じて追加
};

// ========================================
// コンストラクタ（レガシードライバー版）
// ========================================
SSD1306Display::SSD1306Display(i2c_port_t i2c_port, uint8_t address)
    : _i2c_port(i2c_port)
    , _i2c_address(address)
    , _initialized(false)
    , _frame_buffer(nullptr)
    , _display_mutex(nullptr)
    , _i2c_error_count(0)
    , _update_count(0)
{
    ESP_LOGD(TAG, "SSD1306Display オブジェクト作成: ポート=%d, アドレス=0x%02X", 
             _i2c_port, _i2c_address);
}

// ========================================
// デストラクタ
// ========================================
SSD1306Display::~SSD1306Display() 
{
    end();
    ESP_LOGD(TAG, "SSD1306Display オブジェクト削除完了");
}

// ========================================
// コマンド送信（レガシードライバー版）
// ========================================
esp_err_t SSD1306Display::write_command(uint8_t cmd, int retries) 
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt < retries; attempt++) {
        // コマンドリンク作成
        i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
        if (!cmd_handle) {
            _i2c_error_count++;
            return ESP_ERR_NO_MEM;
        }

        // コマンド送信（制御バイト 0x00 + コマンド）
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_handle, 0x00, true);  // 制御バイト（コマンド）
        i2c_master_write_byte(cmd_handle, cmd, true);   // コマンドデータ
        i2c_master_stop(cmd_handle);
        
        // 実行
        ret = i2c_master_cmd_begin(_i2c_port, cmd_handle, SSD1306_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_handle);
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "コマンド送信成功: 0x%02X", cmd);
            break;
        } else {
            ESP_LOGW(TAG, "コマンド送信失敗 (試行%d/%d): 0x%02X, エラー: %s", 
                     attempt + 1, retries, cmd, esp_err_to_name(ret));
            _i2c_error_count++;
            
            if (attempt < retries - 1) {
                vTaskDelay(pdMS_TO_TICKS(5));  // リトライ間隔
            }
        }
    }
    
    return ret;
}

// ========================================
// データ送信（レガシードライバー版）
// ========================================
esp_err_t SSD1306Display::write_data(const uint8_t* data, size_t len, int retries) 
{
    if (!_initialized || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt < retries; attempt++) {
        // コマンドリンク作成
        i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
        if (!cmd_handle) {
            _i2c_error_count++;
            return ESP_ERR_NO_MEM;
        }

        // データ送信（制御バイト 0x40 + データ）
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd_handle, 0x40, true);  // 制御バイト（データ）
        i2c_master_write(cmd_handle, data, len, true);  // データ
        i2c_master_stop(cmd_handle);
        
        // 実行
        ret = i2c_master_cmd_begin(_i2c_port, cmd_handle, SSD1306_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd_handle);
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "データ送信成功: %zu bytes", len);
            break;
        } else {
            ESP_LOGW(TAG, "データ送信失敗 (試行%d/%d): %zu bytes, エラー: %s", 
                     attempt + 1, retries, len, esp_err_to_name(ret));
            _i2c_error_count++;
            
            if (attempt < retries - 1) {
                vTaskDelay(pdMS_TO_TICKS(5));  // リトライ間隔
            }
        }
    }
    
    return ret;
}

// ========================================
// デバイス接続確認（レガシードライバー版）
// ========================================
esp_err_t SSD1306Display::probe_device() 
{
    ESP_LOGD(TAG, "デバイス接続確認: 0x%02X", _i2c_address);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SSD1306デバイス検出成功: 0x%02X", _i2c_address);
    } else {
        ESP_LOGW(TAG, "❌ SSD1306デバイス検出失敗: 0x%02X, エラー: %s", 
                 _i2c_address, esp_err_to_name(ret));
        _i2c_error_count++;
    }
    
    return ret;
}

// ========================================
// 初期化シーケンス送信
// ========================================
esp_err_t SSD1306Display::send_init_sequence() 
{
    ESP_LOGD(TAG, "SSD1306初期化シーケンス送信開始");
    
    // 初期化コマンドシーケンス
    const uint8_t init_commands[] = {
        SSD1306_CMD_DISPLAY_OFF,        // ディスプレイOFF
        SSD1306_CMD_SET_DISPLAY_CLOCK,  // クロック設定
        0x80,                           // 推奨値
        SSD1306_CMD_SET_MULTIPLEX,      // マルチプレックス設定
        0x3F,                           // 64MUX（64行）
        SSD1306_CMD_SET_DISPLAY_OFFSET, // ディスプレイオフセット
        0x00,                           // オフセット無し
        SSD1306_CMD_SET_START_LINE,     // 開始行設定
        SSD1306_CMD_CHARGE_PUMP,        // チャージポンプ設定
        0x14,                           // 内蔵VCC使用
        SSD1306_CMD_MEMORY_MODE,        // メモリアドレッシングモード
        0x00,                           // 水平アドレッシングモード
        SSD1306_CMD_SEG_REMAP,          // セグメントリマップ
        SSD1306_CMD_COM_SCAN_DEC,       // COM出力スキャン方向
        SSD1306_CMD_SET_COM_PINS,       // COMピン設定
        0x12,                           // 代替COMピン設定
        SSD1306_CMD_SET_CONTRAST,       // コントラスト設定
        0xCF,                           // 初期コントラスト値
        SSD1306_CMD_SET_PRECHARGE,      // プリチャージ期間
        0xF1,                           // プリチャージ期間設定
        SSD1306_CMD_SET_VCOM_DETECT,    // VCOMH設定
        0x40,                           // VCOMH設定値
        SSD1306_CMD_DISPLAY_ALL_ON_RESUME, // 全点灯解除
        SSD1306_CMD_NORMAL_DISPLAY,     // 通常表示
        SSD1306_CMD_DEACTIVATE_SCROLL,  // スクロール無効
        SSD1306_CMD_DISPLAY_ON          // ディスプレイON
    };
    
    // コマンドを順次送信
    for (size_t i = 0; i < sizeof(init_commands); i++) {
        esp_err_t ret = write_command(init_commands[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "初期化コマンド送信失敗: 0x%02X at index %zu", 
                     init_commands[i], i);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // コマンド間隔
    }
    
    ESP_LOGI(TAG, "✅ SSD1306初期化シーケンス送信完了");
    return ESP_OK;
}

// ========================================
// 初期化
// ========================================
esp_err_t SSD1306Display::begin() 
{
    ESP_LOGI(TAG, "🔧 SSD1306ディスプレイ初期化開始");
    
    if (_initialized) {
        ESP_LOGW(TAG, "⚠️ 既に初期化済み");
        return ESP_OK;
    }
    
    // ミューテックス作成
    _display_mutex = xSemaphoreCreateMutex();
    if (!_display_mutex) {
        ESP_LOGE(TAG, "❌ ミューテックス作成失敗");
        return ESP_ERR_NO_MEM;
    }
    
    // フレームバッファ確保
    _frame_buffer = (uint8_t*)calloc(SSD1306_BUFFER_SIZE, 1);
    if (!_frame_buffer) {
        ESP_LOGE(TAG, "❌ フレームバッファ確保失敗");
        vSemaphoreDelete(_display_mutex);
        _display_mutex = nullptr;
        return ESP_ERR_NO_MEM;
    }
    
    // デバイス接続確認
    esp_err_t ret = probe_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SSD1306デバイス接続確認失敗");
        end();
        return ret;
    }
    
    // 初期化シーケンス送信
    ret = send_init_sequence();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ SSD1306初期化シーケンス失敗");
        end();
        return ret;
    }
    
    _initialized = true;
    
    // 初期画面表示
    clear();
    draw_string(0, 0, "SSD1306 Ready!");
    draw_string(0, 2, "Pixel Art Cam");
    display();
    
    ESP_LOGI(TAG, "✅ SSD1306ディスプレイ初期化完了");
    return ESP_OK;
}

// ========================================
// 終了処理
// ========================================
void SSD1306Display::end() 
{
    if (_initialized) {
        // ディスプレイOFF
        write_command(SSD1306_CMD_DISPLAY_OFF);
        _initialized = false;
    }
    
    if (_frame_buffer) {
        free(_frame_buffer);
        _frame_buffer = nullptr;
    }
    
    if (_display_mutex) {
        vSemaphoreDelete(_display_mutex);
        _display_mutex = nullptr;
    }
    
    ESP_LOGI(TAG, "SSD1306ディスプレイ終了処理完了");
}

// ========================================
// フレームバッファクリア
// ========================================
void SSD1306Display::clear() 
{
    if (!_initialized || !_frame_buffer) {
        return;
    }
    
    if (xSemaphoreTake(_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(_frame_buffer, 0, SSD1306_BUFFER_SIZE);
        xSemaphoreGive(_display_mutex);
    }
}

// ========================================
// ピクセル設定
// ========================================
void SSD1306Display::set_pixel(int x, int y, bool color) 
{
    if (!_initialized || !_frame_buffer) {
        return;
    }
    
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (xSemaphoreTake(_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int byte_index = x + (y / 8) * SSD1306_WIDTH;
        int bit_index = y % 8;
        
        if (color) {
            _frame_buffer[byte_index] |= (1 << bit_index);
        } else {
            _frame_buffer[byte_index] &= ~(1 << bit_index);
        }
        
        xSemaphoreGive(_display_mutex);
    }
}

// ========================================
// ディスプレイ更新
// ========================================
esp_err_t SSD1306Display::display() 
{
    if (!_initialized || !_frame_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    
    if (xSemaphoreTake(_display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // 列アドレス設定
        ret = write_command(SSD1306_CMD_SET_COLUMN_ADDR);
        if (ret == ESP_OK) ret = write_command(0);      // 開始列
        if (ret == ESP_OK) ret = write_command(127);    // 終了列
        
        // ページアドレス設定
        if (ret == ESP_OK) ret = write_command(SSD1306_CMD_SET_PAGE_ADDR);
        if (ret == ESP_OK) ret = write_command(0);      // 開始ページ
        if (ret == ESP_OK) ret = write_command(7);      // 終了ページ
        
        // フレームバッファ送信
        if (ret == ESP_OK) {
            ret = write_data(_frame_buffer, SSD1306_BUFFER_SIZE);
        }
        
        if (ret == ESP_OK) {
            _update_count++;
        }
        
        xSemaphoreGive(_display_mutex);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    
    return ret;
}

// ========================================
// 簡易文字列描画
// ========================================
void SSD1306Display::draw_string(int x, int y, const char* text) 
{
    if (!text || !_initialized) {
        return;
    }
    
    int char_x = x * 8;  // 文字幅8ピクセル
    int char_y = y * 8;  // 文字高8ピクセル
    
    for (int i = 0; text[i] != '\0' && (char_x + i * 8) < SSD1306_WIDTH; i++) {
        char c = text[i];
        const uint8_t* font_data = nullptr;
        
        // フォントデータ選択（簡易版）
        if (c >= '0' && c <= '9') {
            font_data = font_8x8[c - '0' + 1];  // 0-9
        } else if (c >= 'A' && c <= 'C') {
            font_data = font_8x8[c - 'A' + 11]; // A-C（簡略版）
        } else {
            font_data = font_8x8[0];  // 空白
        }
        
        // 8x8ピクセルで文字描画
        for (int px = 0; px < 8; px++) {
            for (int py = 0; py < 8; py++) {
                bool pixel = (font_data[py] >> (7 - px)) & 1;
                set_pixel(char_x + i * 8 + px, char_y + py, pixel);
            }
        }
    }
}

// ========================================
// 行単位テキスト表示
// ========================================
void SSD1306Display::print_line(int line, const char* text) 
{
    if (line < 0 || line >= 8) {
        return;
    }
    
    // 該当行をクリア
    int start_y = line * 8;
    for (int y = start_y; y < start_y + 8; y++) {
        for (int x = 0; x < SSD1306_WIDTH; x++) {
            set_pixel(x, y, false);
        }
    }
    
    // テキスト描画
    draw_string(0, line, text);
}

// ========================================
// カメラプレビュー表示
// ========================================
esp_err_t SSD1306Display::show_camera_preview(camera_fb_t* fb, int x, int y, 
                                               int width, int height) 
{
    if (!fb || !_initialized || !_frame_buffer) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGW(TAG, "⚠️ RGB565以外のフォーマット: %d", fb->format);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // 範囲チェック
    if (x < 0 || y < 0 || (x + width) > SSD1306_WIDTH || (y + height) > SSD1306_HEIGHT) {
        ESP_LOGW(TAG, "⚠️ プレビュー範囲が画面外: (%d,%d)+(%dx%d)", x, y, width, height);
        return ESP_ERR_INVALID_ARG;
    }
    
    // RGB565データをモノクロに変換してリサイズ
    rgb565_to_mono_resize(fb->buf, fb->width, fb->height, 
                          _frame_buffer, width, height);
    
    ESP_LOGD(TAG, "カメラプレビュー表示: %dx%d → %dx%d", 
             fb->width, fb->height, width, height);
    
    return display();
}

// ========================================
// RGB565 → モノクロ変換 + リサイズ
// ========================================
void SSD1306Display::rgb565_to_mono_resize(const uint8_t* rgb565_data, int src_width, int src_height,
                                            uint8_t* mono_buffer, int dst_width, int dst_height) 
{
    if (!rgb565_data || !mono_buffer) {
        return;
    }
    
    const uint16_t* rgb565 = (const uint16_t*)rgb565_data;
    
    for (int dst_y = 0; dst_y < dst_height; dst_y++) {
        for (int dst_x = 0; dst_x < dst_width; dst_x++) {
            // ソース座標計算（最近傍補間）
            int src_x = (dst_x * src_width) / dst_width;
            int src_y = (dst_y * src_height) / dst_height;
            
            if (src_x >= src_width) src_x = src_width - 1;
            if (src_y >= src_height) src_y = src_height - 1;
            
            // RGB565からRGB成分抽出
            uint16_t pixel = rgb565[src_y * src_width + src_x];
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;
            
            // グレースケール変換（輝度計算）
            uint8_t gray = (r * 299 + g * 587 + b * 114) / 1000;
            
            // 二値化（閾値: 128）
            bool pixel_on = gray > 15;  // RGB565での中間値
            
            // フレームバッファに設定
            set_pixel(dst_x, dst_y, pixel_on);
        }
    }
}

// ========================================
// パレット情報表示
// ========================================
void SSD1306Display::show_palette_info(int palette_index, const char* palette_name) 
{
    char info_text[32];
    snprintf(info_text, sizeof(info_text), "Palette: %d", palette_index);
    
    print_line(7, info_text);  // 最下行に表示
    
    if (palette_name) {
        print_line(6, palette_name);  // パレット名も表示
    }
}

// ========================================
// 通信テスト
// ========================================
esp_err_t SSD1306Display::test_communication() 
{
    ESP_LOGI(TAG, "🧪 SSD1306通信テスト開始");
    
    esp_err_t ret = probe_device();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SSD1306通信テスト成功");
        
        // テストパターン表示
        show_test_pattern(1);
        
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "❌ SSD1306通信テスト失敗");
        return ret;
    }
}

// ========================================
// テストパターン表示
// ========================================
esp_err_t SSD1306Display::show_test_pattern(int pattern) 
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    clear();
    
    switch (pattern) {
        case 0: // チェッカーボード
            for (int y = 0; y < SSD1306_HEIGHT; y++) {
                for (int x = 0; x < SSD1306_WIDTH; x++) {
                    bool pixel_on = ((x / 8) + (y / 8)) % 2;
                    set_pixel(x, y, pixel_on);
                }
            }
            break;
            
        case 1: // 境界線
            for (int x = 0; x < SSD1306_WIDTH; x++) {
                set_pixel(x, 0, true);
                set_pixel(x, SSD1306_HEIGHT - 1, true);
            }
            for (int y = 0; y < SSD1306_HEIGHT; y++) {
                set_pixel(0, y, true);
                set_pixel(SSD1306_WIDTH - 1, y, true);
            }
            draw_string(4, 3, "SSD1306 OK");
            break;
            
        case 2: // グラデーション（縦縞）
            for (int x = 0; x < SSD1306_WIDTH; x++) {
                for (int y = 0; y < SSD1306_HEIGHT; y++) {
                    bool pixel_on = (x % 4) < 2;
                    set_pixel(x, y, pixel_on);
                }
            }
            break;
            
        default: // 全点灯
            for (int y = 0; y < SSD1306_HEIGHT; y++) {
                for (int x = 0; x < SSD1306_WIDTH; x++) {
                    set_pixel(x, y, true);
                }
            }
            break;
    }
    
    return display();
}

// ========================================
// 統計情報表示
// ========================================
void SSD1306Display::print_stats() 
{
    ESP_LOGI(TAG, "\n📊 === SSD1306統計情報 ===");
    ESP_LOGI(TAG, "🔧 初期化状態: %s", _initialized ? "✅ 初期化済み" : "❌ 未初期化");
    ESP_LOGI(TAG, "📡 I2Cポート: %d, アドレス: 0x%02X", _i2c_port, _i2c_address);
    ESP_LOGI(TAG, "📈 画面更新回数: %lu", (unsigned long)_update_count);
    ESP_LOGI(TAG, "⚠️ I2C通信エラー: %lu", (unsigned long)_i2c_error_count);
    ESP_LOGI(TAG, "💾 フレームバッファ: %s", _frame_buffer ? "✅ 確保済み" : "❌ 未確保");
}

// ========================================
// コントラスト設定
// ========================================
esp_err_t SSD1306Display::set_contrast(uint8_t contrast) 
{
    esp_err_t ret = write_command(SSD1306_CMD_SET_CONTRAST);
    if (ret == ESP_OK) {
        ret = write_command(contrast);
    }
    
    ESP_LOGD(TAG, "コントラスト設定: %d", contrast);
    return ret;
}

// ========================================
// 表示ON/OFF
// ========================================
esp_err_t SSD1306Display::set_display_on(bool on) 
{
    uint8_t cmd = on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF;
    esp_err_t ret = write_command(cmd);
    
    ESP_LOGD(TAG, "ディスプレイ: %s", on ? "ON" : "OFF");
    return ret;
}