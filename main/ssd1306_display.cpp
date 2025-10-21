/**
 * @file ssd1306_display.cpp
 * @brief SSD1306 OLED ディスプレイ制御クラス実装
 */

#include "ssd1306_display.h"
#include <stdlib.h>

static const char* TAG = "SSD1306Display";

SSD1306Display::SSD1306Display(i2c_port_t i2c_port, uint8_t device_addr)
    : _i2c_port(i2c_port)
    , _device_address(device_addr)
    , _initialized(false)
    , _frame_buffer(nullptr)
    , _display_on(false)
    , _contrast(127)
{
    ESP_LOGI(TAG, "SSD1306Display コンストラクタ - I2C:%d, アドレス:0x%02X", _i2c_port, _device_address);
}

SSD1306Display::~SSD1306Display()
{
    ESP_LOGI(TAG, "SSD1306Display デストラクタ");
    if (_frame_buffer) {
        free(_frame_buffer);
        _frame_buffer = nullptr;
    }
}

esp_err_t SSD1306Display::init()
{
    ESP_LOGI(TAG, "SSD1306 OLED ディスプレイ初期化開始");
    
    // デバイス存在確認
    esp_err_t ret = probe_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "デバイス検出失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "デバイス検出成功");
    
    // フレームバッファ確保
    _frame_buffer = (uint8_t*)malloc(SSD1306_BUFFER_SIZE);
    if (_frame_buffer == nullptr) {
        ESP_LOGE(TAG, "フレームバッファ確保失敗");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "フレームバッファ確保完了: %d bytes", SSD1306_BUFFER_SIZE);
    
    // SSD1306初期化シーケンス
    const uint8_t init_commands[] = {
        SSD1306_CMD_DISPLAY_OFF,           // ディスプレイOFF
        SSD1306_CMD_SET_CLOCK_DIV, 0x80,   // クロック分周比設定
        SSD1306_CMD_SET_MUX_RATIO, 0x3F,   // MUX比設定 (64-1)
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00, // ディスプレイオフセット
        SSD1306_CMD_SET_START_LINE | 0x00, // 表示開始行
        SSD1306_CMD_CHARGE_PUMP, 0x14,     // チャージポンプON
        SSD1306_CMD_SET_MEMORY_MODE, 0x00, // 水平アドレッシングモード
        SSD1306_CMD_SET_SEGMENT_REMAP | 0x01, // セグメントリマップ
        SSD1306_CMD_SET_COM_SCAN_DIR,      // COM出力スキャン方向
        SSD1306_CMD_SET_COM_PINS, 0x12,    // COM ピン設定
        SSD1306_CMD_SET_CONTRAST, 0x7F,    // コントラスト設定
        SSD1306_CMD_SET_PRECHARGE, 0xF1,   // プリチャージ期間
        SSD1306_CMD_SET_VCOM_DETECT, 0x40, // VCOM検出レベル
        SSD1306_CMD_NORMAL_DISPLAY,        // 通常表示
        SSD1306_CMD_DISPLAY_ON             // ディスプレイON
    };
    
    ret = write_commands(init_commands, sizeof(init_commands));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初期化コマンド送信失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // フレームバッファをクリア
    memset(_frame_buffer, 0, SSD1306_BUFFER_SIZE);
    
    _initialized = true;
    _display_on = true;
    ESP_LOGI(TAG, "SSD1306 OLED ディスプレイ初期化完了");
    
    return ESP_OK;
}

esp_err_t SSD1306Display::probe_device()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(SSD1306_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t SSD1306Display::write_command(uint8_t cmd)
{
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    i2c_master_start(command);
    i2c_master_write_byte(command, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(command, SSD1306_CONTROL_CMD_SINGLE, true);
    i2c_master_write_byte(command, cmd, true);
    i2c_master_stop(command);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, command, pdMS_TO_TICKS(SSD1306_TIMEOUT_MS));
    i2c_cmd_link_delete(command);
    
    return ret;
}

esp_err_t SSD1306Display::write_commands(const uint8_t* cmds, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = write_command(cmds[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t SSD1306Display::write_data(const uint8_t* data, size_t count)
{
    if (count == 0) return ESP_OK;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_CONTROL_DATA_STREAM, true);
    i2c_master_write(cmd, (uint8_t*)data, count, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(SSD1306_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

void SSD1306Display::set_pixel_in_buffer(int16_t x, int16_t y, bool white)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT || !_frame_buffer) {
        return;
    }
    
    int byte_index = x + (y / 8) * SSD1306_WIDTH;
    int bit_index = y % 8;
    
    if (white) {
        _frame_buffer[byte_index] |= (1 << bit_index);
    } else {
        _frame_buffer[byte_index] &= ~(1 << bit_index);
    }
}

void SSD1306Display::clear()
{
    ESP_LOGD(TAG, "画面クリア");
    if (_frame_buffer) {
        memset(_frame_buffer, 0, SSD1306_BUFFER_SIZE);
    }
}

esp_err_t SSD1306Display::display()
{
    ESP_LOGD(TAG, "画面更新");
    if (!_initialized || !_frame_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // アドレス設定コマンド送信
    esp_err_t ret = write_command(SSD1306_CMD_SET_COLUMN_ADDR);
    if (ret != ESP_OK) return ret;
    ret = write_command(0);
    if (ret != ESP_OK) return ret;
    ret = write_command(SSD1306_WIDTH - 1);
    if (ret != ESP_OK) return ret;
    
    ret = write_command(SSD1306_CMD_SET_PAGE_ADDR);
    if (ret != ESP_OK) return ret;
    ret = write_command(0);
    if (ret != ESP_OK) return ret;
    ret = write_command(7);
    if (ret != ESP_OK) return ret;
    
    // フレームバッファを送信
    return write_data(_frame_buffer, SSD1306_BUFFER_SIZE);
}

void SSD1306Display::set_pixel(int16_t x, int16_t y, bool white)
{
    ESP_LOGD(TAG, "ピクセル設定: (%d,%d) = %s", x, y, white ? "白" : "黒");
    set_pixel_in_buffer(x, y, white);
}

void SSD1306Display::draw_string(int16_t x, int16_t y, const char* text, bool white)
{
    ESP_LOGI(TAG, "文字列描画: (%d,%d) \"%s\"", x, y, text);
    // 実装はスタブ - 実際のフォント描画は複雑なので省略
}

void SSD1306Display::draw_hline(int16_t x, int16_t y, int16_t width, bool white)
{
    ESP_LOGD(TAG, "水平線描画: (%d,%d) 幅=%d", x, y, width);
    for (int i = 0; i < width; i++) {
        set_pixel_in_buffer(x + i, y, white);
    }
}

void SSD1306Display::draw_vline(int16_t x, int16_t y, int16_t height, bool white)
{
    ESP_LOGD(TAG, "垂直線描画: (%d,%d) 高さ=%d", x, y, height);
    for (int i = 0; i < height; i++) {
        set_pixel_in_buffer(x, y + i, white);
    }
}

void SSD1306Display::draw_rect(int16_t x, int16_t y, int16_t width, int16_t height, bool white, bool fill)
{
    ESP_LOGI(TAG, "矩形描画: (%d,%d) %dx%d %s", x, y, width, height, fill ? "塗りつぶし" : "枠線");
    if (fill) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                set_pixel_in_buffer(x + i, y + j, white);
            }
        }
    } else {
        draw_hline(x, y, width, white);
        draw_hline(x, y + height - 1, width, white);
        draw_vline(x, y, height, white);
        draw_vline(x + width - 1, y, height, white);
    }
}

esp_err_t SSD1306Display::set_display_power(bool on)
{
    ESP_LOGI(TAG, "ディスプレイ電源: %s", on ? "ON" : "OFF");
    _display_on = on;
    return write_command(on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
}

esp_err_t SSD1306Display::set_contrast(uint8_t contrast)
{
    ESP_LOGI(TAG, "コントラスト設定: %d", contrast);
    _contrast = contrast;
    esp_err_t ret = write_command(SSD1306_CMD_SET_CONTRAST);
    if (ret != ESP_OK) return ret;
    return write_command(contrast);
}

esp_err_t SSD1306Display::set_invert_display(bool invert)
{
    ESP_LOGI(TAG, "表示反転: %s", invert ? "ON" : "OFF");
    return write_command(invert ? SSD1306_CMD_INVERSE_DISPLAY : SSD1306_CMD_NORMAL_DISPLAY);
}

bool SSD1306Display::is_initialized() const
{
    return _initialized;
}

uint8_t SSD1306Display::get_device_address() const
{
    return _device_address;
}

void SSD1306Display::get_screen_size(int* width, int* height) const
{
    if (width) *width = SSD1306_WIDTH;
    if (height) *height = SSD1306_HEIGHT;
}