/**
 * @file ssd1306_display.cpp
 * @brief SSD1306 OLED ディスプレイ制御クラス実装
 */

#include "ssd1306_display.h"
#include <stdlib.h>

static const char* TAG = "SSD1306Display";

// UTF-8 → 美咲フォントインデックス変換テーブル（記号用）
static const struct {
    uint32_t codepoint;
    uint16_t index;
} symbol_table[] = {
    {0x0020, 0},    // ' ' (スペース)
    {0x3001, 1},    // 、
    {0x3002, 2},    // 。
    {0x002C, 3},    // ,
    {0x002E, 4},    // .
    {0x30FB, 5},    // ・
    {0x003A, 6},    // :
    {0x003B, 7},    // ;
    {0x003F, 8},    // ?
    {0x0021, 9},    // !
    {0x309B, 10},   // ゛
    {0x309C, 11},   // ゜
    {0x0060, 12},   // `
    {0x0027, 13},   // '
    {0x00A8, 14},   // ¨
    {0x005E, 15},   // ^
    {0x00AF, 16},   // ¯
    {0x005F, 17},   // _
    {0x30FD, 18},   // ヽ
    {0x30FE, 19},   // ヾ
    {0x309D, 20},   // ゝ
    {0x309E, 21},   // ゞ
    {0x3003, 22},   // 〃
    {0x4EDD, 23},   // 仝
    {0x3005, 24},   // 々
    {0x3006, 25},   // 〆
    {0x25CB, 26},   // ○
    {0x30FC, 27},   // ー
    {0x2015, 28},   // ―
    {0x002D, 29},   // -
    {0x002F, 30},   // /
    {0x005C, 31},   // '\'
    {0x007E, 32},   // ~
    {0x2016, 33},   // ‖
    {0x007C, 34},   // |
    {0x2026, 35},   // …
    {0x2025, 36},   // ‥
    {0x0028, 41},   // (
    {0x0029, 42},   // )
    {0x3014, 43},   // 〔
    {0x3015, 44},   // 〕
    {0x005B, 45},   // [
    {0x005D, 46},   // ]
    {0x007B, 47},   // {
    {0x007D, 48},   // }
    {0x003C, 49},   // <
    {0x003E, 50},   // >
    {0x300A, 51},   // 《
    {0x300B, 52},   // 》
    {0x300C, 53},   // 「
    {0x300D, 54},   // 」
    {0x300E, 55},   // 『
    {0x300F, 56},   // 』
    {0x3010, 57},   // 【
    {0x3011, 58},   // 】
    {0x002B, 59},   // +
    {0x00B1, 61},   // ±
    {0x00D7, 62},   // ×
    {0x00F7, 63},   // ÷
    {0x003D, 65},   // = (半角)
    {0x2260, 66},   // ≠
    {0x2266, 68},   // ≦
    {0x2267, 69},   // ≧
    {0x0022, 76},   // "
    {0xFFE5, 78},   // ￥
    {0x0024, 79},   // $
    {0x0025, 82},   // %
    {0x0023, 83},   // #
    {0x0026, 84},   // &
    {0x002A, 85},   // *
    {0x0040, 86},   // @
};

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

void SSD1306Display::draw_char(uint16_t char_index, int16_t x, int16_t y, bool white)
{
    ESP_LOGD(TAG, "文字描画: インデックス=%d, 位置=(%d,%d)", char_index, x, y);

    // 範囲チェック
    if (char_index >= MISAKI_TOTAL_CHARS) {
        ESP_LOGW(TAG, "文字インデックスが範囲外: %d (最大: %d)", char_index, MISAKI_TOTAL_CHARS - 1);
        return;
    }

    if (!_frame_buffer) {
        ESP_LOGW(TAG, "フレームバッファが未初期化");
        return;
    }

    // 美咲フォントデータを取得
    const misaki_char_t* font_char = &misaki_font[char_index];

    // 8x8ビットマップを描画
    // 美咲フォントは右から左の列順で格納されている
    for (int col = 0; col < MISAKI_CHAR_WIDTH; col++) {
        uint8_t column_data = font_char->bitmap[col];

        // 各列の8ビット（縦方向）を描画
        for (int row = 0; row < MISAKI_CHAR_HEIGHT; row++) {
            if (column_data & (1 << row)) {
                // ビットが1の場合、指定色でピクセルを描画
                set_pixel_in_buffer(x + col, y + row, white);
            } else {
                // ビットが0の場合、背景色（反転色）でピクセルを描画
                set_pixel_in_buffer(x + col, y + row, !white);
            }
        }
    }
}

void SSD1306Display::draw_bitmap(const uint8_t* bitmap, uint16_t width, uint16_t height,
                                  int16_t x, int16_t y, bool invert)
{
    ESP_LOGD(TAG, "ビットマップ描画: サイズ=%dx%d, 位置=(%d,%d), 反転=%s",
             width, height, x, y, invert ? "ON" : "OFF");

    if (!_frame_buffer || !bitmap) {
        ESP_LOGW(TAG, "フレームバッファまたはビットマップが無効");
        return;
    }

    // 1行あたりのバイト数を計算（8ピクセル = 1バイト）
    uint16_t bytes_per_row = (width + 7) / 8;

    // 各行をループ
    for (uint16_t row = 0; row < height; row++) {
        // 描画先のY座標を計算
        int16_t dest_y = y + row;

        // Y座標のクリッピング
        if (dest_y < 0 || dest_y >= SSD1306_HEIGHT) {
            continue;
        }

        // 各バイト（8ピクセル分）をループ
        for (uint16_t byte_col = 0; byte_col < bytes_per_row; byte_col++) {
            // ビットマップデータのインデックスを計算
            uint16_t bitmap_index = row * bytes_per_row + byte_col;
            uint8_t bitmap_byte = bitmap[bitmap_index];

            // 8ビットを展開してピクセルに変換
            for (uint8_t bit = 0; bit < 8; bit++) {
                // 描画先のX座標を計算
                int16_t dest_x = x + (byte_col * 8) + bit;

                // X座標のクリッピング
                if (dest_x < 0 || dest_x >= SSD1306_WIDTH) {
                    continue;
                }

                // ピクセルの幅を超えないようにチェック
                if ((byte_col * 8 + bit) >= width) {
                    break;
                }

                // ビットを取得（MSBが左端）
                bool pixel_on = (bitmap_byte & (0x80 >> bit)) != 0;

                // 反転フラグを適用
                if (invert) {
                    pixel_on = !pixel_on;
                }

                // ピクセルを描画
                set_pixel_in_buffer(dest_x, dest_y, pixel_on);
            }
        }
    }
}

void SSD1306Display::draw_terminal(const Terminal* terminal)
{
    if (!_frame_buffer || !terminal) {
        ESP_LOGW(TAG, "フレームバッファまたはターミナルが無効");
        return;
    }

    int16_t base_x = terminal->get_display_x();
    int16_t base_y = terminal->get_display_y();
    bool invert = terminal->get_invert();
    bool show_border = terminal->get_show_border();
    uint8_t cols = terminal->get_cols();
    uint8_t rows = terminal->get_rows();

    ESP_LOGD(TAG, "ターミナル描画: 位置=(%d,%d), サイズ=%dx%d, 反転=%s, 枠線=%s",
             base_x, base_y, cols, rows, invert ? "ON" : "OFF", show_border ? "ON" : "OFF");

    // 枠線を描画
    if (show_border) {
        int16_t border_width = cols * MISAKI_CHAR_WIDTH + 2;
        int16_t border_height = rows * MISAKI_CHAR_HEIGHT + 2;
        draw_rect(base_x, base_y, border_width, border_height, !invert, false);

        // 枠線分オフセット
        base_x += 1;
        base_y += 1;
    }

    // 各文字を描画
    for (uint8_t row = 0; row < rows; row++) {
        const uint16_t* row_buffer = terminal->get_buffer_row(row);
        if (!row_buffer) continue;

        for (uint8_t col = 0; col < cols; col++) {
            uint16_t char_index = row_buffer[col];

            // 文字位置を計算
            int16_t char_x = base_x + (col * MISAKI_CHAR_WIDTH);
            int16_t char_y = base_y + (row * MISAKI_CHAR_HEIGHT);

            // 文字を描画
            draw_char(char_index, char_x, char_y, !invert);
        }
    }
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

// ========== UTF-8文字列処理 ==========

uint32_t SSD1306Display::decode_utf8(const char* utf8_str, int* bytes_consumed)
{
    if (!utf8_str || !bytes_consumed) {
        if (bytes_consumed) *bytes_consumed = 0;
        return 0;
    }

    uint8_t byte1 = (uint8_t)utf8_str[0];

    // 1バイト文字 (ASCII: 0x00-0x7F)
    if ((byte1 & 0x80) == 0) {
        *bytes_consumed = 1;
        return byte1;
    }

    // 2バイト文字 (0xC0-0xDF)
    if ((byte1 & 0xE0) == 0xC0) {
        uint8_t byte2 = (uint8_t)utf8_str[1];
        if ((byte2 & 0xC0) == 0x80) {
            *bytes_consumed = 2;
            return ((byte1 & 0x1F) << 6) | (byte2 & 0x3F);
        }
    }

    // 3バイト文字 (0xE0-0xEF)
    if ((byte1 & 0xF0) == 0xE0) {
        uint8_t byte2 = (uint8_t)utf8_str[1];
        uint8_t byte3 = (uint8_t)utf8_str[2];
        if ((byte2 & 0xC0) == 0x80 && (byte3 & 0xC0) == 0x80) {
            *bytes_consumed = 3;
            return ((byte1 & 0x0F) << 12) | ((byte2 & 0x3F) << 6) | (byte3 & 0x3F);
        }
    }

    // 4バイト文字 (0xF0-0xF7)
    if ((byte1 & 0xF8) == 0xF0) {
        uint8_t byte2 = (uint8_t)utf8_str[1];
        uint8_t byte3 = (uint8_t)utf8_str[2];
        uint8_t byte4 = (uint8_t)utf8_str[3];
        if ((byte2 & 0xC0) == 0x80 && (byte3 & 0xC0) == 0x80 && (byte4 & 0xC0) == 0x80) {
            *bytes_consumed = 4;
            return ((byte1 & 0x07) << 18) | ((byte2 & 0x3F) << 12) |
                   ((byte3 & 0x3F) << 6) | (byte4 & 0x3F);
        }
    }

    // 不正なUTF-8シーケンス
    *bytes_consumed = 1;
    return 0xFFFD; // Unicode replacement character
}

uint16_t SSD1306Display::lookup_symbol(uint32_t codepoint)
{
    // 線形探索（テーブルサイズが小さいため）
    for (size_t i = 0; i < sizeof(symbol_table) / sizeof(symbol_table[0]); i++) {
        if (symbol_table[i].codepoint == codepoint) {
            return symbol_table[i].index;
        }
    }
    return 0xFFFF; // 見つからない
}

uint16_t SSD1306Display::utf8_to_font_index(const char* utf8_str, int* bytes_consumed)
{
    // UTF-8デコード
    uint32_t codepoint = decode_utf8(utf8_str, bytes_consumed);

    // 数字 '0'-'9' (U+0030-0039) → インデックス203-212
    if (codepoint >= 0x0030 && codepoint <= 0x0039) {
        return 203 + (codepoint - 0x0030);
    }

    // 大文字 'A'-'Z' (U+0041-005A) → インデックス220-245
    if (codepoint >= 0x0041 && codepoint <= 0x005A) {
        return 220 + (codepoint - 0x0041);
    }

    // 小文字 'a'-'z' (U+0061-007A) → インデックス252-277
    if (codepoint >= 0x0061 && codepoint <= 0x007A) {
        return 252 + (codepoint - 0x0061);
    }

    // ひらがな 'ぁ'-'ん' (U+3041-3093) → インデックス282-364
    if (codepoint >= 0x3041 && codepoint <= 0x3093) {
        return 282 + (codepoint - 0x3041);
    }

    // カタカナ 'ァ'-'ヴ' (U+30A1-30F4) → インデックス376-459
    if (codepoint >= 0x30A1 && codepoint <= 0x30F4) {
        return 376 + (codepoint - 0x30A1);
    }

    // 記号テーブルで検索
    uint16_t idx = lookup_symbol(codepoint);
    if (idx != 0xFFFF) {
        return idx;
    }

    // 未対応文字 → インデックス73 (□記号)
    ESP_LOGD(TAG, "未対応文字: U+%04lX", (unsigned long)codepoint);
    return 115;
}

void SSD1306Display::terminal_print(Terminal* terminal, const char* str)
{
    if (!terminal || !str) {
        ESP_LOGW(TAG, "terminal_print: 無効な引数");
        return;
    }

    const char* ptr = str;
    while (*ptr != '\0') {
        int bytes_consumed = 0;
        uint16_t font_index = utf8_to_font_index(ptr, &bytes_consumed);

        if (bytes_consumed > 0) {
            terminal->print_char(font_index);
            ptr += bytes_consumed;
        } else {
            // 不正なシーケンスをスキップ
            ptr++;
        }
    }
}

void SSD1306Display::terminal_println(Terminal* terminal, const char* str)
{
    terminal_print(terminal, str);
    if (terminal) {
        terminal->newline();
    }
}

void SSD1306Display::terminal_print_at(Terminal* terminal, uint8_t row,
                                        const char* str, uint8_t col_offset)
{
    if (!terminal || !str) {
        ESP_LOGW(TAG, "terminal_print_at: 無効な引数");
        return;
    }

    uint8_t rows = terminal->get_rows();
    uint8_t cols = terminal->get_cols();

    if (row >= rows) {
        ESP_LOGW(TAG, "terminal_print_at: 行番号が範囲外 (%d)", row);
        return;
    }

    uint8_t col = col_offset;
    const char* ptr = str;

    while (*ptr != '\0' && col < cols) {
        int bytes_consumed = 0;
        uint16_t font_index = utf8_to_font_index(ptr, &bytes_consumed);

        if (bytes_consumed > 0) {
            terminal->set_char(row, col, font_index);
            col++;
            ptr += bytes_consumed;
        } else {
            // 不正なシーケンスをスキップ
            ptr++;
        }
    }
}