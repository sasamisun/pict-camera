/**
 * @file terminal.cpp
 * @brief ターミナル風テキスト表示クラス実装
 */

#include "terminal.h"
#include "esp_log.h"

static const char* TAG = "Terminal";

Terminal::Terminal(uint8_t cols, uint8_t rows)
    : char_buffer(nullptr)
    , cols(cols)
    , rows(rows)
    , cursor_row(0)
    , cursor_col(0)
    , display_x(0)
    , display_y(0)
    , invert(false)
    , show_border(false)
    , auto_wrap(true)
    , auto_scroll(true)
{
    // 動的バッファ確保
    char_buffer = new uint16_t*[rows];
    for (uint8_t i = 0; i < rows; i++) {
        char_buffer[i] = new uint16_t[cols];
    }

    ESP_LOGI(TAG, "ターミナル作成: %d列 × %d行", cols, rows);
    init();
}

Terminal::~Terminal()
{
    // 動的バッファ解放
    if (char_buffer) {
        for (uint8_t i = 0; i < rows; i++) {
            delete[] char_buffer[i];
        }
        delete[] char_buffer;
        char_buffer = nullptr;
    }
    ESP_LOGI(TAG, "ターミナル破棄");
}

void Terminal::init()
{
    // バッファを空白文字で初期化
    for (uint8_t row = 0; row < rows; row++) {
        for (uint8_t col = 0; col < cols; col++) {
            char_buffer[row][col] = TERMINAL_CHAR_EMPTY;
        }
    }

    // カーソルを左上に配置
    cursor_row = 0;
    cursor_col = 0;

    ESP_LOGI(TAG, "ターミナル初期化完了: %d行 × %d列", rows, cols);
}

// ========== セッター ==========

void Terminal::set_position(int16_t x, int16_t y)
{
    display_x = x;
    display_y = y;
}

void Terminal::set_invert(bool inv)
{
    invert = inv;
}

void Terminal::set_border(bool show)
{
    show_border = show;
}

void Terminal::set_auto_wrap(bool wrap)
{
    auto_wrap = wrap;
}

void Terminal::set_auto_scroll(bool scroll)
{
    auto_scroll = scroll;
}

// ========== 文字操作 ==========

void Terminal::set_char(uint8_t row, uint8_t col, uint16_t char_index)
{
    if (row >= rows || col >= cols) {
        ESP_LOGW(TAG, "set_char: 範囲外 (%d, %d)", row, col);
        return;
    }

    char_buffer[row][col] = char_index;
}

uint16_t Terminal::get_char(uint8_t row, uint8_t col) const
{
    if (row >= rows || col >= cols) {
        ESP_LOGW(TAG, "get_char: 範囲外 (%d, %d)", row, col);
        return TERMINAL_CHAR_EMPTY;
    }

    return char_buffer[row][col];
}

// ========== カーソル操作 ==========

void Terminal::set_cursor(uint8_t row, uint8_t col)
{
    if (row >= rows) row = rows - 1;
    if (col >= cols) col = cols - 1;

    cursor_row = row;
    cursor_col = col;
}

void Terminal::get_cursor(uint8_t* row, uint8_t* col) const
{
    if (row) *row = cursor_row;
    if (col) *col = cursor_col;
}

uint8_t Terminal::get_cursor_row() const
{
    return cursor_row;
}

// ========== ターミナル操作 ==========

void Terminal::clear()
{
    for (uint8_t row = 0; row < rows; row++) {
        clear_line(row);
    }

    cursor_row = 0;
    cursor_col = 0;

    ESP_LOGD(TAG, "画面クリア");
}

void Terminal::clear_line(uint8_t row)
{
    if (row >= rows) {
        ESP_LOGW(TAG, "clear_line: 範囲外 (行%d)", row);
        return;
    }

    for (uint8_t col = 0; col < cols; col++) {
        char_buffer[row][col] = TERMINAL_CHAR_EMPTY;
    }
}

void Terminal::print_char(uint16_t char_index)
{
    // 現在のカーソル位置に文字を設定
    char_buffer[cursor_row][cursor_col] = char_index;

    // カーソルを進める
    cursor_col++;

    // 行末を超えた場合（17文字目以降）
    if (cursor_col > cols) {
        if (auto_wrap) {
            // 自動改行
            cursor_col = 0;
            cursor_row++;

            // 最終行を超えた場合
            if (cursor_row >= rows) {
                if (auto_scroll) {
                    // 自動スクロール
                    scroll_up();
                    cursor_row = rows - 1;
                } else {
                    // スクロールしない場合は最終行に留まる
                    cursor_row = rows - 1;
                }
            }
        } else {
            // 自動改行しない場合は行末に留まる
            cursor_col = cols - 1;
        }
    }
}

void Terminal::newline()
{
    cursor_col = 0;
    cursor_row++;

    // 最終行を超えた場合
    if (cursor_row >= rows) {
        if (auto_scroll) {
            scroll_up();
            cursor_row = rows - 1;
        } else {
            cursor_row = rows - 1;
        }
    }
}

void Terminal::scroll_up()
{
    // 全行を1行ずつ上にシフト
    for (uint8_t row = 0; row < rows - 1; row++) {
        for (uint8_t col = 0; col < cols; col++) {
            char_buffer[row][col] = char_buffer[row + 1][col];
        }
    }

    // 最終行をクリア
    clear_line(rows - 1);

    ESP_LOGD(TAG, "スクロールアップ");
}

// ========== バッファアクセス ==========

const uint16_t* Terminal::get_buffer_row(uint8_t row) const
{
    if (row >= rows) {
        ESP_LOGW(TAG, "get_buffer_row: 範囲外 (行%d)", row);
        return nullptr;
    }

    return char_buffer[row];
}
