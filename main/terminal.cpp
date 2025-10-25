/**
 * @file terminal.cpp
 * @brief ターミナル風テキスト表示クラス実装
 */

#include "terminal.h"
#include "esp_log.h"

static const char* TAG = "Terminal";

Terminal::Terminal()
    : cursor_row(0)
    , cursor_col(0)
    , display_x(0)
    , display_y(0)
    , invert(false)
    , show_border(false)
    , auto_wrap(true)
    , auto_scroll(true)
{
    init();
}

Terminal::~Terminal()
{
    // 静的バッファなので特に解放処理は不要
}

void Terminal::init()
{
    // バッファを空白文字で初期化
    for (uint8_t row = 0; row < TERMINAL_ROWS; row++) {
        for (uint8_t col = 0; col < TERMINAL_COLS; col++) {
            char_buffer[row][col] = TERMINAL_CHAR_EMPTY;
        }
    }

    // カーソルを左上に配置
    cursor_row = 0;
    cursor_col = 0;

    ESP_LOGI(TAG, "ターミナル初期化完了: %d行 × %d列", TERMINAL_ROWS, TERMINAL_COLS);
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
    if (row >= TERMINAL_ROWS || col >= TERMINAL_COLS) {
        ESP_LOGW(TAG, "set_char: 範囲外 (%d, %d)", row, col);
        return;
    }

    char_buffer[row][col] = char_index;
}

uint16_t Terminal::get_char(uint8_t row, uint8_t col) const
{
    if (row >= TERMINAL_ROWS || col >= TERMINAL_COLS) {
        ESP_LOGW(TAG, "get_char: 範囲外 (%d, %d)", row, col);
        return TERMINAL_CHAR_EMPTY;
    }

    return char_buffer[row][col];
}

// ========== カーソル操作 ==========

void Terminal::set_cursor(uint8_t row, uint8_t col)
{
    if (row >= TERMINAL_ROWS) row = TERMINAL_ROWS - 1;
    if (col >= TERMINAL_COLS) col = TERMINAL_COLS - 1;

    cursor_row = row;
    cursor_col = col;
}

void Terminal::get_cursor(uint8_t* row, uint8_t* col) const
{
    if (row) *row = cursor_row;
    if (col) *col = cursor_col;
}

// ========== ターミナル操作 ==========

void Terminal::clear()
{
    for (uint8_t row = 0; row < TERMINAL_ROWS; row++) {
        clear_line(row);
    }

    cursor_row = 0;
    cursor_col = 0;

    ESP_LOGD(TAG, "画面クリア");
}

void Terminal::clear_line(uint8_t row)
{
    if (row >= TERMINAL_ROWS) {
        ESP_LOGW(TAG, "clear_line: 範囲外 (行%d)", row);
        return;
    }

    for (uint8_t col = 0; col < TERMINAL_COLS; col++) {
        char_buffer[row][col] = TERMINAL_CHAR_EMPTY;
    }
}

void Terminal::print_char(uint16_t char_index)
{
    // 現在のカーソル位置に文字を設定
    char_buffer[cursor_row][cursor_col] = char_index;

    // カーソルを進める
    cursor_col++;

    // 行末に達した場合
    if (cursor_col >= TERMINAL_COLS) {
        if (auto_wrap) {
            // 自動改行
            cursor_col = 0;
            cursor_row++;

            // 最終行を超えた場合
            if (cursor_row >= TERMINAL_ROWS) {
                if (auto_scroll) {
                    // 自動スクロール
                    scroll_up();
                    cursor_row = TERMINAL_ROWS - 1;
                } else {
                    // スクロールしない場合は最終行に留まる
                    cursor_row = TERMINAL_ROWS - 1;
                }
            }
        } else {
            // 自動改行しない場合は行末に留まる
            cursor_col = TERMINAL_COLS - 1;
        }
    }
}

void Terminal::newline()
{
    cursor_col = 0;
    cursor_row++;

    // 最終行を超えた場合
    if (cursor_row >= TERMINAL_ROWS) {
        if (auto_scroll) {
            scroll_up();
            cursor_row = TERMINAL_ROWS - 1;
        } else {
            cursor_row = TERMINAL_ROWS - 1;
        }
    }
}

void Terminal::scroll_up()
{
    // 全行を1行ずつ上にシフト
    for (uint8_t row = 0; row < TERMINAL_ROWS - 1; row++) {
        for (uint8_t col = 0; col < TERMINAL_COLS; col++) {
            char_buffer[row][col] = char_buffer[row + 1][col];
        }
    }

    // 最終行をクリア
    clear_line(TERMINAL_ROWS - 1);

    ESP_LOGD(TAG, "スクロールアップ");
}

// ========== バッファアクセス ==========

const uint16_t* Terminal::get_buffer_row(uint8_t row) const
{
    if (row >= TERMINAL_ROWS) {
        ESP_LOGW(TAG, "get_buffer_row: 範囲外 (行%d)", row);
        return nullptr;
    }

    return char_buffer[row];
}
