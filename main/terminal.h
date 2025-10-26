/**
 * @file terminal.h
 * @brief ターミナル風テキスト表示クラス
 *
 * 美咲フォント（8x8ドット）を使用した16列×8行のターミナル
 */

#ifndef TERMINAL_H
#define TERMINAL_H

#include <stdint.h>
#include <string.h>

// ターミナルサイズ定数
#define TERMINAL_COLS 16
#define TERMINAL_ROWS 8
#define TERMINAL_CHAR_EMPTY 0  // 空白文字のインデックス

/**
 * @class Terminal
 * @brief ターミナル風テキスト表示管理クラス
 */
class Terminal {
private:
    // バッファ（動的サイズ対応）
    uint16_t** char_buffer;
    uint8_t cols;  // 列数
    uint8_t rows;  // 行数

    // カーソル管理
    uint8_t cursor_row;
    uint8_t cursor_col;

    // 表示位置とスタイル
    int16_t display_x;
    int16_t display_y;
    bool invert;
    bool show_border;

    // 動作制御
    bool auto_wrap;
    bool auto_scroll;

public:
    /**
     * @brief コンストラクタ
     * @param cols 列数（デフォルト: TERMINAL_COLS = 16）
     * @param rows 行数（デフォルト: TERMINAL_ROWS = 8）
     */
    Terminal(uint8_t cols = TERMINAL_COLS, uint8_t rows = TERMINAL_ROWS);

    /**
     * @brief デストラクタ
     */
    ~Terminal();

    /**
     * @brief 初期化
     */
    void init();

    // ========== セッター ==========

    /**
     * @brief 表示位置を設定
     * @param x 表示開始X座標
     * @param y 表示開始Y座標
     */
    void set_position(int16_t x, int16_t y);

    /**
     * @brief 白黒反転フラグを設定
     * @param invert true: 反転, false: 通常
     */
    void set_invert(bool invert);

    /**
     * @brief 枠線表示フラグを設定
     * @param show true: 表示, false: 非表示
     */
    void set_border(bool show);

    /**
     * @brief 自動改行フラグを設定
     * @param wrap true: 行末で自動改行, false: 行末で停止
     */
    void set_auto_wrap(bool wrap);

    /**
     * @brief 自動スクロールフラグを設定
     * @param scroll true: バッファ末尾で自動スクロール, false: 末尾で停止
     */
    void set_auto_scroll(bool scroll);

    // ========== ゲッター ==========

    /**
     * @brief 行数を取得
     * @return 行数
     */
    uint8_t get_rows() const { return rows; }

    /**
     * @brief 列数を取得
     * @return 列数
     */
    uint8_t get_cols() const { return cols; }

    /**
     * @brief 表示X座標を取得
     * @return X座標
     */
    int16_t get_display_x() const { return display_x; }

    /**
     * @brief 表示Y座標を取得
     * @return Y座標
     */
    int16_t get_display_y() const { return display_y; }

    /**
     * @brief 白黒反転フラグを取得
     * @return true: 反転, false: 通常
     */
    bool get_invert() const { return invert; }

    /**
     * @brief 枠線表示フラグを取得
     * @return true: 表示, false: 非表示
     */
    bool get_show_border() const { return show_border; }

    // ========== 文字操作 ==========

    /**
     * @brief 指定位置の文字を設定
     * @param row 行番号（0-7）
     * @param col 列番号（0-15）
     * @param char_index 美咲フォント文字インデックス（0-751）
     */
    void set_char(uint8_t row, uint8_t col, uint16_t char_index);

    /**
     * @brief 指定位置の文字を取得
     * @param row 行番号（0-7）
     * @param col 列番号（0-15）
     * @return 文字インデックス
     */
    uint16_t get_char(uint8_t row, uint8_t col) const;

    // ========== カーソル操作 ==========

    /**
     * @brief カーソル位置を設定
     * @param row 行番号（0-7）
     * @param col 列番号（0-15）
     */
    void set_cursor(uint8_t row, uint8_t col);

    /**
     * @brief カーソル位置を取得
     * @param row 行番号を格納するポインタ
     * @param col 列番号を格納するポインタ
     */
    void get_cursor(uint8_t* row, uint8_t* col) const;

    /**
     * @brief カーソル行番号を取得
     * @return 現在のカーソル行（0-7）
     */
    uint8_t get_cursor_row() const;

    // ========== ターミナル操作 ==========

    /**
     * @brief バッファ全体をクリア
     */
    void clear();

    /**
     * @brief 指定行をクリア
     * @param row 行番号（0-7）
     */
    void clear_line(uint8_t row);

    /**
     * @brief カーソル位置に文字を出力してカーソルを進める
     * @param char_index 文字インデックス
     */
    void print_char(uint16_t char_index);

    /**
     * @brief 改行（カーソルを次の行の先頭に移動）
     */
    void newline();

    /**
     * @brief 1行上にスクロール（最上行は消え、最下行に空行が追加される）
     */
    void scroll_up();

    // ========== バッファアクセス ==========

    /**
     * @brief 指定行のバッファを取得（読み取り専用）
     * @param row 行番号（0-7）
     * @return 行バッファへのポインタ
     */
    const uint16_t* get_buffer_row(uint8_t row) const;
};

#endif // TERMINAL_H
