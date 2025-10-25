/**
 * @file ssd1306_display.h
 * @brief SSD1306 OLED ディスプレイ制御クラス
 * 
 * ESP-IDF 5.4 レガシーI2Cドライバー対応版
 * I2Cポート番号を引数として受け取り、複数デバイスでI2Cバス共有可能
 */

#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>
#include "misaki_font.h"
#include "image_logo.h"
#include "terminal.h"

// SSD1306 デフォルト設定
#define SSD1306_DEFAULT_ADDR            0x3C    // デフォルトI2Cアドレス
#define SSD1306_TIMEOUT_MS              1000    // 通信タイムアウト

// ディスプレイ解像度
#define SSD1306_WIDTH                   128     // 横幅
#define SSD1306_HEIGHT                  64      // 縦幅
#define SSD1306_BUFFER_SIZE             ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8)

// SSD1306 コマンド定義
#define SSD1306_CMD_DISPLAY_OFF         0xAE    // ディスプレイOFF
#define SSD1306_CMD_DISPLAY_ON          0xAF    // ディスプレイON
#define SSD1306_CMD_SET_CONTRAST        0x81    // コントラスト設定
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6    // 通常表示
#define SSD1306_CMD_INVERSE_DISPLAY     0xA7    // 反転表示
#define SSD1306_CMD_SET_MEMORY_MODE     0x20    // メモリアドレシングモード
#define SSD1306_CMD_SET_COLUMN_ADDR     0x21    // カラムアドレス設定
#define SSD1306_CMD_SET_PAGE_ADDR       0x22    // ページアドレス設定
#define SSD1306_CMD_SET_START_LINE      0x40    // 表示開始行設定
#define SSD1306_CMD_SET_SEGMENT_REMAP   0xA1    // セグメントリマップ
#define SSD1306_CMD_SET_MUX_RATIO       0xA8    // MUX比設定
#define SSD1306_CMD_SET_COM_SCAN_DIR    0xC8    // COM出力スキャン方向
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3    // ディスプレイオフセット
#define SSD1306_CMD_SET_COM_PINS        0xDA    // COM ピン設定
#define SSD1306_CMD_SET_CLOCK_DIV       0xD5    // クロック分周比設定
#define SSD1306_CMD_SET_PRECHARGE       0xD9    // プリチャージ期間設定
#define SSD1306_CMD_SET_VCOM_DETECT     0xDB    // VCOM検出レベル設定
#define SSD1306_CMD_CHARGE_PUMP         0x8D    // チャージポンプ設定

// 制御バイト定義
#define SSD1306_CONTROL_CMD_SINGLE      0x80    // 単一コマンド
#define SSD1306_CONTROL_CMD_STREAM      0x00    // 連続コマンド
#define SSD1306_CONTROL_DATA_STREAM     0x40    // 連続データ

/**
 * @class SSD1306Display
 * @brief SSD1306 OLED ディスプレイ制御クラス
 */
class SSD1306Display {
private:
    i2c_port_t _i2c_port;           // 使用するI2Cポート
    uint8_t _device_address;        // デバイスI2Cアドレス
    bool _initialized;              // 初期化完了フラグ
    uint8_t* _frame_buffer;         // フレームバッファ
    bool _display_on;               // ディスプレイON/OFF状態
    uint8_t _contrast;              // コントラスト値
    
    // 内部ヘルパーメソッド
    esp_err_t write_command(uint8_t cmd);
    esp_err_t write_commands(const uint8_t* cmds, size_t count);
    esp_err_t write_data(const uint8_t* data, size_t count);
    void set_pixel_in_buffer(int16_t x, int16_t y, bool white);

public:
    /**
     * @brief コンストラクタ
     * @param i2c_port 使用するI2Cポート番号（I2C_NUM_0, I2C_NUM_1など）
     * @param device_addr デバイスI2Cアドレス（デフォルト: 0x3C）
     */
    SSD1306Display(i2c_port_t i2c_port, uint8_t device_addr = SSD1306_DEFAULT_ADDR);
    
    /**
     * @brief デストラクタ
     */
    ~SSD1306Display();
    
    /**
     * @brief ディスプレイを初期化
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t init();
    
    /**
     * @brief デバイス存在確認
     * @return ESP_OK: デバイス検出, その他: エラーコード
     */
    esp_err_t probe_device();
    
    /**
     * @brief 画面をクリア（黒で塗りつぶし）
     */
    void clear();
    
    /**
     * @brief フレームバッファをディスプレイに送信
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t display();
    
    /**
     * @brief ピクセルを設定
     * @param x X座標 (0-127)
     * @param y Y座標 (0-63)
     * @param white true: 白, false: 黒
     */
    void set_pixel(int16_t x, int16_t y, bool white);
    
    /**
     * @brief 美咲フォント文字を描画（8x8ドット）
     * @param char_index 文字配列番号 (0-751)
     * @param x 開始X座標
     * @param y 開始Y座標
     * @param white true: 白文字, false: 黒文字
     */
    void draw_char(uint16_t char_index, int16_t x, int16_t y, bool white = true);

    /**
     * @brief ビットマップ画像を描画（2値画像）
     * @param bitmap 画像データ配列へのポインタ（1バイト=8ピクセル、水平方向ビット詰め）
     * @param width 画像の幅（ピクセル）
     * @param height 画像の高さ（ピクセル）
     * @param x 表示開始X座標
     * @param y 表示開始Y座標
     * @param invert true: 白黒反転, false: 通常表示
     */
    void draw_bitmap(const uint8_t* bitmap, uint16_t width, uint16_t height,
                     int16_t x, int16_t y, bool invert = false);

    /**
     * @brief ターミナルを描画
     * @param terminal ターミナルオブジェクトへのポインタ
     */
    void draw_terminal(const Terminal* terminal);

    /**
     * @brief 文字列を描画（シンプルな6x8フォント）
     * @param x 開始X座標
     * @param y 開始Y座標
     * @param text 描画する文字列
     * @param white true: 白文字, false: 黒文字
     */
    void draw_string(int16_t x, int16_t y, const char* text, bool white = true);
    
    /**
     * @brief 水平線を描画
     * @param x 開始X座標
     * @param y Y座標
     * @param width 幅
     * @param white true: 白線, false: 黒線
     */
    void draw_hline(int16_t x, int16_t y, int16_t width, bool white);
    
    /**
     * @brief 垂直線を描画
     * @param x X座標
     * @param y 開始Y座標
     * @param height 高さ
     * @param white true: 白線, false: 黒線
     */
    void draw_vline(int16_t x, int16_t y, int16_t height, bool white);
    
    /**
     * @brief 矩形を描画
     * @param x 開始X座標
     * @param y 開始Y座標
     * @param width 幅
     * @param height 高さ
     * @param white true: 白, false: 黒
     * @param fill true: 塗りつぶし, false: 枠線のみ
     */
    void draw_rect(int16_t x, int16_t y, int16_t width, int16_t height, bool white, bool fill = false);
    
    /**
     * @brief ディスプレイON/OFF制御
     * @param on true: ON, false: OFF
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_display_power(bool on);
    
    /**
     * @brief コントラスト設定
     * @param contrast コントラスト値 (0-255)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_contrast(uint8_t contrast);
    
    /**
     * @brief 表示反転設定
     * @param invert true: 反転, false: 通常
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_invert_display(bool invert);
    
    /**
     * @brief 初期化状態を取得
     * @return true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const;
    
    /**
     * @brief I2Cアドレスを取得
     * @return デバイスI2Cアドレス
     */
    uint8_t get_device_address() const;
    
    /**
     * @brief 画面サイズを取得
     * @param width 幅を格納するポインタ
     * @param height 高さを格納するポインタ
     */
    void get_screen_size(int* width, int* height) const;
};

#endif // SSD1306_DISPLAY_H