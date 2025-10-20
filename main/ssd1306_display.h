/*
 * SSD1306 OLED ディスプレイ制御クラス ヘッダーファイル (レガシーI2C統一版)
 * ESP-IDF 5.4対応 + ピクセルアートカメラ プレビュー表示用
 * 
 * 主な機能:
 * - 128x64 SSD1306 OLEDディスプレイ制御
 * - カメラフレームバッファからプレビュー表示
 * - RGB565 → モノクロ変換 + リサイズ処理
 * - レガシーI2Cドライバ使用（PimoroniEncoderと統一）
 * 
 * 変更点:
 * - driver/i2c_master.h → driver/i2c.h に変更
 * - i2c_master_bus_handle_t → i2c_port_t に変更
 * - レガシーI2CAPIに統一（i2c_cmd_link_create等）
 */

#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"  // レガシーI2Cドライバに変更
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_camera.h"
#include "esp_log.h"

// ========================================
// SSD1306ディスプレイ仕様
// ========================================
#define SSD1306_WIDTH           128    // ディスプレイ幅（ピクセル）
#define SSD1306_HEIGHT          64     // ディスプレイ高さ（ピクセル）
#define SSD1306_DEFAULT_ADDR    0x3C   // デフォルトI2Cアドレス（7bit）
#define SSD1306_ALT_ADDR        0x3D   // 代替I2Cアドレス（ADDRピンがHIGHの場合）

// I2C通信設定（レガシードライバ用）
#define SSD1306_I2C_TIMEOUT_MS  100    // I2Cタイムアウト（ミリ秒）
#define SSD1306_I2C_MAX_RETRIES 3      // I2C通信リトライ回数

// フレームバッファサイズ（128x64 ÷ 8ビット/バイト）
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

// SSD1306コマンド定義
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_DISPLAY_CLOCK   0xD5
#define SSD1306_CMD_SET_MULTIPLEX       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_CHARGE_PUMP         0x8D
#define SSD1306_CMD_MEMORY_MODE         0x20
#define SSD1306_CMD_SEG_REMAP           0xA1
#define SSD1306_CMD_COM_SCAN_DEC        0xC8
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOM_DETECT     0xDB
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_DEACTIVATE_SCROLL   0x2E
#define SSD1306_CMD_SET_COLUMN_ADDR     0x21
#define SSD1306_CMD_SET_PAGE_ADDR       0x22

// ========================================
// SSD1306ディスプレイ制御クラス（レガシーI2C版）
// ========================================
class SSD1306Display {
private:
    // I2C関連（レガシードライバー版）
    i2c_port_t _i2c_port;              // I2Cポート番号（I2C_NUM_0, I2C_NUM_1）
    uint8_t _i2c_address;               // I2Cアドレス（デフォルト0x3C）
    
    // ディスプレイ状態
    bool _initialized;                  // 初期化フラグ
    uint8_t* _frame_buffer;             // フレームバッファ（1024バイト）
    SemaphoreHandle_t _display_mutex;   // ディスプレイアクセス用ミューテックス
    
    // 統計情報
    uint32_t _i2c_error_count;          // I2C通信エラー回数
    uint32_t _update_count;             // 画面更新回数
    
    // 内部メソッド（レガシードライバー版）
    esp_err_t write_command(uint8_t cmd, int retries = SSD1306_I2C_MAX_RETRIES);
    esp_err_t write_data(const uint8_t* data, size_t len, int retries = SSD1306_I2C_MAX_RETRIES);
    esp_err_t probe_device();
    esp_err_t send_init_sequence();
    void rgb565_to_mono_resize(const uint8_t* rgb565_data, int src_width, int src_height,
                               uint8_t* mono_buffer, int dst_width, int dst_height);

public:
    // ========================================
    // コンストラクタ/デストラクタ（レガシーI2C版）
    // ========================================
    
    /**
     * @brief SSD1306ディスプレイオブジェクトを作成
     * @param i2c_port I2Cポート番号（I2C_NUM_0, I2C_NUM_1など）
     * @param address I2Cアドレス（デフォルト: 0x3C）
     */
    SSD1306Display(i2c_port_t i2c_port, uint8_t address = SSD1306_DEFAULT_ADDR);
    
    /**
     * @brief デストラクタ（自動的にend()を呼び出し）
     */
    ~SSD1306Display();

    // ========================================
    // 初期化/終了処理
    // ========================================
    
    /**
     * @brief SSD1306ディスプレイを初期化
     * @return ESP_OK: 成功, その他: エラーコード
     * 
     * 実行内容:
     * - I2Cデバイス接続確認
     * - 初期化シーケンス送信
     * - フレームバッファ確保
     * - テスト表示
     */
    esp_err_t begin();
    
    /**
     * @brief ディスプレイを終了し、リソース解放
     */
    void end();

    // ========================================
    // 基本描画機能
    // ========================================
    
    /**
     * @brief フレームバッファをクリア（全ピクセルOFF）
     */
    void clear();
    
    /**
     * @brief 単一ピクセルを描画
     * @param x X座標（0-127）
     * @param y Y座標（0-63）
     * @param color ピクセル色（true=点灯, false=消灯）
     */
    void set_pixel(int x, int y, bool color);
    
    /**
     * @brief フレームバッファの内容をディスプレイに転送
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t display();

    // ========================================
    // テキスト描画機能
    // ========================================
    
    /**
     * @brief 8x8フォントで文字を描画
     * @param x X座標（文字単位: 0-15）
     * @param y Y座標（行単位: 0-7）
     * @param text 描画する文字列
     */
    void draw_string(int x, int y, const char* text);
    
    /**
     * @brief 状態表示用のシンプルなテキスト描画
     * @param line 行番号（0-7）
     * @param text 表示テキスト
     */
    void print_line(int line, const char* text);

    // ========================================
    // カメラプレビュー機能
    // ========================================
    
    /**
     * @brief カメラフレームバッファをプレビュー表示
     * @param fb カメラフレームバッファ
     * @param x 表示位置X座標（0-127）
     * @param y 表示位置Y座標（0-63）
     * @param width プレビュー幅（最大128）
     * @param height プレビュー高さ（最大64）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t show_camera_preview(camera_fb_t* fb, int x = 0, int y = 0, 
                                  int width = 128, int height = 64);
    
    /**
     * @brief パレット選択状態を表示
     * @param palette_index 現在のパレットインデックス（0-7）
     * @param palette_name パレット名（オプション）
     */
    void show_palette_info(int palette_index, const char* palette_name = nullptr);

    // ========================================
    // 状態管理機能
    // ========================================
    
    /**
     * @brief ディスプレイの輝度を設定
     * @param contrast 輝度値（0-255）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_contrast(uint8_t contrast);
    
    /**
     * @brief ディスプレイの表示ON/OFF
     * @param on true: 表示ON, false: 表示OFF
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_display_on(bool on);
    
    /**
     * @brief 通信テスト
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t test_communication();
    
    /**
     * @brief 統計情報を表示
     */
    void print_stats();
    
    /**
     * @brief 初期化状態の確認
     * @return true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const { return _initialized; }

    // ========================================
    // デバッグ機能
    // ========================================
    
    /**
     * @brief テストパターン表示
     * @param pattern パターン番号（0-3）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t show_test_pattern(int pattern);
    
    /**
     * @brief フレームバッファの内容をダンプ表示
     */
    void dump_frame_buffer();
};

#endif // SSD1306_DISPLAY_H