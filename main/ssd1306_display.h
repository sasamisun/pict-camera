/*
 * SSD1306 OLED ディスプレイ制御クラス ヘッダーファイル
 * ESP-IDF 5.4対応 + ピクセルアートカメラ プレビュー表示用にゃ
 * 
 * 主な機能:
 * - 128x64 SSD1306 OLEDディスプレイ制御
 * - カメラフレームバッファからプレビュー表示
 * - RGB565 → モノクロ変換 + リサイズ処理
 * - I2C共有対応（エンコーダと同じバス使用）
 * 
 * 作成者: にゃんにゃんプログラマー
 */

#ifndef SSD1306_DISPLAY_H
#define SSD1306_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_camera.h"

// ========================================
// SSD1306ディスプレイ仕様にゃ
// ========================================
#define SSD1306_WIDTH           128    // ディスプレイ幅（ピクセル）
#define SSD1306_HEIGHT          64     // ディスプレイ高さ（ピクセル）
#define SSD1306_DEFAULT_ADDR    0x3C   // デフォルトI2Cアドレス（7bit）
#define SSD1306_ALT_ADDR        0x3D   // 代替I2Cアドレス（ADDRピンがHIGHの場合）

// ========================================
// SSD1306ディスプレイ制御クラスにゃ
// ========================================
class SSD1306Display {
public:
    // ========================================
    // コンストラクタ/デストラクタにゃ
    // ========================================
    
    /**
     * @brief SSD1306ディスプレイオブジェクトを作成
     * @param i2c_bus 共有I2Cバスハンドル
     * @param address I2Cアドレス（デフォルト: 0x3C）
     */
    SSD1306Display(i2c_master_bus_handle_t i2c_bus, uint8_t address = SSD1306_DEFAULT_ADDR);
    
    /**
     * @brief デストラクタ（自動的にend()を呼び出し）
     */
    ~SSD1306Display();

    // ========================================
    // 初期化/終了処理にゃ
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
    // 基本描画機能にゃ
    // ========================================
    
    /**
     * @brief フレームバッファをクリア（全ピクセルOFF）
     */
    void clear();
    
    /**
     * @brief 単一ピクセルを描画
     * @param x X座標（0-127）
     * @param y Y座標（0-63）
     * @param color true: ピクセルON, false: ピクセルOFF
     */
    void draw_pixel(int16_t x, int16_t y, bool color);
    
    /**
     * @brief 矩形を描画（塗りつぶし）
     * @param x 開始X座標
     * @param y 開始Y座標
     * @param w 幅
     * @param h 高さ
     * @param color true: 白, false: 黒
     */
    void draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color);
    
    /**
     * @brief テキストを描画（簡易フォント使用）
     * @param x 開始X座標
     * @param y 開始Y座標
     * @param text 表示文字列
     * @param size フォントサイズ（1以上）
     * 
     * 注意: 現在は基本的な英数字のみサポート
     */
    void draw_text(int16_t x, int16_t y, const char* text, uint8_t size = 1);
    
    /**
     * @brief フレームバッファの内容をディスプレイに転送
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t update();

    // ========================================
    // カメラプレビュー専用機能にゃ
    // ========================================
    
    /**
     * @brief カメラフレームバッファをプレビュー表示
     * @param fb カメラフレームバッファ（RGB565フォーマット）
     * @return ESP_OK: 成功, その他: エラーコード
     * 
     * 処理内容:
     * 1. RGB565 → モノクロ変換
     * 2. アスペクト比維持でリサイズ
     * 3. 中央配置で表示
     * 4. ディスプレイ更新
     */
    esp_err_t show_camera_preview(camera_fb_t* fb);

    // ========================================
    // ディスプレイ設定にゃ
    // ========================================
    
    /**
     * @brief コントラスト設定
     * @param contrast コントラスト値（0-255）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_contrast(uint8_t contrast);
    
    /**
     * @brief ディスプレイのON/OFF切り替え
     * @param on true: ON, false: OFF
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_display_on(bool on);
    
    /**
     * @brief 反転表示の設定
     * @param invert true: 反転, false: 通常
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_invert_display(bool invert);

    // ========================================
    // ステータス取得にゃ
    // ========================================
    
    /**
     * @brief 初期化状態を確認
     * @return true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const;
    
    /**
     * @brief I2Cアドレスを取得
     * @return I2Cアドレス（7bit）
     */
    uint8_t get_i2c_address() const;

private:
    // ========================================
    // プライベートメンバ変数にゃ
    // ========================================
    i2c_master_bus_handle_t _i2c_bus_handle;    // 共有I2Cバスハンドル
    i2c_master_dev_handle_t _i2c_dev_handle;    // SSD1306デバイスハンドル
    uint8_t _i2c_address;                       // I2Cアドレス
    bool _initialized;                          // 初期化フラグ
    SemaphoreHandle_t _display_mutex;           // フレームバッファ排他制御用
    uint8_t* _buffer;                           // フレームバッファ（1024バイト）

    // ========================================
    // プライベートメソッドにゃ
    // ========================================
    
    /**
     * @brief I2C経由でコマンドを送信
     * @param cmd 送信コマンド
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t send_command(uint8_t cmd);
    
    /**
     * @brief I2C経由でデータを送信
     * @param data 送信データ
     * @param len データ長
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t send_data(const uint8_t* data, size_t len);
    
    /**
     * @brief SSD1306初期化シーケンスを送信
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t send_init_sequence();
    
    /**
     * @brief 文字描画（8x8フォント）
     * @param x X座標
     * @param y Y座標
     * @param char_index フォント配列のインデックス
     * @param size フォントサイズ
     */
    void draw_char(int16_t x, int16_t y, uint8_t char_index, uint8_t size);
    
    /**
     * @brief RGB565画像をモノクロでリサイズ変換
     * @param fb カメラフレームバッファ
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t convert_and_resize_rgb565(camera_fb_t* fb);
    
    /**
     * @brief ミューテックス取得（タイムアウト付き）
     * @return true: 取得成功, false: タイムアウト
     */
    bool take_mutex();
    
    /**
     * @brief ミューテックス解放
     */
    void give_mutex();
};

#endif // SSD1306_DISPLAY_H