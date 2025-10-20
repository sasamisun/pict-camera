/*
 * PimoroniEncoder.h
 * Pimoroni RGB Encoder Breakout 制御クラス（レガシーI2Cドライバー対応版）
 * 
 * 変更内容:
 * - ESP-IDF 5.4のレガシーI2Cドライバー（driver/i2c.h）に対応
 * - マルチスレッドセーフ設計
 * - エラーハンドリング強化
 * - デバッグ機能充実
 */

#ifndef PIMORONI_ENCODER_H
#define PIMORONI_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"  // レガシーI2Cドライバー
#include "esp_log.h"
#include "esp_err.h"

// ========================================
// IO Expanderのレジスタアドレス
// ========================================
#define IOE_REG_INT         0x00  // 割り込みレジスタ
#define IOE_REG_ENC_1_COUNT 0x11  // エンコーダ1カウント
#define IOE_REG_PWM_BASE    0x12  // PWMベースアドレス

// PWM制御レジスタ
#define REG_PWM_PERIOD_L   0x12   // PWM周期下位バイト
#define REG_PWM_PERIOD_H   0x13   // PWM周期上位バイト
#define REG_PWM_CONTROL    0x14   // PWM制御

// ピンモード定義
#define PIN_MODE_IO    0b00000000  // GPIO モード
#define PIN_MODE_PWM   0b00000001  // PWM モード
#define PIN_MODE_ADC   0b00000010  // ADC モード

//LED
#define REG_LED_RED     0xCB  // 赤LED制御レジスタ
#define REG_LED_GREEN   0x9B  // 緑LED制御レジスタ  
#define REG_LED_BLUE    0x9C  // 青LED制御レジスタ

// ========================================
// ピン定義（RGB Encoder Breakout用）
// ========================================
#define PIN_RED   1   // 赤LED (Pin 1)
#define PIN_GREEN 7   // 緑LED (Pin 7)
#define PIN_BLUE  2   // 青LED (Pin 2)

#define POT_ENC_A 12  // エンコーダA相 (Pin 12)
#define POT_ENC_B 3   // エンコーダB相 (Pin 3)
#define POT_ENC_C 11  // エンコーダC（プルダウン、Pin 11）

// I2C通信設定
#define I2C_TIMEOUT_MS     100    // I2Cタイムアウト（ミリ秒）
#define I2C_MAX_RETRIES    3      // I2C通信リトライ回数

// ========================================
// PimoroniEncoderクラス定義
// ========================================
class PimoroniEncoder {
private:
    // I2C関連（レガシードライバー版）
    i2c_port_t _i2c_port;                          // I2Cポート番号（I2C_NUM_0, I2C_NUM_1）
    uint8_t _i2c_address;                          // I2Cアドレス（デフォルト0x0F）
    
    // エンコーダ状態
    int8_t _last_encoder_value;                    // 前回のエンコーダ値（生値）
    int16_t _current_value;                        // 現在の論理値（範囲制限後）
    int16_t _min_value;                            // 最小値
    int16_t _max_value;                            // 最大値
    
    // システム状態
    bool _initialized;                             // 初期化フラグ
    SemaphoreHandle_t _device_mutex;               // デバイスアクセス用ミューテックス
    
    // LED設定
    float _brightness;                             // LED輝度（0.0-1.0）
    uint16_t _period;                             // PWM周期
    uint8_t _current_r, _current_g, _current_b;    // 現在のRGB値
    
    // 統計情報
    uint32_t _i2c_error_count;                     // I2C通信エラー回数
    uint32_t _last_update_time;                    // 最後の更新時刻
    
    // 内部メソッド（レガシードライバー版）
    esp_err_t read_register(uint8_t reg, uint8_t *value, int retries = I2C_MAX_RETRIES);
    esp_err_t write_register(uint8_t reg, uint8_t value, int retries = I2C_MAX_RETRIES);
    esp_err_t setup_pwm();
    esp_err_t setup_rotary_encoder();
    void set_pin_mode(uint8_t pin, uint8_t mode);
    void set_pwm_output(uint8_t pin, uint16_t value);

public:
    // ========================================
    // コンストラクタ（レガシードライバー版）
    // ========================================
    /**
     * コンストラクタ
     * @param i2c_port I2Cポート番号（I2C_NUM_0, I2C_NUM_1など）
     * @param address I2Cアドレス（デフォルト: 0x0F）
     */
    PimoroniEncoder(i2c_port_t i2c_port, uint8_t address = 0x0F);
    
    // ========================================
    // デストラクタ
    // ========================================
    ~PimoroniEncoder();

    // ========================================
    // デバイス制御
    // ========================================
    /**
     * デバイスの初期化
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t begin();
    
    /**
     * デバイス通信テスト
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t test_communication();
    
    /**
     * デバイスのリセット
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t reset();

    // ========================================
    // エンコーダー制御
    // ========================================
    /**
     * エンコーダー値の取得
     * @return 現在のエンコーダー値
     */
    int16_t get_value();
    
    /**
     * エンコーダー値の設定
     * @param value 設定する値
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_value(int16_t value);
    
    /**
     * エンコーダー値の範囲設定
     * @param min_val 最小値
     * @param max_val 最大値
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_value_range(int16_t min_val, int16_t max_val);

    // ========================================
    // LED制御
    // ========================================
    /**
     * LED色の設定（RGB個別指定）
     * @param r 赤成分（0-255）
     * @param g 緑成分（0-255）
     * @param b 青成分（0-255）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_led_color(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * LED色の設定（32ビット色定数）
     * @param color 32ビット色定数（0xRRGGBB形式）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_led_color(uint32_t color);
    
    /**
     * LED色の設定（別名）
     * @param r 赤成分（0-255）
     * @param g 緑成分（0-255）
     * @param b 青成分（0-255）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_led(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * LED輝度の設定
     * @param brightness 輝度（0.0-1.0）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_led_brightness(float brightness);
    
    /**
     * LEDを消灯
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t clear_led();
    
    /**
     * LEDを消灯（別名）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t led_off();

    // ========================================
    // ステータス・デバッグ
    // ========================================
    /**
     * デバイス情報の表示
     */
    void print_device_info();
    
    /**
     * I2C通信エラー回数の取得
     * @return エラー回数
     */
    uint32_t get_error_count() const { return _i2c_error_count; }
    
    /**
     * 初期化状態の確認
     * @return true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const { return _initialized; }
    
    /**
     * デバイス接続確認
     * @return ESP_OK: 接続OK, その他: エラーコード
     */
    esp_err_t probe_device();
};

#endif // PIMORONI_ENCODER_H