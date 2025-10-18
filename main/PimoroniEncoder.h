/*
 * PimoroniEncoder.h
 * Pimoroni RGB Encoder Breakout 制御クラス (ESP-IDF 5.4対応版)
 * 
 * 新機能:
 * - ESP-IDF 5.4の新しいI2Cマスタードライバー対応にゃ
 * - マルチスレッドセーフ設計にゃ
 * - エラーハンドリング強化にゃ
 * - デバッグ機能充実にゃ
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
#include "driver/i2c_master.h"  // ESP-IDF 5.4の新しいI2Cドライバーにゃ
#include "esp_log.h"
#include "esp_err.h"

// ========================================
// IO Expanderのレジスタアドレスにゃ
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

// ========================================
// ピン定義（RGB Encoder Breakout用）にゃ
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
// PimoroniEncoderクラス定義にゃ
// ========================================
class PimoroniEncoder {
private:
    // I2C関連
    i2c_master_bus_handle_t _i2c_bus_handle;    // I2Cバスハンドル（ESP-IDF 5.4）
    i2c_master_dev_handle_t _i2c_dev_handle;    // I2Cデバイスハンドル
    uint8_t _i2c_address;                       // I2Cアドレス（デフォルト0x0F）
    
    // エンコーダ状態
    int8_t _last_encoder_value;                 // 前回のエンコーダ値（生値）
    int16_t _current_value;                     // 現在の論理値（範囲制限後）
    int16_t _min_value;                         // 最小値
    int16_t _max_value;                         // 最大値
    
    // システム状態
    bool _initialized;                          // 初期化フラグ
    SemaphoreHandle_t _device_mutex;            // デバイスアクセス用ミューテックス
    
    // LED設定
    float _brightness;                          // LED輝度（0.0-1.0）
    uint16_t _period;                          // PWM周期
    uint8_t _current_r, _current_g, _current_b; // 現在のRGB値
    
    // 統計情報
    uint32_t _i2c_error_count;                  // I2C通信エラー回数
    uint32_t _last_update_time;                 // 最後の更新時刻
    
    // 内部メソッドにゃ
    esp_err_t read_register(uint8_t reg, uint8_t *value, int retries = I2C_MAX_RETRIES);
    esp_err_t write_register(uint8_t reg, uint8_t value, int retries = I2C_MAX_RETRIES);
    esp_err_t setup_pwm();
    esp_err_t setup_rotary_encoder();
    void set_pin_mode(uint8_t pin, uint8_t mode);
    void set_pwm_output(uint8_t pin, uint16_t value);
    bool take_mutex(uint32_t timeout_ms = 100);
    void give_mutex();

public:
    // コンストラクタ・デストラクタ
    PimoroniEncoder(i2c_master_bus_handle_t i2c_bus_handle, uint8_t i2c_address = 0x0F);
    virtual ~PimoroniEncoder();
    
    // 初期化・終了処理
    esp_err_t begin();
    void end();
    
    // 値の範囲設定
    void set_value_range(int16_t min_val, int16_t max_val);
    
    // エンコーダ値の更新と取得
    int16_t update();                          // エンコーダ値を更新して返す
    int16_t get_value() const;                 // 現在の値を取得
    void set_value(int16_t val);               // 値を強制設定
    
    // RGB LED制御
    void set_led(uint8_t r, uint8_t g, uint8_t b);      // RGB値指定（0-255）
    void set_led_color(uint32_t color);                 // 0xRRGGBB形式で指定
    void set_led_brightness(float brightness);          // 輝度設定（0.0-1.0）
    void led_off();                                     // LEDを消灯
    void led_pulse(uint32_t color, uint16_t duration_ms); // LED点滅
    
    // 状態確認
    bool is_initialized() const { return _initialized; }
    bool is_device_connected();                         // デバイス接続確認
    
    // 統計・デバッグ情報
    uint32_t get_error_count() const { return _i2c_error_count; }
    uint32_t get_last_update_time() const { return _last_update_time; }
    void print_debug_info();
    void print_register_dump();                         // 全レジスタダンプ
    
    // 高度な機能
    esp_err_t reset_encoder_count();                    // エンコーダカウントリセット
    esp_err_t test_all_leds();                          // LED動作テスト
};

#endif // PIMORONI_ENCODER_H