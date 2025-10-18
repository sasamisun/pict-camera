/*
 * PimoroniEncoder.h
 * Pimoroni RGB Encoder Breakout 制御クラス (ESP-IDF版)
 * 
 * 機能:
 * - ロータリーエンコーダ値の読み取り
 * - RGB LED制御
 * - I2C通信（アドレス: 0x0F）
 */

#ifndef PIMORONI_ENCODER_H
#define PIMORONI_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// ========================================
// IO Expanderのレジスタアドレス
// ========================================
#define IOE_REG_INT         0x00  // 割り込み
#define IOE_REG_ENC_1_COUNT 0x11  // エンコーダ1カウント
#define IOE_REG_PWM_BASE    0x12  // PWMベースアドレス

// PWM制御レジスタ
#define REG_PWM_PERIOD_L   0x12
#define REG_PWM_PERIOD_H   0x13
#define REG_PWM_CONTROL    0x14

// ピンモード
#define PIN_MODE_IO    0b00000000
#define PIN_MODE_PWM   0b00000001
#define PIN_MODE_ADC   0b00000010

// ========================================
// ピン定義（RGB Encoder Breakout用）
// ========================================
#define PIN_RED   1   // 赤LED
#define PIN_GREEN 7   // 緑LED  
#define PIN_BLUE  2   // 青LED

#define POT_ENC_A 12  // エンコーダA相
#define POT_ENC_B 3   // エンコーダB相
#define POT_ENC_C 11  // エンコーダC（プルダウン）

// ========================================
// クラス定義
// ========================================
class PimoroniEncoder {
private:
    i2c_port_t _i2c_port;              // I2Cポート番号
    uint8_t _i2c_address;              // I2Cアドレス（デフォルト0x0F）
    int16_t _last_encoder_value;       // 前回のエンコーダ値
    int16_t _current_value;            // 現在の論理値（範囲制限後）
    int16_t _min_value;                // 最小値
    int16_t _max_value;                // 最大値
    bool _initialized;                 // 初期化フラグ
    float _brightness;                 // LED輝度（0.0-1.0）
    uint16_t _period;                  // PWM周期
    
    // 内部メソッド
    esp_err_t read_register(uint8_t reg, uint8_t *value);
    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t setup_pwm();
    esp_err_t setup_rotary_encoder();
    void set_pin_mode(uint8_t pin, uint8_t mode);
    void set_pwm_output(uint8_t pin, uint16_t value);

public:
    // コンストラクタ
    PimoroniEncoder(i2c_port_t i2c_port = I2C_NUM_1, uint8_t i2c_address = 0x0F);
    
    // 初期化
    esp_err_t begin(gpio_num_t sda_pin, gpio_num_t scl_pin);
    
    // 値の範囲設定
    void set_value_range(int16_t min_val, int16_t max_val);
    
    // エンコーダ値の更新と取得
    int16_t update();              // エンコーダ値を更新して返す
    int16_t get_value();           // 現在の値を取得
    void set_value(int16_t val);   // 値を強制設定
    
    // RGB LED制御
    void set_led(uint8_t r, uint8_t g, uint8_t b);      // 0-255
    void set_led_color(uint32_t color);                 // 0xRRGGBB形式
    void set_led_brightness(float brightness);          // 輝度設定（内部保存）
    
    // 状態確認
    bool is_initialized() { return _initialized; }
    
    // デバッグ情報
    void print_debug_info();
};

#endif // PIMORONI_ENCODER_H