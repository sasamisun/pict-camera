/**
 * @file PimoroniEncoder.h
 * @brief Pimoroni RGB Encoder Breakout (PIM522) 制御クラス
 * 
 * ESP-IDF 5.4 レガシーI2Cドライバー対応版
 * I2Cポート番号を引数として受け取り、複数デバイスでI2Cバス共有可能
 */

#ifndef PIMORONI_ENCODER_H
#define PIMORONI_ENCODER_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

// Pimoroni RGB Encoder のデフォルト設定
#define PIMORONI_ENCODER_DEFAULT_ADDR   0x0F    // デフォルトI2Cアドレス
#define PIMORONI_ENCODER_TIMEOUT_MS     1000    // 通信タイムアウト

// レジスタアドレス定義
#define REG_INT                         0x00    // 割り込みレジスタ
#define REG_ENC_1_COUNT                 0x11    // エンコーダカウント値
#define REG_PWM_PERIOD_L                0x12    // PWM周期（下位）
#define REG_PWM_PERIOD_H                0x13    // PWM周期（上位）  
#define REG_PWM_CONTROL                 0x14    // PWM制御
#define REG_MODE_BASE                   0x04    // モード設定ベースアドレス
#define REG_PWM_VALUE_BASE              0x18    // PWM値ベースアドレス

// RGBピン定義（MS51 内部ピン番号）
#define PIN_RED                         1       // 赤LED
#define PIN_GREEN                       7       // 緑LED  
#define PIN_BLUE                        2       // 青LED

// ピンモード定義
#define PIN_MODE_INPUT                  0x00    // 入力モード
#define PIN_MODE_PWM                    0x01    // PWMモード
#define PIN_MODE_OUTPUT                 0x02    // 出力モード

/**
 * @class PimoroniEncoder
 * @brief Pimoroni RGB Encoder Breakout 制御クラス
 */
class PimoroniEncoder {
private:
    i2c_port_t _i2c_port;           // 使用するI2Cポート
    uint8_t _device_address;        // デバイスI2Cアドレス
    bool _initialized;              // 初期化完了フラグ
    int16_t _last_encoder_count;    // 前回のエンコーダカウント値
    int16_t _current_value;         // 現在の値（オーバーフロー対応済み）
    int16_t _min_value;             // 最小値
    int16_t _max_value;             // 最大値
    
    // 内部ヘルパーメソッド
    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t read_register(uint8_t reg, uint8_t* value);
    esp_err_t set_pin_mode(uint8_t pin, uint8_t mode);
    esp_err_t set_pwm_value(uint8_t pin, uint16_t value);

public:
    /**
     * @brief コンストラクタ
     * @param i2c_port 使用するI2Cポート番号（I2C_NUM_0, I2C_NUM_1など）
     * @param device_addr デバイスI2Cアドレス（デフォルト: 0x0F）
     */
    PimoroniEncoder(i2c_port_t i2c_port, uint8_t device_addr = PIMORONI_ENCODER_DEFAULT_ADDR);
    
    /**
     * @brief デストラクタ
     */
    ~PimoroniEncoder();
    
    /**
     * @brief エンコーダーを初期化
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t init();
    
    /**
     * @brief デバイス存在確認
     * @return ESP_OK: デバイス検出, その他: エラーコード
     */
    esp_err_t probe_device();
    
    /**
     * @brief エンコーダー値を更新（定期的に呼び出す）
     * @return 現在のエンコーダー値
     */
    int16_t update();
    
    /**
     * @brief エンコーダー値の範囲を設定
     * @param min_val 最小値
     * @param max_val 最大値
     */
    void set_value_range(int16_t min_val, int16_t max_val);
    
    /**
     * @brief 現在のエンコーダー値を取得
     * @return 現在の値
     */
    int16_t get_value() const;
    
    /**
     * @brief エンコーダー値を設定
     * @param value 設定する値
     */
    void set_value(int16_t value);
    
    /**
     * @brief RGB LEDの色を設定
     * @param r 赤成分 (0-255)
     * @param g 緑成分 (0-255) 
     * @param b 青成分 (0-255)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_rgb_color(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief RGB LEDの色を設定（24bit色指定）
     * @param color 24bit RGB色 (0xRRGGBB)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_color(uint32_t color);
    
    /**
     * @brief LEDを消灯
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t clear_led();
    
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
};

#endif // PIMORONI_ENCODER_H