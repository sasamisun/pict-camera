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

// レジスタアドレス定義 (Pimoroni公式ライブラリより)
#define REG_INT                         0xF9    // 割り込みレジスタ
#define REG_ENC_1_COUNT                 0x06    // エンコーダ1カウント値
#define REG_ENC_EN                      0x04    // エンコーダ有効化
#define REG_PWMPL                       0x99    // PWM周期（下位）
#define REG_PWMPH                       0x91    // PWM周期（上位）
#define REG_PWMCON0                     0x98    // PWM制御0
#define REG_PWMCON1                     0x9F    // PWM制御1

// GPIO モードレジスタ (ポート0用 - ピン1, 2, 7はポート0に属する)
#define REG_P0M1                        0x71    // ポート0モード1
#define REG_P0M2                        0x72    // ポート0モード2
#define REG_P1M1                        0x73    // ポート1モード1
#define REG_P1M2                        0x74    // ポート1モード2

// PIOCON レジスタ (PWM有効化用)
#define REG_PIOCON0                     0x9E    // PIOCON0 (PWM0-3制御)
#define REG_PIOCON1                     0xC9    // PIOCON1 (PWM4-5制御)

// PWM値レジスタ (個別定義が必要)
#define REG_PWM0L                       0x9A    // PWM0 下位
#define REG_PWM0H                       0x92    // PWM0 上位
#define REG_PWM1L                       0x9B    // PWM1 下位
#define REG_PWM1H                       0x93    // PWM1 上位
#define REG_PWM2L                       0x9C    // PWM2 下位
#define REG_PWM2H                       0x94    // PWM2 上位
#define REG_PWM3L                       0x9D    // PWM3 下位
#define REG_PWM3H                       0x95    // PWM3 上位
#define REG_PWM4L                       0xCA    // PWM4 下位
#define REG_PWM4H                       0xC7    // PWM4 上位
#define REG_PWM5L                       0xCB    // PWM5 下位
#define REG_PWM5H                       0xC8    // PWM5 上位

// PNPレジスタ (PWM極性反転制御)
#define REG_PNP                         0x96    // PWM極性反転レジスタ

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

    /**
     * @brief LED色テストを実行（全8色を順番に点灯）
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t test_led_colors();

    /**
     * @brief エンコーダー接続確認
     * @return true: 接続OK, false: 接続NG
     */
    bool check_encoder_connection();
};

#endif // PIMORONI_ENCODER_H