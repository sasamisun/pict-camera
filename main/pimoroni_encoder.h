#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Pimoroni RGB Encoder Breakout用ドライバ (ESP-IDF移植版) - 修正版
     *
     * このライブラリはPimoroni RGB Encoder Breakoutボードの制御を行います。
     * ロータリーエンコーダーの読み取りとRGB LEDの制御が可能です。
     *
     * ⚠️ 修正内容：LEDピンとPWMチャンネルの正しいマッピング
     */

    // ===== 定数定義 =====

#define PIMORONI_ENCODER_I2C_ADDR 0x0F   // デフォルトI2Cアドレス
#define PIMORONI_ENCODER_CHIP_ID 0xE26A  // チップ識別ID
#define PIMORONI_ENCODER_TIMEOUT_MS 1000 // タイムアウト値(ms)

// RGB LED制御用の色値範囲
#define LED_MIN_VALUE 0   // LED最小値
#define LED_MAX_VALUE 255 // LED最大値

    // 回転方向の定義
    typedef enum
    {
        PIMORONI_ENCODER_CW = 0, // 時計回り
        PIMORONI_ENCODER_CCW = 1 // 反時計回り
    } pimoroni_encoder_direction_t;

// ===== レジスタ定義 (内部使用) =====

// チップ情報関連
#define REG_CHIP_ID_L 0xFA // チップID下位バイト
#define REG_CHIP_ID_H 0xFB // チップID上位バイト
#define REG_VERSION 0xFC   // バージョン情報

// エンコーダー制御関連
#define REG_ENC_EN 0x04      // エンコーダー有効化レジスタ
#define REG_ENC_1_CFG 0x05   // エンコーダー1設定レジスタ
#define REG_ENC_1_COUNT 0x06 // エンコーダー1カウントレジスタ

// PWM制御関連（RGB LED用）
#define REG_PWMCON0 0x98 // PWM制御レジスタ0
#define REG_PWMCON1 0x9F // PWM制御レジスタ1
#define REG_PWMPL 0x99   // PWM周期下位バイト
#define REG_PWMPH 0x91   // PWM周期上位バイト

// 各PWMチャンネルのレジスタ（正しいマッピング）
#define REG_PWM0L 0x9A // PWM0下位バイト
#define REG_PWM0H 0x92 // PWM0上位バイト
#define REG_PWM1L 0x9B // PWM1下位バイト
#define REG_PWM1H 0x93 // PWM1上位バイト
#define REG_PWM2L 0x9C // PWM2下位バイト
#define REG_PWM2H 0x94 // PWM2上位バイト

// GPIO制御関連
#define REG_P0M1 0x71    // Port0モード1レジスタ
#define REG_P0M2 0x72    // Port0モード2レジスタ
#define REG_P1M1 0x73    // Port1モード1レジスタ
#define REG_P1M2 0x74    // Port1モード2レジスタ
#define REG_PIOCON0 0x9E // PIO制御レジスタ0
#define REG_PIOCON1 0xB1 // PIO制御レジスタ1 (修正: 正しいアドレス)
#define REG_PNP 0x96     // PWM極性レジスタ

// 割り込み制御関連
#define REG_INT 0xF9  // 割り込みレジスタ
#define REG_CTRL 0xFE // 制御レジスタ

    // ===== エンコーダーハードウェア設定 =====

#define ENC_TERM_A 12 // エンコーダーA相ピン
#define ENC_TERM_B 3  // エンコーダーB相ピン
#define ENC_TERM_C 11 // エンコーダーボタンピン
#define ENC_CHANNEL 1 // 使用エンコーダーチャンネル

// ===== RGB LEDピン配置（修正版）=====
// 物理ピン番号（1ベースのピン番号）
#define LED_R_PHYSICAL_PIN 1 // 赤LEDの物理ピン
#define LED_G_PHYSICAL_PIN 7 // 緑LEDの物理ピン
#define LED_B_PHYSICAL_PIN 2 // 青LEDの物理ピン

// 🔧 正しいPWMチャンネル番号（元のコードから）
#define LED_R_PWM_CHANNEL 5 // 赤LED: ピン1 → PWM5
#define LED_G_PWM_CHANNEL 1 // 緑LED: ピン7 → PWM1
#define LED_B_PWM_CHANNEL 2 // 青LED: ピン2 → PWM2

// 🔧 PWM5の拡張レジスタ（Page 1アドレス）
#define REG_PWM4L 0xCA // PWM4下位バイト（Page 1）
#define REG_PWM4H 0xC7 // PWM4上位バイト（Page 1）
#define REG_PWM5L 0xCB // PWM5下位バイト（Page 1）
#define REG_PWM5H 0xC8 // PWM5上位バイト（Page 1）

    // ===== 構造体定義 =====

    /**
     * @brief Pimoroni Encoderデバイス構造体
     */
    typedef struct
    {
        i2c_port_t i2c_port;                    // I2Cポート番号
        uint8_t i2c_address;                    // I2Cアドレス
        gpio_num_t interrupt_pin;               // 割り込みピン（オプション）
        pimoroni_encoder_direction_t direction; // エンコーダー回転方向
        float brightness;                       // LED明度（0.01-1.0）
        int16_t encoder_offset;                 // エンコーダーオフセット値
        int16_t encoder_last;                   // 前回のエンコーダー値
    } pimoroni_encoder_t;

    /**
     * @brief Pimoroni Encoder初期化設定構造体
     */
    typedef struct
    {
        i2c_port_t i2c_port;                    // I2Cポート番号
        uint8_t i2c_address;                    // I2Cアドレス（デフォルト: 0x0F）
        gpio_num_t interrupt_pin;               // 割り込みピン（GPIO_NUM_NC = 未使用）
        pimoroni_encoder_direction_t direction; // エンコーダー回転方向
        float brightness;                       // LED明度（0.01-1.0）
        bool skip_chip_id_check;                // チップID確認をスキップ
    } pimoroni_encoder_config_t;

    // ===== 公開API関数 =====

    /**
     * @brief Pimoroni Encoderの初期化
     *
     * @param encoder エンコーダーデバイス構造体
     * @param config 初期化設定
     * @return ESP_OK: 成功, ESP_FAIL: 失敗
     */
    esp_err_t pimoroni_encoder_init(pimoroni_encoder_t *encoder, const pimoroni_encoder_config_t *config);

    /**
     * @brief デバイスの破棄
     *
     * @param encoder エンコーダーデバイス構造体
     * @return ESP_OK: 成功
     */
    esp_err_t pimoroni_encoder_deinit(pimoroni_encoder_t *encoder);

    /**
     * @brief エンコーダーの回転値を読み取り
     *
     * @param encoder エンコーダーデバイス構造体
     * @return エンコーダーの累積回転値（正: 時計回り, 負: 反時計回り）
     */
    int16_t pimoroni_encoder_read(pimoroni_encoder_t *encoder);

    /**
     * @brief エンコーダーの回転値をクリア（ゼロリセット）
     *
     * @param encoder エンコーダーデバイス構造体
     * @return ESP_OK: 成功, ESP_FAIL: 失敗
     */
    esp_err_t pimoroni_encoder_clear(pimoroni_encoder_t *encoder);

    /**
     * @brief エンコーダーの回転方向を設定
     *
     * @param encoder エンコーダーデバイス構造体
     * @param direction 回転方向
     * @return ESP_OK: 成功
     */
    esp_err_t pimoroni_encoder_set_direction(pimoroni_encoder_t *encoder, pimoroni_encoder_direction_t direction);

    /**
     * @brief エンコーダーの回転方向を取得
     *
     * @param encoder エンコーダーデバイス構造体
     * @return 現在の回転方向
     */
    pimoroni_encoder_direction_t pimoroni_encoder_get_direction(const pimoroni_encoder_t *encoder);

    /**
     * @brief RGB LEDの色を設定
     *
     * @param encoder エンコーダーデバイス構造体
     * @param r 赤色値（0-255）
     * @param g 緑色値（0-255）
     * @param b 青色値（0-255）
     * @return ESP_OK: 成功, ESP_FAIL: 失敗
     */
    esp_err_t pimoroni_encoder_set_led(pimoroni_encoder_t *encoder, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief LED明度を設定
     *
     * @param encoder エンコーダーデバイス構造体
     * @param brightness 明度（0.01-1.0）
     * @return ESP_OK: 成功, ESP_FAIL: 失敗
     */
    esp_err_t pimoroni_encoder_set_brightness(pimoroni_encoder_t *encoder, float brightness);

    /**
     * @brief 割り込みフラグを確認
     *
     * @param encoder エンコーダーデバイス構造体
     * @return true: 割り込み発生中, false: 割り込みなし
     */
    bool pimoroni_encoder_available(pimoroni_encoder_t *encoder);

    /**
     * @brief 割り込みフラグをクリア
     *
     * @param encoder エンコーダーデバイス構造体
     * @return ESP_OK: 成功, ESP_FAIL: 失敗
     */
    esp_err_t pimoroni_encoder_clear_interrupt(pimoroni_encoder_t *encoder);

    /**
     * @brief チップIDを取得
     *
     * @param encoder エンコーダーデバイス構造体
     * @return チップID（0xE26A が正常値）
     */
    uint16_t pimoroni_encoder_get_chip_id(pimoroni_encoder_t *encoder);

    /**
     * @brief デフォルト設定で初期化設定構造体を作成
     *
     * @param i2c_port I2Cポート番号
     * @return デフォルト設定の初期化構造体
     */
    pimoroni_encoder_config_t pimoroni_encoder_get_default_config(i2c_port_t i2c_port);

#ifdef __cplusplus
}
#endif