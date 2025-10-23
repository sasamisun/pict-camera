#include "pimoroni_encoder.h"
#include <string.h>
#include <math.h>

// ===== ログタグ定義 =====
static const char *TAG = "pimoroni_encoder";

// ===== 内部関数プロトタイプ =====
static esp_err_t write_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t value);
static esp_err_t read_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t *value);
static esp_err_t read_register_16bit(pimoroni_encoder_t *encoder, uint8_t reg_low, uint8_t reg_high, uint16_t *value);
static esp_err_t set_pin_mode_pwm(pimoroni_encoder_t *encoder, uint8_t pin, bool invert);
static esp_err_t setup_rotary_encoder_hardware(pimoroni_encoder_t *encoder);
static esp_err_t setup_pwm_system(pimoroni_encoder_t *encoder);
static esp_err_t enable_interrupt_output(pimoroni_encoder_t *encoder);

// ===== I2C通信関数 =====

/**
 * @brief レジスタに1バイト書き込み
 */
static esp_err_t write_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    
    esp_err_t ret = i2c_master_write_to_device(
        encoder->i2c_port,
        encoder->i2c_address,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C書き込みエラー reg=0x%02X, value=0x%02X", reg, value);
    }
    
    return ret;
}

/**
 * @brief レジスタから1バイト読み取り
 */
static esp_err_t read_register(pimoroni_encoder_t *encoder, uint8_t reg, uint8_t *value)
{
    esp_err_t ret = i2c_master_write_read_device(
        encoder->i2c_port,
        encoder->i2c_address,
        &reg,
        1,
        value,
        1,
        pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C読み取りエラー reg=0x%02X", reg);
    }
    
    return ret;
}

/**
 * @brief 16ビットレジスタ読み取り（下位→上位の順）
 */
static esp_err_t read_register_16bit(pimoroni_encoder_t *encoder, uint8_t reg_low, uint8_t reg_high, uint16_t *value)
{
    uint8_t low_byte, high_byte;
    esp_err_t ret;
    
    ret = read_register(encoder, reg_low, &low_byte);
    if (ret != ESP_OK) return ret;
    
    ret = read_register(encoder, reg_high, &high_byte);
    if (ret != ESP_OK) return ret;
    
    *value = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
    return ESP_OK;
}

// ===== ハードウェア設定関数 =====

/**
 * @brief ピンをPWMモードに設定
 */
static esp_err_t set_pin_mode_pwm(pimoroni_encoder_t *encoder, uint8_t pin, bool invert)
{
    esp_err_t ret = ESP_OK;
    uint8_t port = (pin <= 7) ? 0 : 1;  // ピン番号からポート判定
    uint8_t pin_bit = pin % 8;           // ポート内のピン位置
    
    // PWMモード設定用のレジスタアドレス
    uint8_t reg_m1 = (port == 0) ? REG_P0M1 : REG_P1M1;
    uint8_t reg_m2 = (port == 0) ? REG_P0M2 : REG_P1M2;
    uint8_t reg_piocon = (pin <= 3) ? REG_PIOCON0 : REG_PIOCON1;
    
    // 現在のモード設定を読み取り
    uint8_t m1_val, m2_val, piocon_val;
    ret |= read_register(encoder, reg_m1, &m1_val);
    ret |= read_register(encoder, reg_m2, &m2_val);
    ret |= read_register(encoder, reg_piocon, &piocon_val);
    
    if (ret != ESP_OK) return ret;
    
    // PWMモード設定（Push-Pull出力）
    m1_val &= ~(1 << pin_bit);           // M1 = 0
    m2_val |= (1 << pin_bit);            // M2 = 1 (Push-Pull)
    
    // PWMチャンネル有効化
    uint8_t pwm_channel;
    switch(pin) {
        case LED_R_PIN: pwm_channel = 1; break;  // 赤LED = PWM1
        case LED_G_PIN: pwm_channel = 2; break;  // 緑LED = PWM2  
        case LED_B_PIN: pwm_channel = 0; break;  // 青LED = PWM0
        default: return ESP_ERR_INVALID_ARG;
    }
    piocon_val |= (1 << pwm_channel);    // PWMチャンネル有効化
    
    // レジスタに書き戻し
    ret |= write_register(encoder, reg_m1, m1_val);
    ret |= write_register(encoder, reg_m2, m2_val);
    ret |= write_register(encoder, reg_piocon, piocon_val);
    
    // 反転設定（共通カソードLED用）
    if (invert) {
        uint8_t pnp_reg;
        ret |= read_register(encoder, 0x96, &pnp_reg);  // PNPレジスタ
        pnp_reg |= (1 << pwm_channel);
        ret |= write_register(encoder, 0x96, pnp_reg);
    }
    
    return ret;
}

/**
 * @brief ロータリーエンコーダーハードウェア設定
 */
static esp_err_t setup_rotary_encoder_hardware(pimoroni_encoder_t *encoder)
{
    esp_err_t ret = ESP_OK;
    
    // エンコーダーピンの設定値作成
    // A相とB相のピン番号を4ビットずつ配置
    uint8_t encoder_config = ENC_TERM_A | (ENC_TERM_B << 4);
    
    // エンコーダー設定レジスタに書き込み
    ret |= write_register(encoder, REG_ENC_1_CFG, encoder_config);
    
    // エンコーダー有効化（チャンネル1を有効、マイクロステップは無効）
    uint8_t enc_enable = 0x01;  // ビット0: チャンネル1有効, ビット1: マイクロステップ無効
    ret |= write_register(encoder, REG_ENC_EN, enc_enable);
    
    // エンコーダーカウントをゼロリセット
    ret |= write_register(encoder, REG_ENC_1_COUNT, 0x00);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ロータリーエンコーダー初期化完了 (A=%d, B=%d, C=%d)", 
                 ENC_TERM_A, ENC_TERM_B, ENC_TERM_C);
    }
    
    return ret;
}

/**
 * @brief PWMシステム設定
 */
static esp_err_t setup_pwm_system(pimoroni_encoder_t *encoder)
{
    esp_err_t ret = ESP_OK;
    
    // PWM周期計算（明度に応じて）
    uint16_t pwm_period = (uint16_t)(255.0f / encoder->brightness);
    
    // PWM周期設定
    ret |= write_register(encoder, REG_PWMPL, (uint8_t)(pwm_period & 0xFF));
    ret |= write_register(encoder, REG_PWMPH, (uint8_t)(pwm_period >> 8));
    
    // PWM制御設定（分周比=2, エッジアライン, PWM実行）
    ret |= write_register(encoder, REG_PWMCON1, 0x01);  // 分周比=2
    ret |= write_register(encoder, REG_PWMCON0, 0x80);  // PWM実行ビット
    
    // RGB LEDピンをPWMモードに設定
    ret |= set_pin_mode_pwm(encoder, LED_R_PIN, true);   // 赤LED (反転)
    ret |= set_pin_mode_pwm(encoder, LED_G_PIN, true);   // 緑LED (反転)
    ret |= set_pin_mode_pwm(encoder, LED_B_PIN, true);   // 青LED (反転)
    
    // PWM設定をロード
    uint8_t pwmcon0;
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40;  // LOADビット設定
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PWMシステム初期化完了 (周期=%d, 明度=%.2f)", pwm_period, encoder->brightness);
    }
    
    return ret;
}

/**
 * @brief 割り込み出力有効化
 */
static esp_err_t enable_interrupt_output(pimoroni_encoder_t *encoder)
{
    if (encoder->interrupt_pin == GPIO_NUM_NC) {
        return ESP_OK;  // 割り込みピン未使用
    }
    
    esp_err_t ret = ESP_OK;
    
    // ESP32側GPIO設定
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << encoder->interrupt_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;
    
    // IOExpander側割り込み出力有効化
    uint8_t int_config = 0x02;  // OUT_ENビット設定
    ret = write_register(encoder, REG_INT, int_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "割り込み出力有効化完了 (GPIO%d)", encoder->interrupt_pin);
    }
    
    return ret;
}

// ===== 公開API実装 =====

esp_err_t pimoroni_encoder_init(pimoroni_encoder_t *encoder, const pimoroni_encoder_config_t *config)
{
    if (!encoder || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Pimoroni Encoder初期化開始...");
    
    // 構造体初期化
    memset(encoder, 0, sizeof(pimoroni_encoder_t));
    encoder->i2c_port = config->i2c_port;
    encoder->i2c_address = config->i2c_address;
    encoder->interrupt_pin = config->interrupt_pin;
    encoder->direction = config->direction;
    encoder->brightness = config->brightness;
    encoder->encoder_offset = 0;
    encoder->encoder_last = 0;
    
    // 明度値の範囲チェック
    if (encoder->brightness < 0.01f) encoder->brightness = 0.01f;
    if (encoder->brightness > 1.0f) encoder->brightness = 1.0f;
    
    esp_err_t ret = ESP_OK;
    
    // チップID確認（スキップ指定がない場合）
    if (!config->skip_chip_id_check) {
        uint16_t chip_id = pimoroni_encoder_get_chip_id(encoder);
        if (chip_id != PIMORONI_ENCODER_CHIP_ID) {
            ESP_LOGE(TAG, "チップID不正: 0x%04X (期待値: 0x%04X)", chip_id, PIMORONI_ENCODER_CHIP_ID);
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "チップID確認OK: 0x%04X", chip_id);
    }
    
    // ハードウェア初期化
    ret |= setup_rotary_encoder_hardware(encoder);
    ret |= setup_pwm_system(encoder);
    ret |= enable_interrupt_output(encoder);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Pimoroni Encoder初期化完了 (I2C=0x%02X)", encoder->i2c_address);
    } else {
        ESP_LOGE(TAG, "Pimoroni Encoder初期化失敗");
    }
    
    return ret;
}

esp_err_t pimoroni_encoder_deinit(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // エンコーダー無効化
    write_register(encoder, REG_ENC_EN, 0x00);
    
    // PWM停止
    write_register(encoder, REG_PWMCON0, 0x00);
    
    // LED消灯
    pimoroni_encoder_set_led(encoder, 0, 0, 0);
    
    ESP_LOGI(TAG, "Pimoroni Encoder破棄完了");
    return ESP_OK;
}

int16_t pimoroni_encoder_read(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        ESP_LOGE(TAG, "無効なエンコーダーポインタ");
        return 0;
    }
    
    uint8_t raw_count;
    esp_err_t ret = read_register(encoder, REG_ENC_1_COUNT, &raw_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "エンコーダー読み取りエラー");
        return encoder->encoder_offset + encoder->encoder_last;
    }
    
    // 符号付き8ビット値に変換
    int16_t current_count = (int16_t)((int8_t)raw_count);
    
    // オーバーフロー/アンダーフロー検出・補正
    int16_t last = encoder->encoder_last;
    if (last > 64 && current_count < -64) {
        encoder->encoder_offset += 256;  // 正方向オーバーフロー
    } else if (last < -64 && current_count > 64) {
        encoder->encoder_offset -= 256;  // 負方向アンダーフロー
    }
    
    encoder->encoder_last = current_count;
    
    // 最終カウント値計算
    int16_t total_count = encoder->encoder_offset + current_count;
    
    // 回転方向を考慮
    if (encoder->direction == PIMORONI_ENCODER_CCW) {
        total_count = -total_count;
    }
    
    return total_count;
}

esp_err_t pimoroni_encoder_clear(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    encoder->encoder_offset = 0;
    encoder->encoder_last = 0;
    
    esp_err_t ret = write_register(encoder, REG_ENC_1_COUNT, 0x00);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "エンコーダーカウントクリア完了");
    }
    
    return ret;
}

esp_err_t pimoroni_encoder_set_direction(pimoroni_encoder_t *encoder, pimoroni_encoder_direction_t direction)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    encoder->direction = direction;
    ESP_LOGD(TAG, "エンコーダー方向設定: %s", 
             (direction == PIMORONI_ENCODER_CW) ? "時計回り" : "反時計回り");
    
    return ESP_OK;
}

pimoroni_encoder_direction_t pimoroni_encoder_get_direction(const pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return PIMORONI_ENCODER_CW;
    }
    
    return encoder->direction;
}

esp_err_t pimoroni_encoder_set_led(pimoroni_encoder_t *encoder, uint8_t r, uint8_t g, uint8_t b)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    // RGB値をPWM値にスケーリング（明度考慮）
    uint16_t r_pwm = (uint16_t)(r * encoder->brightness);
    uint16_t g_pwm = (uint16_t)(g * encoder->brightness);
    uint16_t b_pwm = (uint16_t)(b * encoder->brightness);
    
    // 各色のPWM値設定（PWMロードは最後に一度だけ）
    ret |= write_register(encoder, REG_PWM1L, (uint8_t)(r_pwm & 0xFF));     // 赤LED下位
    ret |= write_register(encoder, REG_PWM1H, (uint8_t)(r_pwm >> 8));       // 赤LED上位
    ret |= write_register(encoder, REG_PWM2L, (uint8_t)(g_pwm & 0xFF));     // 緑LED下位
    ret |= write_register(encoder, REG_PWM2H, (uint8_t)(g_pwm >> 8));       // 緑LED上位
    ret |= write_register(encoder, REG_PWM0L, (uint8_t)(b_pwm & 0xFF));     // 青LED下位
    ret |= write_register(encoder, REG_PWM0H, (uint8_t)(b_pwm >> 8));       // 青LED上位
    
    // PWM設定をロード
    uint8_t pwmcon0;
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40;  // LOADビット設定
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "LED色設定完了: R=%d, G=%d, B=%d (明度=%.2f)", r, g, b, encoder->brightness);
    }
    
    return ret;
}

esp_err_t pimoroni_encoder_set_brightness(pimoroni_encoder_t *encoder, float brightness)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 明度値の範囲制限
    if (brightness < 0.01f) brightness = 0.01f;
    if (brightness > 1.0f) brightness = 1.0f;
    
    encoder->brightness = brightness;
    
    // PWM周期を再計算・設定
    uint16_t pwm_period = (uint16_t)(255.0f / brightness);
    esp_err_t ret = ESP_OK;
    
    ret |= write_register(encoder, REG_PWMPL, (uint8_t)(pwm_period & 0xFF));
    ret |= write_register(encoder, REG_PWMPH, (uint8_t)(pwm_period >> 8));
    
    // PWM設定をロード
    uint8_t pwmcon0;
    ret |= read_register(encoder, REG_PWMCON0, &pwmcon0);
    pwmcon0 |= 0x40;  // LOADビット設定
    ret |= write_register(encoder, REG_PWMCON0, pwmcon0);
    
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "明度設定完了: %.2f (PWM周期=%d)", brightness, pwm_period);
    }
    
    return ret;
}

bool pimoroni_encoder_available(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return false;
    }
    
    // 割り込みピンが設定されている場合はGPIOレベルをチェック
    if (encoder->interrupt_pin != GPIO_NUM_NC) {
        return (gpio_get_level(encoder->interrupt_pin) == 0);  // 割り込み時はLOW
    }
    
    // 割り込みピン未使用の場合はレジスタから直接読み取り
    uint8_t int_status;
    esp_err_t ret = read_register(encoder, REG_INT, &int_status);
    if (ret != ESP_OK) {
        return false;
    }
    
    return (int_status & 0x01) != 0;  // TRIGDビットチェック
}

esp_err_t pimoroni_encoder_clear_interrupt(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 割り込みフラグクリア（TRIGDビットをクリア）
    uint8_t int_status;
    esp_err_t ret = read_register(encoder, REG_INT, &int_status);
    if (ret != ESP_OK) return ret;
    
    int_status &= ~0x01;  // TRIGDビットクリア
    ret = write_register(encoder, REG_INT, int_status);
    
    return ret;
}

uint16_t pimoroni_encoder_get_chip_id(pimoroni_encoder_t *encoder)
{
    if (!encoder) {
        return 0;
    }
    
    uint16_t chip_id;
    esp_err_t ret = read_register_16bit(encoder, REG_CHIP_ID_L, REG_CHIP_ID_H, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "チップID読み取りエラー");
        return 0;
    }
    
    return chip_id;
}

pimoroni_encoder_config_t pimoroni_encoder_get_default_config(i2c_port_t i2c_port)
{
    pimoroni_encoder_config_t config = {
        .i2c_port = i2c_port,
        .i2c_address = PIMORONI_ENCODER_I2C_ADDR,
        .interrupt_pin = GPIO_NUM_NC,  // 割り込みピン未使用
        .direction = PIMORONI_ENCODER_CW,
        .brightness = 1.0f,
        .skip_chip_id_check = false
    };
    
    return config;
}