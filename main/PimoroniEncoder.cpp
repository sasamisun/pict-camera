/*
 * PimoroniEncoder.cpp
 * Pimoroni RGB Encoder Breakout 制御実装 (ESP-IDF版)
 */

#include "PimoroniEncoder.h"

static const char* TAG = "PimoroniEncoder";

// ========================================
// コンストラクタ
// ========================================
PimoroniEncoder::PimoroniEncoder(i2c_port_t i2c_port, uint8_t i2c_address) {
    _i2c_port = i2c_port;
    _i2c_address = i2c_address;
    _last_encoder_value = 0;
    _current_value = 0;
    _min_value = 0;
    _max_value = 7;
    _initialized = false;
    _brightness = 0.5;
    _period = 255;
}

// ========================================
// I2Cレジスタ読み取り
// ========================================
esp_err_t PimoroniEncoder::read_register(uint8_t reg, uint8_t *value) {
    esp_err_t ret;
    
    // レジスタアドレスを送信
    ret = i2c_master_write_to_device(_i2c_port, _i2c_address, &reg, 1, 
                                     pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // データを読み取り
    ret = i2c_master_read_from_device(_i2c_port, _i2c_address, value, 1, 
                                      pdMS_TO_TICKS(100));
    return ret;
}

// ========================================
// I2Cレジスタ書き込み
// ========================================
esp_err_t PimoroniEncoder::write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(_i2c_port, _i2c_address, data, 2, 
                                      pdMS_TO_TICKS(100));
}

// ========================================
// ピンモード設定
// ========================================
void PimoroniEncoder::set_pin_mode(uint8_t pin, uint8_t mode) {
    uint8_t reg = 0x04 + pin;  // MODE レジスタ
    uint8_t current = 0;
    
    // 既存の値を読み取って反転ビットを保持
    if (read_register(reg, &current) == ESP_OK) {
        uint8_t new_value = (current & 0b11111100) | (mode & 0b00000011);
        write_register(reg, new_value);
    }
}

// ========================================
// PWM出力設定
// ========================================
void PimoroniEncoder::set_pwm_output(uint8_t pin, uint16_t value) {
    // PWM値を設定（0-65535）
    uint8_t reg_l = 0x18 + (pin * 2);      // PWM下位バイト
    uint8_t reg_h = 0x18 + (pin * 2) + 1;  // PWM上位バイト
    
    write_register(reg_l, value & 0xFF);
    write_register(reg_h, (value >> 8) & 0xFF);
}

// ========================================
// PWMセットアップ
// ========================================
esp_err_t PimoroniEncoder::setup_pwm() {
    esp_err_t ret;
    
    // PWM周期設定
    ret = write_register(REG_PWM_PERIOD_L, _period & 0xFF);
    if (ret != ESP_OK) return ret;
    
    ret = write_register(REG_PWM_PERIOD_H, (_period >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;
    
    // PWM制御設定（分周比2で高速PWM）
    ret = write_register(REG_PWM_CONTROL, 2);
    if (ret != ESP_OK) return ret;
    
    // RGB LEDピンをPWMモードに設定
    set_pin_mode(PIN_RED, PIN_MODE_PWM);
    set_pin_mode(PIN_GREEN, PIN_MODE_PWM);
    set_pin_mode(PIN_BLUE, PIN_MODE_PWM);
    
    // LEDを初期化（消灯）
    set_led(0, 0, 0);
    
    ESP_LOGI(TAG, "PWM setup completed");
    return ESP_OK;
}

// ========================================
// ロータリーエンコーダのセットアップ
// ========================================
esp_err_t PimoroniEncoder::setup_rotary_encoder() {
    esp_err_t ret;
    
    // エンコーダカウントをクリア
    ret = write_register(IOE_REG_ENC_1_COUNT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear encoder count");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Encoder setup completed");
    return ESP_OK;
}

// ========================================
// 初期化
// ========================================
esp_err_t PimoroniEncoder::begin(gpio_num_t sda_pin, gpio_num_t scl_pin) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing encoder...");
    
    // I2C設定
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;  // 400kHz
    
    // I2Cドライバをインストール
    ret = i2c_param_config(_i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(_i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // デバイス存在確認
    uint8_t test_data = 0;
    ret = read_register(IOE_REG_INT, &test_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device not found at I2C address 0x%02X", _i2c_address);
        return ret;
    }
    
    ESP_LOGI(TAG, "Device found at I2C address 0x%02X", _i2c_address);
    
    // エンコーダのセットアップ
    ret = setup_rotary_encoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Encoder setup failed");
        return ret;
    }
    
    // PWMの設定（LED制御用）
    ret = setup_pwm();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM setup failed");
        return ret;
    }
    
    _initialized = true;
    ESP_LOGI(TAG, "Encoder initialization completed");
    
    return ESP_OK;
}

// ========================================
// 値の範囲設定
// ========================================
void PimoroniEncoder::set_value_range(int16_t min_val, int16_t max_val) {
    _min_value = min_val;
    _max_value = max_val;
    
    // 現在値を範囲内に制限
    if (_current_value < _min_value) _current_value = _min_value;
    if (_current_value > _max_value) _current_value = _max_value;
}

// ========================================
// エンコーダ値の更新
// ========================================
int16_t PimoroniEncoder::update() {
    if (!_initialized) return _current_value;
    
    uint8_t raw_count = 0;
    esp_err_t ret = read_register(IOE_REG_ENC_1_COUNT, &raw_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read encoder count");
        return _current_value;
    }
    
    // signed 8bitに変換（-128 to 127）
    int8_t signed_count = (int8_t)raw_count;
    
    // 変化量を計算
    int16_t diff = signed_count - _last_encoder_value;
    
    // オーバーフローを検出して補正
    if (diff > 100) {
        diff -= 256;  // 127→-128へのアンダーフロー
    } else if (diff < -100) {
        diff += 256;  // -128→127へのオーバーフロー
    }
    
    // 変化があった場合のみ更新
    if (diff != 0) {
        _current_value += diff;
        
        // 範囲制限
        if (_current_value < _min_value) _current_value = _min_value;
        if (_current_value > _max_value) _current_value = _max_value;
        
        _last_encoder_value = signed_count;
    }
    
    return _current_value;
}

// ========================================
// 現在の値を取得
// ========================================
int16_t PimoroniEncoder::get_value() {
    return _current_value;
}

// ========================================
// 値を強制設定
// ========================================
void PimoroniEncoder::set_value(int16_t val) {
    _current_value = val;
    
    // 範囲制限
    if (_current_value < _min_value) _current_value = _min_value;
    if (_current_value > _max_value) _current_value = _max_value;
}

// ========================================
// RGB LED制御（0-255）
// ========================================
void PimoroniEncoder::set_led(uint8_t r, uint8_t g, uint8_t b) {
    if (!_initialized) return;
    
    // 輝度を適用（0.0-1.0）
    uint16_t red   = (uint16_t)(r * _brightness);
    uint16_t green = (uint16_t)(g * _brightness);
    uint16_t blue  = (uint16_t)(b * _brightness);
    
    // PWM値に変換（0-65535の範囲で設定）
    // RGB Encoder Breakoutは反転論理（大きい値で暗く）なので反転
    uint16_t red_pwm   = 65535 - (red * 257);    // 255 * 257 = 65535
    uint16_t green_pwm = 65535 - (green * 257);
    uint16_t blue_pwm  = 65535 - (blue * 257);
    
    // PWM出力
    set_pwm_output(PIN_RED, red_pwm);
    set_pwm_output(PIN_GREEN, green_pwm);
    set_pwm_output(PIN_BLUE, blue_pwm);
}

// ========================================
// RGB LED制御（0xRRGGBB形式）
// ========================================
void PimoroniEncoder::set_led_color(uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    set_led(r, g, b);
}

// ========================================
// LED輝度設定
// ========================================
void PimoroniEncoder::set_led_brightness(float brightness) {
    if (brightness < 0.0) brightness = 0.0;
    if (brightness > 1.0) brightness = 1.0;
    _brightness = brightness;
}

// ========================================
// デバッグ情報表示
// ========================================
void PimoroniEncoder::print_debug_info() {
    uint8_t raw_count = 0;
    read_register(IOE_REG_ENC_1_COUNT, &raw_count);
    
    ESP_LOGI(TAG, "=== Encoder Debug Info ===");
    ESP_LOGI(TAG, "I2C Address: 0x%02X", _i2c_address);
    ESP_LOGI(TAG, "Initialized: %s", _initialized ? "Yes" : "No");
    ESP_LOGI(TAG, "Current Value: %d (Range: %d-%d)", _current_value, _min_value, _max_value);
    ESP_LOGI(TAG, "Raw Count: %d", (int8_t)raw_count);
    ESP_LOGI(TAG, "Brightness: %.2f", _brightness);
    ESP_LOGI(TAG, "==========================");
}