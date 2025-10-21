/**
 * @file PimoroniEncoder.cpp
 * @brief Pimoroni RGB Encoder Breakout (PIM522) 制御クラス実装
 */

#include "PimoroniEncoder.h"

static const char* TAG = "PimoroniEncoder";

PimoroniEncoder::PimoroniEncoder(i2c_port_t i2c_port, uint8_t device_addr)
    : _i2c_port(i2c_port)
    , _device_address(device_addr)
    , _initialized(false)
    , _last_encoder_count(0)
    , _current_value(0)
    , _min_value(0)
    , _max_value(7)
{
    ESP_LOGI(TAG, "PimoroniEncoder コンストラクタ - I2C:%d, アドレス:0x%02X", _i2c_port, _device_address);
}

PimoroniEncoder::~PimoroniEncoder()
{
    ESP_LOGI(TAG, "PimoroniEncoder デストラクタ");
    if (_initialized) {
        clear_led();
    }
}

esp_err_t PimoroniEncoder::init()
{
    ESP_LOGI(TAG, "Pimoroni RGB Encoder 初期化開始");
    
    // デバイス存在確認
    esp_err_t ret = probe_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "デバイス検出失敗: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "デバイス検出成功");
    
    // PWM周期設定 (255 = 8bit)
    ret = write_register(REG_PWM_PERIOD_L, 255);
    if (ret != ESP_OK) return ret;
    
    ret = write_register(REG_PWM_PERIOD_H, 0);
    if (ret != ESP_OK) return ret;
    
    // PWM制御設定 (分周比2)
    ret = write_register(REG_PWM_CONTROL, 2);
    if (ret != ESP_OK) return ret;
    
    // RGB LEDピンをPWMモードに設定
    ret = set_pin_mode(PIN_RED, PIN_MODE_PWM);
    if (ret != ESP_OK) return ret;
    
    ret = set_pin_mode(PIN_GREEN, PIN_MODE_PWM);
    if (ret != ESP_OK) return ret;
    
    ret = set_pin_mode(PIN_BLUE, PIN_MODE_PWM);
    if (ret != ESP_OK) return ret;
    
    // エンコーダー初期値取得
    uint8_t initial_count;
    ret = read_register(REG_ENC_1_COUNT, &initial_count);
    if (ret == ESP_OK) {
        _last_encoder_count = (int8_t)initial_count;
        ESP_LOGI(TAG, "エンコーダー初期値: %d", _last_encoder_count);
    }
    
    // LEDを消灯
    clear_led();
    
    _initialized = true;
    ESP_LOGI(TAG, "Pimoroni RGB Encoder 初期化完了");
    
    return ESP_OK;
}

esp_err_t PimoroniEncoder::probe_device()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t PimoroniEncoder::write_register(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t PimoroniEncoder::read_register(uint8_t reg, uint8_t* value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(PIMORONI_ENCODER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t PimoroniEncoder::set_pin_mode(uint8_t pin, uint8_t mode)
{
    uint8_t reg = REG_MODE_BASE + pin;
    uint8_t current_value;
    
    esp_err_t ret = read_register(reg, &current_value);
    if (ret != ESP_OK) return ret;
    
    uint8_t new_value = (current_value & 0b11111100) | (mode & 0b00000011);
    return write_register(reg, new_value);
}

esp_err_t PimoroniEncoder::set_pwm_value(uint8_t pin, uint16_t value)
{
    uint8_t reg_low = REG_PWM_VALUE_BASE + (pin * 2);
    uint8_t reg_high = reg_low + 1;
    
    esp_err_t ret = write_register(reg_low, value & 0xFF);
    if (ret != ESP_OK) return ret;
    
    return write_register(reg_high, (value >> 8) & 0xFF);
}

int16_t PimoroniEncoder::update()
{
    ESP_LOGD(TAG, "エンコーダー値更新: %d", _current_value);
    return _current_value;
}

void PimoroniEncoder::set_value_range(int16_t min_val, int16_t max_val)
{
    ESP_LOGI(TAG, "エンコーダー範囲設定: %d - %d", min_val, max_val);
    _min_value = min_val;
    _max_value = max_val;
}

int16_t PimoroniEncoder::get_value() const
{
    return _current_value;
}

void PimoroniEncoder::set_value(int16_t value)
{
    ESP_LOGI(TAG, "エンコーダー値設定: %d", value);
    _current_value = value;
}

esp_err_t PimoroniEncoder::set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!_initialized) {
        ESP_LOGW(TAG, "未初期化のため色設定をスキップ");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "RGB色設定: R=%d, G=%d, B=%d", r, g, b);
    
    // Common Anodeなので値を反転 (255-value)
    uint16_t red_pwm = 65535 - (r * 257);
    uint16_t green_pwm = 65535 - (g * 257);
    uint16_t blue_pwm = 65535 - (b * 257);
    
    esp_err_t ret = set_pwm_value(PIN_RED, red_pwm);
    if (ret != ESP_OK) return ret;
    
    ret = set_pwm_value(PIN_GREEN, green_pwm);
    if (ret != ESP_OK) return ret;
    
    return set_pwm_value(PIN_BLUE, blue_pwm);
}

esp_err_t PimoroniEncoder::set_color(uint32_t color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    ESP_LOGI(TAG, "24bit色設定: 0x%06lX", (unsigned long)color);
    return set_rgb_color(r, g, b);
}

esp_err_t PimoroniEncoder::clear_led()
{
    ESP_LOGI(TAG, "LED消灯");
    return set_rgb_color(0, 0, 0);
}

bool PimoroniEncoder::is_initialized() const
{
    return _initialized;
}

uint8_t PimoroniEncoder::get_device_address() const
{
    return _device_address;
}