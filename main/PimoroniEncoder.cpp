/*
 * PimoroniEncoder.cpp
 * Pimoroni RGB Encoder Breakout 制御クラス実装（レガシーI2Cドライバー対応版）
 */

#include "PimoroniEncoder.h"
#include "esp_timer.h"  // esp_timer_get_time用に追加

static const char* TAG = "PimoroniEncoder";

// ========================================
// コンストラクタ（レガシードライバー版）
// ========================================
PimoroniEncoder::PimoroniEncoder(i2c_port_t i2c_port, uint8_t address)
    : _i2c_port(i2c_port)
    , _i2c_address(address)
    , _last_encoder_value(0)
    , _current_value(0)
    , _min_value(0)
    , _max_value(255)
    , _initialized(false)
    , _device_mutex(nullptr)
    , _brightness(1.0f)
    , _period(255)
    , _current_r(0)
    , _current_g(0)
    , _current_b(0)
    , _i2c_error_count(0)
    , _last_update_time(0)
{
    ESP_LOGD(TAG, "PimoroniEncoder オブジェクト作成: ポート=%d, アドレス=0x%02X", 
             _i2c_port, _i2c_address);
}

// ========================================
// デストラクタ
// ========================================
PimoroniEncoder::~PimoroniEncoder() 
{
    if (_device_mutex) {
        vSemaphoreDelete(_device_mutex);
        _device_mutex = nullptr;
    }
    ESP_LOGD(TAG, "PimoroniEncoder オブジェクト削除完了");
}

// ========================================
// レジスタ読み取り（レガシードライバー版）
// ========================================
esp_err_t PimoroniEncoder::read_register(uint8_t reg, uint8_t *value, int retries) 
{
    if (!_initialized || !value) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt < retries; attempt++) {
        // コマンドリンク作成
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) {
            _i2c_error_count++;
            return ESP_ERR_NO_MEM;
        }

        // 送信フェーズ（レジスタアドレス指定）
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        
        // 再スタート条件
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_i2c_address << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        // 実行
        ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "レジスタ読み取り成功: 0x%02X = 0x%02X", reg, *value);
            break;
        } else {
            ESP_LOGW(TAG, "レジスタ読み取り失敗 (試行%d/%d): 0x%02X, エラー: %s", 
                     attempt + 1, retries, reg, esp_err_to_name(ret));
            _i2c_error_count++;
            
            if (attempt < retries - 1) {
                vTaskDelay(pdMS_TO_TICKS(5));  // リトライ間隔
            }
        }
    }
    
    return ret;
}

// ========================================
// レジスタ書き込み（レガシードライバー版）
// ========================================
esp_err_t PimoroniEncoder::write_register(uint8_t reg, uint8_t value, int retries) 
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt < retries; attempt++) {
        // コマンドリンク作成
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) {
            _i2c_error_count++;
            return ESP_ERR_NO_MEM;
        }

        // 送信フェーズ
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, value, true);
        i2c_master_stop(cmd);
        
        // 実行
        ret = i2c_master_cmd_begin(_i2c_port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGV(TAG, "レジスタ書き込み成功: 0x%02X = 0x%02X", reg, value);
            break;
        } else {
            ESP_LOGW(TAG, "レジスタ書き込み失敗 (試行%d/%d): 0x%02X = 0x%02X, エラー: %s", 
                     attempt + 1, retries, reg, value, esp_err_to_name(ret));
            _i2c_error_count++;
            
            if (attempt < retries - 1) {
                vTaskDelay(pdMS_TO_TICKS(5));  // リトライ間隔
            }
        }
    }
    
    return ret;
}

// ========================================
// デバイス接続確認（レガシードライバー版）
// ========================================
esp_err_t PimoroniEncoder::probe_device() 
{
    ESP_LOGD(TAG, "デバイス接続確認: 0x%02X", _i2c_address);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ デバイス検出成功: 0x%02X", _i2c_address);
    } else {
        ESP_LOGW(TAG, "❌ デバイス検出失敗: 0x%02X, エラー: %s", _i2c_address, esp_err_to_name(ret));
        _i2c_error_count++;
    }
    
    return ret;
}

// ========================================
// 通信テスト（レガシードライバー版）
// ========================================
esp_err_t PimoroniEncoder::test_communication() 
{
    ESP_LOGI(TAG, "🧪 デバイス通信テスト開始（レガシー版）");
    
    // テスト1: 単純な書き込みテスト
    ESP_LOGI(TAG, "📝 テスト1: 書き込みテスト");
    uint8_t test_reg = 0x11;  // エンコーダカウントレジスタ
    uint8_t test_write_data = 0x00;
    
    esp_err_t ret = write_register(test_reg, test_write_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ 書き込みテスト成功");
    } else {
        ESP_LOGW(TAG, "⚠️ 書き込みテスト失敗: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // テスト2: 読み取りテスト
    ESP_LOGI(TAG, "📖 テスト2: 読み取りテスト");
    uint8_t read_data = 0;
    
    ret = read_register(test_reg, &read_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ 読み取りテスト成功: レジスタ0x%02X = 0x%02X", test_reg, read_data);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "⚠️ 読み取りテスト失敗: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGE(TAG, "❌ 通信テスト失敗");
    return ESP_FAIL;
}

// ========================================
// PWM設定
// ========================================
esp_err_t PimoroniEncoder::setup_pwm() 
{
    ESP_LOGD(TAG, "PWM設定開始");
    
    // PWM周期設定（16ビット値）
    esp_err_t ret = write_register(REG_PWM_PERIOD_L, _period & 0xFF);
    if (ret != ESP_OK) return ret;
    
    ret = write_register(REG_PWM_PERIOD_H, (_period >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;
    
    // PWM制御レジスタ設定
    ret = write_register(REG_PWM_CONTROL, 0x80);  // PWM有効
    if (ret != ESP_OK) return ret;
    
    ESP_LOGD(TAG, "PWM設定完了: 周期=%d", _period);
    return ESP_OK;
}

// ========================================
// ロータリーエンコーダ設定
// ========================================
esp_err_t PimoroniEncoder::setup_rotary_encoder() 
{
    ESP_LOGD(TAG, "ロータリーエンコーダ設定開始");
    
    // エンコーダピンをGPIOモードに設定
    set_pin_mode(POT_ENC_A, PIN_MODE_IO);
    set_pin_mode(POT_ENC_B, PIN_MODE_IO);
    set_pin_mode(POT_ENC_C, PIN_MODE_IO);  // プッシュボタン
    
    // エンコーダカウンタ初期化
    esp_err_t ret = write_register(IOE_REG_ENC_1_COUNT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "エンコーダカウンタ初期化失敗");
        return ret;
    }
    
    ESP_LOGD(TAG, "ロータリーエンコーダ設定完了");
    return ESP_OK;
}

// ========================================
// ピンモード設定
// ========================================
void PimoroniEncoder::set_pin_mode(uint8_t pin, uint8_t mode) 
{
    // ピンモード設定レジスタ（仮想的な実装）
    uint8_t mode_reg = 0x40 + pin;  // ピンモードレジスタのベースアドレス
    write_register(mode_reg, mode);
}

// ========================================
// PWM出力設定
// ========================================
void PimoroniEncoder::set_pwm_output(uint8_t pin, uint16_t value) 
{
    // PWM値設定（仮想的な実装）
    uint8_t pwm_reg_l = 0x80 + (pin * 2);      // PWM下位バイト
    uint8_t pwm_reg_h = 0x80 + (pin * 2) + 1;  // PWM上位バイト
    
    write_register(pwm_reg_l, value & 0xFF);
    write_register(pwm_reg_h, (value >> 8) & 0xFF);
}

// ========================================
// 初期化（レガシードライバー版）
// ========================================
esp_err_t PimoroniEncoder::begin() 
{
    ESP_LOGI(TAG, "🚀 エンコーダ初期化開始（レガシー版）");
    
    // ミューテックス作成
    _device_mutex = xSemaphoreCreateMutex();
    if (!_device_mutex) {
        ESP_LOGE(TAG, "❌ ミューテックス作成失敗");
        return ESP_ERR_NO_MEM;
    }
    
    // デバイス存在確認
    esp_err_t ret = probe_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ デバイス検出失敗");
        vSemaphoreDelete(_device_mutex);
        _device_mutex = nullptr;
        return ret;
    }
    
    _initialized = true;  // probe_device成功後に設定
    
    // 通信テスト
    ret = test_communication();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ 通信テストで問題検出、初期化は続行");
    }
    
    // PWM設定
    ret = setup_pwm();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ PWM設定失敗");
        return ret;
    }
    
    // LEDピンをPWMモードに設定
    set_pin_mode(PIN_RED, PIN_MODE_PWM);
    set_pin_mode(PIN_GREEN, PIN_MODE_PWM);
    set_pin_mode(PIN_BLUE, PIN_MODE_PWM);
    
    // ロータリーエンコーダ設定
    ret = setup_rotary_encoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ エンコーダ設定失敗");
        return ret;
    }
    
    // 初期LED状態（消灯）
    clear_led();
    
    ESP_LOGI(TAG, "✅ エンコーダ初期化完了（レガシー版）");
    return ESP_OK;
}

// ========================================
// エンコーダ値取得
// ========================================
int16_t PimoroniEncoder::get_value() 
{
    if (!_initialized) {
        return _current_value;  // 初期化前は現在値を返す
    }
    
    if (xSemaphoreTake(_device_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "ミューテックス取得タイムアウト");
        return _current_value;
    }
    
    uint8_t raw_value = 0;
    esp_err_t ret = read_register(IOE_REG_ENC_1_COUNT, &raw_value);
    
    if (ret == ESP_OK) {
        int8_t encoder_delta = (int8_t)raw_value - _last_encoder_value;
        _last_encoder_value = (int8_t)raw_value;
        
        if (encoder_delta != 0) {
            _current_value += encoder_delta;
            
            // 範囲制限
            if (_current_value < _min_value) _current_value = _min_value;
            if (_current_value > _max_value) _current_value = _max_value;
            
            _last_update_time = esp_timer_get_time() / 1000;
            ESP_LOGD(TAG, "エンコーダ更新: %d (raw: %d, delta: %d)", 
                     _current_value, raw_value, encoder_delta);
        }
    }
    
    xSemaphoreGive(_device_mutex);
    return _current_value;
}

// ========================================
// エンコーダ値設定
// ========================================
esp_err_t PimoroniEncoder::set_value(int16_t value) 
{
    if (value < _min_value) value = _min_value;
    if (value > _max_value) value = _max_value;
    
    _current_value = value;
    ESP_LOGD(TAG, "エンコーダ値設定: %d", value);
    return ESP_OK;
}

// ========================================
// エンコーダ値範囲設定
// ========================================
esp_err_t PimoroniEncoder::set_value_range(int16_t min_val, int16_t max_val) 
{
    if (min_val > max_val) {
        ESP_LOGE(TAG, "無効な範囲: min=%d > max=%d", min_val, max_val);
        return ESP_ERR_INVALID_ARG;
    }
    
    _min_value = min_val;
    _max_value = max_val;
    
    // 現在値が範囲外なら調整
    if (_current_value < _min_value) _current_value = _min_value;
    if (_current_value > _max_value) _current_value = _max_value;
    
    ESP_LOGI(TAG, "エンコーダ範囲設定: %d〜%d", min_val, max_val);
    return ESP_OK;
}

// ========================================
// LED色設定（RGB個別指定）
// ========================================
esp_err_t PimoroniEncoder::set_led_color(uint8_t r, uint8_t g, uint8_t b) 
{
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 輝度補正適用
    uint16_t red_val = (uint16_t)(r * _brightness);
    uint16_t green_val = (uint16_t)(g * _brightness);
    uint16_t blue_val = (uint16_t)(b * _brightness);
    
    if (xSemaphoreTake(_device_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // PWM値設定
    set_pwm_output(PIN_RED, red_val);
    set_pwm_output(PIN_GREEN, green_val);
    set_pwm_output(PIN_BLUE, blue_val);
    
    _current_r = r;
    _current_g = g;
    _current_b = b;
    
    xSemaphoreGive(_device_mutex);
    
    ESP_LOGD(TAG, "LED色設定: R=%d, G=%d, B=%d (輝度=%.2f)", r, g, b, _brightness);
    return ESP_OK;
}

// ========================================
// LED色設定（32ビット色定数）
// ========================================
esp_err_t PimoroniEncoder::set_led_color(uint32_t color) 
{
    // 32ビット色定数からRGB成分を抽出
    uint8_t r = (color >> 16) & 0xFF;  // 赤成分
    uint8_t g = (color >> 8) & 0xFF;   // 緑成分
    uint8_t b = color & 0xFF;          // 青成分
    
    ESP_LOGD(TAG, "LED色設定（32ビット）: 0x%06lX -> R=%d, G=%d, B=%d", 
             (unsigned long)color, r, g, b);
    
    return set_led_color(r, g, b);
}

// ========================================
// LED色設定（別名）
// ========================================
esp_err_t PimoroniEncoder::set_led(uint8_t r, uint8_t g, uint8_t b) 
{
    return set_led_color(r, g, b);
}

// ========================================
// LED輝度設定
// ========================================
esp_err_t PimoroniEncoder::set_led_brightness(float brightness) 
{
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    
    _brightness = brightness;
    
    // 現在の色で輝度更新
    return set_led_color(_current_r, _current_g, _current_b);
}

// ========================================
// LED消灯
// ========================================
esp_err_t PimoroniEncoder::clear_led() 
{
    return set_led_color(0, 0, 0);
}

// ========================================
// LED消灯（別名）
// ========================================
esp_err_t PimoroniEncoder::led_off() 
{
    return clear_led();
}

// ========================================
// デバイス情報表示
// ========================================
void PimoroniEncoder::print_device_info() 
{
    ESP_LOGI(TAG, "=== PimoroniEncoder デバイス情報 ===");
    ESP_LOGI(TAG, "I2Cポート: %d", _i2c_port);
    ESP_LOGI(TAG, "I2Cアドレス: 0x%02X", _i2c_address);
    ESP_LOGI(TAG, "初期化状態: %s", _initialized ? "初期化済み" : "未初期化");
    ESP_LOGI(TAG, "エンコーダ値: %d (範囲: %d〜%d)", _current_value, _min_value, _max_value);
    ESP_LOGI(TAG, "LED状態: R=%d, G=%d, B=%d, 輝度=%.2f", 
             _current_r, _current_g, _current_b, _brightness);
    ESP_LOGI(TAG, "I2Cエラー回数: %lu", (unsigned long)_i2c_error_count);
    ESP_LOGI(TAG, "最終更新: %lu ms", (unsigned long)_last_update_time);
    ESP_LOGI(TAG, "==============================");
}

// ========================================
// デバイスリセット
// ========================================
esp_err_t PimoroniEncoder::reset() 
{
    ESP_LOGI(TAG, "デバイスリセット開始");
    
    if (!_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // LEDを消灯
    clear_led();
    
    // エンコーダカウンタリセット
    write_register(IOE_REG_ENC_1_COUNT, 0);
    _last_encoder_value = 0;
    
    // 統計情報リセット
    _i2c_error_count = 0;
    _last_update_time = 0;
    
    ESP_LOGI(TAG, "デバイスリセット完了");
    return ESP_OK;
}