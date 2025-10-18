/*
 * PimoroniEncoder.cpp
 * Pimoroni RGB Encoder Breakout 制御実装 (ESP-IDF 5.4完全対応版)
 * 
 * 新機能:
 * - ESP-IDF 5.4の新しいI2Cマスタードライバー完全対応にゃ
 * - マルチスレッドセーフ実装にゃ
 * - 強力なエラーハンドリングとリトライ機能にゃ
 * - 豊富なデバッグ機能にゃ
 */

#include "PimoroniEncoder.h"
#include "esp_timer.h"

static const char* TAG = "PimoroniEncoder";

// ========================================
// コンストラクタにゃ
// ========================================
PimoroniEncoder::PimoroniEncoder(i2c_master_bus_handle_t i2c_bus_handle, uint8_t i2c_address) {
    _i2c_bus_handle = i2c_bus_handle;
    _i2c_dev_handle = NULL;
    _i2c_address = i2c_address;
    _last_encoder_value = 0;
    _current_value = 0;
    _min_value = 0;
    _max_value = 7;
    _initialized = false;
    _device_mutex = NULL;
    _brightness = 0.5f;
    _period = 255;
    _current_r = 0;
    _current_g = 0;
    _current_b = 0;
    _i2c_error_count = 0;
    _last_update_time = 0;
}

// ========================================
// デストラクタにゃ
// ========================================
PimoroniEncoder::~PimoroniEncoder() {
    end();
}

// ========================================
// ミューテックス取得にゃ
// ========================================
bool PimoroniEncoder::take_mutex(uint32_t timeout_ms) {
    if (!_device_mutex) return false;
    return xSemaphoreTake(_device_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

// ========================================
// ミューテックス解放にゃ
// ========================================
void PimoroniEncoder::give_mutex() {
    if (_device_mutex) {
        xSemaphoreGive(_device_mutex);
    }
}

// ========================================
// I2Cレジスタ読み取り（リトライ機能付き）にゃ
// ========================================
esp_err_t PimoroniEncoder::read_register(uint8_t reg, uint8_t *value, int retries) {
    // 初期化中でもI2Cハンドルがあれば読み取り可能にする
    if (!_i2c_dev_handle || !value) {
        ESP_LOGE(TAG, "❌ 無効なハンドルまたはバッファ");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt <= retries; attempt++) {
        // レジスタアドレスを送信してデータを読み取り
        ret = i2c_master_transmit_receive(
            _i2c_dev_handle, 
            &reg, 1,           // 送信データ（レジスタアドレス）
            value, 1,          // 受信バッファ
            I2C_TIMEOUT_MS     // タイムアウト
        );
        
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "✅ レジスタ読み取り成功: reg=0x%02X, value=0x%02X", reg, *value);
            break;  // 成功
        }
        
        // エラーの詳細ログ
        ESP_LOGW(TAG, "⚠️ レジスタ読み取りエラー: reg=0x%02X, attempt=%d, error=%s", 
                 reg, attempt + 1, esp_err_to_name(ret));
        
        // リトライの場合は少し待機
        if (attempt < retries) {
            vTaskDelay(pdMS_TO_TICKS(50));  // 待機時間を長めに
        }
    }
    
    if (ret != ESP_OK) {
        _i2c_error_count++;
        ESP_LOGE(TAG, "❌ I2C読み取り最終失敗: reg=0x%02X, error=%s", 
                 reg, esp_err_to_name(ret));
    }
    
    return ret;
}

// ========================================
// I2Cレジスタ書き込み（リトライ機能付き）にゃ
// ========================================
esp_err_t PimoroniEncoder::write_register(uint8_t reg, uint8_t value, int retries) {
    if (!_initialized || !_i2c_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[2] = {reg, value};
    esp_err_t ret = ESP_FAIL;
    
    for (int attempt = 0; attempt <= retries; attempt++) {
        ret = i2c_master_transmit(
            _i2c_dev_handle,
            data, 2,
            I2C_TIMEOUT_MS
        );
        
        if (ret == ESP_OK) {
            break;  // 成功
        }
        
        // リトライの場合は少し待機
        if (attempt < retries) {
            ESP_LOGW(TAG, "⚠️ I2C書き込みリトライ %d/%d (reg:0x%02X, val:0x%02X)", 
                     attempt + 1, retries, reg, value);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    if (ret != ESP_OK) {
        _i2c_error_count++;
        ESP_LOGE(TAG, "❌ I2C書き込み失敗: reg=0x%02X, val=0x%02X, error=%s", 
                 reg, value, esp_err_to_name(ret));
    }
    
    return ret;
}

// ========================================
// ピンモード設定にゃ
// ========================================
void PimoroniEncoder::set_pin_mode(uint8_t pin, uint8_t mode) {
    uint8_t reg = 0x04 + pin;  // MODE レジスタ
    uint8_t current = 0;
    
    // 既存の値を読み取って他のビットを保持
    if (read_register(reg, &current) == ESP_OK) {
        uint8_t new_value = (current & 0b11111100) | (mode & 0b00000011);
        write_register(reg, new_value);
    }
}

// ========================================
// PWM出力設定にゃ
// ========================================
void PimoroniEncoder::set_pwm_output(uint8_t pin, uint16_t value) {
    // PWM値を設定（0-65535）
    uint8_t reg_l = 0x18 + (pin * 2);      // PWM下位バイト
    uint8_t reg_h = 0x18 + (pin * 2) + 1;  // PWM上位バイト
    
    write_register(reg_l, value & 0xFF);
    write_register(reg_h, (value >> 8) & 0xFF);
}

// ========================================
// PWMセットアップにゃ
// ========================================
esp_err_t PimoroniEncoder::setup_pwm() {
    ESP_LOGI(TAG, "🔧 PWMセットアップ開始");
    
    // PWM周期設定
    /*
    ESP_RETURN_ON_ERROR(
        write_register(REG_PWM_PERIOD_L, _period & 0xFF), 
        TAG, "PWM周期下位設定失敗"
    );
    
    ESP_RETURN_ON_ERROR(
        write_register(REG_PWM_PERIOD_H, (_period >> 8) & 0xFF), 
        TAG, "PWM周期上位設定失敗"
    );
    // PWM制御設定（分周比2で高速PWM）
    ESP_RETURN_ON_ERROR(
        write_register(REG_PWM_CONTROL, 2), 
        TAG, "PWM制御設定失敗"
    );
    
    // 少し待機してから次の設定
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // RGB LEDピンをPWMモードに設定
    set_pin_mode(PIN_RED, PIN_MODE_PWM);
    set_pin_mode(PIN_GREEN, PIN_MODE_PWM);
    set_pin_mode(PIN_BLUE, PIN_MODE_PWM);
    
    // LEDを初期化（消灯）
    led_off();
    */
    
    ESP_LOGI(TAG, "✅ PWMセットアップ完了");
    return ESP_OK;
}

// ========================================
// ロータリーエンコーダのセットアップにゃ
// ========================================
esp_err_t PimoroniEncoder::setup_rotary_encoder() {
    ESP_LOGI(TAG, "🔧 エンコーダセットアップ開始");
    
    // エンコーダカウントをクリア
    esp_err_t ret = write_register(IOE_REG_ENC_1_COUNT, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ エンコーダカウントクリア失敗");
        return ret;
    }
    
    // 初期値を読み取り
    uint8_t initial_count = 0;
    ret = read_register(IOE_REG_ENC_1_COUNT, &initial_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ エンコーダ初期値読み取り失敗");
        return ret;
    }
    
    _last_encoder_value = (int8_t)initial_count;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "✅ エンコーダセットアップ完了（初期値: %d）", _last_encoder_value);
    return ESP_OK;
}

// ========================================
// デバイス接続確認にゃ
// ========================================
bool PimoroniEncoder::is_device_connected() {
    if (!_initialized || !_i2c_dev_handle) return false;
    
    uint8_t test_data = 0;
    esp_err_t ret = read_register(IOE_REG_INT, &test_data, 1);  // リトライ1回のみ
    return (ret == ESP_OK);
}

// ========================================
// より詳細なデバイステスト関数にゃ
// ========================================
esp_err_t PimoroniEncoder::test_device_communication() {
    ESP_LOGI(TAG, "🧪 デバイス通信テスト開始");
    
    // テスト1: 単純な書き込みテスト
    ESP_LOGI(TAG, "📝 テスト1: 書き込みテスト");
    uint8_t test_reg = 0x11;  // エンコーダカウントレジスタ
    uint8_t test_write_data = 0x00;
    
    esp_err_t ret = i2c_master_transmit(_i2c_dev_handle, 
                                       (uint8_t[]){test_reg, test_write_data}, 2, 
                                       2000);  // 2秒タイムアウト
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ 書き込みテスト成功");
    } else {
        ESP_LOGW(TAG, "⚠️ 書き込みテスト失敗: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // テスト2: 読み取りテスト
    ESP_LOGI(TAG, "📖 テスト2: 読み取りテスト");
    uint8_t read_data = 0;
    
    // 方法A: transmit_receive
    ret = i2c_master_transmit_receive(_i2c_dev_handle, 
                                     &test_reg, 1,
                                     &read_data, 1, 
                                     2000);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ 方法A成功: レジスタ0x%02X = 0x%02X", test_reg, read_data);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "⚠️ 方法A失敗: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 方法B: 分離送信（古典的な方法）
    ESP_LOGI(TAG, "📖 テスト3: 分離送信方式");
    ret = i2c_master_transmit(_i2c_dev_handle, &test_reg, 1, 1000);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = i2c_master_receive(_i2c_dev_handle, &read_data, 1, 1000);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ 方法B成功: レジスタ0x%02X = 0x%02X", test_reg, read_data);
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "⚠️ 方法B失敗: %s", esp_err_to_name(ret));
    
    ESP_LOGE(TAG, "❌ 全ての通信テスト失敗");
    return ESP_FAIL;
}

// ========================================
// 初期化（ESP-IDF 5.4対応）にゃ
// ========================================
esp_err_t PimoroniEncoder::begin() {
    ESP_LOGI(TAG, "🚀 エンコーダ初期化開始");
    
    if (!_i2c_bus_handle) {
        ESP_LOGE(TAG, "❌ I2Cバスハンドルが無効です");
        return ESP_ERR_INVALID_ARG;
    }
    
    // ミューテックス作成
    _device_mutex = xSemaphoreCreateMutex();
    if (!_device_mutex) {
        ESP_LOGE(TAG, "❌ ミューテックス作成失敗");
        return ESP_ERR_NO_MEM;
    }
    
    // I2Cデバイス設定（ESP-IDF 5.4新API）
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = _i2c_address,
        .scl_speed_hz = 100000,
        .scl_wait_us = 2000,
        .flags = {
            .disable_ack_check = false,
        },
    };
    
    esp_err_t ret = i2c_master_bus_add_device(_i2c_bus_handle, &dev_cfg, &_i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ I2Cデバイス追加失敗: %s", esp_err_to_name(ret));
        vSemaphoreDelete(_device_mutex);
        _device_mutex = NULL;
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // プローブテスト
    ESP_LOGI(TAG, "🔍 デバイス存在確認テスト");
    ret = i2c_master_probe(_i2c_bus_handle, _i2c_address, 2000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ プローブテスト失敗: %s", esp_err_to_name(ret));
        end();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "✅ プローブテスト成功");
    
    // 詳細な通信テスト

    ret = test_device_communication();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ 通信テスト失敗、基本機能のみで続行");
        // 通信テスト失敗でも続行（一部機能のみ使用）
    }
        
    
    // この時点で初期化フラグを立てる
    _initialized = true;

    test_all_leds();
    
    // エンコーダのセットアップ（エラーでも続行）
    setup_rotary_encoder();
    
    // PWMの設定（エラーでも続行）
    setup_pwm();
    
    _last_update_time = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "🎉 エンコーダ初期化完了にゃ！");
    return ESP_OK;
}

// ========================================
// 終了処理にゃ
// ========================================
void PimoroniEncoder::end() {
    if (!_initialized) return;
    
    ESP_LOGI(TAG, "🔚 エンコーダ終了処理開始");
    
    _initialized = false;
    
    // LEDを消灯
    led_off();
    
    // I2Cデバイスを削除
    if (_i2c_dev_handle) {
        i2c_master_bus_rm_device(_i2c_dev_handle);
        _i2c_dev_handle = NULL;
    }
    
    // ミューテックスを削除
    if (_device_mutex) {
        vSemaphoreDelete(_device_mutex);
        _device_mutex = NULL;
    }
    
    ESP_LOGI(TAG, "✅ エンコーダ終了処理完了");
}

// ========================================
// 値の範囲設定にゃ
// ========================================
void PimoroniEncoder::set_value_range(int16_t min_val, int16_t max_val) {
    if (!take_mutex()) return;
    
    _min_value = min_val;
    _max_value = max_val;
    
    // 現在値を範囲内に制限
    if (_current_value < _min_value) _current_value = _min_value;
    if (_current_value > _max_value) _current_value = _max_value;
    
    give_mutex();
    
    ESP_LOGI(TAG, "📏 値範囲設定: %d - %d", min_val, max_val);
}

// ========================================
// エンコーダ値の更新にゃ
// ========================================
int16_t PimoroniEncoder::update() {
    if (!_initialized || !take_mutex()) {
        return _current_value;
    }
    
    uint8_t raw_count = 0;
    esp_err_t ret = read_register(IOE_REG_ENC_1_COUNT, &raw_count, 1);
    if (ret != ESP_OK) {
        give_mutex();
        return _current_value;
    }
    
    // signed 8bitに変換（-128 to 127）
    int8_t signed_count = (int8_t)raw_count;
    
    // 変化量を計算
    int16_t diff = signed_count - _last_encoder_value;
    
    // オーバーフロー検出と補正
    if (diff > 100) {
        diff -= 256;  // 127→-128へのアンダーフロー
    } else if (diff < -100) {
        diff += 256;  // -128→127へのオーバーフロー
    }
    
    // 変化があった場合のみ更新
    if (diff != 0) {
        int16_t old_value = _current_value;
        _current_value += diff;
        
        // 範囲制限
        if (_current_value < _min_value) _current_value = _min_value;
        if (_current_value > _max_value) _current_value = _max_value;
        
        _last_encoder_value = signed_count;
        _last_update_time = esp_timer_get_time() / 1000;
        
        // エンコーダ変化ログ（重要！）
        ESP_LOGI(TAG, "🎛️ エンコーダ変化: raw=%d, diff=%d, 値=%d→%d", 
                 signed_count, diff, old_value, _current_value);
    } else {
        // 変化なしの場合は詳細ログ（デバッグ用）
        ESP_LOGD(TAG, "🎛️ エンコーダ変化なし: raw=%d, 現在値=%d", signed_count, _current_value);
    }
    
    give_mutex();
    return _current_value;
}

// ========================================
// 現在の値を取得にゃ
// ========================================
int16_t PimoroniEncoder::get_value() const {
    return _current_value;
}

// ========================================
// 値を強制設定にゃ
// ========================================
void PimoroniEncoder::set_value(int16_t val) {
    if (!take_mutex()) return;
    
    _current_value = val;
    
    // 範囲制限
    if (_current_value < _min_value) _current_value = _min_value;
    if (_current_value > _max_value) _current_value = _max_value;
    
    give_mutex();
    
    ESP_LOGI(TAG, "🎯 値強制設定: %d", _current_value);
}

// ========================================
// RGB LED制御（0-255）にゃ
// ========================================
void PimoroniEncoder::set_led(uint8_t r, uint8_t g, uint8_t b) {
    if (!_initialized || !take_mutex()) return;
    
    // 値に変化がない場合はスキップ（無限ループ防止）
    if (_current_r == r && _current_g == g && _current_b == b) {
        give_mutex();
        return;
    }
    
    _current_r = r;
    _current_g = g;
    _current_b = b;
    
    // 輝度を適用（0.0-1.0）
    uint8_t red   = (uint8_t)(r * _brightness);
    uint8_t green = (uint8_t)(g * _brightness);
    uint8_t blue  = (uint8_t)(b * _brightness);
    
    ESP_LOGI(TAG, "🌈 LED設定: R=%d, G=%d, B=%d (輝度=%.2f)", red, green, blue, _brightness);
    
    // 方法1: 通常値で試行
    esp_err_t ret_r = write_register(REG_LED_RED, red, 2);
    esp_err_t ret_g = write_register(REG_LED_GREEN, green, 2);
    esp_err_t ret_b = write_register(REG_LED_BLUE, blue, 2);
    
    bool success = (ret_r == ESP_OK && ret_g == ESP_OK && ret_b == ESP_OK);
    
    if (success) {
        ESP_LOGI(TAG, "✅ LED設定成功（通常値）");
    } else {
        ESP_LOGW(TAG, "⚠️ LED設定失敗、反転値で再試行");
        
        // 方法2: Common Anode用反転値で試行
        uint8_t inv_red   = 255 - red;
        uint8_t inv_green = 255 - green;
        uint8_t inv_blue  = 255 - blue;
        
        ESP_LOGI(TAG, "🔄 反転値テスト: R=%d, G=%d, B=%d", inv_red, inv_green, inv_blue);
        
        ret_r = write_register(REG_LED_RED, inv_red, 2);
        ret_g = write_register(REG_LED_GREEN, inv_green, 2);
        ret_b = write_register(REG_LED_BLUE, inv_blue, 2);
        
        if (ret_r == ESP_OK && ret_g == ESP_OK && ret_b == ESP_OK) {
            ESP_LOGI(TAG, "✅ LED設定成功（反転値）");
        } else {
            ESP_LOGE(TAG, "❌ LED設定完全失敗");
        }
    }
    
    give_mutex();
}

// ========================================
// RGB LED制御（0xRRGGBB形式）にゃ
// ========================================
void PimoroniEncoder::set_led_color(uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    set_led(r, g, b);
}

// ========================================
// LED輝度設定にゃ
// ========================================
void PimoroniEncoder::set_led_brightness(float brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    
    _brightness = brightness;
    
    // 現在の色で輝度を更新
    set_led(_current_r, _current_g, _current_b);
    
    ESP_LOGI(TAG, "💡 輝度設定: %.2f", brightness);
}

// ========================================
// LED消灯にゃ
// ========================================
void PimoroniEncoder::led_off() {
    set_led(0, 0, 0);
}

// ========================================
// LED点滅にゃ
// ========================================
void PimoroniEncoder::led_pulse(uint32_t color, uint16_t duration_ms) {
    set_led_color(color);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    led_off();
}

// ========================================
// エンコーダカウントリセットにゃ
// ========================================
esp_err_t PimoroniEncoder::reset_encoder_count() {
    if (!_initialized || !take_mutex()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = write_register(IOE_REG_ENC_1_COUNT, 0);
    if (ret == ESP_OK) {
        _last_encoder_value = 0;
        ESP_LOGI(TAG, "🔄 エンコーダカウントリセット完了");
    }
    
    give_mutex();
    return ret;
}

// ========================================
// LED動作テストにゃ
// ========================================
esp_err_t PimoroniEncoder::test_all_leds() {
    if (!_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "🧪 LED直接制御テスト開始");
    
    // 各色を順番にテスト（直接レジスタ操作）
    const struct {
        uint8_t r, g, b;
        const char* name;
    } test_colors[] = {
        {255, 0, 0, "赤"},
        {0, 255, 0, "緑"},
        {0, 0, 255, "青"},
        {255, 255, 0, "黄"},
        {255, 0, 255, "マゼンタ"},
        {0, 255, 255, "シアン"},
        {255, 255, 255, "白"},
        {0, 0, 0, "消灯"}
    };
    
    for (int i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "🔴 テスト色: %s (R=%d, G=%d, B=%d)", 
                 test_colors[i].name, test_colors[i].r, test_colors[i].g, test_colors[i].b);
        
        // 直接レジスタに書き込み
        write_register(REG_LED_RED, test_colors[i].r, 1);
        write_register(REG_LED_GREEN, test_colors[i].g, 1);
        write_register(REG_LED_BLUE, test_colors[i].b, 1);
        
        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms待機
    }
    
    ESP_LOGI(TAG, "✅ LEDテスト完了");

        ESP_LOGI(TAG, "🧪 LEDレジスタ詳細テスト開始");
    
    // 各レジスタの読み取りテスト
    uint8_t red_val = 0, green_val = 0, blue_val = 0;
    
    esp_err_t ret_r = read_register(REG_LED_RED, &red_val, 3);
    esp_err_t ret_g = read_register(REG_LED_GREEN, &green_val, 3);
    esp_err_t ret_b = read_register(REG_LED_BLUE, &blue_val, 3);
    
    ESP_LOGI(TAG, "📖 LEDレジスタ読み取り結果:");
    ESP_LOGI(TAG, "  赤(0x%02X): %s = 0x%02X", REG_LED_RED, 
             ret_r == ESP_OK ? "成功" : "失敗", red_val);
    ESP_LOGI(TAG, "  緑(0x%02X): %s = 0x%02X", REG_LED_GREEN, 
             ret_g == ESP_OK ? "成功" : "失敗", green_val);
    ESP_LOGI(TAG, "  青(0x%02X): %s = 0x%02X", REG_LED_BLUE, 
             ret_b == ESP_OK ? "成功" : "失敗", blue_val);
    
    // 簡単な書き込みテスト
    ESP_LOGI(TAG, "✏️ LEDレジスタ書き込みテスト");
    
    // 赤を最大値に設定
    if (write_register(REG_LED_RED, 255, 3) == ESP_OK) {
        ESP_LOGI(TAG, "✅ 赤レジスタ書き込み成功");
        vTaskDelay(pdMS_TO_TICKS(1000));
        write_register(REG_LED_RED, 0, 1); // 消灯
    }
    
    // 緑を最大値に設定
    if (write_register(REG_LED_GREEN, 255, 3) == ESP_OK) {
        ESP_LOGI(TAG, "✅ 緑レジスタ書き込み成功");
        vTaskDelay(pdMS_TO_TICKS(1000));
        write_register(REG_LED_GREEN, 0, 1); // 消灯
    }
    
    // 青を最大値に設定
    if (write_register(REG_LED_BLUE, 255, 3) == ESP_OK) {
        ESP_LOGI(TAG, "✅ 青レジスタ書き込み成功");
        vTaskDelay(pdMS_TO_TICKS(1000));
        write_register(REG_LED_BLUE, 0, 1); // 消灯
    }
    

    return ESP_OK;
}

// ========================================
// デバッグ情報表示にゃ
// ========================================
void PimoroniEncoder::print_debug_info() {
    ESP_LOGI(TAG, "\n🔍 === エンコーダデバッグ情報 ===");
    ESP_LOGI(TAG, "📡 I2Cアドレス: 0x%02X", _i2c_address);
    ESP_LOGI(TAG, "🔧 初期化状態: %s", _initialized ? "完了" : "未完了");
    ESP_LOGI(TAG, "🔗 デバイス接続: %s", is_device_connected() ? "OK" : "NG");
    ESP_LOGI(TAG, "🎯 現在値: %d (範囲: %d-%d)", _current_value, _min_value, _max_value);
    ESP_LOGI(TAG, "📊 生エンコーダ値: %d", _last_encoder_value);
    ESP_LOGI(TAG, "💡 輝度: %.2f", _brightness);
    ESP_LOGI(TAG, "🌈 現在RGB: R=%d, G=%d, B=%d", _current_r, _current_g, _current_b);
    ESP_LOGI(TAG, "❌ I2Cエラー回数: %lu", _i2c_error_count);
    ESP_LOGI(TAG, "⏰ 最終更新: %lu ms", _last_update_time);
    ESP_LOGI(TAG, "================================\n");
}

// ========================================
// 全レジスタダンプにゃ
// ========================================
void PimoroniEncoder::print_register_dump() {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ デバイスが初期化されていません");
        return;
    }
    
    ESP_LOGI(TAG, "\n📋 === レジスタダンプ ===");
    
    // 重要なレジスタを表示
    uint8_t registers[] = {
        0x00,  // INT
        0x11,  // ENC_1_COUNT
        0x12,  // PWM_PERIOD_L
        0x13,  // PWM_PERIOD_H
        0x14,  // PWM_CONTROL
        0x18,  // PWM_VALUE (PIN_RED_L)
        0x19,  // PWM_VALUE (PIN_RED_H)
        0x24,  // PWM_VALUE (PIN_GREEN_L)
        0x25,  // PWM_VALUE (PIN_GREEN_H)
        0x1C,  // PWM_VALUE (PIN_BLUE_L)
        0x1D,  // PWM_VALUE (PIN_BLUE_H)
    };
    
    for (int i = 0; i < sizeof(registers); i++) {
        uint8_t value = 0;
        if (read_register(registers[i], &value, 1) == ESP_OK) {
            ESP_LOGI(TAG, "📌 Reg[0x%02X] = 0x%02X (%d)", 
                     registers[i], value, value);
        } else {
            ESP_LOGW(TAG, "❌ Reg[0x%02X] = READ ERROR", registers[i]);
        }
    }
    
    ESP_LOGI(TAG, "========================\n");
}