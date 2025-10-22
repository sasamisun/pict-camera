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

    // PWM周期設定 (255 = 8bit PWM)
    ret = write_register(REG_PWMPL, 255);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM周期(下位)設定失敗");
        return ret;
    }

    ret = write_register(REG_PWMPH, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM周期(上位)設定失敗");
        return ret;
    }

    // PWM制御設定 (分周比2, PWMRUN有効化)
    // PWMCON0: bit7=PWMRUN, bit[2:0]=CLKDIV
    ret = write_register(REG_PWMCON0, 0x80 | 0x02);  // PWMRUN=1, CLKDIV=2
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM制御設定失敗");
        return ret;
    }
    
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
    ESP_LOGI(TAG, "ピン%dをモード%dに設定開始", pin, mode);

    // すべてのRGBピンはポート0に属するため、P0M1/P0M2を使用
    // GPIO モード設定: 00=QB, 01=PP(Push-Pull), 10=IN, 11=OD
    // PWMの場合は Push-Pull(01) に設定

    if (mode == PIN_MODE_PWM) {
        // 1. GPIO を Push-Pull 出力モードに設定
        uint8_t p0m1_val, p0m2_val;
        esp_err_t ret;

        ret = read_register(REG_P0M1, &p0m1_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "P0M1読み取り失敗");
            return ret;
        }

        ret = read_register(REG_P0M2, &p0m2_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "P0M2読み取り失敗");
            return ret;
        }

        // Push-Pull = 01 -> M1=0, M2=1
        p0m1_val &= ~(1 << pin);  // M1ビットをクリア
        p0m2_val |= (1 << pin);   // M2ビットをセット

        ret = write_register(REG_P0M1, p0m1_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "P0M1書き込み失敗");
            return ret;
        }

        ret = write_register(REG_P0M2, p0m2_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "P0M2書き込み失敗");
            return ret;
        }

        ESP_LOGI(TAG, "ピン%d: GPIO Push-Pull設定完了 (P0M1=0x%02X, P0M2=0x%02X)",
                 pin, p0m1_val, p0m2_val);

        // 2. PIOCON レジスタで PWM 機能を有効化
        // ピン1, 2, 7 はすべて PWM0, PWM1, PWM2 に対応
        // PIOCON0 の bit0, bit1, bit2 を設定
        uint8_t piocon0_val;
        ret = read_register(REG_PIOCON0, &piocon0_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PIOCON0読み取り失敗");
            return ret;
        }

        // ピン番号に対応するPWMチャネルのIOPWMビットを設定
        // PIN_RED(1) -> PWM1, PIN_GREEN(7) -> PWM7は存在しないので確認が必要
        // 実際には、ピン1=PWM1, ピン2=PWM2, ピン7=PWM7(?)
        // Pimoroniのコードを見ると、ピン番号がそのままPWMチャネルになる模様
        if (pin < 8) {  // PIOCON0 は PWM0-7 を制御
            piocon0_val |= (1 << pin);
        }

        ret = write_register(REG_PIOCON0, piocon0_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PIOCON0書き込み失敗");
            return ret;
        }

        ESP_LOGI(TAG, "ピン%d: PWM有効化完了 (PIOCON0=0x%02X)", pin, piocon0_val);
    }

    return ESP_OK;
}

esp_err_t PimoroniEncoder::set_pwm_value(uint8_t pin, uint16_t value)
{
    // ピン番号からPWMレジスタへのマッピング
    // PIN_RED(1) -> PWM1, PIN_BLUE(2) -> PWM2, PIN_GREEN(7) -> PWM7
    uint8_t reg_low, reg_high;

    switch (pin) {
        case 0:
            reg_low = REG_PWM0L;
            reg_high = REG_PWM0H;
            break;
        case 1:
            reg_low = REG_PWM1L;
            reg_high = REG_PWM1H;
            break;
        case 2:
            reg_low = REG_PWM2L;
            reg_high = REG_PWM2H;
            break;
        case 3:
            reg_low = REG_PWM3L;
            reg_high = REG_PWM3H;
            break;
        case 4:
            reg_low = REG_PWM4L;
            reg_high = REG_PWM4H;
            break;
        case 5:
            reg_low = REG_PWM5L;
            reg_high = REG_PWM5H;
            break;
        case 7:
            // ピン7はPWM7に対応すると仮定
            // もしPWM5までしかない場合は、ピン7もPWM5にマッピングする
            reg_low = REG_PWM5L;
            reg_high = REG_PWM5H;
            ESP_LOGW(TAG, "ピン7はPWM5にマッピング（PWM7が存在しないため）");
            break;
        default:
            ESP_LOGE(TAG, "未対応のピン番号: %d", pin);
            return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "PWM値設定: ピン%d, 値=%u, レジスタ(L=0x%02X, H=0x%02X)",
             pin, value, reg_low, reg_high);

    esp_err_t ret = write_register(reg_low, value & 0xFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM下位バイト書き込み失敗");
        return ret;
    }

    ret = write_register(reg_high, (value >> 8) & 0xFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM上位バイト書き込み失敗");
        return ret;
    }

    return ESP_OK;
}

// エンコーダー値を実際にハードウェアから読み取って更新する関数
int16_t PimoroniEncoder::update()
{
    if (!_initialized) {
        ESP_LOGD(TAG, "未初期化のためエンコーダー値更新をスキップ");
        return _current_value;
    }
    
    // ハードウェアからエンコーダーカウント値を読み取り
    uint8_t raw_count;
    esp_err_t ret = read_register(REG_ENC_1_COUNT, &raw_count);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "エンコーダー値読み取り失敗: %s", esp_err_to_name(ret));
        return _current_value;
    }
    
    // 8bit signed値に変換
    int8_t current_count = (int8_t)raw_count;
    
    // 前回値との差分を計算（オーバーフロー対応）
    int16_t diff = current_count - _last_encoder_count;
    
    // オーバーフロー検出と補正
    if (diff > 100) {
        diff -= 256;  // 127→-128への遷移
    } else if (diff < -100) {
        diff += 256;  // -128→127への遷移
    }
    
    // 差分がある場合のみ値を更新
    if (diff != 0) {
        _current_value += diff;
        
        // 設定された範囲内に制限
        if (_current_value < _min_value) {
            _current_value = _min_value;
        } else if (_current_value > _max_value) {
            _current_value = _max_value;
        }
        
        _last_encoder_count = current_count;
        ESP_LOGI(TAG, "エンコーダー値変更: %d (raw: %d, diff: %d)", _current_value, current_count, diff);
    }
    
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

    // 8bit(0-255)を16bit(0-65535)に変換: value * 256 + value
    // これにより 0→0, 255→65535 の正確なマッピングを実現
    uint16_t red_16bit = (r << 8) | r;
    uint16_t green_16bit = (g << 8) | g;
    uint16_t blue_16bit = (b << 8) | b;

    // Common Anodeなので値を反転 (65535 - value)
    uint16_t red_pwm = 65535 - red_16bit;
    uint16_t green_pwm = 65535 - green_16bit;
    uint16_t blue_pwm = 65535 - blue_16bit;

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

// エンコーダーのテスト用関数群を追加

esp_err_t PimoroniEncoder::test_led_colors()
{
    if (!_initialized) {
        ESP_LOGE(TAG, "エンコーダー未初期化");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "LED色テスト開始");
    
    // 基本色テスト
    const uint32_t test_colors[] = {
        0xFF0000, // 赤
        0x00FF00, // 緑
        0x0000FF, // 青
        0xFFFF00, // 黄
        0xFF00FF, // マゼンタ
        0x00FFFF, // シアン
        0xFFFFFF, // 白
        0x000000  // 消灯
    };
    
    const char* color_names[] = {
        "赤", "緑", "青", "黄", "マゼンタ", "シアン", "白", "消灯"
    };
    
    for (int i = 0; i < 8; i++) {
        ESP_LOGI(TAG, "色テスト: %s (0x%06lX)", color_names[i], (unsigned long)test_colors[i]);
        esp_err_t ret = set_color(test_colors[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "色設定失敗: %s", color_names[i]);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // 500ms待機
    }
    
    ESP_LOGI(TAG, "LED色テスト完了");
    return ESP_OK;
}

bool PimoroniEncoder::check_encoder_connection()
{
    if (!_initialized) {
        ESP_LOGE(TAG, "エンコーダー未初期化");
        return false;
    }
    
    // エンコーダーレジスタの読み取りテスト
    uint8_t test_value;
    esp_err_t ret = read_register(REG_ENC_1_COUNT, &test_value);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "エンコーダー接続確認OK: レジスタ値 = %d", test_value);
        return true;
    } else {
        ESP_LOGE(TAG, "エンコーダー接続確認NG: %s", esp_err_to_name(ret));
        return false;
    }
}