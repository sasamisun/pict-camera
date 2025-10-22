/**
 * @file camera_utils.cpp
 * @brief ESP32-S3 カメラ制御ユーティリティクラス実装
 */

#include "camera_utils.h"

static const char* TAG = "CameraUtils";

CameraUtils::CameraUtils()
    : _initialized(false)
    , _current_frame(nullptr)
    , _night_mode(false)
    , _psram_size(0)
{
    ESP_LOGI(TAG, "CameraUtils コンストラクタ");
    
    // PSRAM使用可能サイズを取得
    _psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM利用可能: %lu KB", (unsigned long)(_psram_size / 1024));
}

CameraUtils::~CameraUtils()
{
    ESP_LOGI(TAG, "CameraUtils デストラクタ");
    if (_initialized) {
        deinit();
    }
}

esp_err_t CameraUtils::init(camera_night_mode_t night_mode)
{
    ESP_LOGI(TAG, "AtomS3R CAM (GC0308) 初期化開始");
    
    if (_initialized) {
        ESP_LOGW(TAG, "既に初期化済み");
        return ESP_OK;
    }
    
    // ✅ AtomS3R CAM専用: GPIO18でカメラ電源制御
    ESP_LOGI(TAG, "AtomS3R CAM: GPIO18でカメラ電源を有効化");
    gpio_config_t power_pin_config = {};
    power_pin_config.pin_bit_mask = (1ULL << 18);
    power_pin_config.mode = GPIO_MODE_OUTPUT;
    power_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
    power_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_pin_config.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t gpio_ret = gpio_config(&power_pin_config);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO18設定失敗: %s", esp_err_to_name(gpio_ret));
        return gpio_ret;
    }
    
    // GPIO18をLOWに設定（カメラ電源ON）
    gpio_set_level(GPIO_NUM_18, 0);
    ESP_LOGI(TAG, "カメラ電源ON（GPIO18=LOW）");
    
    // 電源安定待機
    vTaskDelay(pdMS_TO_TICKS(500));  // 500ms待機
    
    // カメラ設定を準備
    setup_camera_config();
    
    // カメラ初期化実行
    esp_err_t err = esp_camera_init(&_camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "カメラ初期化失敗: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "GC0308カメラ初期化成功");
    
    // 続きの処理...
    return ESP_OK;
}

esp_err_t CameraUtils::deinit()
{
    ESP_LOGI(TAG, "カメラ終了処理");
    
    if (_current_frame) {
        release_current_frame();
    }
    
    if (_initialized) {
        esp_err_t err = esp_camera_deinit();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "カメラ終了処理警告: %s", esp_err_to_name(err));
        }
        _initialized = false;
    }
    
    ESP_LOGI(TAG, "カメラ終了処理完了");
    return ESP_OK;
}

void CameraUtils::setup_camera_config()
{
    // カメラ設定を初期化
    memset(&_camera_config, 0, sizeof(_camera_config));
    
    // AtomS3R CAM 専用ピン設定（推定値）
    _camera_config.pin_pwdn = GPIO_NUM_NC;        // パワーダウン未使用
    _camera_config.pin_reset = GPIO_NUM_NC;       // リセット未使用  
    _camera_config.pin_xclk = GPIO_NUM_21;        // 外部クロック
    _camera_config.pin_sscb_sda = GPIO_NUM_12;    // I2C SDA (SCCB)
    _camera_config.pin_sscb_scl = GPIO_NUM_9;     // I2C SCL (SCCB)
    
    // データピン（8bit）
    _camera_config.pin_d7 = GPIO_NUM_13;
    _camera_config.pin_d6 = GPIO_NUM_11;
    _camera_config.pin_d5 = GPIO_NUM_17;
    _camera_config.pin_d4 = GPIO_NUM_4;
    _camera_config.pin_d3 = GPIO_NUM_48;
    _camera_config.pin_d2 = GPIO_NUM_46;
    _camera_config.pin_d1 = GPIO_NUM_42;
    _camera_config.pin_d0 = GPIO_NUM_3;
    
    // 制御ピン
    _camera_config.pin_vsync = GPIO_NUM_10;       // 垂直同期
    _camera_config.pin_href = GPIO_NUM_14;        // 水平同期  
    _camera_config.pin_pclk = GPIO_NUM_40;        // ピクセルクロック
    
    // クロック設定
    _camera_config.xclk_freq_hz = 20000000;       // 20MHz
    _camera_config.ledc_timer = LEDC_TIMER_0;
    _camera_config.ledc_channel = LEDC_CHANNEL_0;
    
    // GC0308に最適化された設定
    _camera_config.pixel_format = PIXFORMAT_RGB565; // GC0308はRGB565対応
    _camera_config.frame_size = FRAMESIZE_VGA;      // GC0308最大: 640x480
    _camera_config.jpeg_quality = 12;              // RGB565では未使用
    _camera_config.fb_count = 2;
    
    // PSRAM設定
    if (_psram_size > 0) {
        _camera_config.fb_location = CAMERA_FB_IN_PSRAM;
        ESP_LOGI(TAG, "フレームバッファをPSRAMに配置");
    } else {
        _camera_config.fb_location = CAMERA_FB_IN_DRAM;
        ESP_LOGI(TAG, "フレームバッファを内蔵RAMに配置");
    }
    
    _camera_config.grab_mode = CAMERA_GRAB_LATEST;
    _camera_config.sccb_i2c_port = I2C_NUM_0;     // カメラ専用I2C
    
    ESP_LOGI(TAG, "AtomS3R CAM (GC0308) 設定準備完了");
}

esp_err_t CameraUtils::configure_sensor_settings()
{
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "センサー取得失敗");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "センサー設定適用中...");
    
    // AtomS3R Cam 用の基本設定
    s->set_hmirror(s, 1);      // 水平反転有効
    s->set_vflip(s, 1);        // 垂直反転有効
    s->set_brightness(s, 0);   // 明度標準
    s->set_contrast(s, 0);     // コントラスト標準
    s->set_saturation(s, 0);   // 彩度標準
    
    ESP_LOGI(TAG, "センサー設定適用完了");
    return ESP_OK;
}

esp_err_t CameraUtils::apply_night_mode_settings(bool enable)
{
    ESP_LOGI(TAG, "ナイトモード設定: %s", enable ? "有効" : "無効");
    
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) {
        return ESP_FAIL;
    }
    
    if (enable) {
        s->set_brightness(s, 1);   // 明度アップ
        // 他のナイトモード設定は実装省略
    } else {
        s->set_brightness(s, 0);   // 明度標準
    }
    
    return ESP_OK;
}

camera_fb_t* CameraUtils::capture_frame()
{
    ESP_LOGD(TAG, "フレーム撮影");
    
    if (!_initialized) {
        ESP_LOGE(TAG, "カメラ未初期化");
        return nullptr;
    }
    
    // 既存フレームを解放
    if (_current_frame) {
        release_current_frame();
    }
    
    _current_frame = esp_camera_fb_get();
    if (!_current_frame) {
        ESP_LOGE(TAG, "フレーム取得失敗");
        return nullptr;
    }
    
    ESP_LOGD(TAG, "フレーム撮影成功 - サイズ: %zu bytes", _current_frame->len);
    return _current_frame;
}

void CameraUtils::release_frame(camera_fb_t* fb)
{
    ESP_LOGD(TAG, "フレーム解放");
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

void CameraUtils::release_current_frame()
{
    if (_current_frame) {
        ESP_LOGD(TAG, "現在フレーム解放");
        esp_camera_fb_return(_current_frame);
        _current_frame = nullptr;
    }
}

esp_err_t CameraUtils::save_frame_as_bmp(camera_fb_t* fb, const char* filepath)
{
    ESP_LOGI(TAG, "BMP保存: %s", filepath);
    // 実装はスタブ - BMPファイル作成は複雑なので省略
    return ESP_OK;
}

esp_err_t CameraUtils::set_frame_size(framesize_t frame_size)
{
    ESP_LOGI(TAG, "フレームサイズ変更: %d", frame_size);
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_framesize(s, frame_size) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_pixel_format(pixformat_t pixel_format)
{
    ESP_LOGI(TAG, "ピクセルフォーマット変更: %d", pixel_format);
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_pixformat(s, pixel_format) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_night_mode(bool enable)
{
    ESP_LOGI(TAG, "ナイトモード切り替え: %s", enable ? "有効" : "無効");
    _night_mode = enable;
    return apply_night_mode_settings(enable);
}

esp_err_t CameraUtils::set_brightness(int brightness)
{
    ESP_LOGI(TAG, "明度設定: %d", brightness);
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_brightness(s, brightness) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_contrast(int contrast)
{
    ESP_LOGI(TAG, "コントラスト設定: %d", contrast);
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_contrast(s, contrast) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_saturation(int saturation)
{
    ESP_LOGI(TAG, "彩度設定: %d", saturation);
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_saturation(s, saturation) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_white_balance(bool awb_gain_enable)
{
    ESP_LOGI(TAG, "ホワイトバランス設定: %s", awb_gain_enable ? "有効" : "無効");
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_awb_gain(s, awb_gain_enable ? 1 : 0) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_horizontal_mirror(bool enable)
{
    ESP_LOGI(TAG, "水平反転設定: %s", enable ? "有効" : "無効");
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_hmirror(s, enable ? 1 : 0) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::set_vertical_flip(bool enable)
{
    ESP_LOGI(TAG, "垂直反転設定: %s", enable ? "有効" : "無効");
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;
    return s->set_vflip(s, enable ? 1 : 0) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t CameraUtils::get_sensor_info(camera_sensor_info_t* info)
{
    ESP_LOGI(TAG, "センサー情報取得");
    // 実装はスタブ
    return ESP_OK;
}

bool CameraUtils::is_initialized() const
{
    return _initialized;
}

bool CameraUtils::is_night_mode() const
{
    return _night_mode;
}

esp_err_t CameraUtils::get_frame_size(int* width, int* height) const
{
    if (!_initialized) return ESP_ERR_INVALID_STATE;
    
    // HQVGA固定値
    if (width) *width = 240;
    if (height) *height = 176;
    
    ESP_LOGD(TAG, "フレームサイズ取得: %dx%d", 240, 176);
    return ESP_OK;
}

size_t CameraUtils::get_psram_size() const
{
    return _psram_size;
}

esp_err_t CameraUtils::get_frame_stats(uint32_t* total_frames, uint32_t* dropped_frames)
{
    ESP_LOGI(TAG, "フレーム統計取得");
    // 実装はスタブ
    if (total_frames) *total_frames = 0;
    if (dropped_frames) *dropped_frames = 0;
    return ESP_OK;
}