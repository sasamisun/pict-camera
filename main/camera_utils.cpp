/*
 * camera_utils.cpp
 * カメラ操作専用ユーティリティクラス実装 (ESP-IDF 5.4対応版)
 * 
 * 新機能:
 * - 堅牢なフレーム取得・管理にゃ
 * - 高品質BMP保存機能にゃ
 * - カメラ設定自動制御にゃ
 * - 詳細な統計情報取得にゃ
 * - エラーハンドリング強化にゃ
 */

#include "camera_utils.h"

static const char* TAG = "CameraUtils";

// ========================================
// フレームサイズから解像度取得（ユーティリティ関数）にゃ
// ========================================
bool get_frame_size_resolution(framesize_t frame_size, int* width, int* height) {
    if (!width || !height) return false;
    
    switch (frame_size) {
        case FRAMESIZE_96X96:    *width = 96;  *height = 96;  break;
        case FRAMESIZE_QQVGA:    *width = 160; *height = 120; break;
        case FRAMESIZE_QCIF:     *width = 176; *height = 144; break;
        case FRAMESIZE_HQVGA:    *width = 240; *height = 176; break;
        case FRAMESIZE_240X240:  *width = 240; *height = 240; break;
        case FRAMESIZE_QVGA:     *width = 320; *height = 240; break;
        case FRAMESIZE_CIF:      *width = 400; *height = 296; break;
        case FRAMESIZE_HVGA:     *width = 480; *height = 320; break;
        case FRAMESIZE_VGA:      *width = 640; *height = 480; break;
        case FRAMESIZE_SVGA:     *width = 800; *height = 600; break;
        case FRAMESIZE_XGA:      *width = 1024; *height = 768; break;
        case FRAMESIZE_HD:       *width = 1280; *height = 720; break;
        case FRAMESIZE_SXGA:     *width = 1280; *height = 1024; break;
        case FRAMESIZE_UXGA:     *width = 1600; *height = 1200; break;
        case FRAMESIZE_FHD:      *width = 1920; *height = 1080; break;
        case FRAMESIZE_P_HD:     *width = 720; *height = 1280; break;
        case FRAMESIZE_P_3MP:    *width = 864; *height = 1536; break;
        case FRAMESIZE_QXGA:     *width = 2048; *height = 1536; break;
        case FRAMESIZE_QHD:      *width = 2560; *height = 1440; break;
        case FRAMESIZE_WQXGA:    *width = 2560; *height = 1600; break;
        case FRAMESIZE_P_FHD:    *width = 1080; *height = 1920; break;
        case FRAMESIZE_QSXGA:    *width = 2560; *height = 1920; break;
        default: return false;
    }
    return true;
}

// ========================================
// ピクセルフォーマット名取得にゃ
// ========================================
const char* get_pixel_format_name(pixformat_t format) {
    switch (format) {
        case PIXFORMAT_RGB565:   return "RGB565";
        case PIXFORMAT_YUV422:   return "YUV422";
        case PIXFORMAT_YUV420:   return "YUV420";
        case PIXFORMAT_GRAYSCALE: return "GRAYSCALE";
        case PIXFORMAT_JPEG:     return "JPEG";
        case PIXFORMAT_RGB888:   return "RGB888";
        case PIXFORMAT_RAW:      return "RAW";
        case PIXFORMAT_RGB444:   return "RGB444";
        case PIXFORMAT_RGB555:   return "RGB555";
        default: return "UNKNOWN";
    }
}

// ========================================
// BMPファイルサイズ計算にゃ
// ========================================
uint32_t calculate_bmp_file_size(int width, int height) {
    // ヘッダーサイズ + 画像データサイズ（4バイト境界アライン）
    int row_size = ((width * 3 + 3) / 4) * 4;
    return sizeof(lgfx::bitmap_header_t) + (row_size * height);
}

// ========================================
// BMPヘッダー作成・書き込み関数にゃ
// ========================================
bool frame2bmp(camera_fb_t *fb, uint8_t **out_buf, size_t *out_len) {
    if (!fb || !out_buf || !out_len) {
        ESP_LOGE(TAG, "❌ frame2bmp: 無効なパラメータ");
        return false;
    }
    
    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "❌ frame2bmp: サポートされていないピクセルフォーマット");
        return false;
    }
    
    int width = fb->width;
    int height = fb->height;
    int row_size = ((width * 3 + 3) / 4) * 4;  // 4バイト境界アライン
    
    // BMPファイル全体のサイズ計算
    size_t header_size = 54;  // BITMAPFILEHEADER + BITMAPINFOHEADER
    size_t image_size = row_size * height;
    size_t total_size = header_size + image_size;
    
    // メモリ確保
    uint8_t* bmp_buf = (uint8_t*)malloc(total_size);
    if (!bmp_buf) {
        ESP_LOGE(TAG, "❌ frame2bmp: メモリ確保失敗 (%zu bytes)", total_size);
        return false;
    }
    
    uint8_t* ptr = bmp_buf;
    
    // BITMAPFILEHEADER (14 bytes)
    *ptr++ = 'B'; *ptr++ = 'M';                           // bfType
    *(uint32_t*)ptr = total_size; ptr += 4;               // bfSize
    *(uint32_t*)ptr = 0; ptr += 4;                        // bfReserved1, bfReserved2
    *(uint32_t*)ptr = header_size; ptr += 4;              // bfOffBits
    
    // BITMAPINFOHEADER (40 bytes)
    *(uint32_t*)ptr = 40; ptr += 4;                       // biSize
    *(int32_t*)ptr = width; ptr += 4;                     // biWidth
    *(int32_t*)ptr = height; ptr += 4;                    // biHeight
    *(uint16_t*)ptr = 1; ptr += 2;                        // biPlanes
    *(uint16_t*)ptr = 24; ptr += 2;                       // biBitCount
    *(uint32_t*)ptr = 0; ptr += 4;                        // biCompression
    *(uint32_t*)ptr = image_size; ptr += 4;               // biSizeImage
    *(int32_t*)ptr = 2835; ptr += 4;                      // biXPelsPerMeter (72 DPI)
    *(int32_t*)ptr = 2835; ptr += 4;                      // biYPelsPerMeter (72 DPI)
    *(uint32_t*)ptr = 0; ptr += 4;                        // biClrUsed
    *(uint32_t*)ptr = 0; ptr += 4;                        // biClrImportant
    
    // 画像データ変換（RGB565 -> RGB24、下から上へ）
    uint16_t* rgb565_data = (uint16_t*)fb->buf;
    uint8_t* row_buffer = (uint8_t*)malloc(row_size);
    if (!row_buffer) {
        ESP_LOGE(TAG, "❌ frame2bmp: 行バッファ確保失敗");
        free(bmp_buf);
        return false;
    }
    
    for (int y = height - 1; y >= 0; y--) {
        memset(row_buffer, 0, row_size);
        
        for (int x = 0; x < width; x++) {
            uint16_t rgb565 = rgb565_data[y * width + x];
            
            // RGB565をRGB888に変換
            uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565 & 0x1F) * 255 / 31;
            
            // BGRの順で格納（BMPフォーマット）
            row_buffer[x * 3] = b;
            row_buffer[x * 3 + 1] = g;
            row_buffer[x * 3 + 2] = r;
        }
        
        // 1行分をコピー
        memcpy(ptr, row_buffer, row_size);
        ptr += row_size;
    }
    
    free(row_buffer);
    
    *out_buf = bmp_buf;
    *out_len = total_size;
    
    ESP_LOGD(TAG, "✅ frame2bmp: BMP変換完了 (%dx%d, %zu bytes)", width, height, total_size);
    return true;
}

// ========================================
// コンストラクタにゃ
// ========================================
CameraUtils::CameraUtils(const char* mount_point) {
    _mount_point = mount_point ? mount_point : "/sdcard";
    _capture_mutex = NULL;
    _last_capture_time = 0;
    
    // 統計情報初期化
    memset(&_stats, 0, sizeof(frame_stats_t));
    
    ESP_LOGI(TAG, "📸 CameraUtils初期化 (マウントポイント: %s)", _mount_point);
}

// ========================================
// デストラクタにゃ
// ========================================
CameraUtils::~CameraUtils() {
    end();
    ESP_LOGI(TAG, "🧹 CameraUtils終了処理完了");
}

// ========================================
// 初期化にゃ
// ========================================
esp_err_t CameraUtils::begin() {
    ESP_LOGI(TAG, "🔧 CameraUtils初期化開始");
    
    // ミューテックス作成
    _capture_mutex = xSemaphoreCreateMutex();
    if (!_capture_mutex) {
        ESP_LOGE(TAG, "❌ キャプチャミューテックス作成失敗");
        return ESP_ERR_NO_MEM;
    }
    
    // カメラ動作確認
    if (!is_camera_ready()) {
        ESP_LOGW(TAG, "⚠️ カメラが準備できていません");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "✅ CameraUtils初期化完了");
    return ESP_OK;
}

// ========================================
// 終了処理にゃ
// ========================================
void CameraUtils::end() {
    if (_capture_mutex) {
        vSemaphoreDelete(_capture_mutex);
        _capture_mutex = NULL;
    }
}

// ========================================
// フレーム検証にゃ
// ========================================
esp_err_t CameraUtils::validate_frame(camera_fb_t* fb) {
    if (!fb) {
        ESP_LOGE(TAG, "❌ NULLフレームバッファ");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!fb->buf || fb->len == 0) {
        ESP_LOGE(TAG, "❌ 無効なフレームデータ");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (fb->width <= 0 || fb->height <= 0) {
        ESP_LOGE(TAG, "❌ 無効なフレームサイズ: %dx%d", fb->width, fb->height);
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ESP_OK;
}

// ========================================
// 統計情報更新にゃ
// ========================================
void CameraUtils::update_stats(camera_fb_t* fb, uint32_t capture_time) {
    if (!fb) return;
    
    _stats.frame_count++;
    _stats.last_frame_size = fb->len;
    _stats.total_bytes_captured += fb->len;
    _stats.pixel_format = fb->format;
    
    // フレームサイズを推定
    int width, height;
    if (get_frame_size_resolution(FRAMESIZE_HQVGA, &width, &height)) {
        if (width == fb->width && height == fb->height) {
            _stats.frame_size = FRAMESIZE_HQVGA;
        }
    }
    
    // 平均キャプチャ時間更新
    if (_stats.frame_count > 1) {
        _stats.avg_frame_time_ms = (_stats.avg_frame_time_ms + capture_time) / 2;
    } else {
        _stats.avg_frame_time_ms = capture_time;
    }
}

// ========================================
// フレーム取得にゃ
// ========================================
camera_fb_t* CameraUtils::capture_frame(uint32_t timeout_ms) {
    if (!_capture_mutex) {
        ESP_LOGE(TAG, "❌ CameraUtilsが初期化されていません");
        return nullptr;
    }
    
    // ミューテックス取得
    if (xSemaphoreTake(_capture_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "⚠️ キャプチャミューテックス取得タイムアウト");
        return nullptr;
    }
    
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    // フレーム取得
    camera_fb_t* fb = esp_camera_fb_get();
    
    uint32_t capture_time = (esp_timer_get_time() / 1000) - start_time;
    
    if (fb) {
        // フレーム検証
        if (validate_frame(fb) == ESP_OK) {
            update_stats(fb, capture_time);
            _last_capture_time = esp_timer_get_time() / 1000;
            ESP_LOGD(TAG, "📷 フレーム取得成功 (%dx%d, %zu bytes, %lu ms)", 
                     fb->width, fb->height, fb->len, (unsigned long)capture_time);
        } else {
            _stats.error_count++;
            esp_camera_fb_return(fb);
            fb = nullptr;
        }
    } else {
        _stats.error_count++;
        ESP_LOGW(TAG, "⚠️ フレーム取得失敗");
    }
    
    xSemaphoreGive(_capture_mutex);
    return fb;
}

// ========================================
// リトライ付きフレーム取得にゃ
// ========================================
camera_fb_t* CameraUtils::capture_frame_retry(int max_retries) {
    camera_fb_t* fb = nullptr;
    
    for (int retry = 0; retry <= max_retries; retry++) {
        fb = capture_frame(FRAME_TIMEOUT_MS);
        if (fb) {
            if (retry > 0) {
                ESP_LOGI(TAG, "📷 フレーム取得成功 (リトライ %d回)", retry);
            }
            break;
        }
        
        if (retry < max_retries) {
            ESP_LOGW(TAG, "⚠️ フレーム取得失敗、リトライ %d/%d", retry + 1, max_retries);
            vTaskDelay(pdMS_TO_TICKS(100)); // 100ms待機
        }
    }
    
    if (!fb) {
        ESP_LOGE(TAG, "❌ フレーム取得失敗（最大リトライ数に達しました）");
    }
    
    return fb;
}

// ========================================
// フレーム解放にゃ
// ========================================
void CameraUtils::release_frame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
        ESP_LOGD(TAG, "🔄 フレーム解放完了");
    }
}

// ========================================
// BMPヘッダー書き込みにゃ
// ========================================
esp_err_t CameraUtils::write_bmp_header(FILE* file, int width, int height) {
    if (!file) return ESP_ERR_INVALID_ARG;
    
    // BMPヘッダー構造体を使用
    lgfx::bitmap_header_t header = {};
    
    int row_size = ((width * 3 + 3) / 4) * 4;
    
    header.bfType = 0x4D42;                              // "BM"
    header.bfSize = sizeof(header) + (row_size * height);
    header.bfOffBits = sizeof(header);
    header.biSize = 40;
    header.biWidth = width;
    header.biHeight = height;
    header.biPlanes = 1;
    header.biBitCount = 24;
    header.biCompression = 0;
    header.biSizeImage = 0;
    header.biXPelsPerMeter = 2835;
    header.biYPelsPerMeter = 2835;
    header.biClrUsed = 0;
    header.biClrImportant = 0;
    
    size_t written = fwrite(&header, 1, sizeof(header), file);
    if (written != sizeof(header)) {
        ESP_LOGE(TAG, "❌ BMPヘッダー書き込み失敗");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// ========================================
// オリジナル画像BMP保存にゃ
// ========================================
bool CameraUtils::save_original_bmp(camera_fb_t* fb, uint32_t timestamp, int file_counter) {
    if (!fb) {
        ESP_LOGE(TAG, "❌ 無効なフレームバッファ");
        return false;
    }
    
    // ファイル名生成
    char filename[128];
    snprintf(filename, sizeof(filename), "%s/%010lu_%04d_Original.bmp", 
             _mount_point, (unsigned long)timestamp, file_counter);
    
    ESP_LOGI(TAG, "💾 オリジナル画像保存開始: %s", filename);
    
    // frame2bmp関数を使用してBMP変換
    uint8_t* bmp_data = nullptr;
    size_t bmp_size = 0;
    
    if (!frame2bmp(fb, &bmp_data, &bmp_size)) {
        ESP_LOGE(TAG, "❌ BMP変換失敗");
        return false;
    }
    
    // ファイル書き込み
    FILE* file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "❌ ファイルオープン失敗: %s", filename);
        free(bmp_data);
        return false;
    }
    
    size_t written = fwrite(bmp_data, 1, bmp_size, file);
    fclose(file);
    free(bmp_data);
    
    if (written != bmp_size) {
        ESP_LOGE(TAG, "❌ ファイル書き込み失敗: %zu/%zu bytes", written, bmp_size);
        return false;
    }
    
    ESP_LOGI(TAG, "✅ オリジナル画像保存完了: %s (%zu bytes)", filename, bmp_size);
    return true;
}

// ========================================
// フレームをBMPファイルに保存にゃ
// ========================================
bool CameraUtils::frame_to_bmp_file(camera_fb_t* fb, const char* filename) {
    if (!fb || !filename) {
        ESP_LOGE(TAG, "❌ 無効なパラメータ");
        return false;
    }
    
    uint8_t* bmp_data = nullptr;
    size_t bmp_size = 0;
    
    if (!frame2bmp(fb, &bmp_data, &bmp_size)) {
        ESP_LOGE(TAG, "❌ BMP変換失敗");
        return false;
    }
    
    FILE* file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "❌ ファイルオープン失敗: %s", filename);
        free(bmp_data);
        return false;
    }
    
    size_t written = fwrite(bmp_data, 1, bmp_size, file);
    fclose(file);
    free(bmp_data);
    
    bool success = (written == bmp_size);
    if (success) {
        ESP_LOGI(TAG, "✅ BMP保存完了: %s", filename);
    } else {
        ESP_LOGE(TAG, "❌ BMP保存失敗: %s", filename);
    }
    
    return success;
}

// ========================================
// カメラ設定取得にゃ
// ========================================
esp_err_t CameraUtils::get_camera_settings(camera_settings_t& settings) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE(TAG, "❌ センサー取得失敗");
        return ESP_ERR_NOT_FOUND;
    }
    
    // 現在の設定を読み取り（実際のAPIでは読み取り機能が限定的）
    memset(&settings, 0, sizeof(settings));
    ESP_LOGW(TAG, "⚠️ カメラ設定読み取りは限定的です");
    
    return ESP_OK;
}

// ========================================
// カメラ設定適用にゃ
// ========================================
esp_err_t CameraUtils::set_camera_settings(const camera_settings_t& settings) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE(TAG, "❌ センサー取得失敗");
        return ESP_ERR_NOT_FOUND;
    }
    
    // 各設定を適用
    esp_err_t ret = ESP_OK;
    
    if (sensor->set_brightness) {
        sensor->set_brightness(sensor, settings.brightness);
        ESP_LOGD(TAG, "🔧 明度設定: %d", settings.brightness);
    }
    
    if (sensor->set_contrast) {
        sensor->set_contrast(sensor, settings.contrast);
        ESP_LOGD(TAG, "🔧 コントラスト設定: %d", settings.contrast);
    }
    
    if (sensor->set_saturation) {
        sensor->set_saturation(sensor, settings.saturation);
        ESP_LOGD(TAG, "🔧 彩度設定: %d", settings.saturation);
    }
    
    if (sensor->set_hmirror) {
        sensor->set_hmirror(sensor, settings.h_mirror ? 1 : 0);
        ESP_LOGD(TAG, "🔧 水平反転: %s", settings.h_mirror ? "ON" : "OFF");
    }
    
    if (sensor->set_vflip) {
        sensor->set_vflip(sensor, settings.v_flip ? 1 : 0);
        ESP_LOGD(TAG, "🔧 垂直反転: %s", settings.v_flip ? "ON" : "OFF");
    }
    
    ESP_LOGI(TAG, "✅ カメラ設定適用完了");
    return ret;
}

// ========================================
// カメラ設定リセットにゃ
// ========================================
esp_err_t CameraUtils::reset_camera_settings() {
    camera_settings_t default_settings = {
        .brightness = 0,
        .contrast = 0,
        .saturation = 0,
        .sharpness = 0,
        .ae_level = 0,
        .h_mirror = true,   // AtomS3R Cam用
        .v_flip = true,     // AtomS3R Cam用
        .aec = true,
        .aec_dsp = true
    };
    
    return set_camera_settings(default_settings);
}

// ========================================
// 個別設定メソッド群にゃ
// ========================================
esp_err_t CameraUtils::set_brightness(int level) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor || !sensor->set_brightness) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    sensor->set_brightness(sensor, level);
    ESP_LOGI(TAG, "🔧 明度設定: %d", level);
    return ESP_OK;
}

esp_err_t CameraUtils::set_contrast(int level) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor || !sensor->set_contrast) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    sensor->set_contrast(sensor, level);
    ESP_LOGI(TAG, "🔧 コントラスト設定: %d", level);
    return ESP_OK;
}

esp_err_t CameraUtils::set_saturation(int level) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor || !sensor->set_saturation) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    sensor->set_saturation(sensor, level);
    ESP_LOGI(TAG, "🔧 彩度設定: %d", level);
    return ESP_OK;
}

esp_err_t CameraUtils::set_mirror(bool h_mirror, bool v_flip) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        return ESP_ERR_NOT_FOUND;
    }
    
    if (sensor->set_hmirror) {
        sensor->set_hmirror(sensor, h_mirror ? 1 : 0);
    }
    if (sensor->set_vflip) {
        sensor->set_vflip(sensor, v_flip ? 1 : 0);
    }
    
    ESP_LOGI(TAG, "🔧 ミラー設定: H=%s, V=%s", 
             h_mirror ? "ON" : "OFF", v_flip ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t CameraUtils::set_auto_exposure(bool enable) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // ESP-camera 2.1.3では自動露出関連のAPIが削除されているため
    // 設定は記録するが実際の制御はスキップ
    ESP_LOGW(TAG, "⚠️ 自動露出制御: esp-camera 2.1.3では非サポート");
    return ESP_OK;
}

// ========================================
// カメラ準備確認にゃ
// ========================================
bool CameraUtils::is_camera_ready() {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGW(TAG, "⚠️ カメラセンサー未取得");
        return false;
    }
    
    // テストキャプチャで動作確認
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (test_fb) {
        esp_camera_fb_return(test_fb);
        return true;
    }
    
    return false;
}

// ========================================
// テストキャプチャにゃ
// ========================================
esp_err_t CameraUtils::test_capture() {
    ESP_LOGI(TAG, "🧪 テストキャプチャ開始");
    
    camera_fb_t* fb = capture_frame(5000);  // 5秒タイムアウト
    if (!fb) {
        ESP_LOGE(TAG, "❌ テストキャプチャ失敗");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ テストキャプチャ成功: %dx%d, %zu bytes, %s", 
             fb->width, fb->height, fb->len, get_pixel_format_name(fb->format));
    
    release_frame(fb);
    return ESP_OK;
}

// ========================================
// カメラ情報表示にゃ
// ========================================
void CameraUtils::print_camera_info() {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGW(TAG, "⚠️ カメラセンサー情報取得不可");
        return;
    }
    
    ESP_LOGI(TAG, "\n📸 === カメラ情報 ===");
    ESP_LOGI(TAG, "🎯 センサーID: 0x%02X", sensor->id.PID);
    ESP_LOGI(TAG, "🔧 現在の設定:");
    ESP_LOGI(TAG, "  📐 フォーマット: %s", get_pixel_format_name(_stats.pixel_format));
    ESP_LOGI(TAG, "  📏 フレームサイズ: %d", _stats.frame_size);
    ESP_LOGI(TAG, "🏃 準備状態: %s", is_camera_ready() ? "OK" : "NG");
    ESP_LOGI(TAG, "====================\n");
}

// ========================================
// 統計情報表示にゃ
// ========================================
void CameraUtils::print_stats() const {
    ESP_LOGI(TAG, "\n📊 === カメラ統計情報 ===");
    ESP_LOGI(TAG, "📷 キャプチャ回数: %lu", (unsigned long)_stats.frame_count);
    ESP_LOGI(TAG, "❌ エラー回数: %lu", (unsigned long)_stats.error_count);
    ESP_LOGI(TAG, "⏱️ 平均キャプチャ時間: %lu ms", (unsigned long)_stats.avg_frame_time_ms);
    ESP_LOGI(TAG, "📏 最終フレームサイズ: %lu bytes", (unsigned long)_stats.last_frame_size);
    ESP_LOGI(TAG, "💾 累計キャプチャ量: %lu bytes", (unsigned long)_stats.total_bytes_captured);
    ESP_LOGI(TAG, "📐 ピクセルフォーマット: %s", get_pixel_format_name(_stats.pixel_format));
    ESP_LOGI(TAG, "========================\n");
}

// ========================================
// 統計情報リセットにゃ
// ========================================
void CameraUtils::reset_stats() {
    memset(&_stats, 0, sizeof(frame_stats_t));
    ESP_LOGI(TAG, "🔄 カメラ統計情報リセット完了");
}

// ========================================
// フレーム情報表示にゃ
// ========================================
void CameraUtils::print_frame_info(camera_fb_t* fb) {
    if (!fb) {
        ESP_LOGW(TAG, "⚠️ NULLフレーム");
        return;
    }
    
    ESP_LOGI(TAG, "🖼️ フレーム情報:");
    ESP_LOGI(TAG, "  📏 サイズ: %dx%d", fb->width, fb->height);
    ESP_LOGI(TAG, "  💾 データ量: %zu bytes", fb->len);
    ESP_LOGI(TAG, "  🎨 フォーマット: %s", get_pixel_format_name(fb->format));
    ESP_LOGI(TAG, "  ⏰ タイムスタンプ: %lu", (unsigned long)fb->timestamp.tv_sec);
}

// ========================================
// カメラ設定検証にゃ
// ========================================
bool CameraUtils::validate_camera_config() {
    sensor_t* sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE(TAG, "❌ センサー未取得");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ カメラ設定検証完了");
    return true;
}

// ========================================
// フレーム情報JSON出力にゃ
// ========================================
void print_frame_json(camera_fb_t* fb) {
    if (!fb) {
        printf("{\"error\":\"null_frame\"}\n");
        return;
    }
    
    printf("{\n");
    printf("  \"width\": %d,\n", fb->width);
    printf("  \"height\": %d,\n", fb->height);
    printf("  \"length\": %zu,\n", fb->len);
    printf("  \"format\": \"%s\",\n", get_pixel_format_name(fb->format));
    printf("  \"timestamp\": %lu\n", (unsigned long)fb->timestamp.tv_sec);
    printf("}\n");
}