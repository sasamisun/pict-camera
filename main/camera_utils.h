/**
 * @file camera_utils.h
 * @brief ESP32-S3 カメラ制御ユーティリティクラス
 * 
 * ESP-IDF 5.4 + esp-camera 2.1.3 対応版
 * AtomS3R専用設定でカメラの初期化・撮影・保存を行う
 */

#ifndef CAMERA_UTILS_H
#define CAMERA_UTILS_H

#include <cstring>
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2c.h"

// 制御ピン
#define CAMERA_PIN_VSYNC    GPIO_NUM_10     // 垂直同期
#define CAMERA_PIN_HREF     GPIO_NUM_14     // 水平同期
#define CAMERA_PIN_PCLK     GPIO_NUM_40     // ピクセルクロック

// カメラ設定
#define CAMERA_XCLK_FREQ    20000000        // 20MHz
#define CAMERA_I2C_PORT     I2C_NUM_0       // カメラ専用I2Cポート

// 画像設定
#define DEFAULT_FRAME_SIZE  FRAMESIZE_HQVGA // 240x176
#define DEFAULT_PIXEL_FORMAT PIXFORMAT_RGB565
#define DEFAULT_JPEG_QUALITY 12             // JPEG品質（RGB565では未使用）
#define FB_COUNT            2               // フレームバッファ数

/**
 * @enum camera_night_mode_t
 * @brief ナイトモード設定
 */
typedef enum {
    CAMERA_NIGHT_MODE_OFF = 0,      // 通常モード
    CAMERA_NIGHT_MODE_ON = 1        // ナイトモード（高感度）
} camera_night_mode_t;

/**
 * @class CameraUtils
 * @brief ESP32-S3 カメラ制御ユーティリティクラス
 */
class CameraUtils {
private:
    bool _initialized;                      // 初期化完了フラグ
    camera_config_t _camera_config;        // カメラ設定
    camera_fb_t* _current_frame;            // 現在のフレームバッファ
    bool _night_mode;                       // ナイトモード状態
    size_t _psram_size;                     // PSRAM使用可能サイズ
    
    // 内部ヘルパーメソッド
    void setup_camera_config();
    esp_err_t configure_sensor_settings();
    esp_err_t apply_night_mode_settings(bool enable);

public:
    /**
     * @brief コンストラクタ
     */
    CameraUtils();
    
    /**
     * @brief デストラクタ
     */
    ~CameraUtils();
    
    /**
     * @brief カメラを初期化
     * @param night_mode ナイトモードを有効にするか
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t init(camera_night_mode_t night_mode = CAMERA_NIGHT_MODE_OFF);
    
    /**
     * @brief カメラを終了処理
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t deinit();
    
    /**
     * @brief フレームを撮影
     * @return camera_fb_t*: フレームバッファ, NULL: エラー
     */
    camera_fb_t* capture_frame();
    
    /**
     * @brief フレームバッファを解放
     * @param fb 解放するフレームバッファ
     */
    void release_frame(camera_fb_t* fb);
    
    /**
     * @brief 現在のフレームを解放
     */
    void release_current_frame();
    
    /**
     * @brief BMPファイルとして保存
     * @param fb フレームバッファ
     * @param filepath 保存先ファイルパス
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t save_frame_as_bmp(camera_fb_t* fb, const char* filepath);
    
    /**
     * @brief フレームサイズを設定
     * @param frame_size フレームサイズ
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_frame_size(framesize_t frame_size);
    
    /**
     * @brief ピクセルフォーマットを設定
     * @param pixel_format ピクセルフォーマット
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_pixel_format(pixformat_t pixel_format);
    
    /**
     * @brief ナイトモードの切り替え
     * @param enable true: 有効, false: 無効
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_night_mode(bool enable);
    
    /**
     * @brief 明度調整
     * @param brightness 明度 (-2 ～ 2)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_brightness(int brightness);
    
    /**
     * @brief コントラスト調整
     * @param contrast コントラスト (-2 ～ 2)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_contrast(int contrast);
    
    /**
     * @brief 彩度調整
     * @param saturation 彩度 (-2 ～ 2)
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_saturation(int saturation);
    
    /**
     * @brief ホワイトバランス調整
     * @param awb_gain_enable ホワイトバランスゲイン有効/無効
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_white_balance(bool awb_gain_enable);
    
    /**
     * @brief 水平反転設定
     * @param enable true: 反転, false: 通常
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_horizontal_mirror(bool enable);
    
    /**
     * @brief 垂直反転設定
     * @param enable true: 反転, false: 通常
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t set_vertical_flip(bool enable);
    
    /**
     * @brief カメラセンサー情報を取得
     * @param info センサー情報を格納する構造体のポインタ
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t get_sensor_info(camera_sensor_info_t* info);
    
    /**
     * @brief 初期化状態を取得
     * @return true: 初期化済み, false: 未初期化
     */
    bool is_initialized() const;
    
    /**
     * @brief ナイトモード状態を取得
     * @return true: ナイトモード, false: 通常モード
     */
    bool is_night_mode() const;
    
    /**
     * @brief 現在のフレームサイズを取得
     * @param width 幅を格納するポインタ
     * @param height 高さを格納するポインタ
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t get_frame_size(int* width, int* height) const;
    
    /**
     * @brief PSRAM使用状況を取得
     * @return PSRAM使用可能サイズ（バイト）
     */
    size_t get_psram_size() const;
    
    /**
     * @brief フレーム統計情報を取得
     * @param total_frames 総フレーム数を格納するポインタ
     * @param dropped_frames ドロップしたフレーム数を格納するポインタ
     * @return ESP_OK: 成功, その他: エラーコード
     */
    esp_err_t get_frame_stats(uint32_t* total_frames, uint32_t* dropped_frames);
};

#endif // CAMERA_UTILS_H