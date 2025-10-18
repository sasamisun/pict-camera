/*
 * camera_utils.h
 * カメラ操作専用ユーティリティクラス
 * 
 * 機能:
 * - フレーム取得・管理にゃ
 * - オリジナル画像BMP保存にゃ
 * - カメラ設定制御にゃ
 * - フレーム統計情報取得にゃ
 */

#ifndef CAMERA_UTILS_H
#define CAMERA_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ========================================
// 定数定義にゃ
// ========================================
#define FRAME_TIMEOUT_MS    5000    // フレーム取得タイムアウト
#define MAX_CAPTURE_RETRIES 3       // 撮影リトライ回数

// ========================================
// フレーム統計情報にゃ
// ========================================
typedef struct {
    uint32_t frame_count;           // 取得フレーム数
    uint32_t error_count;           // エラー回数
    uint32_t avg_frame_time_ms;     // 平均フレーム取得時間
    uint32_t last_frame_size;       // 最後のフレームサイズ
    uint32_t total_bytes_captured;  // 累計キャプチャバイト数
    pixformat_t pixel_format;       // ピクセルフォーマット
    framesize_t frame_size;         // フレームサイズ
} frame_stats_t;

// ========================================
// カメラ設定構造体にゃ
// ========================================
typedef struct {
    int brightness;     // 明度 (-2 to 2)
    int contrast;       // コントラスト (-2 to 2)
    int saturation;     // 彩度 (-2 to 2)
    int sharpness;      // シャープネス (-2 to 2)
    int ae_level;       // 自動露出レベル (-2 to 2)
    bool h_mirror;      // 水平反転
    bool v_flip;        // 垂直反転
    bool aec;           // 自動露出制御
    bool aec_dsp;       // DSP自動露出
} camera_settings_t;

// ========================================
// CameraUtilsクラスにゃ
// ========================================
class CameraUtils {
private:
    // 設定
    const char* _mount_point;
    SemaphoreHandle_t _capture_mutex;
    
    // 統計情報
    frame_stats_t _stats;
    uint32_t _last_capture_time;
    
    // 内部メソッド
    esp_err_t validate_frame(camera_fb_t* fb);
    uint32_t calculate_bmp_size(int width, int height);
    esp_err_t write_bmp_header(FILE* file, int width, int height);
    void update_stats(camera_fb_t* fb, uint32_t capture_time);

public:
    // コンストラクタ・デストラクタ
    CameraUtils(const char* mount_point = "/sdcard");
    virtual ~CameraUtils();
    
    // 初期化・終了
    esp_err_t begin();
    void end();
    
    // フレーム操作
    camera_fb_t* capture_frame(uint32_t timeout_ms = FRAME_TIMEOUT_MS);
    camera_fb_t* capture_frame_retry(int max_retries = MAX_CAPTURE_RETRIES);
    void release_frame(camera_fb_t* fb);
    
    // BMP保存
    bool save_original_bmp(camera_fb_t* fb, uint32_t timestamp, int file_counter);
    bool frame_to_bmp_file(camera_fb_t* fb, const char* filename);
    
    // カメラ設定
    esp_err_t set_camera_settings(const camera_settings_t& settings);
    esp_err_t get_camera_settings(camera_settings_t& settings);
    esp_err_t reset_camera_settings();
    
    // 個別設定メソッド
    esp_err_t set_brightness(int level);      // -2 to 2
    esp_err_t set_contrast(int level);        // -2 to 2
    esp_err_t set_saturation(int level);      // -2 to 2
    esp_err_t set_mirror(bool h_mirror, bool v_flip);
    esp_err_t set_auto_exposure(bool enable);
    
    // 統計情報
    const frame_stats_t& get_stats() const { return _stats; }
    void print_stats() const;
    void reset_stats();
    
    // ユーティリティ
    bool is_camera_ready();
    esp_err_t test_capture();
    void print_camera_info();
    
    // デバッグ
    void print_frame_info(camera_fb_t* fb);
    bool validate_camera_config();
};

// ========================================
// ユーティリティ関数にゃ
// ========================================

// フレームサイズから解像度取得
bool get_frame_size_resolution(framesize_t frame_size, int* width, int* height);

// ピクセルフォーマット名取得
const char* get_pixel_format_name(pixformat_t format);

// BMPファイルサイズ計算
uint32_t calculate_bmp_file_size(int width, int height);

// フレーム情報をJSON形式で出力
void print_frame_json(camera_fb_t* fb);

#endif // CAMERA_UTILS_H