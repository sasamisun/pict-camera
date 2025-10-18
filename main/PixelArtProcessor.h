/*
 * PixelArtProcessor.h
 * ピクセルアート画像処理専用クラス
 * 
 * 機能:
 * - RGB565からグレースケール変換にゃ
 * - カラーパレット適用にゃ
 * - BMP形式での保存にゃ
 * - マルチコア対応高速処理にゃ
 */

#ifndef PIXEL_ART_PROCESSOR_H
#define PIXEL_ART_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>                     // sqrtf関数用にゃ
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ========================================
// 定数定義にゃ
// ========================================
#define MAX_PALETTES        8       // 最大パレット数
#define COLORS_PER_PALETTE  8       // パレット当たりの色数
#define GRAY_LEVELS         8       // グレースケールレベル数（0-7）

// BT.709 輝度計算係数（高精度）
#define LUMINANCE_R_COEFF   0.2126f
#define LUMINANCE_G_COEFF   0.7152f
#define LUMINANCE_B_COEFF   0.0722f

// ========================================
// BMPヘッダー構造体にゃ
// ========================================
typedef struct {
    uint16_t bfType;          // "BM"
    uint32_t bfSize;          // ファイルサイズ
    uint16_t bfReserved1;     // 予約領域1
    uint16_t bfReserved2;     // 予約領域2
    uint32_t bfOffBits;       // データ開始位置
    uint32_t biSize;          // 情報ヘッダサイズ
    int32_t biWidth;          // 画像幅
    int32_t biHeight;         // 画像高さ
    uint16_t biPlanes;        // プレーン数
    uint16_t biBitCount;      // ビット深度
    uint32_t biCompression;   // 圧縮形式
    uint32_t biSizeImage;     // 画像サイズ
    int32_t biXPelsPerMeter;  // 水平解像度
    int32_t biYPelsPerMeter;  // 垂直解像度
    uint32_t biClrUsed;       // 使用色数
    uint32_t biClrImportant;  // 重要色数
} __attribute__((packed)) bitmap_header_t;

// ========================================
// 画像処理統計情報にゃ
// ========================================
typedef struct {
    uint32_t total_pixels;           // 総ピクセル数
    uint32_t processing_time_ms;     // 処理時間（ミリ秒）
    uint32_t avg_luminance;          // 平均輝度
    uint32_t histogram[GRAY_LEVELS]; // グレースケールヒストグラム
} image_stats_t;

// ========================================
// PixelArtProcessorクラスにゃ
// ========================================
class PixelArtProcessor {
private:
    // パレットデータ
    const uint32_t (*_color_palettes)[COLORS_PER_PALETTE];
    int _palette_count;
    
    // 統計情報
    image_stats_t _last_stats;
    uint32_t _total_processed_images;
    
    // 内部処理メソッド
    uint8_t rgb565_to_gray_level(uint16_t rgb565);
    uint8_t rgb888_to_gray_level(uint8_t r, uint8_t g, uint8_t b);
    void calculate_histogram(const uint8_t* gray_data, int width, int height);
    esp_err_t write_bmp_header(FILE* file, int width, int height);
    
public:
    // コンストラクタ・デストラクタ
    PixelArtProcessor(const uint32_t palettes[][COLORS_PER_PALETTE], int palette_count);
    virtual ~PixelArtProcessor();
    
    // 輝度計算
    uint8_t* calculate_luminance(camera_fb_t* fb);
    uint8_t* calculate_luminance_fast(camera_fb_t* fb);  // 高速版
    
    // BMP保存
    bool save_palette_bmp(const uint8_t* gray_data, int width, int height,
                         int palette_index, uint32_t timestamp, int file_counter,
                         const char* mount_point);
    
    // RGB565からBMP変換・保存
    bool frame_to_bmp(camera_fb_t* fb, const char* filename);
    
    // パレット操作
    bool is_valid_palette_index(int palette_index) const;
    uint32_t get_palette_color(int palette_index, int color_index) const;
    void set_custom_palette(int palette_index, const uint32_t colors[COLORS_PER_PALETTE]);
    
    // 統計情報
    const image_stats_t& get_last_stats() const { return _last_stats; }
    uint32_t get_total_processed_images() const { return _total_processed_images; }
    void print_stats() const;
    void reset_stats();
    
    // ユーティリティ
    void preview_palette(int palette_index, uint8_t* preview_data, int preview_size) const;
    bool validate_image_dimensions(int width, int height) const;
};

// ========================================
// ユーティリティ関数にゃ
// ========================================

// RGB565分解
static inline void rgb565_to_rgb888(uint16_t rgb565, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = ((rgb565 >> 11) & 0x1F) * 255 / 31;  // 5ビット -> 8ビット
    *g = ((rgb565 >> 5) & 0x3F) * 255 / 63;   // 6ビット -> 8ビット
    *b = (rgb565 & 0x1F) * 255 / 31;          // 5ビット -> 8ビット
}

// RGB888結合
static inline uint32_t rgb888_to_color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// 色距離計算（単純なユークリッド距離）
static inline float color_distance(uint32_t color1, uint32_t color2) {
    int r1 = (color1 >> 16) & 0xFF, g1 = (color1 >> 8) & 0xFF, b1 = color1 & 0xFF;
    int r2 = (color2 >> 16) & 0xFF, g2 = (color2 >> 8) & 0xFF, b2 = color2 & 0xFF;
    
    int dr = r1 - r2, dg = g1 - g2, db = b1 - b2;
    return sqrtf(dr*dr + dg*dg + db*db);
}

#endif // PIXEL_ART_PROCESSOR_H