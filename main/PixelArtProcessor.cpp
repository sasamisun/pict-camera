/*
 * PixelArtProcessor.cpp
 * ピクセルアート画像処理専用クラス実装 (ESP-IDF 5.4対応版)
 * 
 * 新機能:
 * - RGB565からグレースケール高速変換にゃ
 * - 8色カラーパレット適用にゃ
 * - BMP形式での高品質保存にゃ
 * - ヒストグラム分析機能にゃ
 * - エラーハンドリング強化にゃ
 */

#include "PixelArtProcessor.h"

static const char* TAG = "PixelArtProcessor";

// ========================================
// コンストラクタにゃ
// ========================================
PixelArtProcessor::PixelArtProcessor(const uint32_t palettes[][COLORS_PER_PALETTE], int palette_count) {
    _color_palettes = palettes;
    _palette_count = palette_count;
    _total_processed_images = 0;
    
    // 統計情報初期化
    memset(&_last_stats, 0, sizeof(image_stats_t));
    
    ESP_LOGI(TAG, "🎨 PixelArtProcessor初期化完了 (パレット数: %d)", palette_count);
}

// ========================================
// デストラクタにゃ
// ========================================
PixelArtProcessor::~PixelArtProcessor() {
    ESP_LOGI(TAG, "🧹 PixelArtProcessor終了処理完了");
}

// ========================================
// RGB565を8段階グレースケールに変換にゃ
// ========================================
uint8_t PixelArtProcessor::rgb565_to_gray_level(uint16_t rgb565) {
    // RGB565を各成分に分解
    uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;  // 5ビット -> 8ビット
    uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;   // 6ビット -> 8ビット
    uint8_t b = (rgb565 & 0x1F) * 255 / 31;          // 5ビット -> 8ビット
    
    return rgb888_to_gray_level(r, g, b);
}

// ========================================
// RGB888を8段階グレースケールに変換にゃ
// ========================================
uint8_t PixelArtProcessor::rgb888_to_gray_level(uint8_t r, uint8_t g, uint8_t b) {
    // BT.709標準の輝度計算
    float luminance = LUMINANCE_R_COEFF * r + LUMINANCE_G_COEFF * g + LUMINANCE_B_COEFF * b;
    
    // 0-255を0-7の8段階に量子化
    uint8_t gray_level = (uint8_t)(luminance / 32.0f);
    
    // 範囲チェック（安全のため）
    if (gray_level >= GRAY_LEVELS) {
        gray_level = GRAY_LEVELS - 1;
    }
    
    return gray_level;
}

// ========================================
// カメラフレームから輝度データを計算にゃ
// ========================================
uint8_t* PixelArtProcessor::calculate_luminance(camera_fb_t* fb) {
    if (!fb || !fb->buf) {
        ESP_LOGE(TAG, "❌ 無効なフレームバッファ");
        return nullptr;
    }
    
    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "❌ サポートされていないピクセルフォーマット: %d", fb->format);
        return nullptr;
    }
    
    int width = fb->width;
    int height = fb->height;
    int total_pixels = width * height;
    
    ESP_LOGI(TAG, "🔄 輝度計算開始: %dx%d (%d pixels)", width, height, total_pixels);
    
    // 処理開始時間を記録
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    // 輝度データ用メモリ確保
    uint8_t* gray_data = (uint8_t*)malloc(total_pixels);
    if (!gray_data) {
        ESP_LOGE(TAG, "❌ 輝度データ用メモリ確保失敗");
        return nullptr;
    }
    
    // RGB565データのポインタ
    uint16_t* rgb565_data = (uint16_t*)fb->buf;
    
    // 統計用変数
    uint32_t luminance_sum = 0;
    uint32_t histogram[GRAY_LEVELS] = {0};
    
    // 各ピクセルを処理
    for (int i = 0; i < total_pixels; i++) {
        // RGB565値を取得（エンディアン考慮）
        uint16_t rgb565 = rgb565_data[i];
        
        // グレースケールレベルに変換
        uint8_t gray_level = rgb565_to_gray_level(rgb565);
        gray_data[i] = gray_level;
        
        // 統計情報更新
        luminance_sum += gray_level;
        histogram[gray_level]++;
    }
    
    // 処理時間計算
    uint32_t processing_time = (esp_timer_get_time() / 1000) - start_time;
    
    // 統計情報更新
    _last_stats.total_pixels = total_pixels;
    _last_stats.processing_time_ms = processing_time;
    _last_stats.avg_luminance = luminance_sum / total_pixels;
    memcpy(_last_stats.histogram, histogram, sizeof(histogram));
    
    ESP_LOGI(TAG, "✅ 輝度計算完了 (%lu ms, 平均輝度: %lu)", 
             (unsigned long)processing_time, (unsigned long)_last_stats.avg_luminance);
    
    return gray_data;
}

// ========================================
// 高速版輝度計算（マルチコア対応予定）にゃ
// ========================================
uint8_t* PixelArtProcessor::calculate_luminance_fast(camera_fb_t* fb) {
    // 現在は通常版と同じ実装
    // 将来的にはマルチコア処理やSIMD最適化を実装予定
    return calculate_luminance(fb);
}

// ========================================
// ヒストグラム計算にゃ
// ========================================
void PixelArtProcessor::calculate_histogram(const uint8_t* gray_data, int width, int height) {
    if (!gray_data) return;
    
    uint32_t histogram[GRAY_LEVELS] = {0};
    int total_pixels = width * height;
    
    for (int i = 0; i < total_pixels; i++) {
        uint8_t gray_level = gray_data[i];
        if (gray_level < GRAY_LEVELS) {
            histogram[gray_level]++;
        }
    }
    
    memcpy(_last_stats.histogram, histogram, sizeof(histogram));
}

// ========================================
// BMPヘッダー書き込みにゃ
// ========================================
esp_err_t PixelArtProcessor::write_bmp_header(FILE* file, int width, int height) {
    if (!file) return ESP_ERR_INVALID_ARG;
    
    // BMPヘッダー作成
    bitmap_header_t header = {};
    
    // ファイルヘッダー
    header.bfType = 0x4D42;                    // "BM"
    header.bfReserved1 = 0;
    header.bfReserved2 = 0;
    header.bfOffBits = sizeof(bitmap_header_t);
    
    // 情報ヘッダー
    header.biSize = 40;                        // BITMAPINFOHEADER サイズ
    header.biWidth = width;
    header.biHeight = height;
    header.biPlanes = 1;
    header.biBitCount = 24;                    // 24ビットカラー
    header.biCompression = 0;                  // 無圧縮
    header.biSizeImage = 0;                    // 無圧縮の場合は0でOK
    header.biXPelsPerMeter = 2835;             // 72 DPI
    header.biYPelsPerMeter = 2835;
    header.biClrUsed = 0;
    header.biClrImportant = 0;
    
    // 行サイズ計算（4バイト境界アライン）
    int row_size = ((width * 3 + 3) / 4) * 4;
    header.bfSize = header.bfOffBits + (row_size * height);
    
    // ヘッダー書き込み
    size_t written = fwrite(&header, 1, sizeof(header), file);
    if (written != sizeof(header)) {
        ESP_LOGE(TAG, "❌ BMPヘッダー書き込み失敗");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// ========================================
// パレット変換BMP保存にゃ
// ========================================
bool PixelArtProcessor::save_palette_bmp(const uint8_t* gray_data, int width, int height,
                                        int palette_index, uint32_t timestamp, int file_counter,
                                        const char* mount_point) {
    if (!gray_data || !is_valid_palette_index(palette_index)) {
        ESP_LOGE(TAG, "❌ 無効なパラメータ");
        return false;
    }
    
    // ファイル名生成
    char filename[128];
    snprintf(filename, sizeof(filename), "%s/%010lu_%04d_palette%01d.bmp", 
             mount_point, (unsigned long)timestamp, file_counter, palette_index);
    
    ESP_LOGI(TAG, "💾 パレット画像保存開始: %s", filename);
    
    // ファイルオープン
    FILE* file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "❌ ファイルオープン失敗: %s", filename);
        return false;
    }
    
    // BMPヘッダー書き込み
    if (write_bmp_header(file, width, height) != ESP_OK) {
        fclose(file);
        return false;
    }
    
    // 行サイズ計算（4バイト境界アライン）
    int row_size = ((width * 3 + 3) / 4) * 4;
    uint8_t* row_buffer = (uint8_t*)malloc(row_size);
    if (!row_buffer) {
        ESP_LOGE(TAG, "❌ 行バッファ確保失敗");
        fclose(file);
        return false;
    }
    
    // 画像データ書き込み（BMPは下から上へ）
    for (int y = height - 1; y >= 0; y--) {
        // 行バッファをクリア
        memset(row_buffer, 0, row_size);
        
        for (int x = 0; x < width; x++) {
            // グレースケール値を取得
            int pixel_index = y * width + x;
            uint8_t gray_level = gray_data[pixel_index];
            
            // パレットから色を取得
            uint32_t color = get_palette_color(palette_index, gray_level);
            uint8_t r = (color >> 16) & 0xFF;
            uint8_t g = (color >> 8) & 0xFF;
            uint8_t b = color & 0xFF;
            
            // BGRの順で書き込み（BMPフォーマット）
            int buffer_index = x * 3;
            row_buffer[buffer_index] = b;
            row_buffer[buffer_index + 1] = g;
            row_buffer[buffer_index + 2] = r;
        }
        
        // 1行分書き込み
        if (fwrite(row_buffer, 1, row_size, file) != row_size) {
            ESP_LOGE(TAG, "❌ 画像データ書き込み失敗 (行: %d)", y);
            free(row_buffer);
            fclose(file);
            return false;
        }
    }
    
    // クリーンアップ
    free(row_buffer);
    fclose(file);
    
    _total_processed_images++;
    
    ESP_LOGI(TAG, "✅ パレット画像保存完了: %s", filename);
    return true;
}

// ========================================
// RGB565フレームをBMPファイルに保存にゃ
// ========================================
bool PixelArtProcessor::frame_to_bmp(camera_fb_t* fb, const char* filename) {
    if (!fb || !filename) {
        ESP_LOGE(TAG, "❌ 無効なパラメータ");
        return false;
    }
    
    ESP_LOGI(TAG, "💾 フレーム保存開始: %s", filename);
    
    FILE* file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE(TAG, "❌ ファイルオープン失敗: %s", filename);
        return false;
    }
    
    int width = fb->width;
    int height = fb->height;
    
    // BMPヘッダー書き込み
    if (write_bmp_header(file, width, height) != ESP_OK) {
        fclose(file);
        return false;
    }
    
    // RGB565データのポインタ
    uint16_t* rgb565_data = (uint16_t*)fb->buf;
    
    // 行サイズ計算
    int row_size = ((width * 3 + 3) / 4) * 4;
    uint8_t* row_buffer = (uint8_t*)malloc(row_size);
    if (!row_buffer) {
        ESP_LOGE(TAG, "❌ 行バッファ確保失敗");
        fclose(file);
        return false;
    }
    
    // 画像データ書き込み
    for (int y = height - 1; y >= 0; y--) {
        memset(row_buffer, 0, row_size);
        
        for (int x = 0; x < width; x++) {
            int pixel_index = y * width + x;
            uint16_t rgb565 = rgb565_data[pixel_index];
            
            // RGB565をRGB888に変換
            uint8_t r, g, b;
            rgb565_to_rgb888(rgb565, &r, &g, &b);
            
            // BGRの順で書き込み
            int buffer_index = x * 3;
            row_buffer[buffer_index] = b;
            row_buffer[buffer_index + 1] = g;
            row_buffer[buffer_index + 2] = r;
        }
        
        if (fwrite(row_buffer, 1, row_size, file) != row_size) {
            ESP_LOGE(TAG, "❌ 画像データ書き込み失敗");
            free(row_buffer);
            fclose(file);
            return false;
        }
    }
    
    free(row_buffer);
    fclose(file);
    
    ESP_LOGI(TAG, "✅ フレーム保存完了: %s", filename);
    return true;
}

// ========================================
// パレットインデックス有効性確認にゃ
// ========================================
bool PixelArtProcessor::is_valid_palette_index(int palette_index) const {
    return (palette_index >= 0 && palette_index < _palette_count);
}

// ========================================
// パレット色取得にゃ
// ========================================
uint32_t PixelArtProcessor::get_palette_color(int palette_index, int color_index) const {
    if (!is_valid_palette_index(palette_index) || 
        color_index < 0 || color_index >= COLORS_PER_PALETTE) {
        ESP_LOGW(TAG, "⚠️ 無効なパレット/色インデックス: palette=%d, color=%d", 
                 palette_index, color_index);
        return 0x000000;  // 黒を返す
    }
    
    return _color_palettes[palette_index][color_index];
}

// ========================================
// カスタムパレット設定にゃ
// ========================================
void PixelArtProcessor::set_custom_palette(int palette_index, const uint32_t colors[COLORS_PER_PALETTE]) {
    if (!is_valid_palette_index(palette_index) || !colors) {
        ESP_LOGE(TAG, "❌ 無効なパレット設定パラメータ");
        return;
    }
    
    // 注意: constキャストは通常避けるべきだが、カスタムパレット機能のため例外的に使用
    uint32_t* palette = const_cast<uint32_t*>(_color_palettes[palette_index]);
    memcpy(palette, colors, sizeof(uint32_t) * COLORS_PER_PALETTE);
    
    ESP_LOGI(TAG, "🎨 カスタムパレット設定完了: インデックス %d", palette_index);
}

// ========================================
// 統計情報表示にゃ
// ========================================
void PixelArtProcessor::print_stats() const {
    ESP_LOGI(TAG, "\n📊 === 画像処理統計情報 ===");
    ESP_LOGI(TAG, "🖼️ 総処理画像数: %lu", (unsigned long)_total_processed_images);
    ESP_LOGI(TAG, "📐 最終画像サイズ: %lu pixels", (unsigned long)_last_stats.total_pixels);
    ESP_LOGI(TAG, "⏱️ 最終処理時間: %lu ms", (unsigned long)_last_stats.processing_time_ms);
    ESP_LOGI(TAG, "💡 平均輝度: %lu", (unsigned long)_last_stats.avg_luminance);
    
    ESP_LOGI(TAG, "📈 グレースケールヒストグラム:");
    for (int i = 0; i < GRAY_LEVELS; i++) {
        ESP_LOGI(TAG, "  レベル%d: %lu pixels", i, (unsigned long)_last_stats.histogram[i]);
    }
    ESP_LOGI(TAG, "=============================\n");
}

// ========================================
// 統計情報リセットにゃ
// ========================================
void PixelArtProcessor::reset_stats() {
    memset(&_last_stats, 0, sizeof(image_stats_t));
    _total_processed_images = 0;
    ESP_LOGI(TAG, "🔄 統計情報リセット完了");
}

// ========================================
// パレットプレビュー生成にゃ
// ========================================
void PixelArtProcessor::preview_palette(int palette_index, uint8_t* preview_data, int preview_size) const {
    if (!is_valid_palette_index(palette_index) || !preview_data || preview_size <= 0) {
        ESP_LOGE(TAG, "❌ 無効なプレビューパラメータ");
        return;
    }
    
    // 各グレースケールレベルの代表色を順番に配置
    int colors_per_pixel = preview_size / GRAY_LEVELS;
    if (colors_per_pixel <= 0) colors_per_pixel = 1;
    
    for (int i = 0; i < preview_size && i < GRAY_LEVELS * colors_per_pixel; i++) {
        int gray_level = i / colors_per_pixel;
        if (gray_level >= GRAY_LEVELS) gray_level = GRAY_LEVELS - 1;
        
        uint32_t color = get_palette_color(palette_index, gray_level);
        preview_data[i] = (uint8_t)((color >> 16) & 0xFF);  // とりあえず赤成分のみ
    }
    
    ESP_LOGI(TAG, "🖼️ パレット%dのプレビュー生成完了", palette_index);
}

// ========================================
// 画像サイズ検証にゃ
// ========================================
bool PixelArtProcessor::validate_image_dimensions(int width, int height) const {
    if (width <= 0 || height <= 0) {
        ESP_LOGE(TAG, "❌ 無効な画像サイズ: %dx%d", width, height);
        return false;
    }
    
    if (width > 1024 || height > 1024) {
        ESP_LOGW(TAG, "⚠️ 大きな画像サイズ: %dx%d", width, height);
        return true;  // 警告だが処理は継続
    }
    
    return true;
}