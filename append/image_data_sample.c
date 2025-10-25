/**
 * image_data 使用サンプル
 */

#include "image_data.h"
#include <stdio.h>

// モノクロ画像描画サンプル
void draw_image_data(int x, int y) {
    for (int row = 0; row < IMAGE_DATA_HEIGHT; row++) {
        for (int byte_col = 0; byte_col < IMAGE_DATA_BYTES_PER_ROW; byte_col++) {
            int data_index = row * IMAGE_DATA_BYTES_PER_ROW + byte_col;
            uint8_t byte_data = image_data[data_index];
            
            for (int bit = 0; bit < 8; bit++) {
                int pixel_x = x + byte_col * 8 + bit;
                int pixel_y = y + row;
                
                if (byte_data & (1 << (7 - bit))) {
                    // 黒ピクセルを描画
                    // display_draw_pixel(pixel_x, pixel_y, 1);
                }
            }
        }
    }
}

// 使用例
void example_usage(void) {
    printf("画像サイズ: %dx%d\n", IMAGE_DATA_WIDTH, IMAGE_DATA_HEIGHT);
    printf("データサイズ: %d バイト\n", IMAGE_DATA_SIZE);
    
    // 画像を座標(0, 0)に描画
    draw_image_data(0, 0);
}
