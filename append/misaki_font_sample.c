/**
 * 美咲フォント使用サンプル
 * ESP-IDF + SSD1306用
 */

#include "misaki_font.h"
#include <stdio.h>

// ユーティリティ関数の実装
const misaki_char_t* misaki_get_char(uint16_t char_index) {
    if (char_index >= MISAKI_TOTAL_CHARS) {
        return NULL;  // 範囲外
    }
    return &misaki_font[char_index];
}

uint16_t misaki_get_char_count(void) {
    return MISAKI_TOTAL_CHARS;
}

                // SSD1306への文字描画サンプル関数
void draw_misaki_char(int x, int y, uint16_t char_index) {
    const misaki_char_t* char_data = misaki_get_char(char_index);
    if (char_data == NULL) return;

    // 8x8ドットを描画（上の行から順に処理）
    for (int col = 0; col < 8; col++) {
        uint8_t column_data = char_data->bitmap[col];
        for (int row = 0; row < 8; row++) {
            // 上の行（row=0）が最上位ビット（bit7）
            if (column_data & (1 << (7 - row))) {
                // SSD1306のピクセル描画関数を呼び出し
                // ssd1306_draw_pixel(x + col, y + row, 1);
            }
        }
    }
}

// 使用例
void example_usage(void) {
    printf("美咲フォント総文字数: %d\n", misaki_get_char_count());
    
    // 0番目の文字を座標(10, 20)に描画
    draw_misaki_char(10, 20, 0);
}
