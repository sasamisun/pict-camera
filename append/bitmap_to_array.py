#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
美咲フォントビットマップをCの2次元配列に変換するスクリプト
ターゲット: ESP-IDF環境のSSD1306表示用

ビットマップ仕様:
- 1文字: 8x8ドット
- 配置: 94文字/行 × 8行 = 752文字
- 出力: C言語の2次元配列ヘッダーファイル
"""

import os
import sys
from PIL import Image
import numpy as np
from datetime import datetime

class MisakiFontConverter:
    def __init__(self, bitmap_path, output_path="misaki_font.h"):
        """
        コンストラクタ
        
        Args:
            bitmap_path (str): ビットマップファイルのパス
            output_path (str): 出力ヘッダーファイルのパス
        """
        self.bitmap_path = bitmap_path
        self.output_path = output_path
        
        # フォント仕様
        self.CHAR_WIDTH = 8      # 文字幅（ドット）
        self.CHAR_HEIGHT = 8     # 文字高（ドット）
        self.CHARS_PER_ROW = 94  # 1行あたりの文字数
        self.TOTAL_ROWS = 8      # 総行数
        self.TOTAL_CHARS = 752   # 総文字数
        
        self.font_data = []      # フォントデータ格納用

    def load_bitmap(self):
        """
        ビットマップファイルを読み込み、グレースケールに変換
        """
        try:
            print(f"ビットマップファイルを読み込み中: {self.bitmap_path}")
            
            # 画像を開いてグレースケールに変換
            img = Image.open(self.bitmap_path).convert('L')
            self.bitmap = np.array(img)
            
            print(f"画像サイズ: {img.size} (幅x高)")
            print(f"期待サイズ: {self.CHARS_PER_ROW * self.CHAR_WIDTH}x{self.TOTAL_ROWS * self.CHAR_HEIGHT}")
            
            # サイズチェック
            expected_width = self.CHARS_PER_ROW * self.CHAR_WIDTH
            expected_height = self.TOTAL_ROWS * self.CHAR_HEIGHT
            
            if img.size != (expected_width, expected_height):
                print(f"⚠️ 警告: 画像サイズが期待値と異なります")
                print(f"実際: {img.size}, 期待: ({expected_width}, {expected_height})")
            
            return True
            
        except Exception as e:
            print(f"❌ エラー: ビットマップファイルの読み込みに失敗: {e}")
            return False

    def extract_char_bitmap(self, char_row, char_col):
        """
        指定された位置の文字ビットマップを抽出
        
        Args:
            char_row (int): 文字の行位置 (0-7)
            char_col (int): 文字の列位置 (0-93)
            
        Returns:
            numpy.ndarray: 8x8の文字ビットマップ
        """
        # ピクセル座標を計算
        start_x = char_col * self.CHAR_WIDTH
        start_y = char_row * self.CHAR_HEIGHT
        end_x = start_x + self.CHAR_WIDTH
        end_y = start_y + self.CHAR_HEIGHT
        
        # 文字領域を抽出
        char_bitmap = self.bitmap[start_y:end_y, start_x:end_x]
        
        return char_bitmap

    def bitmap_to_bytes(self, char_bitmap):
        """
        8x8ビットマップを8バイトのデータに変換
        各バイトは縦8ドットを表現（SSD1306形式）
        
        Args:
            char_bitmap (numpy.ndarray): 8x8グレースケールビットマップ
            
        Returns:
            list: 8バイトのリスト
        """
        bytes_data = []
        
        # 閾値でモノクロ化（128未満を黒=1、128以上を白=0とする）
        threshold = 128
        
        # 各列（x座標）について処理
        for x in range(self.CHAR_WIDTH):
            byte_value = 0
            
            # 各行（y座標）について処理（上位ビットから）
            for y in range(self.CHAR_HEIGHT):
                if char_bitmap[y, x] < threshold:  # 黒ピクセル
                    byte_value |= (1 << y)  # 対応するビットを立てる
            
            bytes_data.append(byte_value)
        
        return bytes_data

    def convert_all_chars(self):
        """
        全文字のビットマップを変換してデータ配列に格納
        """
        print("文字データを変換中...")
        
        char_count = 0
        
        # 全行・全列を処理
        for row in range(self.TOTAL_ROWS):
            for col in range(self.CHARS_PER_ROW):
                # 文字ビットマップを抽出
                char_bitmap = self.extract_char_bitmap(row, col)
                
                # バイトデータに変換
                char_bytes = self.bitmap_to_bytes(char_bitmap)
                
                # データを格納
                self.font_data.append(char_bytes)
                
                char_count += 1
                
                # 進捗表示
                if char_count % 100 == 0:
                    print(f"  {char_count}/{self.TOTAL_CHARS} 文字完了")
        
        print(f"✅ 全 {char_count} 文字の変換完了")

    def generate_header_file(self):
        """
        Cヘッダーファイルを生成
        """
        print(f"ヘッダーファイルを生成中: {self.output_path}")
        
        try:
            with open(self.output_path, 'w', encoding='utf-8') as f:
                # ヘッダー部分
                f.write("/**\n")
                f.write(" * 美咲フォント データ配列\n")
                f.write(" * ESP-IDF + SSD1306用\n")
                f.write(" * \n")
                f.write(f" * 生成日時: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f" * 総文字数: {self.TOTAL_CHARS}\n")
                f.write(" * 文字サイズ: 8x8ドット\n")
                f.write(" * データ形式: 各文字8バイト（縦方向ビットマップ）\n")
                f.write(" */\n\n")
                
                # インクルードガード
                f.write("#ifndef MISAKI_FONT_H\n")
                f.write("#define MISAKI_FONT_H\n\n")
                
                f.write("#include <stdint.h>\n")
                f.write("#include <stddef.h>\n\n")
                
                # 定数定義
                f.write("// フォント仕様定数\n")
                f.write(f"#define MISAKI_CHAR_WIDTH  {self.CHAR_WIDTH}\n")
                f.write(f"#define MISAKI_CHAR_HEIGHT {self.CHAR_HEIGHT}\n")
                f.write(f"#define MISAKI_TOTAL_CHARS {self.TOTAL_CHARS}\n")
                f.write(f"#define MISAKI_CHARS_PER_ROW {self.CHARS_PER_ROW}\n")
                f.write(f"#define MISAKI_TOTAL_ROWS {self.TOTAL_ROWS}\n\n")
                
                # 文字データ型定義
                f.write("// 文字データ構造体\n")
                f.write("typedef struct {\n")
                f.write("    uint8_t bitmap[8];  // 8x8ビットマップデータ（縦方向）\n")
                f.write("} misaki_char_t;\n\n")
                
                # フォントデータ配列宣言
                f.write("// フォントデータ配列（FLASHメモリ配置）\n")
                f.write("#ifdef __cplusplus\n")
                f.write("extern \"C\" {\n")
                f.write("#endif\n\n")
                
                f.write("extern const misaki_char_t misaki_font[MISAKI_TOTAL_CHARS];\n\n")
                
                # 配列データ定義開始
                f.write("// フォントデータ実体\n")
                f.write("const misaki_char_t misaki_font[MISAKI_TOTAL_CHARS] = {\n")
                
                # 各文字のデータを出力
                for i, char_data in enumerate(self.font_data):
                    # コメント（文字位置）
                    row = i // self.CHARS_PER_ROW
                    col = i % self.CHARS_PER_ROW
                    f.write(f"    /* {i:3d} (行{row}, 列{col:2d}) */ {{")
                    
                    # バイトデータを16進数で出力
                    hex_data = [f"0x{byte:02X}" for byte in char_data]
                    f.write(", ".join(hex_data))
                    
                    # 最後の文字以外はカンマを付ける
                    if i < len(self.font_data) - 1:
                        f.write("},\n")
                    else:
                        f.write("}\n")
                
                f.write("};\n\n")
                
                # 関数プロトタイプ
                f.write("// ユーティリティ関数\n")
                f.write("const misaki_char_t* misaki_get_char(uint16_t char_index);\n")
                f.write("uint16_t misaki_get_char_count(void);\n\n")
                
                f.write("#ifdef __cplusplus\n")
                f.write("}\n")
                f.write("#endif\n\n")
                
                f.write("#endif // MISAKI_FONT_H\n")
            
            print("✅ ヘッダーファイル生成完了")
            return True
            
        except Exception as e:
            print(f"❌ エラー: ヘッダーファイル生成に失敗: {e}")
            return False

    def generate_sample_c_file(self):
        """
        サンプル使用方法を示すCファイルを生成
        """
        sample_path = "misaki_font_sample.c"
        print(f"サンプルファイルを生成中: {sample_path}")
        
        try:
            with open(sample_path, 'w', encoding='utf-8') as f:
                f.write("/**\n")
                f.write(" * 美咲フォント使用サンプル\n")
                f.write(" * ESP-IDF + SSD1306用\n")
                f.write(" */\n\n")
                
                f.write("#include \"misaki_font.h\"\n")
                f.write("#include <stdio.h>\n\n")
                
                f.write("// ユーティリティ関数の実装\n")
                f.write("const misaki_char_t* misaki_get_char(uint16_t char_index) {\n")
                f.write("    if (char_index >= MISAKI_TOTAL_CHARS) {\n")
                f.write("        return NULL;  // 範囲外\n")
                f.write("    }\n")
                f.write("    return &misaki_font[char_index];\n")
                f.write("}\n\n")
                
                f.write("uint16_t misaki_get_char_count(void) {\n")
                f.write("    return MISAKI_TOTAL_CHARS;\n")
                f.write("}\n\n")
                
                f.write("// SSD1306への文字描画サンプル関数\n")
                f.write("void draw_misaki_char(int x, int y, uint16_t char_index) {\n")
                f.write("    const misaki_char_t* char_data = misaki_get_char(char_index);\n")
                f.write("    if (char_data == NULL) return;\n\n")
                f.write("    // 8x8ドットを描画\n")
                f.write("    for (int col = 0; col < 8; col++) {\n")
                f.write("        uint8_t column_data = char_data->bitmap[col];\n")
                f.write("        for (int row = 0; row < 8; row++) {\n")
                f.write("            if (column_data & (1 << row)) {\n")
                f.write("                // SSD1306のピクセル描画関数を呼び出し\n")
                f.write("                // ssd1306_draw_pixel(x + col, y + row, 1);\n")
                f.write("            }\n")
                f.write("        }\n")
                f.write("    }\n")
                f.write("}\n\n")
                
                f.write("// 使用例\n")
                f.write("void example_usage(void) {\n")
                f.write("    printf(\"美咲フォント総文字数: %d\\n\", misaki_get_char_count());\n")
                f.write("    \n")
                f.write("    // 0番目の文字を座標(10, 20)に描画\n")
                f.write("    draw_misaki_char(10, 20, 0);\n")
                f.write("}\n")
            
            print("✅ サンプルファイル生成完了")
            return True
            
        except Exception as e:
            print(f"❌ エラー: サンプルファイル生成に失敗: {e}")
            return False

    def run(self):
        """
        変換処理を実行
        """
        print("🚀 美咲フォント変換処理を開始")
        print("=" * 50)
        
        # ビットマップ読み込み
        if not self.load_bitmap():
            return False
        
        # 全文字変換
        self.convert_all_chars()
        
        # ヘッダーファイル生成
        if not self.generate_header_file():
            return False
        
        # サンプルファイル生成
        if not self.generate_sample_c_file():
            return False
        
        print("=" * 50)
        print("🎉 変換処理完了！")
        print(f"📄 生成ファイル:")
        print(f"  - {self.output_path}")
        print(f"  - misaki_font_sample.c")
        print(f"📊 統計:")
        print(f"  - 総文字数: {len(self.font_data)}")
        print(f"  - データサイズ: {len(self.font_data) * 8} バイト")
        
        return True


def main():
    """
    メイン処理
    """
    print("美咲フォント → C配列変換ツール")
    print("=" * 40)
    
    # コマンドライン引数チェック
    if len(sys.argv) < 2:
        print("使用方法: python bitmap_to_array.py <ビットマップファイルパス> [出力ファイル名]")
        print("例: python bitmap_to_array.py misaki_font.bmp")
        print("例: python bitmap_to_array.py misaki_font.png my_font.h")
        return
    
    bitmap_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else "misaki_font.h"
    
    # ファイル存在チェック
    if not os.path.exists(bitmap_path):
        print(f"❌ エラー: ビットマップファイルが見つかりません: {bitmap_path}")
        return
    
    # 変換処理実行
    converter = MisakiFontConverter(bitmap_path, output_path)
    success = converter.run()
    
    if success:
        print("\n✅ すべての処理が正常に完了しました！")
    else:
        print("\n❌ 処理中にエラーが発生しました")


if __name__ == "__main__":
    main()