#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
画像ファイルをCの配列に変換するスクリプト
ターゲット: ESP-IDF環境のSSD1306/その他ディスプレイ用

出力形式:
- 幅、高さ、データの順で配列を生成
- モノクロ（1bit）またはグレースケール（8bit）対応
- SSD1306縦方向ビットマップ形式も選択可能
"""

import os
import sys
from PIL import Image
import numpy as np
from datetime import datetime
import argparse

class ImageToArrayConverter:
    def __init__(self, image_path, output_path, array_name="image_data"):
        """
        コンストラクタ
        
        Args:
            image_path (str): 画像ファイルのパス
            output_path (str): 出力ヘッダーファイルのパス
            array_name (str): 配列名
        """
        self.image_path = image_path
        self.output_path = output_path
        self.array_name = array_name
        
        # 画像データ
        self.width = 0
        self.height = 0
        self.image_data = []
        self.format_type = "mono"  # mono, gray, ssd1306
        
    def load_image(self):
        """
        画像ファイルを読み込み
        """
        try:
            print(f"画像ファイルを読み込み中: {self.image_path}")
            
            # 画像を開く
            img = Image.open(self.image_path)
            self.width, self.height = img.size
            
            print(f"画像サイズ: {self.width} x {self.height}")
            print(f"画像モード: {img.mode}")
            
            # グレースケールに変換
            if img.mode != 'L':
                print("グレースケールに変換中...")
                img = img.convert('L')
            
            self.image_array = np.array(img)
            
            return True
            
        except Exception as e:
            print(f"❌ エラー: 画像ファイルの読み込みに失敗: {e}")
            return False
    
    def convert_to_mono(self, threshold=128):
        """
        モノクロ（1bit per pixel）に変換
        8ピクセルを1バイトにパック
        
        Args:
            threshold (int): 二値化の閾値（0-255）
        """
        print(f"モノクロ変換中（閾値: {threshold}）...")
        
        self.format_type = "mono"
        self.image_data = []
        
        # 幅を8の倍数に調整（必要に応じてパディング）
        padded_width = ((self.width + 7) // 8) * 8
        bytes_per_row = padded_width // 8
        
        for y in range(self.height):
            row_data = []
            for byte_idx in range(bytes_per_row):
                byte_value = 0
                for bit_idx in range(8):
                    x = byte_idx * 8 + bit_idx
                    if x < self.width:
                        # 閾値で二値化（黒=1、白=0）
                        if self.image_array[y, x] < threshold:
                            byte_value |= (1 << (7 - bit_idx))  # MSBから格納
                    # パディング部分は0のまま
                
                row_data.append(byte_value)
            
            self.image_data.extend(row_data)
        
        self.bytes_per_row = bytes_per_row
        print(f"変換完了: {self.width}x{self.height} → {len(self.image_data)} バイト")
    
    def convert_to_gray(self):
        """
        グレースケール（8bit per pixel）に変換
        """
        print("グレースケール変換中...")
        
        self.format_type = "gray"
        self.image_data = []
        
        # 行ごとに処理
        for y in range(self.height):
            for x in range(self.width):
                pixel_value = self.image_array[y, x]
                self.image_data.append(pixel_value)
        
        print(f"変換完了: {self.width}x{self.height} → {len(self.image_data)} バイト")
    
    def convert_to_ssd1306(self, threshold=128):
        """
        SSD1306縦方向ビットマップ形式に変換
        8行を1バイトにパック（縦方向）
        
        Args:
            threshold (int): 二値化の閾値（0-255）
        """
        print(f"SSD1306形式変換中（閾値: {threshold}）...")
        
        self.format_type = "ssd1306"
        self.image_data = []
        
        # 高さを8の倍数に調整
        padded_height = ((self.height + 7) // 8) * 8
        pages = padded_height // 8
        
        # ページごと（8行ずつ）に処理
        for page in range(pages):
            for x in range(self.width):
                byte_value = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if y < self.height:
                        # 閾値で二値化（黒=1、白=0）
                        if self.image_array[y, x] < threshold:
                            byte_value |= (1 << bit)  # LSBから格納
                    # パディング部分は0のまま
                
                self.image_data.append(byte_value)
        
        self.pages = pages
        print(f"変換完了: {self.width}x{self.height} → {len(self.image_data)} バイト ({pages} ページ)")
    
    def generate_header_file(self):
        """
        Cヘッダーファイルを生成
        """
        print(f"ヘッダーファイルを生成中: {self.output_path}")
        
        try:
            with open(self.output_path, 'w', encoding='utf-8') as f:
                # ヘッダー部分
                f.write("/**\n")
                f.write(" * 画像データ配列\n")
                f.write(f" * 元ファイル: {os.path.basename(self.image_path)}\n")
                f.write(" * \n")
                f.write(f" * 生成日時: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f" * 画像サイズ: {self.width} x {self.height}\n")
                f.write(f" * データ形式: {self.format_type}\n")
                f.write(f" * データサイズ: {len(self.image_data)} バイト\n")
                f.write(" */\n\n")
                
                # インクルードガード
                guard_name = f"{self.array_name.upper()}_H"
                f.write(f"#ifndef {guard_name}\n")
                f.write(f"#define {guard_name}\n\n")
                
                f.write("#include <stdint.h>\n")
                f.write("#include <stddef.h>\n\n")
                
                # 定数定義
                f.write("// 画像仕様定数\n")
                f.write(f"#define {self.array_name.upper()}_WIDTH  {self.width}\n")
                f.write(f"#define {self.array_name.upper()}_HEIGHT {self.height}\n")
                f.write(f"#define {self.array_name.upper()}_SIZE   {len(self.image_data)}\n")
                
                if self.format_type == "mono":
                    f.write(f"#define {self.array_name.upper()}_BYTES_PER_ROW {self.bytes_per_row}\n")
                elif self.format_type == "ssd1306":
                    f.write(f"#define {self.array_name.upper()}_PAGES {self.pages}\n")
                
                f.write("\n")
                
                # C++対応
                f.write("#ifdef __cplusplus\n")
                f.write("extern \"C\" {\n")
                f.write("#endif\n\n")
                                
                # 配列定義
                f.write("// 画像データ実体\n")
                f.write(f"static const uint8_t {self.array_name}[{len(self.image_data)}] = {{\n")
                
                # データを16進数で出力（16バイトずつ改行）
                for i, byte_value in enumerate(self.image_data):
                    if i % 16 == 0:
                        f.write("    ")
                    
                    f.write(f"0x{byte_value:02X}")
                    
                    if i < len(self.image_data) - 1:
                        f.write(", ")
                    
                    if i % 16 == 15 or i == len(self.image_data) - 1:
                        f.write("\n")
                
                f.write("};\n\n")
                
                f.write("#ifdef __cplusplus\n")
                f.write("}\n")
                f.write("#endif\n\n")
                
                f.write(f"#endif // {guard_name}\n")
            
            print("✅ ヘッダーファイル生成完了")
            return True
            
        except Exception as e:
            print(f"❌ エラー: ヘッダーファイル生成に失敗: {e}")
            return False
    
    def generate_sample_c_file(self):
        """
        サンプル使用方法を示すCファイルを生成
        """
        sample_path = f"{self.array_name}_sample.c"
        print(f"サンプルファイルを生成中: {sample_path}")
        
        try:
            with open(sample_path, 'w', encoding='utf-8') as f:
                f.write("/**\n")
                f.write(f" * {self.array_name} 使用サンプル\n")
                f.write(" */\n\n")
                
                f.write(f"#include \"{os.path.basename(self.output_path)}\"\n")
                f.write("#include <stdio.h>\n\n")
                
                if self.format_type == "mono":
                    # モノクロ画像の描画例
                    f.write("// モノクロ画像描画サンプル\n")
                    f.write(f"void draw_{self.array_name}(int x, int y) {{\n")
                    f.write("    for (int row = 0; row < %s_HEIGHT; row++) {\n" % self.array_name.upper())
                    f.write("        for (int byte_col = 0; byte_col < %s_BYTES_PER_ROW; byte_col++) {\n" % self.array_name.upper())
                    f.write("            int data_index = row * %s_BYTES_PER_ROW + byte_col;\n" % self.array_name.upper())
                    f.write(f"            uint8_t byte_data = {self.array_name}[data_index];\n")
                    f.write("            \n")
                    f.write("            for (int bit = 0; bit < 8; bit++) {\n")
                    f.write("                int pixel_x = x + byte_col * 8 + bit;\n")
                    f.write("                int pixel_y = y + row;\n")
                    f.write("                \n")
                    f.write("                if (byte_data & (1 << (7 - bit))) {\n")
                    f.write("                    // 黒ピクセルを描画\n")
                    f.write("                    // display_draw_pixel(pixel_x, pixel_y, 1);\n")
                    f.write("                }\n")
                    f.write("            }\n")
                    f.write("        }\n")
                    f.write("    }\n")
                    f.write("}\n\n")
                
                elif self.format_type == "gray":
                    # グレースケール画像の描画例
                    f.write("// グレースケール画像描画サンプル\n")
                    f.write(f"void draw_{self.array_name}(int x, int y) {{\n")
                    f.write("    for (int row = 0; row < %s_HEIGHT; row++) {\n" % self.array_name.upper())
                    f.write("        for (int col = 0; col < %s_WIDTH; col++) {\n" % self.array_name.upper())
                    f.write("            int data_index = row * %s_WIDTH + col;\n" % self.array_name.upper())
                    f.write(f"            uint8_t gray_value = {self.array_name}[data_index];\n")
                    f.write("            \n")
                    f.write("            // グレースケール値でピクセルを描画\n")
                    f.write("            // display_draw_pixel_gray(x + col, y + row, gray_value);\n")
                    f.write("        }\n")
                    f.write("    }\n")
                    f.write("}\n\n")
                
                elif self.format_type == "ssd1306":
                    # SSD1306形式の描画例
                    f.write("// SSD1306画像描画サンプル\n")
                    f.write(f"void draw_{self.array_name}_ssd1306(int x, int y) {{\n")
                    f.write("    for (int page = 0; page < %s_PAGES; page++) {\n" % self.array_name.upper())
                    f.write("        for (int col = 0; col < %s_WIDTH; col++) {\n" % self.array_name.upper())
                    f.write("            int data_index = page * %s_WIDTH + col;\n" % self.array_name.upper())
                    f.write(f"            uint8_t column_data = {self.array_name}[data_index];\n")
                    f.write("            \n")
                    f.write("            for (int bit = 0; bit < 8; bit++) {\n")
                    f.write("                int pixel_x = x + col;\n")
                    f.write("                int pixel_y = y + page * 8 + bit;\n")
                    f.write("                \n")
                    f.write("                if (column_data & (1 << bit)) {\n")
                    f.write("                    // ピクセルを描画\n")
                    f.write("                    // ssd1306_draw_pixel(pixel_x, pixel_y, 1);\n")
                    f.write("                }\n")
                    f.write("            }\n")
                    f.write("        }\n")
                    f.write("    }\n")
                    f.write("}\n\n")
                
                # 使用例
                f.write("// 使用例\n")
                f.write("void example_usage(void) {\n")
                f.write(f"    printf(\"画像サイズ: %dx%d\\n\", {self.array_name.upper()}_WIDTH, {self.array_name.upper()}_HEIGHT);\n")
                f.write(f"    printf(\"データサイズ: %d バイト\\n\", {self.array_name.upper()}_SIZE);\n")
                f.write("    \n")
                f.write("    // 画像を座標(0, 0)に描画\n")
                f.write(f"    draw_{self.array_name}(0, 0);\n")
                f.write("}\n")
            
            print("✅ サンプルファイル生成完了")
            return True
            
        except Exception as e:
            print(f"❌ エラー: サンプルファイル生成に失敗: {e}")
            return False
    
    def run(self, format_type="mono", threshold=128):
        """
        変換処理を実行
        
        Args:
            format_type (str): 出力形式（mono, gray, ssd1306）
            threshold (int): 二値化の閾値（mono, ssd1306の場合）
        """
        print("🚀 画像変換処理を開始")
        print("=" * 50)
        
        # 画像読み込み
        if not self.load_image():
            return False
        
        # 形式に応じて変換
        if format_type == "mono":
            self.convert_to_mono(threshold)
        elif format_type == "gray":
            self.convert_to_gray()
        elif format_type == "ssd1306":
            self.convert_to_ssd1306(threshold)
        else:
            print(f"❌ エラー: 未対応の形式: {format_type}")
            return False
        
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
        print(f"  - {self.array_name}_sample.c")
        print(f"📊 統計:")
        print(f"  - 画像サイズ: {self.width} x {self.height}")
        print(f"  - データサイズ: {len(self.image_data)} バイト")
        print(f"  - 形式: {self.format_type}")
        
        return True


def main():
    """
    メイン処理
    """
    parser = argparse.ArgumentParser(description="画像ファイルをC配列に変換")
    parser.add_argument("image_path", help="入力画像ファイルのパス")
    parser.add_argument("-o", "--output", default="image_data.h", help="出力ヘッダーファイル名")
    parser.add_argument("-n", "--name", default="image_data", help="配列名")
    parser.add_argument("-f", "--format", choices=["mono", "gray", "ssd1306"], default="mono", 
                       help="出力形式 (mono: モノクロ, gray: グレースケール, ssd1306: SSD1306形式)")
    parser.add_argument("-t", "--threshold", type=int, default=128, 
                       help="二値化の閾値 (0-255, mono/ssd1306形式の場合)")
    
    args = parser.parse_args()
    
    print("画像 → C配列変換ツール")
    print("=" * 40)
    
    # ファイル存在チェック
    if not os.path.exists(args.image_path):
        print(f"❌ エラー: 画像ファイルが見つかりません: {args.image_path}")
        return
    
    # 変換処理実行
    converter = ImageToArrayConverter(args.image_path, args.output, args.name)
    success = converter.run(args.format, args.threshold)
    
    if success:
        print("\n✅ すべての処理が正常に完了しました！")
    else:
        print("\n❌ 処理中にエラーが発生しました")


if __name__ == "__main__":
    main()