import numpy as np
from PIL import Image

# 讀取txt文件中的設定
def read_txt_file(file_path):
    w = h = bri = None
    array_data = []
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        # 讀取 w, h, bri
        for line in lines:
            if line.startswith('w:'):
                w = int(line.split(":")[1].strip())
            elif line.startswith('h:'):
                h = int(line.split(":")[1].strip())
            elif line.startswith('bri:'):
                bri = int(line.split(":")[1].strip())
            elif line.startswith('array:'):
                # 開始讀取 array 部分
                continue
            else:
                # 讀取每行的 RGB 數據並轉換為數字
                if line.strip():  # 忽略空行
                    array_data.extend([int(x) for x in line.split()])
    
    return w, h, bri, array_data

# 生成圖片
def generate_image(w, h, bri, array_data, output_filename="output_image.png"):
    # 1280 x 720 的 array 需要 reshape 成 (h, w, 3) 來表示 RGB
    array_data = np.array(array_data).reshape((h, w, 3))
    
    # 創建空白圖片 (h, w, 3)，並根據 array_data 填充 RGB 值
    img_data = np.zeros((h, w, 3), dtype=np.uint8)

    for i in range(h):
        for j in range(w):
            # 根據每個像素的 RGB 值, 用 bri 來縮放
            r, g, b = array_data[i, j]
            img_data[i, j] = [min(255, bri * r), min(255, bri * g), min(255, bri * b)]
    
    # 使用 PIL 生成圖片
    img = Image.fromarray(img_data)
    img.save(output_filename)

# 主程序，先讀取 txt 設定並生成圖片
def main():
    # 讀取 1.txt 配置文件
    w, h, bri, array_data = read_txt_file("pattern_condi.txt")

    # 生成 1.png 圖像
    generate_image(w, h, bri, array_data, output_filename="flicker.png")

if __name__ == "__main__":
    main()

