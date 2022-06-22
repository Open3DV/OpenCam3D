import cv2
import numpy as np
import sys

def depthmap_status(img):
    mask = (img > 0.001)
    valid_img = img[mask]
    valid_img = valid_img.reshape(-1)
    depth_min = np.percentile(valid_img, 2)
    depth_max = np.percentile(valid_img, 98)
    return depth_max, depth_min, mask
   
def depth2color(depth_tiff_path, color_map_path):
    img = cv2.imread(depth_tiff_path, -1)
    depth_max, depth_min, mask = depthmap_status(img)

    img[img>depth_max] = depth_max
    img[img<depth_min] = depth_min

    img_norm = (img-depth_min) / (depth_max - depth_min) *255
    img8 = np.array(img_norm, dtype=np.uint8)
    img_color = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
    img_color[~mask] = 0
    cv2.imwrite(color_map_path, img_color)

if __name__ == '__main__':
    input_path = sys.argv[1]
    print(input_path)
    output_path = input_path[:-4]+'bmp'
    print(output_path)

    depth2color(input_path, output_path)

