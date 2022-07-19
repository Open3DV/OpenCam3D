import numpy as np
import cv2
import sys

def depthmap_status(img):
    mask = (img > 0.001)
    valid_img = img[mask]
    valid_img = valid_img.reshape(-1)
    depth_min = np.percentile(valid_img, 2)
    depth_max = np.percentile(valid_img, 98)
    return depth_max, depth_min, mask

def Loading_Depth_From_Tiff(depth_file):
    depth = cv2.imread(depth_file, -1)
    depth = np.float32(np.array(depth))

    return depth


def Loading_Params_From_Txt(params_file):
    params = np.loadtxt(params_file)
    camera_mtx = np.zeros((3, 3))
    camera_mtx[0, 0] = params[0]
    camera_mtx[0, 2] = params[2]
    camera_mtx[1, 1] = params[4]
    camera_mtx[1, 2] = params[5]
    camera_mtx[2, 2] = 1
    camera_dist = params[9:14]

    return camera_mtx, camera_dist

def depth2cloud_undistort(depth, color, camera_mtx, camera_dist):
    pointcloud = []

    for iy in range(1200):
        for ix in range(1920):
            # 先对图片去畸变
            ixiy = cv2.undistortPoints(np.float32(np.array([ix, iy])), camera_mtx, camera_dist)
            z = depth[iy, ix]
            r, g, b = color[iy, ix]
            if z > 0:
                x = ixiy[0, 0, 0] * z
                y = ixiy[0, 0, 1] * z
                pts = np.array([x, y, z, r, g, b])
                pointcloud.append(pts)

    pointcloud = np.array(pointcloud)


    return pointcloud

def depth2color(depth_tiff_path):
    img = cv2.imread(depth_tiff_path, -1)
    depth_max, depth_min, mask = depthmap_status(img)

    img[img>depth_max] = depth_max
    img[img<depth_min] = depth_min

    img_norm = (img-depth_min) / (depth_max - depth_min) *255
    img8 = np.array(img_norm, dtype=np.uint8)
    img_color = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
    img_color[~mask] = 0
    return img_color

def depth2cloud(depth_tiff_path, color_bmp_path, param_txt_path):
    camera_mtx, camera_dist = Loading_Params_From_Txt(param_txt_path)
    depth = Loading_Depth_From_Tiff(depth_tiff_path)
    if color_bmp_path == "0":
        color = depth2color(depth_tiff_path)
    else:
        color = cv2.imread(color_bmp_path)


    pointcloud = depth2cloud_undistort(depth, color, camera_mtx, camera_dist)

    output_xyz_path = depth_tiff_path[:-4]+'xyz'
    print(output_xyz_path)

    np.savetxt(output_xyz_path, pointcloud)


if __name__ == '__main__':
    argv_lenth = len(sys.argv)
    if argv_lenth == 1:
        print("使用示例：python depthmap2cloud.py ./depth.tiff param.txt ./color.bmp\n"
              "或：python depthmap2cloud.py ./depth.tiff param.txt")
        exit(0)

    input_tiff_path = sys.argv[1]
    input_param_path = sys.argv[2]

    if argv_lenth == 4:
        input_bmp_path = sys.argv[3]
        print("根据输入图片给点云上色")
    else:
        input_bmp_path = "0"
        print("生成点云伪彩色")

    print(input_tiff_path)

    depth2cloud(input_tiff_path, input_bmp_path, input_param_path)
