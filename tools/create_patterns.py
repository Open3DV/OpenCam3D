import csv
import cv2
import numpy as np
import math
import os
import sys

os.chdir(sys.path[0])# 使用文件所在目录
PI = 3.1415926
filename = "pattern.csv"

if not os.path.exists('patterns'):os.makedirs('patterns')

with open(filename) as f:
    reader = csv.reader(f)
    header_row = next(reader)

    n = 0
    gray_offset = 0.5

    for row in reader:
        img_x = np.zeros([720, 1280, 3], dtype=np.uint8)
        img_y = np.zeros([720, 1280, 3], dtype=np.uint8)


        direct = row[2]
        period = float(row[3])
        step = int(row[4])
        phase = step*float(row[5])/360

        print('phase:%d'% phase)
        print('step:%d'% step)

        if "vertical" == direct:
            for y_ in range(720):
                for x_ in range(1280):
                    x = x_ + 0.5
                    y = y_ + 0.5
                    # p0 = -math.cos(x / period * PI * 2 + phase * 2*PI / step)
                    # p1 = -math.cos((x + 1) / period * PI * 2 + phase * 2*PI / step)
                    # img_x[y, x, :] = (p1 - p0) / (1 / period * PI * 2) * 120 + 127.0 + gray_offset

                    img_x[y_, x_, :] = math.sin(x / period * PI * 2 + phase * 2*PI / step) * 128 + 127.0 + gray_offset

                    # if 4 == step:
                    #     img_x[y_, x_, :] = math.sin(x / period * PI * 2 + phase * PI / 2) * 120 + 127.0 + gray_offset
                    # elif 6 == step:
                    #     img_x[y_, x_, :] = math.sin(x / period * PI * 2 + phase * PI / 3) * 120 + 127.0 + gray_offset



            cv2.imwrite('patterns/%02d_x.bmp' % n, img_x)
            n += 1
            print(n)
        elif "horizontal" == direct:
            for y_ in range(720):
                for x_ in range(1280):
                    x = x_ + 0.5
                    y = y_ + 0.5
                    # p0 = -math.cos(y / period * PI * 2 + phase * 2*PI / step)
                    # p1 = -math.cos((y + 1) / period * PI * 2 + phase * 2*PI / step)
                    # img_y[y, x, :] = (p1 - p0) / (1 / period * PI * 2) * 120 + 127.0 + gray_offset

                    img_y[y_, x_, :] = math.sin(y / period * PI * 2 + phase * 2*PI / step) * 128 + 127.0 + gray_offset
                    # if 4 == step:
                    #     img_y[y_, x_, :] = math.sin(y / period * PI * 2 + phase * PI / 2) * 120 + 127.0 + gray_offset
                    # elif 6 == step:
                    #     img_y[y_, x_, :] = math.sin(y / period * PI * 2 + phase * PI / 3) * 120 + 127.0 + gray_offset



            cv2.imwrite('patterns/%02d_y.bmp' % n, img_y)
            n += 1
            print(n)

    for pixel in [255, 200, 150, 100, 50, 0]:
            img_p = np.zeros([720, 1280, 3], dtype=np.uint8)
            for y_ in range(720):
                for x_ in range(1280):
                    img_p[y_, x_, :] = pixel
            cv2.imwrite('patterns/%02d_p.bmp' % n, img_p)
            n += 1
            print(n)






