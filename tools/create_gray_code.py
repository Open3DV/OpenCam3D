import csv
import cv2
import numpy as np
import math
import os
import sys

os.chdir(sys.path[0])# 使用文件所在目录


if not os.path.exists('patterns_gray'):
    os.makedirs('patterns_gray')

n = 0 
 
img_x = np.zeros([720, 1280, 3], dtype=np.uint8) 
 
periods = [1,2,4,8,16,32,64,128,256]

for period in periods:  
    for y_ in range(720):
        flag = 255
        inv_flag = 0 
        bit_width = 1280/(period*2)
        invert_num = 0
        for x_ in range(1280):
            if 0 == x_%bit_width:
                invert_num = invert_num+1
                # print('invert_num: %02d'%invert_num)
                if 0 == flag:
                    flag = 255
                else:
                    flag = 0
                if invert_num >4:
                    invert_num = invert_num -4
                if 2 < invert_num:
                    inv_flag = 255
                else:
                    inv_flag = 0 
            if 255 == inv_flag:
                img_x[y_, x_, :] = 255 - flag
            else: 
                img_x[y_, x_, :] = flag
    print('patterns_gray\\gray_%02d_x.bmp'%n)
    cv2.imwrite('patterns_gray\\gray_%02d_x.bmp'%n, img_x)
    n = n+1

 




