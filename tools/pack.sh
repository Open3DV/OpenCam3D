#!/bin/sh
exe="open_cam3d_gui" #需要发布的程序名称
pwd="/home/guanjin/dexforce/code/OpenCam3D/gui/build/bin" #所建文件夹的路径
files=$(ldd $exe | awk '{if (match($3,"/")){ printf("%s "),$3 } }')
cp $files $pwd
