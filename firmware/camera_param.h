#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#define WRITE_PACKAGE_SIZE      256//1024
#define READ_PACKAGE_SIZE       256

struct CameraCalibParam
{
    float camera_intrinsic[9];
    float camera_distortion[5];
    float projector_intrinsic[9];
    float projector_distortion[5];
    float rotation_matrix[9];
    float translation_matrix[3];
};

#endif
