#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

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
