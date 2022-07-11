#include "camera.h"


Camera::Camera()
{
    camera_opened_state_ = false;  
  
    image_width_= 0;
    image_height_= 0;

    
}

Camera::~Camera()
{

}


bool Camera::getImageSize(int &width,int &height)
{
    width = image_width_;
    height = image_height_;
}

bool Camera::openCamera()
{
    return false;
} 

bool Camera::closeCamera()
{

    return false;
}
  
bool Camera::switchToInternalTriggerMode()
{

    return false;
}

bool Camera::switchToExternalTriggerMode()
{

    return false;
}

 