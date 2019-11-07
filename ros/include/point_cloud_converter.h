#ifndef POINT_CLOUD_CONVERTER_H
#define POINT_CLOUD_CONVERTER_H

#include <sensor_msgs/PointCloud2.h>
#include "./api.h"

class PointCloudConverter
{
    public:
        void convertToPointCloud(HPS3D_HandleTypeDef *handle, sensor_msgs::PointCloud2 *outputPointCloud);
};

#endif