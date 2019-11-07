#include "../include/point_cloud_converter.h"//api interface

using namespace std;

void PointCloudConverter::convertToPointCloud(HPS3D_HandleTypeDef *handle, sensor_msgs::PointCloud2 *outputPointCloud)
{
  outputPointCloud->header.stamp = ros::Time::now();

  int numberOfPoints = RES_HEIGHT * RES_WIDTH;
  int numberOfFields = outputPointCloud->fields.size();

  for (int i = 0; i < RES_HEIGHT; i++)
  {
    for (int j = 0; j < RES_WIDTH; j++)
    {
      float32_t xCoordinate = -handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].x / 1000;
      uint8_t* xCoordinateBytes = (uint8_t *)&xCoordinate;
      float32_t yCoordinate = handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].y / 1000;
      uint8_t* yCoordinateBytes = (uint8_t *)&yCoordinate;
      float32_t zCoordinate = handle->MeasureData.point_cloud_data[0].point_data[i * RES_WIDTH + j].z / 1000;
      uint8_t* zCoordinateBytes = (uint8_t *)&zCoordinate;

      int arrayPosition = (i * RES_WIDTH + j) * (numberOfFields * 4);
      outputPointCloud->data[arrayPosition] = xCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 1] = xCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 2] = xCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 3] = xCoordinateBytes[3];

      outputPointCloud->data[arrayPosition + 4] = yCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 5] = yCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 6] = yCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 7] = yCoordinateBytes[3];

      outputPointCloud->data[arrayPosition + 8] = zCoordinateBytes[0];
      outputPointCloud->data[arrayPosition + 9] = zCoordinateBytes[1];
      outputPointCloud->data[arrayPosition + 10] = zCoordinateBytes[2];
      outputPointCloud->data[arrayPosition + 11] = zCoordinateBytes[3];
    }
  }
}