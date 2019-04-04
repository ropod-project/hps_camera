#include "ros/ros.h"//ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include "../include/api.h"//api interface

HPS3D_HandleTypeDef handle;

ros::Publisher camera_pub;

void convertPointCloud(HPS3D_HandleTypeDef *handle, sensor_msgs::PointCloud2 *outputPointCloud)
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
      // uint8_t* zCoordinateBytes = reinterpret_cast<uint8_t*>(&zCoordinate);
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_hps_camera");
  ros::NodeHandle n;
  camera_pub = n.advertise<sensor_msgs::PointCloud2>("hps_camera/depth/points", 1);

  uint8_t fileName[10][20];
  uint32_t dev_cnt = 0;
  RET_StatusTypeDef ret = RET_OK;

  // List the optional devices
  dev_cnt = HPS3D_GetDeviceList((uint8_t *)"/dev/", (uint8_t *)"ttyACM", fileName);
  handle.DeviceName = fileName[0];

  do
  {
    ret = HPS3D_Connect(&handle);
    if (ret != RET_OK)
    {
      printf("Device open failed！ret = %d\n", ret);
			break;
    }

    HPS3D_SetOpticalEnable(&handle, true);
    HPS3D_SetPointCloudEn(true);

    ret = HPS3D_ConfigInit(&handle);
    if (ret != RET_OK)
    {
      HPS3D_RemoveDevice(&handle);
      printf("Initialization failed:%d, device removed\n", ret);
			break;
    }

    handle.RunMode = RUN_SINGLE_SHOT;
		HPS3D_SetRunMode(&handle);
  } while(0);

  uint64_t seq = 0;
  sensor_msgs::PointCloud2 outputPointCloud;
  outputPointCloud.header.seq = seq;
  outputPointCloud.header.frame_id = "hps_camera";
  outputPointCloud.height = RES_HEIGHT;//*measureData.point_cloud_data.height;
  outputPointCloud.width = RES_WIDTH;//*measureData.point_cloud_data.width;
  outputPointCloud.point_step = 12;
  outputPointCloud.row_step = RES_WIDTH * outputPointCloud.point_step;
  outputPointCloud.is_dense = true;

  sensor_msgs::PointField pointFieldX;
  pointFieldX.name = "x";
  pointFieldX.offset = 0;
  pointFieldX.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldX.count = 1;

  sensor_msgs::PointField pointFieldY;
  pointFieldY.name = "y";
  pointFieldY.offset = 4;
  pointFieldY.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldY.count = 1;

  sensor_msgs::PointField pointFieldZ;
  pointFieldZ.name = "z";
  pointFieldZ.offset = 8;
  pointFieldZ.datatype = sensor_msgs::PointField::FLOAT32;
  pointFieldZ.count = 1;

  outputPointCloud.fields.push_back(pointFieldX);
  outputPointCloud.fields.push_back(pointFieldY);
  outputPointCloud.fields.push_back(pointFieldZ);

  outputPointCloud.data.resize(RES_HEIGHT * RES_WIDTH * outputPointCloud.fields.size() * 4);

  while(ros::ok())
  {
    if (handle.RunMode == RUN_SINGLE_SHOT)
    {
			ret = HPS3D_SingleMeasurement(&handle);
			if (ret == RET_OK)
			{
        if (handle.RetPacketType == FULL_DEPTH_PACKET)
        {
          seq += 1;
          outputPointCloud.header.seq = seq;
          convertPointCloud(&handle, &outputPointCloud);
          camera_pub.publish(outputPointCloud);
        } else {
          printf("Process stopped: This packet type is not supported yet.");
          break;
        }
      }
    }
  }

  return 0;
}
