#include <string>
#include "ros/ros.h"//ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointField.h>
#include "../include/api.h"//api interface

using namespace std;

HPS3D_HandleTypeDef handle;

ros::Publisher point_cloud_pub;
ros::Publisher depth_image_pub;

void convertDepthImage(HPS3D_HandleTypeDef *handle, sensor_msgs::Image *outputImage)
{
  // printf("Lenght: %u", sizeof(handle->MeasureData.simple_depth_data->distance)/sizeof(uint16_t));
  outputImage->header.stamp = ros::Time::now();
  outputImage->height = RES_HEIGHT; // 60
  outputImage->width = RES_WIDTH; // 160
  outputImage->encoding = "mono16";
  outputImage->step = RES_WIDTH * 2;

  int numberOfPixels = RES_HEIGHT * RES_WIDTH;
  
  int sizeOfOutputDataArray = numberOfPixels * 2;
  outputImage->data.resize(sizeOfOutputDataArray);

  int indexInputDataArray = 0;

  for (int i = 0; i < sizeOfOutputDataArray; i+=2) 
  {
    // Extract the lower byte.
    outputImage->data[i] = (uint8_t) handle->MeasureData.full_depth_data->distance[indexInputDataArray];
    // Extract the upper byte.
    outputImage->data[i+1] = (uint8_t) (handle->MeasureData.full_depth_data->distance[indexInputDataArray] >> 8);
    indexInputDataArray += 1;
  }
}

void convertPointCloud(HPS3D_HandleTypeDef *handle, sensor_msgs::PointCloud2 *outputPointCloud)
{
  outputPointCloud->header.stamp = ros::Time::now();

  int numberOfPoints = RES_HEIGHT * RES_WIDTH;
  int numberOfFields = outputPointCloud->fields.size();

  for (int i = 0; i < RES_HEIGHT; i++)
  {
    for (int j = 0; j < RES_WIDTH; j++)
    {
      uint16_t *distances = handle->MeasureData.simple_depth_data->distance;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_hps_camera");
  ros::NodeHandle n;
  point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("hps_camera/depth/points", 1);
  depth_image_pub = n.advertise<sensor_msgs::Image>("hps_camera/depth/image", 1);

  bool debugMode;
  ros::param::param("~debugMode", debugMode, true);

  uint8_t fileName[10][20];
  uint32_t dev_cnt = 0;
  RET_StatusTypeDef ret = RET_OK;

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
  outputPointCloud.height = RES_HEIGHT;
  outputPointCloud.width = RES_WIDTH;
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

  sensor_msgs::Image outputImage;
  outputImage.header.seq = seq;
  outputImage.header.frame_id = "hps_camera";
  outputImage.height = RES_HEIGHT;
  outputImage.width = RES_WIDTH;
  outputImage.encoding = "mono16";
  outputImage.step = RES_WIDTH;

  HDRConf hdrConf;
  DistanceFilterConfTypeDef distanceFilterConf;
  SmoothFilterConfTypeDef smoothFilterConf;
  MountingAngleParamTypeDef mountingAngleParamConf;
  // There is not setter for the opticalParamConf, yet.
  OpticalParamConfTypeDef opticalParamConf;

  int hdrConfHdrMode;
  double hdrConfQualityOverexposed;
  double hdrConfQualityOverexposedSerious;
  double hdrConfQualityWeak;
  double hdrConfQualityWeakSerious;
  int hdrConfSimpleHdrMaxIntegration;
  int hdrConfSimpleHdrMinIntegration;
  int hdrConfSuperHdrFrameNumber;
  int hdrConfSuperHdrMaxIntegration;
  int hdrConfHdrDisableIntegrationTime;
  ros::param::param("~hdrConfHdrMode", hdrConfHdrMode, 1);
  ros::param::param("~hdrConfQualityOverexposed", hdrConfQualityOverexposed, 500.);
  ros::param::param("~hdrConfQualityOverexposedSerious", hdrConfQualityOverexposedSerious, 800.);
  ros::param::param("~hdrConfQualityWeak", hdrConfQualityWeak, 120.);
  ros::param::param("~hdrConfQualityWeakSerious", hdrConfQualityWeakSerious, 80.);
  ros::param::param("~hdrConfSimpleHdrMaxIntegration", hdrConfSimpleHdrMaxIntegration, 2000);
  ros::param::param("~hdrConfSimpleHdrMinIntegration", hdrConfSimpleHdrMinIntegration, 200);
  ros::param::param("~hdrConfSuperHdrFrameNumber", hdrConfSuperHdrFrameNumber, 3);
  ros::param::param("~hdrConfSuperHdrMaxIntegration", hdrConfSuperHdrMaxIntegration, 20000);
  ros::param::param("~hdrConfHdrDisableIntegrationTime", hdrConfHdrDisableIntegrationTime, 2000);
  hdrConf.hdr_mode = static_cast<HDRModeTypeDef>(hdrConfHdrMode);
  hdrConf.qualtity_overexposed = hdrConfQualityOverexposed;
  hdrConf.qualtity_overexposed_serious = hdrConfQualityOverexposedSerious;
  hdrConf.qualtity_weak = hdrConfQualityWeak;
  hdrConf.qualtity_weak_serious = hdrConfQualityWeakSerious;
  hdrConf.simple_hdr_max_integration = hdrConfSimpleHdrMaxIntegration;
  hdrConf.simple_hdr_min_integration = hdrConfSimpleHdrMinIntegration;
  hdrConf.super_hdr_frame_number = hdrConfSuperHdrFrameNumber;
  hdrConf.super_hdr_max_integration = hdrConfSuperHdrMaxIntegration;
  hdrConf.hdr_disable_integration_time = hdrConfHdrDisableIntegrationTime;
  HPS3D_SetHDRConfig(&handle, hdrConf);

  int distanceFilterConfFilterType;
  double distanceFilterConfKalmanK;
  int distanceFilterConfKalmanThreshold;
  int distanceFilterConfNumCheck;
  ros::param::param("~distanceFilterConfFilterType", distanceFilterConfFilterType, 0);
  ros::param::param("~distanceFilterConfKalmanK", distanceFilterConfKalmanK, 0.3);
  ros::param::param("~distanceFilterConfKalmanThreshold", distanceFilterConfKalmanThreshold, 200);
  ros::param::param("~distanceFilterConfNumCheck", distanceFilterConfNumCheck, 2);
  distanceFilterConf.filter_type = static_cast<DistanceFilterTypeDef>(distanceFilterConfFilterType);
  distanceFilterConf.kalman_K = distanceFilterConfKalmanK;
  distanceFilterConf.kalman_threshold = distanceFilterConfKalmanThreshold;
  distanceFilterConf.num_check = distanceFilterConfNumCheck;
  HPS3D_SetSimpleKalman(&handle, distanceFilterConf);

  int smoothFilterConfType;
  int smoothFilterConfArg1;
  ros::param::param("~smoothFilterConfType", smoothFilterConfType, 0);
  ros::param::param("~smoothFilterConfArg1", smoothFilterConfArg1, 0);
  smoothFilterConf.type = static_cast<SmoothFilterTypeDef>(smoothFilterConfType);
  smoothFilterConf.arg1 = smoothFilterConfArg1;
  HPS3D_SetSmoothFilter(&handle, smoothFilterConf);

  bool mountingAngleParamConfEnable;
  int mountingAngleParamConfAngleVertical;
  int mountingAngleParamConfHeight;
  ros::param::param("~mountingAngleParamConfEnable", mountingAngleParamConfEnable, false);
  ros::param::param("~mountingAngleParamConfAngleVertical", mountingAngleParamConfAngleVertical, 0);
  ros::param::param("~mountingAngleParamConfHeight", mountingAngleParamConfHeight, 0);
  mountingAngleParamConf.enable = mountingAngleParamConfEnable;
  mountingAngleParamConf.angle_vertical = mountingAngleParamConfAngleVertical;
  mountingAngleParamConf.height = mountingAngleParamConfHeight;
  HPS3D_SetMountingAngleParamConf(&handle, mountingAngleParamConf);

  int distanceOffset;
  ros::param::param("~distanceOffset", distanceOffset, 0);
  HPS3D_SetDistanceOffset(&handle, distanceOffset);

  bool edgeDetectionEnable;
  int edgeDetectionThresholdValue;
  ros::param::param("~edgeDetectionEnable", edgeDetectionEnable, false);
  ros::param::param("~edgeDetectionThresholdValue", edgeDetectionThresholdValue, 0);
  HPS3D_SetEdgeDetectionEnable(edgeDetectionEnable);
  HPS3D_SetEdgeDetectionValue(edgeDetectionThresholdValue);

  if (debugMode)
  {
    HPS3D_GetHDRConfig(&handle, &hdrConf);
    string currentHdrMode;
    switch (hdrConf.hdr_mode)
    {
      case HDR_DISABLE:
        currentHdrMode = "HDR_DISABLE";
        break;
      case AUTO_HDR:
        currentHdrMode = "AUTO_HDR";
        break;
      case SUPER_HDR:
        currentHdrMode = "SUPER_HDR";
        break;
      case SIMPLE_HDR:
        currentHdrMode = "SIMPLE_HDR";
        break;
    }

    printf("HDR configuration.\n HDR mode: %s %d\n Quality overexposed: %f\n Quality overexposed serious: %f\n Quality weak: %f\n Quality weak serious: %f\n Simple HDR max integration: %d\n Simple HDR min integration: %d\n Super HDR frame number: %d\n Super HDR max integration: %d\n HDR disable integration time: %d\n\n", currentHdrMode.c_str(), hdrConf.hdr_mode, hdrConf.qualtity_overexposed, hdrConf.qualtity_overexposed_serious, hdrConf.qualtity_weak, hdrConf.qualtity_weak_serious, hdrConf.simple_hdr_max_integration, hdrConf.simple_hdr_min_integration, hdrConf.super_hdr_frame_number, hdrConf.super_hdr_max_integration, hdrConf.hdr_disable_integration_time);

    HPS3D_GetDistanceFilterConf(&handle, &distanceFilterConf);
    string currentDistanceFilter;
    switch (distanceFilterConf.filter_type)
    {
      case DISTANCE_FILTER_DISABLE:
        currentDistanceFilter = "DISTANCE_FILTER_DISABLE";
        break;
      case DISTANCE_FILTER_SIMPLE_KALMAN:
        currentDistanceFilter = "DISTANCE_FILTER_SIMPLE_KALMAN";
        break;
    }

    printf("Distance filter configuration.\n Filter type: %s %d\n Kalman filter scale factor: %f\n Noise threshold: %d\n Threshold check frame number: %d\n\n", currentDistanceFilter.c_str(), distanceFilterConf.filter_type, distanceFilterConf.kalman_K, distanceFilterConf.kalman_threshold, distanceFilterConf.num_check);

    HPS3D_GetSmoothFilterConf(&handle, &smoothFilterConf);
    string currentSmoothFilter;
    switch (smoothFilterConf.type)
    {
      case SMOOTH_FILTER_DISABLE:
        currentSmoothFilter = "SMOOTH_FILTER_DISABLE";
        break;
      case SMOOTH_FILTER_AVERAGE:
        currentSmoothFilter = "SMOOTH_FILTER_AVERAGE";
        break;
      case SMOOTH_FILTER_GAUSS:
        currentSmoothFilter = "SMOOTH_FILTER_GAUSS";
        break;
    }

    printf("Smooth filter configuration.\n Smoothing filter type: %s %d\n Filtering parameter %d\n\n", currentSmoothFilter.c_str(), smoothFilterConf.type, smoothFilterConf.arg1);

    HPS3D_GetOpticalParamConf(&handle, &opticalParamConf);
    printf("Optical param configuration.\n Enabled: %d\n Horizontal viewing angle: %d\n Vertical viewing angle: %d\n Horizontal illumination angle: %d\n Vertical illumination angle: %d\n\n", opticalParamConf.enable, opticalParamConf.viewing_angle_horiz, opticalParamConf.viewing_angle_vertical, opticalParamConf.illum_angle_horiz, opticalParamConf.illum_angle_vertical);

    HPS3D_GetMountingParamConf(&handle, &mountingAngleParamConf);
    printf("Mounting angle param configuration.\n Enabled: %d\n Vertical installation angle: %d\n Mounting height: %d\n\n", mountingAngleParamConf.enable, mountingAngleParamConf.angle_vertical, mountingAngleParamConf.height);

    printf("Edge detection value: %d\n", HPS3D_GetEdgeDetectionValue());

    int16_t offset;
    HPS3D_GetDistanceOffset(&handle, &offset);
    printf("Distance compensation: %d\n", offset);
  }

  while(ros::ok())
  {
    if (handle.RunMode == RUN_SINGLE_SHOT)
    {
			ret = HPS3D_SingleMeasurement(&handle);
			if (ret == RET_OK)
			{
        if (handle.RetPacketType == FULL_DEPTH_PACKET)
        {

          HDRConf hdrConf;
          HPS3D_GetHDRConfig(&handle, &hdrConf);

          if (debugMode && hdrConf.hdr_mode == AUTO_HDR)
          {
            printf("HDR disable integration time: %d\n\n", hdrConf.hdr_disable_integration_time);
          }

          seq += 1;
          outputPointCloud.header.seq = seq;
          convertPointCloud(&handle, &outputPointCloud);
          point_cloud_pub.publish(outputPointCloud);

          outputImage.header.seq = seq;
          convertDepthImage(&handle, &outputImage);
          depth_image_pub.publish(outputImage);
        } else {
          printf("Process stopped: This packet type is not supported yet.");
          break;
        }
      }
    }
  }

  HPS3D_RemoveDevice(&handle);
  return 0;
}
