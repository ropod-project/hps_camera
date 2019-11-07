#include "../include/depth_image_converter.h"//api interface

using namespace std;

void DepthImageConverter::convertToGrayscaleDepthImage(HPS3D_HandleTypeDef *handle, sensor_msgs::Image *outputImage)
{
  bool result = setOutputImageParameter(outputImage, "mono16");

  uint16_t * distances = handle->MeasureData.full_depth_data->distance;

  int sizeOfOutputDataArray = numberOfPixels * 2;

  int indexInputDataArray = 0;
  for (int i = 0; i < sizeOfOutputDataArray; i+=2) 
  {
    if (distances[i] > 12000) {
      outputImage->data[i] = -1;
      outputImage->data[i+1] = -1;
    } else {
      // Extract the lower byte.
      outputImage->data[i+1] = (uint8_t) distances[indexInputDataArray];
      // Extract the upper byte.
      outputImage->data[i] = (uint8_t) distances[indexInputDataArray] >> 8;
    }
    indexInputDataArray += 1;
  }
}

void DepthImageConverter::convertToRgbDepthImage(HPS3D_HandleTypeDef *handle, sensor_msgs::Image *outputImage)
{
  bool result = setOutputImageParameter(outputImage, "rgb8");

  uint16_t *distances = handle->MeasureData.full_depth_data->distance;

  uint16_t maxDistance = 0;
  uint16_t minDistance = 12000;
  for (int i = 0; i < numberOfPixels; i++) {
    if (distances[i] < minDistance) {
      minDistance = distances[i];
    }

    if (distances[i] > maxDistance && distances[i] <= 12000) {
      maxDistance = distances[i];
    }
  }
  float32_t normalizedDistances[numberOfPixels];

  for (int i = 0; i < numberOfPixels; i++) {
    if (distances[i] > maxDistance) {
      normalizedDistances[i] = -1;
    } else {
      normalizedDistances[i] = (1. * distances[i] - minDistance) / (maxDistance - minDistance);
    }
  }

  int sizeOfOutputDataArray = numberOfPixels * 3;
  uint8_t rgbDepthImage[sizeOfOutputDataArray];
  uint8_t color[3] = {0, 0, 0};

  int indexDistanceArray = 0;
  for (int i = 0; i < sizeOfOutputDataArray; i += 3) {
    bool result = transitionOfHueRange(normalizedDistances[indexDistanceArray], color);

    rgbDepthImage[i] = color[0];
    rgbDepthImage[i + 1] = color[1];
    rgbDepthImage[i + 2] = color[2];
    indexDistanceArray += 1;
  }

  for (int i = 0; i < sizeOfOutputDataArray; i++) {
    outputImage->data[i] = rgbDepthImage[i];
  }
}

bool DepthImageConverter::transitionOfHueRange(float32_t normalizedDistance, uint8_t *color)
{
  float32_t hueInFullSpectrum = normalizedDistance * 360;
  float32_t hueInRange = hueInFullSpectrum * (rgbSettings.hueRange.end - rgbSettings.hueRange.start) / 360 + rgbSettings.hueRange.start;
  return hslColorToRGB(hueInRange, color);
}

bool DepthImageConverter::hslColorToRGB(float32_t hue, uint8_t *color)
{
  float32_t chroma = (1 - std::fabs(2 * rgbSettings.lightness - 1)) * rgbSettings.saturation;
  float32_t hueT = hue / 60;
  float32_t x = chroma * (1 - std::fabs(std::fmod(hueT, 2) - 1));

  float32_t tempRed = 0;
  float32_t tempGreen = 0;
  float32_t tempBlue = 0;
  
  if (0 <= hueT && hueT <= 1) {
    tempRed = chroma;
    tempGreen = x;
    tempBlue = 0;
  } else if (1 <= hueT && hueT <= 2) {
    tempRed = x;
    tempGreen = chroma;
    tempBlue = 0;
  } else if (2 <= hueT && hueT <= 3) {
    tempRed = 0;
    tempGreen = chroma;
    tempBlue = x;
  } else if (3 <= hueT && hueT <= 4) {
    tempRed = 0; 
    tempGreen = x; 
    tempBlue = chroma;
  } else if (4 <= hueT && hueT <= 5) {
    tempRed = x;
    tempGreen = 0;
    tempBlue = chroma;
  } else if (5 <= hueT && hueT < 6) {
    tempRed = chroma;
    tempGreen = 0;
    tempBlue = x;
  }

  float32_t m = rgbSettings.lightness * (1 - rgbSettings.saturation);

  float32_t red = tempRed + m;
  float32_t green = tempGreen + m;
  float32_t blue = tempBlue + m;

  color[0] = red * 255;
  color[1] = green * 255;
  color[2] = blue * 255;

  return true;
}

void DepthImageConverter::printDepthImageInformation(HPS3D_HandleTypeDef *handle) 
{
  uint16_t *distances = handle->MeasureData.full_depth_data->distance;
  uint16_t grayscale[numberOfPixels];

  uint16_t maxDistance = *std::max_element(distances, distances + numberOfPixels);
  uint16_t minDistance = *std::min_element(distances, distances + numberOfPixels);

  printf("HPS Average Distance: %u\n", handle->MeasureData.full_depth_data->distance_average);
  printf("HPS Max Distance: %u\n", handle->MeasureData.full_depth_data->distance_max);
  printf("HPS Min Distance: %u\n\n", handle->MeasureData.full_depth_data->distance_min);

  uint32_t averageDistance = 0;
  int numberOfPixelsAboveLimit = 0;
  int numberOfZeros = 0;
  bool hpsMinDistanceExists = false;
  int logicalMaxValue = 0;
  int distanceCenterPixel = 0;
  for (int i = 0; i < numberOfPixels; i++)
  { 
    if (i == 4800) {
      distanceCenterPixel = distances[i];
    }

    if (handle->MeasureData.full_depth_data->distance_min == distances[i]) {
      hpsMinDistanceExists = true;
    }

    if (distances[i] > 12000) {
      numberOfPixelsAboveLimit += 1;
    } else {
      averageDistance += distances[i];
    }

    if (distances[i] == 0) {
      numberOfZeros += 1;
    }

    if (distances[i] > logicalMaxValue && distances[i] <= 12000) {
      logicalMaxValue = distances[i];
    }

  }

  averageDistance /= (numberOfPixels - numberOfPixelsAboveLimit);

  printf("Average Distance: %u\n", averageDistance);
  printf("Max Distance: %u\n", maxDistance);
  printf("Min Distance: %u\n---------------\n", minDistance);
  printf("Number of pixels greater than 12000: %i\n", numberOfPixelsAboveLimit);
  printf("Number of zeros: %i\n", numberOfZeros);
  printf("Logical max value: %i\n", logicalMaxValue);
  printf("Distance center pixel: %i\n", distanceCenterPixel);
  printf("HPS Min distance exists in 'distances' %d\n------------\n", hpsMinDistanceExists);
}

bool DepthImageConverter::setOutputImageParameter(sensor_msgs::Image *outputImage, const char *encoding)
{
  if (encoding == "mono16") {
    outputImage->header.stamp = ros::Time::now();
    outputImage->height = RES_HEIGHT; // 60
    outputImage->width = RES_WIDTH; // 160
    outputImage->encoding = "mono16";
    outputImage->step = RES_WIDTH * 2;
    outputImage->data.resize(numberOfPixels * 2);
    return true;
  } else if (encoding == "rgb8") {
    outputImage->header.stamp = ros::Time::now();
    outputImage->height = RES_HEIGHT; // 60
    outputImage->width = RES_WIDTH; // 160
    outputImage->encoding = "rgb8";
    outputImage->step = RES_WIDTH * 3;
    outputImage->data.resize(numberOfPixels * 3);
    return true;
  } else {
    return false;
  }
}
