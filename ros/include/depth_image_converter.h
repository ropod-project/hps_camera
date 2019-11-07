#ifndef DEPTH_IMAGE_CONVERTER_H
#define DEPTH_IMAGE_CONVERTER_H

#include <sensor_msgs/Image.h>
#include "./api.h"

struct HueRange
{
    int start;
    int end;
};

struct RgbSettings
{
    float32_t lightness;
    float32_t saturation;
    HueRange hueRange;
};

class DepthImageConverter
{
private:
    const int numberOfPixels = RES_HEIGHT * RES_WIDTH; 
    RgbSettings rgbSettings;

public:
    DepthImageConverter()
    {
        rgbSettings = {0.5, 1, {120, 0}};
    };
    DepthImageConverter(RgbSettings inputRgbSettings){
        rgbSettings = inputRgbSettings;
    };

    void convertToGrayscaleDepthImage(HPS3D_HandleTypeDef *handle, sensor_msgs::Image *outputImage);
    void convertToRgbDepthImage(HPS3D_HandleTypeDef *handle, sensor_msgs::Image *outputImage);
    bool transitionOfHueRange(float32_t normalizedDistance, uint8_t *color);
    bool hslColorToRGB(float32_t hue, uint8_t *color);
    void printDepthImageInformation(HPS3D_HandleTypeDef *handle);
    bool setOutputImageParameter(sensor_msgs::Image *outputImage, const char *encoding);
};

#endif
