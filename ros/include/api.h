/**********************************************************************
* COPYRIGHT NOTICE - HYPERSEN TECHNOLOGY
*
* Copyright (c) 2018, Hypersen Technology, Inc.
*
* All rights reserved.
*
*======================================================================
* \file api.h
* \brief TODO
* \author Kevin
* \email Kevin_Wang@hypersen.com
* \version 1.0.0
* \date 2018年11月13日 上午11:43:31
* \license private & classified
*---------------------------------------------------------------------
* Remark: This project is still under construction.
*======================================================================
* Change History:
*---------------------------------------------------------------------
* <Date>			| <Version>	| <Author>			| <Description>
*---------------------------------------------------------------------
* 2018年11月13日			| V1.0.0	| Kevin				| Create file
*======================================================================
* Detailed Notes:
*---------------------------------------------------------------------
* <Version>		| <Description>
*---------------------------------------------------------------------
* V1.0.0		| TODO
*---------------------------------------------------------------------

**********************************************************************/

#ifndef HPS3D_API_H_
#define HPS3D_API_H_

#ifdef __cplusplus
extern "C"  /*C++*/
{
#endif

#include <stdbool.h>
#include <string.h>


#ifdef _WIN32 /*windows平台*/
	#include <windows.h>
#endif

#define DLL_API _declspec(dllexport)

/*类型的宏 - Type macro*/
typedef signed char 	int8_t;
typedef unsigned char 	uint8_t;
typedef unsigned short 	uint16_t;
typedef short 			int16_t;
typedef unsigned int 	uint32_t;
typedef int 			int32_t;
typedef float 			float32_t;
typedef double 			float64_t;


#define 	DEV_NUM 			 (10)							/*设备数量 - Equipment quantity*/
#define     DEV_NAME_SIZE		 (20)							/*设备名长度 - Device name length*/
#define 	ROI_NUM 			 (8)							/*ROI的数量 - Number of ROIs*/
#define 	OBSTACLE_NUM 		 (20)							/*支持障碍物数量 - Number of supported obstacles*/
#define 	OBSERVER_NUM  		 (10)							/*观察者数量 - Number of observers*/

/*分辨率	- Resolution*/
#define		RES_WIDTH			 (160)							/*深度图的长度 - Length of depth map*/
#define		RES_HEIGHT			 (60)							/*深度图的宽度 - Witdth of depth map*/
#define		MAX_PIX_NUM 		 (RES_WIDTH * RES_HEIGHT)		/*最大的深度图像素 - Maximum depth map pixel*/

/*特殊测量数据值 - Special measurement data value*/
#define	 	LOW_AMPLITUDE   	(65300) 						/*信号幅值低 - Low signal amplitude*/
#define	  	SATURATION 			(65400)     					/*饱和位饱和 - Saturated bit saturation*/
#define	 	ADC_OVERFLOW  		(65500)   						/*ADC溢出 - ADC overflow*/
#define	 	INVALID_DATA 		(65530)    						/*无效数据 - Invalid data*/

/*函数返回的结果 - The result returned by the function*/
typedef enum
{
	RET_OK 		= 0x01,		/*返回成功 - Return success*/
	RET_ERROR 	= 0x02,		/*返回错误 - Return error*/
	RET_BUSY 	= 0x03,		/*返回忙 - Return busy*/
	RET_CONNECT_FAILED,     /*连接失败 - Connection failed*/
	RET_CREAT_PTHREAD_ERR,  /*线程创建失败 - Thread creation failed*/
	RET_WRITE_ERR,          /*写失败 - Write failure*/
	RET_READ_ERR,           /*读失败 - Failed to read*/
	RET_PACKET_HEAD_ERR,    /*报头错误 - Header error*/
	RET_PACKET_ERR,    		/*报文错误 - Message error*/
	RET_BUFF_EMPTY,			/*缓冲区为空 - Buffer is empty*/
	RET_VER_MISMATCH,  		/*版本不匹配 - Version does not match*/
}RET_StatusTypeDef;

/*设备版本 - Device version*/
typedef struct
{
	uint8_t year;			/*年 - Year*/
	uint8_t month;			/*月 - Month*/
	uint8_t day;			/*日 - Day*/
	uint8_t major;			/*主板本 - Motherboard*/
	uint8_t minor;			/*次版本 - Minor version*/
	uint8_t rev;			/*修订次数 - Revisions*/
}Version_t;

/*运行模式选择 - Operating mode selection*/
typedef enum
{
	MinOfRunModeType = 0,
	RUN_IDLE = 0,			/*运行停止 - Stop running*/
	RUN_SINGLE_SHOT,		/*单次测量 - Single measurement*/
	RUN_CONTINUOUS,			/*多次测量 - Multiple measurements*/
	NumberOfRunModeType
}RunModeTypeDef;

/*设置数据包的类型 - Set the type of packet*/
typedef enum
{
	PACKET_FULL = 0,		/*完整数据包（包含深度数据）- Complete package (including deep data)*/
	PACKET_SIMPLE			/*简单数据包（不包含深度数据）- Simple data package (without deep data)*/
}OutPacketTypeDef;


/*ROI的阈值警报类型 - ROI threshold alert type*/
typedef enum
{
	ROI_ALARM_DISABLE = 0,  /*关闭ROI区域警报，仅输出ROI区域的信息和数据 - Close the ROI area alarm and output only the information and data of the ROI area*/
	ROI_ALARM_GPIO       	/*ROI区域警报类型为GPIO OUT电平输出 - ROI area alert type is GPIO*/
}ROIAlarmTypeDef;

/*单点的迟滞配置 - Single point hysteresis configuration*/
typedef struct
{
	uint8_t threshold_id;  		     /*敏感区域阈值id - Sensitive area threshold id*/
	uint32_t threshold_value; 		/*敏感区域阈值 - Sensitive area threshold*/
	uint32_t hysteresis; 			/*迟滞大小 - Hysteresis size*/
	bool enable;					/*迟滞使能 - Hysteresis enable*/
	bool positive;					/*true:正向比较，如果输入值大于阈值则返回True - Positive comparison, return True if the input value is greater than the threshold
									  false:反向比较，如果输入值小于阈值则返回False - Reverse comparison, return False if the input value is less than the threshold*/
}HysteresisSingleConfTypeDef;

/*ROI的参考值类型 - ROI reference value type*/
typedef enum
{
	ROI_REF_DIST_AVR = 1,			/*ROI区域的距离平均值作为参考值 - The average distance of the ROI area is used as a reference value.*/
	ROI_REF_DIST_MIN,				/*ROI区域的距离最小值作为参考值 - The minimum distance of the ROI area is used as a reference value.*/
	ROI_REF_DIST_MAX,				/*ROI区域的距离最大值作为参考值 - The maximum distance of the ROI area is used as a reference value.*/
	ROI_REF_SAT_COUNT,				/*ROI区域的饱和像素点数量作为参考值 - The number of saturated pixels in the ROI area is used as a reference value.*/
	ROI_REF_AMPLITUDE,				/*ROI区域的幅值平均值作为参考值 - The average value of the amplitude of the ROI area is used as a reference value.*/
	ROI_REF_VAILD_AMPLITUDE,		/*ROI区域的有效幅值平均值作为参考值 - The average value of the effective amplitude of the ROI area is used as a reference value.*/
	ROI_REF_THRESHOLD_PIX_NUM		/*超过设定阈值的像素点数，相比最大值比较和最小值比较具有更高的可靠性 - The number of pixels exceeding the set threshold is higher than the maximum comparison and the minimum comparison.*/
}ROIReferenceTypeDef;

/*ROI配置的结构体 - ROI configured structure*/
 typedef struct
{
	bool enable;										/*使能标识 - Enable flag*/
	uint8_t roi_id;										/*ROI的ID - ROI ID*/
	uint16_t left_top_x;								/*左上角x坐标 - Upper left corner x coordinate*/
	uint16_t left_top_y;								/*左上角y坐标 - Upper left corner y coordinate*/
	uint16_t right_bottom_x;							/*右下角x坐标 - Right lower corner x coordinate*/
	uint16_t right_bottom_y;							/*右下角y坐标 - Right lower corner y coordinate*/
	HysteresisSingleConfTypeDef hysteresis_conf[3];		/*单点的迟滞配置 - Single point hysteresis configuration*/
	ROIReferenceTypeDef ref_type[3];					/*ROI的参考值类型,与hysteresis_conf一一对应 - ROI reference value type, one-to-one correspondence with hysteresis_conf*/
	ROIAlarmTypeDef alarm_type[3];						/*ROI的阈值警报类型,与hysteresis_conf一一对应 - ROI threshold alarm type, one-to-one correspondence with hysteresis_conf*/
	uint16_t pixel_number_threshold[3];					/*超过阈值的像素点数阈值,与hysteresis_conf一一对应 - The threshold of the number of pixels exceeding the threshold, one-to-one correspondence with hysteresis_conf*/
}ROIConfTypeDef;

/*Auto HDR调节模式 - Auto HDR adjustment mode*/
typedef enum
{
	HDR_DISABLE = 0,				/*HDR不使能，手动设置/获取 积分时间 - HDR is not enabled, manually set / get integration time*/
	AUTO_HDR,						/*自动HDR，设置/获取 曝光幅值/过度曝光幅值/信号弱幅值/信号极弱幅值 - Auto HDR, Set/Acquire Exposure Amplitude/Overexposure Amplitude/Signal Weak Amplitude/Signal Weak Amplitude*/
	SUPER_HDR,						/*超级HDR，设置/获取 合成帧数/最大积分时间 - Super HDR, set/get composite frame number / maximum integration time*/
	SIMPLE_HDR						/*简单HDR，设置/获取 最大/最小积分时间 - Simple HDR, set/get maximum/minimum integration time*/
}HDRModeTypeDef;

/*HDR配置 - HDR configuration*/
typedef struct
{
	HDRModeTypeDef hdr_mode;					/*HDR模式选择 - HDR mode selection*/
	float32_t qualtity_overexposed;				/*AUTO_HDR 曝光幅值 - AUTO_HDR exposure amplitude*/
	float32_t qualtity_overexposed_serious;		/*AUTO_HDR 过度曝光赋值 - AUTO_HDR overexposure assignment*/
	float32_t qualtity_weak;					/*AUTO_HDR 信号弱幅值 - AUTO_HDR signal weak amplitude*/
	float32_t qualtity_weak_serious;			/*AUTO_HDR 信号极弱幅值 - AUTO_HDR signal is extremely weak amplitude*/
	uint32_t simple_hdr_max_integration;		/*SIMPLE_HDR 最大积分时间 us - SIMPLE_HDR maximum integration time us*/
	uint32_t simple_hdr_min_integration;		/*SIMPLE_HDR 最小积分时间 us - SIMPLE_HDR minimum integration time us*/
	uint8_t super_hdr_frame_number;				/*SUPER_HDR 合成帧数 - SUPER_HDR composite frame number*/
	uint32_t super_hdr_max_integration;			/*SUPER_HDR 最大积分时间 us - SUPER_HDR maximum integration time us*/
	uint32_t hdr_disable_integration_time;		/*HDR_DISABLE 手动积分时间 us - HDR_DISABLE manual integration time us*/
}HDRConf;

/*平滑滤波器的类型 - Type of smoothing filter*/
typedef enum
{
	SMOOTH_FILTER_DISABLE = 0,		/*平滑滤波器关闭 - Smoothing filter off*/
	SMOOTH_FILTER_AVERAGE = 1,		/*均值滤波器 - Mean filter*/
	SMOOTH_FILTER_GAUSS				/*高斯滤波器 - Gaussian filter*/
}SmoothFilterTypeDef;

/*平滑滤波器的配置结构体 - Smoothing filter configuration structure*/
typedef struct
{
	SmoothFilterTypeDef type;		/*设置平滑滤波器模式 - Set the smoothing filter mode*/
	uint32_t arg1;					/*滤波参数 - Filtering parameter*/
}SmoothFilterConfTypeDef;

/*GPIO配置的相关定义 - Related definitions of GPIO configuration*/
/*GPIO_OUT功能 - GPIO_OUT function*/
typedef enum
{
	GPOUT_FUNC_DISABLE = 0,				/*GPIO报警关闭 - GPIO alarm is off*/
	GPOUT_FUNC_ROI_THRESHOLD0_ALARM,	/*GPIO输出阈值0警报 - GPIO output threshold 0 alarm*/
	GPOUT_FUNC_ROI_THRESHOLD1_ALARM,	/*GPIO输出阈值1警报 - GPIO output threshold 1 alarm*/
	GPOUT_FUNC_ROI_THRESHOLD2_ALARM		/*GPIO输出阈值2警报 - GPIO output threshold 2 alarm*/
}GPOutFunctionTypeDef;

/*GPIO_IN功能 - GPIO_IN function*/
typedef enum
{
	GPIN_FUNC_DISABLE = 0,			/*GPIO功能关闭 - GPIO function is off*/
	GPIN_FUNC_CAPTURE   			/*开启测量  注：开启测量后，则不受命令控制，只受IO输入控制 - Turn on measurement Note: After the measurement is turned on, it is not controlled by the command and is only controlled by the IO input.*/
}GPInFunctionTypeDef;

/*GPIO极性 - GPIO polarity*/
typedef enum
{
	GPIO_POLARITY_LOW = 0,			/*GPIO极性为低 - GPIO polarity is low*/
	GPIO_POLARITY_HIGH				/*GPIO极性为高 - GPIO polarity is high*/
}GPIOPolarityTypeDef;

/*GPIO引脚 - GPIO pin*/
typedef enum
{
	GPIN_1 = 1,						/*GPIO输入 - GPIO input*/
	GPOUT_1 = 10					/*GPIO输出 - GPIO output*/
}GPIOTypeDef;

/*GPIO输出配置 - GPIO output configuration*/
typedef struct
{
	GPIOTypeDef gpio;				/*GPIO引脚 - GPIO pin*/
	GPIOPolarityTypeDef polarity;	/*GPIO极性 - GPIO polarity*/
	GPOutFunctionTypeDef function;	/*GPIO功能 - GPIO function*/
}GPIOOutConfTypeDef;

/*GPIO输入配置 - GPIO input configuration*/
typedef struct
{
	GPIOTypeDef gpio;				/*GPIO引脚 - GPIO pin*/
	GPIOPolarityTypeDef polarity;	/*GPIO极性 - GPIO polarity*/
	GPInFunctionTypeDef function;	/*GPIO功能 - GPIO function*/
}GPIOInConfTypeDef;


/*距离滤波器类型 - Distance filter type*/
typedef enum
{
	DISTANCE_FILTER_DISABLE = 0,	/*距离滤波器关闭 - Distance filter off*/
	DISTANCE_FILTER_SIMPLE_KALMAN	/*简单卡尔曼滤波器 - Simple Kalman filter*/
}DistanceFilterTypeDef;

typedef struct
{
	DistanceFilterTypeDef filter_type;		/*距离滤波器类型 - Distance filter type*/
	float32_t kalman_K; 					/*简单卡尔曼滤波器比例系数 - Simple Kalman filter scale factor*/
	uint32_t kalman_threshold;				/*噪声阈值 - Noise threshold*/
	uint32_t num_check;						/*阈值检查帧数 - Threshold check frame number*/
}DistanceFilterConfTypeDef;

/*安装角度变换参数,旋转坐标系使用 - Install angle transformation parameters, use the rotation coordinate system*/
typedef struct
{
	bool enable;					/*安装角度使能 - Mounting angle enable*/
	uint8_t angle_vertical;     	/*垂直方向安装角度（°）:主光轴与地垂线间的夹角 - Vertical installation angle (°): the angle between the main optical axis and the ground perpendicular*/
	uint16_t height;				/*相对于地面的安装高度(mm) - Mounting height relative to the ground(mm)*/
}MountingAngleParamTypeDef;

/*解析数据包的类型 - Parse the type of packet*/
typedef enum
{
	NULL_PACKET = 0x00,				/*解析数据为空 - Parsing data is empty*/
	SIMPLE_ROI_PACKET = 0x01,		/*简单ROI数据包（不含深度图数据）- Simple ROI packet (without depth map data)*/
	FULL_ROI_PACKET,				/*完整ROI数据包（含深度图数据）- Complete ROI packet (with depth map data)*/
	FULL_DEPTH_PACKET,				/*完整深度数据包（含深度图数据）- Full depth packet (with depth map data)*/
	SIMPLE_DEPTH_PACKET,			/*简单深度数据包（不含深度图数据）- Simple depth packet (without depth map data)*/
	OBSTACLE_PACKET,				/*障碍物数据包 - Obstacle packet*/
    SYSTEM_ERROR					/*系统错误 - System error*/
}RetPacketTypedef;

/*简单数据包的ROI数据 - Simple packet ROI data*/
typedef struct
{
	uint8_t group_id;						/*组ID - Group ID*/
	uint8_t id;								/*ROI ID*/
	uint16_t amplitude;						/*平均幅值 - Average amplitude*/
	uint16_t valid_amplitude;				/*平均有效幅值 - Average effective amplitude*/
	uint16_t distance_average;				/*平均距离值 - Average distance value*/
	uint16_t distance_max;					/*最大距离值 - Maximum distance value*/
	uint16_t distance_min;					/*最小距离值 - Minimum distance value*/
	uint16_t dist_max_x;					/*最大距离的x坐标 - The x coordinate of the maximum distance*/
	uint16_t dist_max_y;					/*最大距离的y坐标 - Y coordinate of the maximum distance*/
	uint16_t dist_min_x;					/*最小距离的x坐标 - The x coordinate of the minimum distance*/
	uint16_t dist_min_y;					/*最小距离的y坐标 - The y coordinate of the minimum distance*/
	uint16_t saturation_count;				/*饱和像素点数 - Saturated pixel points*/
	uint8_t threshold_state;				/*当前测量值是否超出阈值:bit0:zone0, bit1:zone1, bit2:zone2 - Whether the current measured value exceeds the threshold:bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t out_of_threshold_pix_num[3];	/*[0]:超过thresold0的像素点数,[1]:...,[2]:... - [0]:More than thresold0 pixels,[1]:...,[2]:...*/
	uint16_t frame_cnt;						/*帧计数器 - Frame counter*/
}SimpleFullRoiDataTypeDef;

/*全部数据包的ROI深度图数据 - ROI depth map data for all packets*/
typedef struct
{
	uint8_t roi_num;						/*ROI总数量 - Total number of ROI*/
	uint8_t group_id;						/*组ID - Group ID*/
	uint8_t id;								/*ROI ID*/
	uint16_t left_top_x;					/*左上角x坐标 - Upper left corner x coordinate*/
	uint16_t left_top_y;					/*左上角y坐标 - Upper left corner y coordinate*/
	uint16_t right_bottom_x;				/*右下角x坐标 - Right lower corner x coordinate*/
	uint16_t right_bottom_y;				/*右下角y坐标 - Right lower corner y coordinate*/
	uint32_t pixel_number;					/*ROI像素点 - ROI pixel*/
	uint16_t amplitude;						/*平均幅值 - Average amplitude*/
	uint16_t valid_amplitude;				/*平均有效幅值 - Average effective amplitude*/
	uint16_t distance_average;				/*平均距离值 - Average distance value*/
	uint16_t distance_max;					/*最大距离值 - Maximum distance value*/
	uint16_t distance_min;					/*最小距离值 - Minimum distance value*/
	uint16_t saturation_count;				/*饱和像素点数 - Saturated pixel points*/
	uint16_t threshold_state;				/*当前测量值是否超出阈值:bit0:zone0, bit1:zone1, bit2:zone2 - Whether the current measured value exceeds the threshold:bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t dist_max_x;					/*最大距离的x坐标 - The x coordinate of the maximum distance*/
	uint16_t dist_max_y;					/*最大距离的y坐标 - Y coordinate of the maximum distance*/
	uint16_t dist_min_x;					/*最小距离的x坐标 - The x coordinate of the minimum distance*/
	uint16_t dist_min_y;					/*最小距离的y坐标 - The y coordinate of the minimum distance*/
	uint32_t frame_cnt;						/*帧计数器 - Frame counter*/
	uint16_t distance[MAX_PIX_NUM];			/*深度数据，按顺序储存 - Depth data, stored in order*/
}FullRoiDataTypeDef;

/*深度图数据 - Depth map data*/
typedef struct
{
	uint16_t distance_average;				/*平均距离值 - Average distance value*/
	uint16_t amplitude_average;				/*平均有效幅值 - Average effective amplitude*/
	uint16_t amplitude_average_whole;		/*平均所有幅值 - Average all amplitudes*/
	uint16_t amplitude_low_count;			/*低信号像素的数量 - Number of low signal pixels*/
	uint16_t saturation_count;				/*饱和像素点数 - Saturated pixel points*/
	uint16_t distance_max;					/*最大距离值 - Maximum distance value*/
	uint16_t distance_min;					/*最小距离值 - Minimum distance value*/
	int16_t temperature;					/*温度值 - Temperature value*/
	uint16_t frame_cnt;						/*帧计数器 - Frame counter*/
	uint16_t interference_num;				/*受干扰的像素点 - Disturbed pixel*/
	uint16_t distance[MAX_PIX_NUM];			/*深度数据，按顺序储存 - Depth data, stored in order*/
}DepthDataTypeDef;


/*每点的点云数据 - Point cloud data per point*/
typedef struct
{
	float32_t x;					/*x,y,z空间坐标 - x,y,z space coordinates*/
	float32_t y;
	float32_t z;
}PerPointCloudDataTypeDef;

/*有序点云数据 - Ordered point cloud data*/
typedef struct
{
	PerPointCloudDataTypeDef point_data[MAX_PIX_NUM];	/*点云坐标，数组是为了存储多个ROI - Point cloud coordinates, arrays are used to store multiple ROIs*/
	uint16_t width;										/*宽度：一行，点的数目 - Width: one line, the number of points*/
	uint16_t height;									/*高度：行的总数 - Height: total number of rows*/
	uint32_t points;									/*点云图，点的总数 - Point cloud map, total number of points*/
}PointCloudDataTypeDef;

/*障碍物配置相关参数 - Obstacle configuration related parameters*/
typedef struct
{
	bool enable;       				/*使能标志位 - Enable flag*/
	uint16_t frame_head; 			/*数据帧，帧头特征字节，例如0XEB81 - Data frame, frame header feature byte, such as 0XEB81*/
	uint8_t number;  				/*需要提取的障碍物数量 例如3 - The number of obstacles that need to be extracted, for example 3*/
	uint16_t vaild_min_dist; 		/*有效范围的最小距离值mm 例如 600 - The minimum distance value of the effective range is mm, for example 600*/
	uint16_t vaild_max_dist; 		/*有效范围的最大距离值mm 例如 3500 - The maximum distance value of the effective range is mm, for example 3500*/
	uint16_t invaild_value;  		/*无效区域的固定参数值mm 例如 5000 - Fixed parameter value of invalid area mm eg 5000*/
	uint16_t frame_size;			/*保存当前缓冲区有效字节数 - Save the current buffer valid bytes*/
}ObstacleConfigTypedef;

/*障碍物数据 - Obstacle data*/
typedef struct
{
	uint8_t Id;   										/*障碍物区域ID - Obstacle area ID*/
	uint32_t FrameCount;  								/*帧计数值 - Frame count value*/
	uint16_t PixelNumber;			 					/*障碍物区域内像素点总数 - Total number of pixels in the obstacle area*/
	uint16_t DistanceAverage; 							/*障碍物区域内平均距离值 - Average distance value in the obstacle area*/
	PerPointCloudDataTypeDef LeftPoint; 				/*障碍物区域左端点坐标值 - Obstruction area left endpoint coordinate value*/
	PerPointCloudDataTypeDef RightPoint; 				/*障碍物区域右端点坐标值 - Obstruction area right endpoint coordinate value*/
	PerPointCloudDataTypeDef UpperPoint; 				/*障碍物区域上端点坐标值 - End point coordinate value on the obstacle area*/
	PerPointCloudDataTypeDef UnderPoint; 				/*障碍物区域下端点坐标值 - End point coordinate value under the obstacle area*/
	PerPointCloudDataTypeDef MinPoint;					/*障碍物区域最小点坐标值 - Minimum point coordinate value of the obstacle area*/
	PerPointCloudDataTypeDef PixelBuffer[MAX_PIX_NUM];   /*保存障碍物所有像素点信息buffer - Save obstacles all pixel information buffer*/
}ObstacleDataTypedef;


/*结构体体封装结构体 用于数据的返回 - Structure body package structure for data return*/
typedef struct
{
	SimpleFullRoiDataTypeDef *simple_roi_data;	/*简单ROI数据包 - Simple ROI packet*/
	FullRoiDataTypeDef *full_roi_data;			/*完整ROI数据包 - Complete ROI packet*/
	DepthDataTypeDef *simple_depth_data;		/*简单深度图数据包 - Simple depth map packet*/
	DepthDataTypeDef *full_depth_data;			/*完整ROI数据包 - Complete ROI packet*/
	PointCloudDataTypeDef *point_cloud_data;	/*点云数据包 - Point cloud packet*/
	ObstacleDataTypedef *Obstacle_data;			/*障碍物数据包 - Obstacle packet*/
	uint8_t *Obstacle_frame_data_buff; 			/*用于存放障碍物数据包的缓冲区 - Buffer for storing obstacle packets*/
}MeasureDataTypeDef;

/*通讯方式 - Communication method*/
typedef enum
{
	SYNC = 0x01,  						/*同步方式 - Synchronously*/
	ASYNC = 0x02 						/*异步方式 - Asynchronous mode*/
}HPS3D_SynchronousTypedef;

/*handle*/
typedef struct
{
	uint8_t *DeviceName; 				/*当前所有可连接设备的名称(自动筛选) - The name of all currently connectable devices (automatic filtering)*/
	uint32_t DeviceFd;  				/*存放当前连接设备的文件描述符 - Store the file descriptor of the currently connected device*/
	uint8_t DeviceAddr; 				/*存放当前连接设备的设备地址(也是帧ID) - Store the device address (also the frame ID) of the currently connected device*/
	HPS3D_SynchronousTypedef SyncMode;  /*同步或异步方式 - Synchronous or asynchronous*/
	RunModeTypeDef RunMode;   			/*运行模式 - Operating mode*/
	MeasureDataTypeDef MeasureData;     /*同步测量数据,当异步方式时测量结果不会保存在此(可通过观察者对其操作) - Synchronous measurement data, when measured in asynchronous mode, the measurement results are not saved here (can be operated by the observer)*/
	RetPacketTypedef RetPacketType;     /*同步测量返回包类型,当异步方式时测量返回包类型结果不会保存在此(可通过观察者对其操作) - Synchronous measurement returns the packet type. When the asynchronous mode is measured, the result of returning the packet type will not be saved here (can be operated by the observer)*/
	OutPacketTypeDef OutputPacketType; 	/*输出数据包类型 - Output packet type*/
	bool ConnectStatus;  	 			/*连接状态 - Connection Status*/
	uint8_t RoiNumber;					/*保存当前设备支持的ROI数量 - Save the number of ROIs supported by the current device*/
	uint8_t ThresholdNumber;			/*保存当前设备ROI支持的阈值数量 - Save the threshold number of current device ROI support*/
	uint8_t ViewAngleHorizontal;  		/*水平方向视场角 - Horizontal field of view*/
	uint8_t ViewAngleVertical;			/*垂直方向视场角 - Vertical field of view*/

}HPS3D_HandleTypeDef;

/*光学参数 - Optical parameter*/
typedef struct
{
	bool enable;						/*光学参数使能（开启后，测量的深度数据为垂直距离）- Optical parameter enable (the measured depth data is vertical distance after turning on)*/
	uint8_t viewing_angle_horiz;    	/*水平方向可视角 - Horizontal direction*/
	uint8_t viewing_angle_vertical; 	/*垂直方向可视角 - Vertical viewing angle*/
	uint8_t illum_angle_horiz;      	/*水平方向照明发射角 - Horizontal illumination angle*/
	uint8_t illum_angle_vertical;		/*垂直方向照明发射角 - Vertical illumination angle*/
}OpticalParamConfTypeDef;

/*多机干扰的配置 - Multi-machine interference configuration*/
typedef struct
{
	bool enable;						/*多机干扰检测使能 - Multi-machine interference detection enable*/
	uint32_t integ_time;				/*多机干扰检测积分时间 - Multi-machine interference detection integration time*/
	uint16_t amplitude_threshold;		/*多机干扰检测阈值 - Multi-machine interference detection threshold*/
	uint8_t capture_num;				/*多机干扰检测采样次数 - Multi-machine interference detection sampling times*/
	uint8_t number_check;				/*多机干扰检测采样次数检查 - Multi-machine interference detection sampling count check*/
}InterferenceDetectConfTypeDef;

/*传输类型 - Transmission type*/
typedef enum
{
	TRANSPORT_USB = 0,					/*USB传输 - USB transfer*/
	TRANSPORT_CAN,						/*CAN传输 - CAN transmission*/
	TRANSPORT_RS232,					/*RS232传输 - RS232 transmission*/
	TRANSPORT_RS485						/*RS485传输 - RS485 transmission*/
}TransportTypeDef;

/*观察者订阅事件 - Observer subscription event*/
typedef enum
{
	ISubject_Event_DataRecvd = 0x01,	/*数据记录事件 - Data logging event*/
	ISubject_Event_DevConnect = 0x02,	/*设备连接事件 - Device connection event*/
	ISubject_Event_DevDisconnect = 0x03 /*设备断开连接事件 - Device disconnection event*/
}AsyncISubjectEvent;

/*观察者订阅事件结构体参数 - Observer subscribes to event structure parameters*/
typedef struct
{
	uint8_t ObserverID;  				/*用于区分观察者ID - Used to distinguish observer ID*/
	bool NotifyEnable;   				/*用于使能观察者 - Used to enable observers*/
	AsyncISubjectEvent AsyncEvent; 		/*观察者订阅事件 - Observer subscription event*/
	MeasureDataTypeDef MeasureData; 	/*用于存放测量结果 - Used to store measurement results*/
	RetPacketTypedef RetPacketType; 	/*用于区分返回包类型 - Used to distinguish the return packet type*/
}AsyncIObserver_t;



/**************************************函数接口*************************************/
/**************************************Function interface*************************************/

/***********************************1.命令函数接口***********************************/
/**说明：在命令函数接口里，均有HPS3D_SetRunMode()函数，设置运行模式为暂停(RUN_IDLE)，才可发送命令。
 		在发送完命令后，记得开启原来的运行模式！！！
		（除HPS3D_SetRunMode本函数外）
 */
 /**Note: In the command function interface, there are HPS3D_SetRunMode() function, and the running mode is set to pause (RUN_IDLE) before the command can be sent.
 		After sending the command, remember to open the original running mode.！！！
 		（In addition to the HPS3D_SetRunMode function）
  */

/**
 * @brief	设置测量模式 - Set measurement mode
 * @param[in]   handle->DeviceAddr   设备地址 - Device address
 * @param[in]   handle->DeviceFd     设备文件描述符fd - Device file descriptor fd
 * @param[in]   handle->RunMode 	 采集模式 - Acquisition mode
 * @note
 * @retval	成功返回 RET_OK - Return successfully
 */
extern RET_StatusTypeDef HPS3D_SetRunMode(HPS3D_HandleTypeDef *handle);

/**
 * @brief	得到设备地址 - Get the device address
 * @param[in]   handle->DeviceFd    设备文件描述符fd - Device file descriptor fd
 * @param[out]  handle->DeviceAddr  输出设备地址 - Output device address
 * @note
 * @retval	成功返回RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDevAddr(HPS3D_HandleTypeDef *handle);

/**
 * @brief	设置设备地址 - Set device address
 * @param[in]	handle->DeviceAddr    原设备地址 - Original device address
 * @param[in]   handle->DeviceFd      设备文件描述符fd - Device file descriptor fd
 * @param[in]	new_addr     		  新设备地址 - New device address
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDevAddr(HPS3D_HandleTypeDef *handle, uint8_t new_addr);

/**
 * @brief	获取设备版本信息 - Get device version information
 * @param[in]	handle->DeviceAddr    设备地址 - Device address
 * @param[in]   handle->DeviceFd      设备文件描述符fd - Device file descriptor fd
 * @param[out]  version_t 		 	  输出设备版本信息 - Output device version information
 * @param[out]	version_t.year		  年 - Year
 * @param[out]  version_t.month		  月 - Month
 * @param[out]  version_t.day		  日 - Day
 * @param[out]  version_t.major		  主版本号 - Major version number
 * @param[out]	version_t.minor		  次版本号 - Minor version number
 * @param[out]	version_t.rev		  修订次数 - Revisions
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDeviceVersion(HPS3D_HandleTypeDef *handle, Version_t *version_t);

/**
 * @brief	设定测量数据返回包类型(简单包或完整包) - Set measurement data to return the package type (simple or complete package)
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[in]	handle->PacketType	   数据包类型选择输入 - Packet type selection input
 * 				-PACKET_FULL	       完整数据包（包含深度数据）- Complete package (including deep data)
 *				-PACKET_SIMPLE		   简单数据包（不包含深度数据）- Simple data package (without deep data)
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPacketType(HPS3D_HandleTypeDef *handle);

/**
 * @brief	获取数据包类型 - Get packet type
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[out]	handle->PacketType     输出数据包的类型 - Type of output packet
 * 				-PACKET_FULL	       完整数据包（包含深度数据）- Complete package (including deep data)
 *				-PACKET_SIMPLE		   简单数据包（不包含深度数据）- Simple data package (without deep data)
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetPacketType(HPS3D_HandleTypeDef *handle);

/**
 * @brief	保存到用户设定表 - Save to user settings table
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileSaveToCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	清除用户设置表 - Clear user settings table
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileClearCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	恢复出厂设置 - Reset
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileRestoreFactory(HPS3D_HandleTypeDef *handle);

/**
 * @brief	获得传输类型 - Get the transfer type
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[out]	transport_type	 	   输出传输类型 - Output transfer type
 * 				-TRANSPORT_USB		   USB传输 - USB transfer
 * 				-TRANSPORT_CAN		   CAN传输 - CAN transmission
 * 				-TRANSPORT_RS232       RS232传输 - RS232 transmission
 * 				-TRANSPORT_RS485	   RS485传输 - RS485 transmission
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetTransportType(HPS3D_HandleTypeDef *handle, TransportTypeDef *transport_type);

/**
 * @brief	选择ROI组 - Select ROI group
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[in]	group_id     		   ROI组选择输入 - ROI group selection input
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SelectROIGroup(HPS3D_HandleTypeDef *handle, uint8_t group_id);

/**
 * @brief	获取当前ROI组ID - Get the current ROI group ID
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[out]  group_id   			   输出当前ROI组ID - Output current ROI group ID
 * @note
 * @retval  成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetROIGroupID(HPS3D_HandleTypeDef *handle, uint8_t *group_id);

/**
 * @brief	设置ROI的警报类型 - Set the alert type for the ROI
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[in]	group_id    	   	   ROI组选择输入 - ROI group selection input
 * @param[in]	threshold_id 	   	   阈值选择输入 - Threshold selection input
 * @param[in]	roi_alarm_type     	   警报类型选择输入 - Alarm type selection input
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIAlarmType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIAlarmTypeDef roi_alarm_type);

/**
 * @brief	设置ROI的参考值类型 - Set the reference type of the ROI
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd -Device file descriptor fd
 * @param[in]	roi_id       		   ROI组ID选择输入 - ROI group ID selection input
 * @param[in]	threshold_id 		   阈值选择输入 - Threshold selection input
 * @param[in]	type	     		   参考值类型选择输入 - Reference value type selection input
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIReferenceType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIReferenceTypeDef ref_type);

/**
 * @brief	设定ROI区域 - Set the ROI area
 * @param[in]	handle->DeviceAddr       设备地址 - Device address
 * @param[in]   handle->DeviceFd         设备文件描述符fd -Device file descriptor fd
 * @param[in]	roi_conf.roi_id          ROI组ID选择输入 - ROI group ID selection input
 * @param[in]	roi_conf.left_top_x		 左上角x坐标 - Upper left corner x coordinate
 * @param[in]	roi_conf.left_top_y		 左上角y坐标 - Upper left corner y coordinate
 * @param[in]	roi_conf.right_bottom_x  右下角x坐标 - Right lower corner x coordinate
 * @param[in]   roi_conf.right_bottom_y  右下角y坐标 - Right lower corner y coordinate
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIRegion(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf);

/**
 * @brief	设置ROI使能 - Set ROI enable
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[in]	roi_id          	   ROI组ID选择输入 - ROI group ID selection input
 * @param[in]	en     				   使能 - Enable
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, bool en);

/**
 * @brief	设置ROI阈值使能 - Set the ROI threshold to enable
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[in]	roi_id	               ROI ID选择输入 - ROI ID selection input
 * @param[in]	threshold_id 		   阈值选择输入 - Threshold selection input
 * @param[in]	en     		 		   使能 - Enable
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, bool en);

/**
 * @brief	设置ROI阈值配置 - Set ROI threshold configuration
 * @param[in]	handle->DeviceAddr     				设备地址 - Device address
 * @param[in]   handle->DeviceFd            		设备文件描述符fd - Device file descriptor fd
 * @param[in]	roi_id       						ROI ID选择输入 - ROI ID selection input
 * @param[in]	threshold_id 						阈值id选择输入 - Threshold id selection input
 * @param[in]   pix_num_threshold   				超过阈值的像素点数 - Number of pixels exceeding the threshold
 * @param[in]	hysteresis_conf.threshold_value		阈值 - Threshold
 * @param[in]	hysteresis_conf.hysteresis  		迟滞（死区）大小 - Hysteresis (dead zone) size
 * @param[in]	hysteresis_conf.positive			比较器极性，true:正向比较, false：反向比较 - Comparator polarity, true: positive comparison, false: reverse comparison
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdConf(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, uint16_t pix_num_threshold, HysteresisSingleConfTypeDef hysteresis_conf);

/**
 * @brief	获取当前设备支持的ROI数量和阈值数量 - Get the number of ROIs and thresholds supported by the current device
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @param[in]   handle->DeviceFd       设备文件描述符fd - Device file descriptor fd
 * @param[out]	roi_number      	   ROI 的数量 - Number of ROIs
 * @param[out]	threshold_number 	   阈值的数量 - Number of thresholds
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetNumberOfROI(HPS3D_HandleTypeDef *handle, uint8_t *roi_number, uint8_t *threshold_number);

/**
 * @brief	获取指定的ROI配置 - Get the specified ROI configuration
 * @param[in]	handle->DeviceAddr     									设备地址 - Device address
 * @param[in]   handle->DeviceFd            							设备文件描述符fd - Device file descriptor fd
 * @param[in]   roi_id  		       									指定ROI - Specify ROI
 * @param[out]  roi_conf			   									输出配置 - Output configuration
 * @param[out]	roi_conf.roi_id											ROI的ID - ROI ID
 * @param[out]	roi_conf.enable											ROI使能 - ROI enable
 * @param[out]	roi_conf.left_top_x										左上角x坐标 - Upper left corner x coordinate
 * @param[out]	roi_conf.left_top_y										左上角y坐标 - Upper left corner y coordinate
 * @param[out]	roi_conf.right_bottom_x									右上角x坐标 - Upper right corner x coordinate
 * @param[out]	roi_conf.right_bottom_y									右上角y坐标 - Upper right corner y coordinate
 *
 * （三组阈值输出）- (three sets of threshold outputs)
 * @param[out]	roi_conf.ref_type[threshold_id]							ROI的参考值类型 - ROI reference value type
 * @param[out]	roi_conf.alarm_type[threshold_id]						ROI的阈值警报类型 - ROI threshold alert type
 * @param[out]	roi_conf.pixel_number_threshold[threshold_id]			超过阈值的像素点数 - Number of pixels exceeding the threshold
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].threshold_id		阈值ID - Threshold ID
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].hysteresis		迟滞大小 - Hysteresis size
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].threshold_value	阈值 - Threshold
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].positive			阈值极性 - Threshold polarity
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].enable			阈值使能 - Threshold enable
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetROIConfById(HPS3D_HandleTypeDef *handle, uint8_t roi_id, ROIConfTypeDef *roi_conf);

/**
 * @brief	设置指定的GPIO输出端口的配置 - Set the configuration of the specified GPIO output port
 * @param[in]	handle->DeviceAddr       设备地址 - Device address
 * @param[in]   handle->DeviceFd         设备文件描述符fd - Device file descriptor fd
 * @param[in]	gpio_out_conf 		     GPIO配置 - GPIO configuration
 * @param[in]	gpio_out_conf.gpio	     GPIO输出口选择 - GPIO output port selection
 * @param[in]	gpio_out_conf.function   GPIO输出功能设置 - GPIO output function setting
 * @param[in]	gpio_out_conf.polarity   GPIO输出极性 - GPIO output polarity
 * @note        gpio_out_conf：只能配置IO输出 - Gpio_out_conf: can only configure IO output
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOOut(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	获取指定GPIO输出端口的配置 - Get the configuration of the specified GPIO output port
 * @param[in]	handle->DeviceAddr       设备地址 - Device address
 * @param[in]   handle->DeviceFd         设备文件描述符fd - Device file descriptor fd
 * @param[in]	gpio_out_conf->gpio	     GPIO端口号选择 - GPIO port number selection
 * @param[out]	gpio_out_conf    	     GPIO输出配置 - GPIO output configuration
 * @param[out]	gpio_out_conf.function   GPIO输出功能设置 - GPIO output function setting
 * @param[out]	gpio_out_conf.polarity   GPIO输出极性 - GPIO output polarity
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOOutConf(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef *gpio_out_conf);

/**
 * @brief	设置指定的GPIO输入端口的配置 - Set the configuration of the specified GPIO input port
 * @param[in]	handle->DeviceAddr      设备地址 - Device address
 * @param[in]   handle->DeviceFd        设备文件描述符fd - Device file descriptor fd
 * @param[in]	gpio_in_conf 		    GPIO配置 - GPIO configuration
 * @param[in]	gpio_in_conf.gpio	    GPIO输入口选择 - GPIO input selection
 * @param[in]	gpio_in_conf.function   GPIO输入功能设置 - GPIO input function setting
 * @param[in]	gpio_in_conf.polarity   GPIO输入极性 - GPIO input polarity
 * @note		gpio_in_conf：只能配置IO输入 - Gpio_in_conf: can only configure IO input
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOIn(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef gpio_in_conf);

/**
 * @brief	获取指定GPIO输入端口的配置 - Get the configuration of the specified GPIO input port
 * @param[in]	handle->DeviceAddr      设备地址 - Device address
 * @param[in]   handle->DeviceFd        设备文件描述符fd - Device file descriptor fd
 * @param[in]	gpio		  		    GPIO端口号选择 - GPIO port number selection
 * @param[out]	gpio_in_conf   	  	    获取GPIO配置的结构体指针 - Get the structure pointer of the GPIO configuration
 * @param[out]	gpio_in_conf.function   GPIO输入功能设置 - GPIO input function setting
 * @param[out]	gpio_in_conf.polarity   GPIO输入极性 - GPIO input polarity
 * @note
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOInConf(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef *gpio_in_conf);

/**
 * @brief	设置HDR模式 - Set HDR mode
 * @param[in]	handle->DeviceAddr     	设备地址 - Device address
 * @param[in]   handle->DeviceFd        设备文件fd - Device file fd
 * @param[in]	hdr_mode   		 		输入HDR的模式 - Input HDR mode
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRMode(HPS3D_HandleTypeDef *handle, HDRModeTypeDef hdr_mode);

/**
 * @brief	设置HDR - Set HDR
 * @param[in]	handle->DeviceAddr     					设备地址 - Device address
 * @param[in]   handle->DeviceFd        				设备文件描述符fd - Device file descriptor fd
 * @param[in]	hdr_conf    		   					输入HDR的配置 - Enter HDR configuration
 * @param[in]	hdr_conf.hdr_mode						HDR模式选择 - HDR mode selection
 * 1、选择AUTO-HDR： - Select AUTO-HDR:
 * @param[in]	hdr_conf.qualtity_overexposed			AUTO-HDR曝光幅值 - AUTO-HDR exposure amplitude
 * @param[in]	hdr_conf.qualtity_overexposed_serious	AUTO-HDR过度曝光幅值 - AUTO-HDR overexposure amplitude
 * @param[in]	hdr_conf.qualtity_weak					AUTO-HDR信号弱幅值 - AUTO-HDR signal weak amplitude
 * @param[in]	hdr_conf.qualtity_weak_serious			AUTO-HDR信号极弱幅值 - Very weak amplitude of AUTO-HDR signal
 * 2、选择SIMPLE-HDR： - Select SIMPLE-HDR:
 * @param[in]	hdr_conf.simple_hdr_max_integration		SIMPLE-HDR最大积分时间 - SIMPLE-HDR maximum integration time
 * @param[in]	hdr_conf.simple_hdr_min_integration		SIMPLE-HDR最小积分时间 - SIMPLE-HDR minimum integration time
 * 3、选择SUPER-HDR： - Select SUPER-HDR:
 * @param[in]	hdr_conf.super_hdr_frame_number			SUPER-HDR合成帧数 - SUPER-HDR composite frame number
 * @param[in]	hdr_conf.super_hdr_max_integration		SUPER-HDR最大积分时间 - SUPER-HDR maximum integration time
 * 4、选择HDR-DISABLE: - Choose HDR-DISABLE:
 * @param[in]	hdr_conf.hdr_disable_integration_time	HDR-DISABLE手动积分时间 - HDR-DISABLE manual integration time
 * @note		1、hdr_mode必须设置成HDR_DISABLE、AUTO_HDR、SUPER_HDR, SIMPLE_HDR,否则返回RET_ERROR; - 1, hdr_mode must be set to HDR_DISABLE, AUTO_HDR, SUPER_HDR, SIMPLE_HDR; Otherwise return RET_ERROR;
 * 			    2、其余配置按照HDRConf结构体里面的数据类型配置 - The rest of the configuration is configured according to the data type in the HDRConf structure.
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf hdr_conf);

/**
 * @brief	获取HDR配置 - Get HDR configuration
 * @param[in]	handle->DeviceAddr     					设备地址 - Device address
 * @param[in]   handle->DeviceFd        				设备文件描述符fd - Device file descriptor fd
 * @param[out]	hdr_conf	  		   					获取HDR配置 - Get HDR configuration
 * @param[out]	hdr_conf.hdr_mode						HDR模式选择 - HDR mode selection
 * @param[out]	hdr_conf.qualtity_overexposed			AUTO-HDR曝光幅值 - AUTO-HDR exposure amplitude
 * @param[out]	hdr_conf.qualtity_overexposed_serious	AUTO-HDR过度曝光幅值 - AUTO-HDR overexposure amplitude
 * @param[out]	hdr_conf.qualtity_weak					AUTO-HDR信号弱幅值 - AUTO-HDR signal weak amplitude
 * @param[out]	hdr_conf.qualtity_weak_serious			AUTO-HDR信号极弱幅值 - Very weak amplitude of AUTO-HDR signal
 * @param[out]	hdr_conf.simple_hdr_max_integration		SIMPLE-HDR最大积分时间 - SIMPLE-HDR maximum integration time
 * @param[out]	hdr_conf.simple_hdr_min_integration		SIMPLE-HDR最小积分时间 - SIMPLE-HDR minimum integration time
 * @param[out]	hdr_conf.super_hdr_frame_number			SUPER-HDR合成帧数 - SUPER-HDR composite frame number
 * @param[out]	hdr_conf.super_hdr_max_integration		SUPER-HDR最大积分时间 - SUPER-HDR maximum integration time
 * @param[out]	hdr_conf.hdr_disable_integration_time	HDR-DISABLE手动积分时间 - HDR-DISABLE manual integration time
 * @note
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf *hdr_conf);

/**
 * @brief	设置距离滤波器类型 - Set the distance filter type
 * @param[in]	handle->DeviceAddr     	  		设备地址 - Device address
 * @param[in]   handle->DeviceFd          		设备文件描述符fd - Device file descriptor fd
 * @param[in]	distance_filter_conf 	  		滤波器类型 - Filter type
 * 				-DISTANCE_FILTER_DISABLE  		距离滤波器不使能 - Distance filter is not enabled
 * 				-DISTANCE_FILTER_SIMPLE_KALMAN	简单卡尔曼滤波器 - Simple Kalman filter
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDistanceFilterType(HPS3D_HandleTypeDef *handle, DistanceFilterTypeDef distance_filter_conf);

/**
 * @brief	配置距离滤波器 - Configuring distance filter
 * @param[in]	handle->DeviceAddr     	  				设备地址 - Device address
 * @param[in]   handle->DeviceFd          				设备文件描述符fd - Device file descriptor fd
 * @param[in]	distance_filter_conf	  				距离滤波器的配置 - Distance filter configuration
 * @param[in]	distance_filter_conf.kalman_K			距离滤波器的比例系数K - Distance factor K of the distance filter
 * @param[in]	distance_filter_conf.num_check			距离滤波器阈值检查帧数 - Distance filter threshold check frame number
 * @param[in]	distance_filter_conf.kalman_threshold	距离滤波器噪声阈值 - Distance filter noise threshold
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSimpleKalman(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef distance_filter_conf);

/**
 * @brief	获取距离滤波器配置 - Get distance filter configuration
 * @param[in]	handle->DeviceAddr     	  				设备地址 - Device address
 * @param[in]   handle->DeviceFd          				设备文件描述符fd - Device file descriptor fd
 * @param[out]	distance_filter_conf   	  				距离滤波器的结构体指针 - Structure pointer of the distance filter
 * @param[out]	distance_filter_conf.filter_type		距离滤波器的类型 - Distance filter type
 * @param[out]	distance_filter_conf.kalman_K			距离滤波器的比例系数K - Distance factor K of the distance filter
 * @param[out]	distance_filter_conf.num_check			距离滤波器阈值检查帧数 - Distance filter threshold check frame number
 * @param[out]	distance_filter_conf.kalman_threshold	距离滤波器噪声阈值 - Distance filter noise threshold
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDistanceFilterConf(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef *distance_filter_conf);

/**
 * @brief	设置平滑滤波器 - Set the smoothing filter
 * @param[in]	handle->DeviceAddr     	  	设备地址 - Device address
 * @param[in]   handle->DeviceFd          	设备文件描述符fd - Device file descriptor fd
 * @param[in]	smooth_filter_conf	  	  	平滑滤波器的设置 - Smoothing filter settings
 * @param[in]	smooth_filter_conf.type		平滑滤波器的类型 - Type of smoothing filter
 * @param[in]	smooth_filter_conf.arg1		平滑滤波器的滤波参数 - Smoothing filter parameters
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSmoothFilter(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef smooth_filter_conf);

/**
 * @brief	获取平滑滤波器的配置 - Get the configuration of the smoothing filter
 * @param[in]	handle->DeviceAddr     	  	设备地址 - Device address
 * @param[in]   handle->DeviceFd          	设备文件描述符fd - Device file descriptor fd
 * @param[out]	smooth_filter_conf	  	  	平滑滤波器配置的结构体指针 - Structure pointer for smoothing filter configuration
 * @param[out]	smooth_filter_conf.type		平滑滤波器的类型 - Type of smoothing filter
 * @param[out]	smooth_filter_conf.arg1		平滑滤波器的滤波参数 - Smoothing filter parameters
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetSmoothFilterConf(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef *smooth_filter_conf);

/**
 * @brief	设定光学参数使能 - Set optical parameter enable
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	en			  			  使能信号 - Enable signal
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetOpticalEnable(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	获取光学参数 - Obtain optical parameters
 * @param[in]	handle->DeviceAddr     	 		 			设备地址 - Device address
 * @param[in]   handle->DeviceFd          					设备文件描述符fd - Device file descriptor fd
 * @param[out]	optical_param_conf	 	  					获取光学参数的结构体指针 - Get the structure pointer of the optical parameter
 * @param[out]	optical_param_conf.enable					光学参数使能 - Optical parameter enable
 * @param[out]	optical_param_conf.viewing_angle_horiz		光学参数水平方向可视角 - Optical parameter horizontal direction
 * @param[out]	optical_param_conf.viewing_angle_vertical	光学参数垂直方向可视角 - Optical parameters, vertical direction, viewing angle
 * @param[out]	optical_param_conf.illum_angle_horiz		光学参数水平方向照明发射角 - Optical parameter horizontal direction illumination emission angle
 * @param[out]	optical_param_conf.illum_angle_vertical		光学参数垂直方向照明发射角 - Optical parameter vertical direction illumination emission angle
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetOpticalParamConf(HPS3D_HandleTypeDef *handle, OpticalParamConfTypeDef *optical_param_conf);

/**
* @brief	设置距离补偿 - Set distance compensation
* @param[in]	handle->DeviceAddr     	  设备地址 - Device address
* @param[in]    handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
* @param[in]	offset		  			  补偿距离 - Compensation distance
* @note
* @retval	成功返回 RET_OK - Successfully returned RET_OK
*/
extern RET_StatusTypeDef HPS3D_SetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t offset);

/**
* @brief	获得距离补偿 - Obtain distance compensation
* @param[in]	handle->DeviceAddr     	  设备地址 - Device address
* @param[in]    handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
* @param[out]	offset		  			  输出距离补偿 - Output distance compensation
* @note
* @retval	成功返回 RET_OK - Successfully returned RET_OK
*/
extern RET_StatusTypeDef HPS3D_GetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t *offset);

/**
 * @brief	设置多机干扰检测使能 - Set multi-machine interference detection enable
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	en		  	 			  使能信号 - Enable signal
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectEn(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	设置多机干扰检测积分时间 - Set multi-machine interference detection integration time
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	us		  	  			  积分时间 - Integration time
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectIntegTime(HPS3D_HandleTypeDef *handle, uint32_t us);

/**
 * @brief	设置多机干扰检测阈值 - Set the multi-machine interference detection threshold
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	thre		  			  阈值 - Threshold
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectAmplitudeThreshold(HPS3D_HandleTypeDef *handle, uint16_t thre);

/**
 * @brief	设置多机干扰检测采样次数 - Set the number of multi-machine interference detection samples
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	capture_number  		  采样次数 - Number of samples
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectCaptureNumber(HPS3D_HandleTypeDef *handle, uint8_t capture_number);

/**
 * @brief	设置多机干扰检测采样次数检查 - Set the multi-machine interference detection sampling count check
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	capture_number_check   	  采样次数 - Number of samples
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectNumberCheck(HPS3D_HandleTypeDef *handle, uint8_t capture_number_check);

/**
 * @brief	获得多机干扰的配置 - Get multi-machine interference configuration
 * @param[in]	handle->DeviceAddr     	  						设备地址 - Device address
 * @param[in]   handle->DeviceFd          						设备文件描述符fd - Device file descriptor fd
 * @param[out]	interference_detect_conf  						获得多机干扰配置的结构体指针 - Structure pointer for multi-machine interference configuration
 * @param[out]	interference_detect_conf.enable					多机干扰检测使能 - Multi-machine interference detection enable
 * @param[out]	interference_detect_conf.integ_time				多机干扰检测积分时间 - Multi-machine interference detection integration time
 * @param[out]	interference_detect_conf.amplitude_threshold	多机干扰检测阈值 - Multi-machine interference detection threshold
 * @param[out]	interference_detect_conf.capture_num			多机干扰检测采样次数 - Multi-machine interference detection sampling times
 * @param[out]	interference_detect_conf.number_check			多机干扰检测采样次数检查 - Multi-machine interference detection sampling count check
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetInterferenceDetectConf(HPS3D_HandleTypeDef *handle, InterferenceDetectConfTypeDef *interference_detect_conf);

/**
 * @brief	设定安装角度变换使能 - Set the installation angle change enable
 * @param[in]	handle->DeviceAddr     	  设备地址 - Device address
 * @param[in]   handle->DeviceFd          设备文件描述符fd - Device file descriptor fd
 * @param[in]	en		      			  使能信号 - Enable signal
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
/*extern RET_StatusTypeDef HPS3D_SetMountingAngleEnable(HPS3D_HandleTypeDef *handle, bool en);*/ /*此接口不再使用，采用以下接口替换 - This interface is no longer used and is replaced by the following interface*/

/**
 * @brief	设定安装角度变换参数 - Set the installation angle transformation parameters
 * @param[in]	handle->DeviceAddr     	  					设备地址 - Device address
 * @param[in]   handle->DeviceFd          					设备文件描述符fd - Device file descriptor fd
 * @param[in]	mounting_angle_param_conf 					安装角度变换参数的配置 - Installation angle transformation parameter configuration
 * @param[in]	mounting_angle_param_conf.angle_vertical	垂直方向安装角度（°）- Vertical installation angle (°)
 * @param[in]	mounting_angle_param_conf.height			相对于地面的安装高度 - Mounting height relative to the ground
 * @note
 * @retval	返回设定安装角度变换参数的结果，成功返回 RET_OK - Returns the result of setting the installation angle transformation parameter, and returns RET_OK successfully.
 */
extern RET_StatusTypeDef HPS3D_SetMountingAngleParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef mounting_angle_param_conf);

/**
 * @brief	获取安装角度变换参数 - Get the installation angle transformation parameter
 * @param[in]	handle->DeviceAddr     	  					设备地址 - Device address
 * @param[in]   handle->DeviceFd          					设备文件描述符fd - Device file descriptor fd
 * @param[out]	mounting_angle_param_conf 					获得安装角度变换参数的结构体指针 - Get the structure pointer of the installation angle transformation parameter
 * @param[out]	mounting_angle_param_conf.enable			安装角度变换参数使能 - Mounting angle transformation parameter enable
 * @param[out]	mounting_angle_param_conf.angle_vertical	垂直方向安装角度（°）- Vertical installation angle (°)
 * @param[out]	mounting_angle_param_conf.height			相对于地面的安装高度(mm) - Mounting height relative to the ground (mm)
 * @note
 * @retval	返回获取安装角度变换参数的结果，成功返回 RET_OK - Returns the result of getting the installation angle transformation parameter, and returns RET_OK successfully.
 */
extern RET_StatusTypeDef HPS3D_GetMountingParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef *mounting_angle_param_conf);



/*************************************2.集成函数接口**********************************/
/*************************************2.Integrated function interface**********************************/

/**
 * @brief	获取目录下指定前缀文件(自动寻找设备) - Get the specified prefix file under the directory (automatically find the device)
 * @param[in]	dirPath 设备文件根目录 - Device file root directory
 * @param[in]   prefix  设备文件名前缀 - Device file name prefix
 * @param[out]  fileName 用于保存当前当前目录下寻找到的设备 - Used to save the device found in the current current directory
 * @note		例：n = HPS3D_GetDeviceList("/dev/","ttyACM",fileName); - example
 * @retval	返回获取成功的数量 0表示失败 - Returns the number of successful acquisitions 0 indicates failure
 */
extern uint32_t HPS3D_GetDeviceList(uint8_t * dirPath,uint8_t *prefix,uint8_t fileName[DEV_NUM][DEV_NAME_SIZE]);

/**
 * @brief	设备连接 - Device connection
 * @param[in]	handle->DeviceName  	设备名路径 - Device address
 * @param[out]	handle->DeviceFd		设备文件描述符fd - Device file descriptor fd
 * @param[out]	handle->ConnectStatus	连接状态true - Connection status true
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_Connect(HPS3D_HandleTypeDef *handle);

/**
 * @brief	断开连接 - Disconnect
 * @param[in]	handle->DeviceFd		设备文件描述符fd - Device file descriptor fd
 * @param[out]	handle->ConnectStatus	连接状态false - Connection status false
 * @note	函数执行的操作： - The operation performed by the function:
 * 			1.设置运行模式停止 - Set the run mode to stop
 * 			2.close(fd)
 * 			3.ConnectStatus = false
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_DisConnect(HPS3D_HandleTypeDef *handle);

/**
 * @brief	设备初始化 - Device initialization
 * @param[in]	handle->mode				运行模式 - Operating mode
 * @param[in]	handle->SyncMode			通讯模式 - Communication mode
 * @param[out]	handle->DeviceAddr			输出设备地址 - Output device address
 * @param[out]	handle->OpticalEnable    	输出光学使能信号true or false - Output optical enable signal true or false
 * @note	函数执行的操作： - The operation performed by the function:
 * 			1.创建线程,创建读串口数据线程 - Create a thread, create a read serial port data thread
 * 			2.先设置运行模式为停止模式（发命令时需要关闭运行模式）	HPS3D_SetRunMode(...) - First set the running mode to stop mode (you need to close the running mode when issuing commands) HPS3D_SetRunMode(...)
 * 			3.发送获得设备地址命令	HPS3D_GetDevAddr(...)	[out]handle->DeviceAddr - Send Get Device Address command HPS3D_GetDevAddr(...)	[out]handle->DeviceAddr
 * 			4.发送获得光学参数使能命令	HPS3D_GetOpticalParamConf(...)	[out]handle->OpticalEnable - Send optical parameter enable command HPS3D_GetOpticalParamConf(...)	[out]handle->OpticalEnable
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_ConfigInit(HPS3D_HandleTypeDef *handle);

/**
 * @brief	设置ROI的阈值 - Set the threshold of the ROI
 * @param[in]	handle->DeviceAddr     									设备地址 - Device address
 * @param[in]	handle->DeviceFd										设备文件描述符fd - Device file descriptor fd
 * @param[in]	threshold_id		  									阈值id - Threshold id
 * @param[in]	roi_conf			   									ROI配置结构体 - ROI configuration structure
 * @param[in]	roi_conf.roi_id											ROI的ID - ROI ID
 * @param[in]	roi_conf.ref_type[threshold_id]							ROI的参考值类型 - ROI reference value type
 * @param[in]	roi_conf.alarm_type[threshold_id]						ROI的阈值警报类型 - ROI threshold alert type
 * @param[in]	roi_conf.pixel_number_threshold[threshold_id]			超过阈值的像素点数 - Number of pixels exceeding the threshold
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].hysteresis		单点迟滞大小 - Single point hysteresis size
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].threshold_value	单点阈值 - Single point threshold
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].positive			单点阈值极性 - Single point threshold polarity
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].enable			单点阈值使能 - Single point threshold enable
 * @note		函数执行的操作： - The operation performed by the function:
 * 				1.先设置运行模式为停止模式（发命令时需要关闭运行模式）	HPS3D_SetRunMode(...) - First set the running mode to stop mode (you need to close the running mode when issuing commands) HPS3D_SetRunMode(...)
 * 				2.发送设置ROI阈值配置命令	HPS3D_SetROIThresholdConf(...) - Send Set ROI Threshold Configuration Command HPS3D_SetROIThresholdConf(...)
 * 				3.发送设置ROI的参考值类型命令	HPS3D_SetROIReferenceType(...) - Send the reference value type command to set the ROI HPS3D_SetROIReferenceType(...)
 * 				4.发送设置ROI的警报类型命令		HPS3D_SetROIAlarmType(...) - Send alert type command to set ROI HPS3D_SetROIAlarmType(...)
 * 				5.发送设置ROI阈值使能命令	HPS3D_SetROIThresholdEnable(...) - Send Set ROI Threshold Enable Command HPS3D_SetROIThresholdEnable(...)
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetThreshold(HPS3D_HandleTypeDef *handle, uint8_t threshold_id, ROIConfTypeDef roi_conf);

/**
 * @brief	设置单个ROI - Set a single ROI
 * @param[in]	handle->DeviceAddr     									设备地址 - Device address
 * @param[in]	handle->DeviceFd										设备文件描述符fd - Device file descriptor fd
 * @param[in]   roi_conf 			   									ROI的设置结构体 - ROI setting structure
 * @param[in]	roi_conf.roi_id											ROI的ID - ROI ID
 * @param[in]	roi_conf.enable											ROI使能 - ROI enable
 * @param[in]	roi_conf.left_top_x										左上角x坐标 - Upper left corner x coordinate
 * @param[in]	roi_conf.left_top_y										左上角y坐标 - Upper left corner y coordinate
 * @param[in]	roi_conf.right_bottom_x									右上角x坐标 - Upper right corner x coordinate
 * @param[in]	roi_conf.right_bottom_y									右上角y坐标 - Upper right corner y coordinate
 *
 * 设置三组阈值 - Set three sets of thresholds
 * @param[in]	roi_conf.ref_type[threshold_id]							ROI的参考值类型 - ROI reference value type
 * @param[in]	roi_conf.alarm_type[threshold_id]						ROI的阈值警报类型 - ROI threshold alert type
 * @param[in]	roi_conf.pixel_number_threshold[threshold_id]			超过阈值的像素点数 - Number of pixels exceeding the threshold
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].roi_id			单点阈值属于哪个ROI - Which ROI does the single point threshold belong to?
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].hysteresis		单点迟滞大小 - Single point hysteresis size
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].threshold_value	单点阈值 - Single point threshold
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].positive			单点阈值极性 - Single point threshold polarity
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].enable			单点阈值使能 - Single point threshold enable
 *
 * @param[in]   gpio_out_conf		  	 								GPIO配置结构体 - GPIO configuration structure
 * @param[in]	gpio_out_conf.gpio	   									GPIO输出口选择 - GPIO output port selection
 * @param[in]	gpio_out_conf.function 									GPIO输出功能设置 - GPIO output function setting
 * @param[in]	gpio_out_conf.polarity 									GPIO输出极性 - GPIO output polarity
 * @note		gpio_out_conf：只能配置IO输出 - Gpio_out_conf: can only configure IO output
 *				函数执行的操作： - The operation performed by the function:
 *				1.先设置运行模式为停止模式（发命令时需要关闭运行模式）HPS3D_SetRunMode(...) - First set the running mode to stop mode (you need to close the running mode when issuing commands) HPS3D_SetRunMode(...)
 *				2.发送设置ROI使能命令	HPS3D_SetROIEnable(...) - Send Set ROI Enable Command HPS3D_SetROIEnable(...)
 *				3.发送设定ROI区域命令	HPS3D_SetROIRegion(...) - Send Set ROI Area Command HPS3D_SetROIRegion(...)
 *				4.发送设置ROI的阈值命令（三组阈值都设置）	HPS3D_SetThreshold(...1/2/3) - Send the threshold command to set the ROI (three sets of thresholds are set) HPS3D_SetThreshold(...1/2/3)
 *				5.发送设置指定的GPIO输出端口的配置命令	HPS3D_SetGPIOOut(...) - Send configuration command to set the specified GPIO output port HPS3D_SetGPIOOut(...)
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSingleRoi(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	异步模式(连续测量模式) 添加观察者 - Asynchronous mode (continuous measurement mode) Add observer
 * @param[in]	Observer_t               		观察者结构体 - Observer structure
 * @param[in]	Observer_t->ObserverFunAddr     观察者函数地址 - Observer function address
 * @param[in]   Observer_t->NotifyEnable 		观察者使能 - Observer enable
 * @param[in]   Observer_t->AsyncEvent  		观察者通知事件 - Observer notification event
 * @param[in]   Observer_t->ObserverID   		观察者ID - Observer ID
 * @note 在退出观察者模式时必须调用移除观察者函数进行内存释放 - You must call the Remove Observer function for memory release when exiting Observer mode.
 * @retval 成功返回RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_AddObserver(void * (*fun)(HPS3D_HandleTypeDef *,AsyncIObserver_t *),HPS3D_HandleTypeDef *handle,AsyncIObserver_t *Observer_t);

/**
 * @brief	异步模式(连续测量模式) 移除观察者 - Asynchronous mode (continuous measurement mode) removes the observer
 * @param[in]	Observer_t             		  观察者结构体 - Observer structure
 * @param[in]	Observer_t->ObserverFunAddr   观察者函数地址 - Observer function address
 * @param[in]   Observer_t->ObserverID 		  观察者ID - Observer ID
 * @param[in]   Observer_t->AsyncEvent 		  观察者通知事件 - Observer notification event
 * @note
 * @retval 成功返回RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_RemoveObserver(AsyncIObserver_t *Observer_t);

/**
 * @brief	设备卸载与资源回收(线程退出与资源释放) - Device unloading and resource reclamation (thread exit and resource release)
 * @param[in]	handle->DeviceAddr     设备地址 - Device address
 * @note	函数执行的操作： - The operation performed by the function:
 * 			1.HPS3D_DisConnect(...)
 * 			2.如果handle->SyncMode == ASYNC或handle->RunMode == RUN_CONTINUOUS，则等待线程退出 - Wait for the thread to exit if handle->SyncMode == ASYNC or handle->RunMode == RUN_CONTINUOUS
 * @retval 成功返回RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_RemoveDevice(HPS3D_HandleTypeDef *handle);

/**
 * @brief		设置debug使能 - Set debug enable
 * @param[in]	en				使能信号（默认开启）- Enable signal (on by default)
 * @note
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDebugEnable(bool en);

/**
 * @brief		获得debug使能 - Get debug enable
 * @param
 * @note
 * @retval	 成功返回 true or false - Successfully returns true or false
 */
extern bool HPS3D_GetDebugEnable(void);

/**
 * @brief		接收回调函数的地址 - Receive the address of the callback function
 * @param[in]	void *Call_Back     接收回调函数地址   回调函数为void *fun(uint8_t *str, uint16_t *str_len){...} - Receive callback function address callback function is void *fun(uint8_t *str, uint16_t *str_len){...}
 * @param[out]	返回给回调函数str和strlen - Return to callback function str and strlen
 * @note
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
RET_StatusTypeDef HPS3D_SetDebugFunc(void (*Call_Back)(uint8_t *str));

/**
 * @brief	设置点云数据转换使能 - Set point cloud data conversion enable
 * @param[in]	en     使能信号 - Enable signal
 * @note
 * @retval	成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPointCloudEn(bool en);

/**
 * @brief	得到点云数据转换使能信号 - Get point cloud data conversion enable signal
 * @param
 * @note
 * @retval	true or false
 */
extern bool HPS3D_GetPointCloudEn(void);

/**
 * @brief		基本解析数据转换为点云数据输出 - Basic analytical data conversion to point cloud data output
 * @param[in]	 MeasureData.full_roi_data/MeasureData.full_depth_data		解析的数据（结构体）- Analyzed data (structure)
 * @param[in]	 RetPacketType												解析的数据类型 - Parsed data type
 * @param[out]  MeasureData.point_cloud_data								可以调用点云数据转换使能接口，设置输出的数据是否为点云数据 - You can call the point cloud data conversion enable interface to set whether the output data is point cloud data.
 * @note
 * @see
 * @code
 *
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
RET_StatusTypeDef HPS3D_BaseToPointCloud(MeasureDataTypeDef *MeasureData, RetPacketTypedef RetPacketType);

/**
 * @brief		将所有的ROI进行拼接，拼接成一张160x60的深度图 - Splicing all ROIs into a 160x60 depth map
 * @param[in]	MeasureData			需要保存的测量数据 - Measurement data that needs to be saved
 * @param[in]   RetPacketType       测量数据的类型,必须是FULL_ROI_PACKET类型 - The type of measurement data must be of type FULL_ROI_PACKET
 * @param[out]	distance     		拼接完的深度图160x60 - Spliced depth map 160x60
 * @note
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_RoiDepthCompound(MeasureDataTypeDef *MeasureData,RetPacketTypedef RetPacketType, uint16_t *distance);


/**
 * @brief		单次测量 - Single measurement
 * @param[in]	handle->DeviceAddr	设备地址 - Device address
 * @param[out]	handle->RetPacketType	测量数据的类型 - Type of measurement data
 * @param[out]	handle->MeasureData		需要保存的测量数据 - Measurement data that needs to be saved
 * @note   该方式为同步测量，即调用此函数后理解得到测量返回值 - This method is synchronous measurement, that is, after calling this function, the measured return value is understood.
 * @see
 * @code
 *
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern RET_StatusTypeDef HPS3D_SingleMeasurement(HPS3D_HandleTypeDef *handle);

/**
 * @brief		初始化障碍物提取参数 - Initialize obstacle extraction parameters
 * @param[in]	Conf	配置障碍物参数 - Configuring obstacle parameters
 * @note
 * @see
 * @code
 *
 * @retval	 成功返回 RET_OK - Successfully returned RET_OK
 */
extern ObstacleConfigTypedef HPS3D_GetObstacleConfigInit(void);

/**
 * @brief		获取障碍物提取配置参数 - Obtain obstacle extraction configuration parameters
 * @param
 * @note
 * @see
 * @code
 * @retval	 返回障碍物配置参数信息 - Return obstacle configuration parameter information
 */
extern RET_StatusTypeDef HPS3D_ObstacleConfigInit(ObstacleConfigTypedef *Conf);

/**
 * @brief		设置障碍物像素点个数阈值 - Set the threshold number of obstacle pixels
 * @param
 * @note
 * @see
 * @code
 * @retval none
 */
extern void HPS3D_SetObstaclePixelNumberThreshold(uint32_t pixel_num_thr);
/**
 * @brief		获取障碍物像素点个数阈值 - Obtain the threshold number of obstacle pixels
 * @param
 * @note
 * @see
 * @code
 * @retval  返回数量值 - Return quantity value
 */
extern uint32_t HPS3D_GetObstaclePixelNumberThreshold(void);


/**
 * @brief		设置障碍物提取阈值偏置 - Set obstacle extraction threshold offset
 * @param
 * @note
 * @see
 * @code
 * @retval  返回空 - Return empty
 */
extern void HPS3D_SetThresholdOffset(int32_t thr_offset);

/**
 * @brief		获取障碍物提取阈值偏置 - Obtain obstacle extraction threshold offset
 * @param
 * @note
 * @see
 * @code
 * @retval  阈值 - Threshold
 */
extern int32_t HPS3D_GetThresholdOffset(void);

/**
 * @brief		获取SDK版本号 - Get the SDK version number
 * @param
 * @note
 * @see
 * @code
 * @retval	 版本信息 - Version Information
 */
extern Version_t HPS3D_GetSDKVersion(void);

/* brief		将特殊测量输出值转换为指定特殊值参数配置 - Convert special measured output values to specified special value parameter configurations
 * param[in]	enable 使能设置 - Enable setting
 * param[in]    value  特殊测量结果重新赋值 - Special measurement results are reassigned
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值 - Return status value
 */
extern RET_StatusTypeDef HPS3D_ConfigSpecialMeasurementValue(bool enable,uint16_t value);

/* brief		设置边缘噪声滤除使能 - Set edge noise filtering enable
 * param[in]	enable： true使能，false关闭 - True enable, false close
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值 - Return status value
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionEnable(bool en);

/* brief		获取边缘噪声滤除使能 - Get edge noise filtering enable
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值 - Return status value
 */
extern bool HPS3D_GetEdgeDetectionEnable(void);


/* brief		设置边缘噪声阈值 - Set the edge noise threshold
 * param[in]	enable： true使能，false关闭 - True enable, false close
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值 - Return status value
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionValue(int32_t threshold_value);

/* brief		获取边缘噪声阈值 - Get the edge noise threshold
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval	 返回状态值 - Return status value
 */
extern int32_t HPS3D_GetEdgeDetectionValue(void);

/**
  * @brief	保存点云数据为ply格式文件 - Save point cloud data to ply format file
  * @param[in] filename  文件名 - file name
  * @param[in] point_cloud_data 测量结果中的电源数据 - Power data in the measurement results
  * @note
  * @see
  * @code
  * @retval 返回状态值 - Return status value
  */
/**/
extern RET_StatusTypeDef HPS3D_SavePlyFile(uint8_t *filename,PointCloudDataTypeDef point_cloud_data);

#ifdef __cplusplus
}
#endif

#endif /* API_H_ */
