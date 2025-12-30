#ifndef MET_SOC_EXTRINSIC_H_
#define MET_SOC_EXTRINSIC_H_
#include <stdint.h>
#include <iostream>
#include "MET_CameraIdx.h"
/*topic name: Veh/Extrinsic*/
/*****************************************************************************************************************************
**                                                  Revision Control History                                                                   **
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                          */
/*  version:  1     2025-09-01     SH                      Initial version                                                  */
/****************************************************************************************************************************/
#define MET_EXTRINSIC_HEADFILE_VERSION (1u)

#define MET_CAM_NUM (11u)

/* CamParam[0]      [1]        [2]        [3]        [4]        [5]        [6]        [7]        [8]        [9]        [10]
        CAM-7       CAM-8      CAM-2      CAM-12     CAM-3      CAM-4      CAM-5      CAM-6      CAM-9      CAM-10     CAM-11     
        前视/主视    前视/30°    后视        环视/前环   环视/后环    环视/左环   环视/右环   侧视/左前    侧视/右前   侧视/左后   侧视/右后                      
*/



typedef struct MET_SOC_IMU_PARAM
{
    float IMU_x; /*unit mm    range(-2000 ~ 2000)*/
    float IMU_y; /*unit mm    range(-10000 ~ 10000)*/
    float IMU_z; /*unit mm    range(0 ~ 5000)*/
    float IMU_x_Rotation; /*unit:degree (-90~ 90)*/
    float IMU_y_Rotation; /*unit:degree (-180 ~ 180)*/
    float IMU_z_Rotation; /*unit:degree (-180 ~ 180)*/
    float IMU_x_Rotation_Cal; /*unit:degree (-90 ~ 90)*/
    float IMU_y_Rotation_Cal; /*unit:degree (-180 ~ 180)*/
    float IMU_z_Rotation_Cal; /*unit:degree (-180 ~ 180)*/
    uint32_t IMU_Type;
}MET_SOC_IMU_PARAM;




typedef struct MET_SOC_CAM_PARAM
{
    float cam_x; /*unit mm*/
    float cam_y; /*unit mm*/
    float cam_z; /*unit mm*/
    float cam_pitch; /*unit degree*/
    float cam_yaw; /*unit degree*/
    float cam_roll; /*unit degree*/
}MET_SOC_CAM_PARAM;


typedef struct MET_SOC_LIDAR_PARAM
{
    float lidar_x; /*unit mm*/
    float lidar_y; /*unit mm*/
    float lidar_z; /*unit mm*/
    float lidar_pitch;/*unit degree*/
    float lidar_yaw;/*unit degree*/
    float lidar_roll;/*unit degree*/
}MET_SOC_LIDAR_PARAM;



typedef struct MET_SOC_EXTRINSIC
{
    uint32_t frameId;
    uint64_t timestamp;
    MET_SOC_IMU_PARAM ImuParam;
    MET_SOC_LIDAR_PARAM LidarParam;
    MET_SOC_CAM_PARAM CamParam[MET_CAM_NUM]; 
}MET_SOC_EXTRINSIC;
#endif
