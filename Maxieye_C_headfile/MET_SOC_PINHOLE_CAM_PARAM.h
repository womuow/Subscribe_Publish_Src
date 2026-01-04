#ifndef MET_SOC_PINHOLE_CAM_PARAM_H_
#define MET_SOC_PINHOLE_CAM_PARAM_H_

#include <stdint.h>

#include "MET_DataStructure.h"

#define SOC_PINHOLE_CAM_PARAM_VERSION 3

typedef struct {
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float k6;

    float p1;
    float p2;
} PinholeDistortion;

typedef struct CameraFrontExtData
{
    uint32_t autofix_success_count;
} CameraFrontExtData;

typedef struct {
    CamExtrinsic extrinsic;
    CamIntrinsic intrinsic;
    PinholeDistortion distortion;
    MET_CamParams camParam;
} MET_SOC_PinholeCamParam;

#endif  // MET_SOC_PINHOLE_CAM_PARAM_H_