#ifndef MET_SOC_CAMS_PARAM_H_
#define MET_SOC_CAMS_PARAM_H_

#include "met/MET_DataStructure.h"
#include "MET_SOC_FISHEYE_CAM_PARAM.h"
#include "MET_SOC_PINHOLE_CAM_PARAM.h"
#include "met/MET_CameraIdx.h"

/* topic name: ParamServer/cams_param */
/*****************************************************************************************************************************
**                                                  Revision Control History                                                **
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                                   */
/*  version:  1     2025-09-11     YW                      Initial version                                                  */
/****************************************************************************************************************************/
#define MET_SOC_CAMS_PARAM_VERSION (1u)

enum class CameraDistortType {
    PINHOLE,
    FISHEYE
};

enum class  CamParamUpdateType {
    TAC,
    SPC,
    AUTOFIX,
    Init        ///< param_server init
};

typedef union {
    MET_SOC_PinholeCamParam pinholeCam;
    MET_SOC_FisheyeCamParam fisheyeCam;
} MET_SOC_CamParamData;

typedef struct MET_SOC_CamParam
{
    bool support_update;
    CameraDistortType distort_type;
    MetCamIdx cam_idx;

    MET_SOC_CamParamData data;

} MET_SOC_CamParam;

typedef struct MET_SOC_CamsParam
{
    uint32_t update_count;                          ///< Number of updates from TAC、SPC、Autofix、Init. update_count=1 when param_server inited

    CamParamUpdateType update_type;                 ///< Type of update

    uint16_t cams_num;                              ///< Number of cams

    MET_SOC_CamParam cams[MetCamIdx::kMetCamNum];

} MET_SOC_CamsParam;

#endif  // MET_SOC_CAMS_PARAM_H_