#ifndef MET_SOC_MAGNA_PARAM_CS_H
#define MET_SOC_MAGNA_PARAM_CS_H

#include "stdint.h"

/* service name: magna_param */
/*****************************************************************************************************************************
**                                                  Revision Control History                                                **
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                                   */
/*  version:  1     2025-12-11     YW                      Initial version                                                  */
/*  version:  2     2025-12-18     YW                      1.Add req TAC input param; 2.Delete fov param                    */
/*  version:  3     2025-12-26     YW                      1.Add param fault; 2. Add client error code                      */
/****************************************************************************************************************************/

#define MET_SOC_MAGNA_PARAM_CS_VERSION 3
#define MET_SOC_MAGNA_CALIB_MAX_APRILTAG_NUM 10

// req type
enum class MagnaParamReqType : uint8_t {

	// cam param
    SendIntParam = 0,           // meta_req_data: MagnaCamIntParam 	     meta_res_data: null
    ReqIntParam = 1,            // meta_req_data: null               	 meta_res_data: MagnaCamIntParam

	// tac calib
    ReqTACInputParam = 2,       // meta_req_data: null 				     meta_res_data: MagnaTACCalibInput
    ReqTACResult = 3,	        // meta_req_data: null 				     meta_res_data: MagnaCamExtParam
    ReqTACCanStart = 4,	        // meta_req_data: null 				     meta_res_data: null
    StartTAC = 5,		        // meta_req_data: null 				     meta_res_data: null
    SendTACResult = 6,          // meta_req_data: MagnaCalibResult       meta_res_data: null

	// spc calib
    ReqSPCResult = 7,	        // meta_req_data: null 				     meta_res_data: MagnaCamExtParam
    ReqSPCCanStart = 8,         // meta_req_data: null 				     meta_res_data: null
    StartSPC = 9,               // meta_req_data: null 				     meta_res_data: null
    SendSPCResult = 10,         // meta_req_data: MagnaCalibResult       meta_res_data: null

	// autofix
    ReqAutofixResult = 11,	    // meta_req_data: null 				     meta_res_data: MagnaCamExtParam
    SendAutofixResult = 12,     // meta_req_data: MagnaCalibResult       meta_res_data: null

    // power down
    PowerDown = 13,             // meta_req_data: MagnaClientStatusCode  meta_res_data: null

    // fault
    ParamValid = 14,            // meta_req_data: null 				     meta_res_data: null
    ParamInvalid = 15           // meta_req_data: null 				     meta_res_data: null

};


// flag
enum class MagnaStatusCode : uint8_t {
    STATUS_OK = 0,
    STATUS_NOK = 1,
    START_COND_NOT_SATISFIED = 2,
    DTC_CALIB_IMPACT_EXISTS = 3,
    SAVE_PARAM_FAILED = 4,
    DTC_CLEAR_FAILED = 5
};

enum class MagnaClientStatusCode : uint8_t {
    STATUS_OK = 0,
    STATUS_NOK = 1,
    PowerDownSaveParamFailed = 2
};

// param
typedef struct MagnaCamExtParam {
    float yaw;
    float roll;
    float pitch;
} MagnaCamExtParam;

typedef struct MagnaCamIntParam{
    /* Common */
    float pixelPitch_X;
    float pixelPitch_Y;
    float focalLength;
    float focalLength_X;
    float focalLength_Y;
    float undistort_focalLength;
	float undistort_focalLength_X;
	float undistort_focalLength_Y;

    /* distort image info */
    int16_t distort_imgWidth;
    int16_t distort_imgHeight;
    float distort_principalPoint_X;
    float distort_principalPoint_Y;

    /* undistort image info */
    int16_t undistort_imgWidth;
    int16_t undistort_imgHeight;
    float undistort_principalPoint_X;
    float undistort_principalPoint_Y;

    /* distortion*/
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float k6;

    float p1;
    float p2;

} MagnaCamIntParam;


// tac input param
typedef struct MagnaAprilTag_Info
{
	float plate_pos_x; // unit: m
	float plate_pos_y; // unit: m
	float plate_pos_z; // unit: m
	float plate_width; // unit: m
	int16_t april_tag_id;
} MagnaAprilTag_Info;

typedef struct MagnaTACCalibInput
{
	// camera pose
    float cam_pos_x;   // unit: m
    float cam_pos_y;   // unit: m
    float cam_pos_z;   // unit: m

	// calibration physical parameters
	MagnaAprilTag_Info april_tag_info[MET_SOC_MAGNA_CALIB_MAX_APRILTAG_NUM];
	int16_t april_tag_num;

} MagnaTACCalibInput;



// calib
enum class MagnaCalibResultCode : uint8_t {
    CALIB_SUCCESS = 0,             // 标定成功
    CALIB_FAILURE = 1,             // 标定失败
    CALIB_EXCEED_STANDARD = 2,     // 标定超差
    CALIB_PARAM_SAVE_FAILED = 3    // 保存标定结果失败
};

typedef struct MagnaCalibResult {
    MagnaCalibResultCode result_code;
    float yaw;
    float roll;
    float pitch;
} MagnaCalibResult;


// req,res
typedef struct MagnaParamCSReq {

	MagnaParamReqType req_type;
    uint8_t meta_req_data[256];

} MagnaParamCSReq;

typedef struct MagnaParamCSRes {

    bool is_success;                // 是否收到此次请求， true: YES, false: NO
    MagnaStatusCode status_code;
    uint8_t meta_res_data[256];

} MagnaParamCSRes;

#endif // MET_SOC_MAGNA_PARAM_CS_H