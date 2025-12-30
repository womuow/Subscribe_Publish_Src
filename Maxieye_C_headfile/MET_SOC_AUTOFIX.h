#ifndef MET_SOC_AUTOFIX_H_
#define MET_SOC_AUTOFIX_H_
/*===========================================================================*\
 * FILE: MET_SOC_AUTOFIX.h
 *===========================================================================
 * Copyright 2022 MAXIEYE Technologies, Inc., All Rights Reserved.
 * MAXIEYE Confidential
 *---------------------------------------------------------------------------
 * Configuration Management Keywords:
 * % version: v3.0
 * % topic: Autofix/camera
 * % date_created: 2025.09.05
 * % author: Heng.Hao
 *---------------------------------------------------------------------------
 * DESCRIPTION:
 *   This is a header file for autofix result of front camera.
 *
 * TRACEABILITY INFO:
 *   Design Document(s): None
 *
 *   Requirements Document(s): None
 *
 *   Applicable Standards:
 *     "C_Coding_Best_Pratice" (https://ofxohbirwc.feishu.cn/docs/doccnTkZgyrXbTRt7kHmHJWjkMh)
 *
 * DEVIATIONS FROM STANDARDS:
 *   None.
 \*===========================================================================*/

 /*===========================================================================*\
  * Include header files
  \*===========================================================================*/
#include <stdint.h>


/*===========================================================================*\
* Exported preprocessor
\*===========================================================================*/
#define MET_SOC_AUTOFIX_VERSION 3
/*===========================================================================*\
 * Exported type definitions
 \*===========================================================================*/
typedef struct BEVAutofix_Debug_Front_Info
{
	bool f_vp_error;		  // true means vanish point can't be caculated in current frame
	bool f_insufficient_lane; // true means host lanes don't meet calibration condition in current frame
	bool f_large_yawrate;	  // true means vehicle's yaw rate too large in current frame, host does't drive in line
	bool f_low_speed;		  // true means vehicle's speed too low in current frame, which doesn't meet calibration condition
	bool f_large_heading;	  // true means angle between lane line and VCS X axis is too large, host doesn't drive along lane line in current frame
	bool f_invalid_line_type; // true means host left lane or right's lane type isn't single line in current frame
	bool f_large_wz;		  // true means road gradient changes rapidly in VCS, which could because road surface doesn't flat, or host pitch doesn't stable in current frame
	bool f_not_single_line;	  // true means lane line edge detection doesn't stable in current frame
	bool f_invalid_roll;	  // true means roll calibration failed in current frame
	float reserve[20];
} BEVAutofix_Debug_Front_Info;

typedef struct BEVAutofix_Debug_Side_Info
{
	bool f_not_single_line;	  // true means lane line edge detection doesn't stable in current frame
	bool f_invalid_point_num; // true means no enough lane line edge points for calibration in current frame
	bool f_invalid_front;	  // true means front camera calibration condition could not be fulfilled
	bool f_invalid_roll;	  // true means roll calibration failed in current frame
	bool f_invalid_pitch;	  // true means pitch calibration failed in current frame
	float reserve[20];
} BEVAutofix_Debug_Side_Info;

typedef struct BEVAutofix_Debug_Back_Info
{
	bool f_vp_error; 		 // true means vanish point can't be caculated in current frame
	bool f_not_single_line;	 // true means lane line edge detection doesn't stable in current frame
	bool f_invalid_front;	 // true means front camera calibration condition could not be fulfilled
	bool f_invalid_roll;	 // true means roll calibration failed in current frame
	float reserve[20];
} BEVAutofix_Debug_Back_Info;

typedef struct BEVAutofixDebugBuffer
{
	BEVAutofix_Debug_Front_Info camF_debug_info; // debug message for front camera
	BEVAutofix_Debug_Back_Info camFN_debug_info; // debug message for front narrow camera
	BEVAutofix_Debug_Back_Info camB_debug_info;	 // debug message for back camera
	BEVAutofix_Debug_Side_Info camLF_debug_info; // debug message for left front camera
	BEVAutofix_Debug_Side_Info camRF_debug_info; // debug message for right front camera
	BEVAutofix_Debug_Side_Info camLB_debug_info; // debug message for left back camera
	BEVAutofix_Debug_Side_Info camRB_debug_info; // debug message for right back camera
	BEVAutofix_Debug_Back_Info AVMF_debug_info;	 // debug message for AVM front camera
	BEVAutofix_Debug_Back_Info AVMB_debug_info;	 // debug message for AVM back camera
	BEVAutofix_Debug_Side_Info AVML_debug_info;	 // debug message for AVM left camera
	BEVAutofix_Debug_Side_Info AVMR_debug_info;	 // debug message for AVM right camera
}BEVAutofixDebugBuffer;

typedef struct BEVAutofix_Info
{
	float roll; 			 	  // roll calibration result, calibration range is [initRoll-2, initRoll+2] degree
	float yaw;					  // yaw calibration result, calibration range is [initYaw-2, initYaw+2] degree
	float pitch; 				  // pitch calibration result, calibration range is [initPitch-2, initPitch+2] degree
	float percentage; 			  // current camera calibration process
	float confidence; 			  // current camera calibration result confidence
	bool f_is_in_autofix_process; // true means current camera is calibrated in current project
	bool f_calib_valid;			  // true means calibration has been successed since powerd on
	bool f_calib_timeout;		  // true means current calibration process is timeout
	bool f_current_frame_valid;	  // true means current frame meets calibration conditions
}BEVAutofix_Info;

typedef struct BEVAutofixBuffer
{
	float confidence; 			// the confidence of current multiple cameras calibration process result
	bool f_calib_valid; 		// indicate whether calibration process has been successed in current power-on cycle: 1:been successed; 0:been not successed
	BEVAutofix_Info camF_info;  // front camera calibration result
	BEVAutofix_Info camFN_info; // front narrow camera calibration result
	BEVAutofix_Info camB_info;	// back camera calibration result
	BEVAutofix_Info camLF_info; // left front camera calibration result
	BEVAutofix_Info camRF_info; // right front camera calibration result
	BEVAutofix_Info camLB_info; // left back camera calibration result
	BEVAutofix_Info camRB_info; // right back camera calibration result
	BEVAutofix_Info AVMF_info;	// AVM front camera result
	BEVAutofix_Info AVMB_info;	// AVM back camera result
	BEVAutofix_Info AVML_info;	// AVM left camera result
	BEVAutofix_Info AVMR_info;	// AVM right camera result
} BEVAutofixBuffer;

typedef struct MET_SOC_BEVAutofixResult
{
	unsigned int frameID;
	uint64_t pic_time_stamp;
	uint64_t result_time_stamp;
	BEVAutofixBuffer autofixBuffer;			  // autofix main output structure
	BEVAutofixDebugBuffer autofixDebugBuffer; // autofix debug message
} MET_SOC_BEVAutofixResult;

/*===========================================================================*\
 * Exported object declaration
 \*===========================================================================*/


 /*===========================================================================*\
  * Exported function prototypes
  \*===========================================================================*/

/*===========================================================================*\
 * File Revision History (top to bottom: first revision to last revision)
 *===========================================================================
 * Date        UserId       Description
 * 25.09.05    Heng.Hao	 Add comments and type fix
 *===========================================================================
 * Date        UserId       Description
 * 25.08.09    Heng.Hao	 AVM Autofix Version
 *===========================================================================
 * Date        UserId       Description
 * 24.08.21    Heng.Hao	 Initial Version
 \*===========================================================================*/

#endif /* MET_SOC_AUTOFIX_H_ */
