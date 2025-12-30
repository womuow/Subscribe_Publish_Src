/*
*  version_idx:4
*	topic: Trk/cor_track
*/

#ifndef MET_SOC_COR_BUF_H
#define MET_SOC_COR_BUF_H

#include <stdint.h>

#define MET_SOC_COR_VERSION 4

typedef struct MET_SOC_CameraBlockInfo
{
	int camIdx;                
	float coverArea;           /* Proportion of the camera's field of view that is occluded, ranging from 0.0 to 1.0. */
	unsigned char isCovered;   /* Indicates whether the camera is covered. 0 means not covered, 1 means covered. */
}MET_SOC_CameraBlockInfo;

typedef struct MET_SOC_CameraCheckRst
{
	unsigned int    frameId;						//frame index of the image
	uint64_t	    picTimestamp;	  				// camera exposure complete time
	uint64_t	    resultTimestamp;  				// COR finish time

	MET_SOC_CameraBlockInfo  cameraBlockInfo[11];	//array of occlusion information for six cameras
} MET_SOC_CameraCheckRst;

#endif/*MET_SOC_COR_BUF_H*/

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.00.00  2025-02-21  LJK     Update COR result elements
*  02.00.00  2025-07-10  LJK	 Adapt COR result to 6 cameras system
*  03.00.00  2025-08-14  LJK	 Add camera exposure time and COR finish time
*  04.00.00  2025-08-21  LJK     Max camera number pair with met/MET_CameraIdx.h
*  -------------------------------------------------------------------------------------------------------------------
*
**********************************************************************************************************************/
