/**********************************************************************************************************************
*  COPYRIGHT (c) 2021 by MaxieyeTech.                                              All rights reserved.
*  -------------------------------------------------------------------------------------------------------------------
*  FILE DESCRIPTION
*  -------------------------------------------------------------------------------------------------------------------
*         \file  MET_IPC_TSR_TRACK_BUF.h
*        \brief  Interface of TL(Traffic Light) tracker output
*
*      \details  Crc module is used to calculate cyclic codes, which encode messages by adding a fixed-length check
*
 * version : 3
 * buf_name : tsr_track 
 * project:665
 * 
 * topic : Trk/tsr_track
 * project:MDU25
*********************************************************************************************************************/

#ifndef MET_IPC_TSR_TRACK_BUF_H_
#define MET_IPC_TSR_TRACK_BUF_H_

#include <stdint.h>

/*********************************************************************************************************************
*  LOCAL CONSTANT MACROS
********************************************************************************************************************/
#define MET_SOC_TSR_TRACK_VERSION		1
#define MET_TS_OUTPUT_OBJNUM			16

/*********************************************************************************************************************
*  DATA STRUCTURE DEFINITIONS
********************************************************************************************************************/
typedef struct METTSRstElement
{
	int             trkIndex;
	short			xmin;		// coordinate on source image
	short			ymin;
	short			xmax;
	short			ymax;

	int				age;		// tracking age
	int				missCnt;
	unsigned char	isCutIn;
	unsigned char	validFlag;	// 0-non exist, 1-exsit

	float           RangeAngle;	// unit:degree
	float			wx;		// unit: m
	float			wy;
	float			wz;
	float			confidence;
	float           SignValue;
	int             EnumValue;//refer to TSR_Enums-China.xlsx
	int             type; //0-none 1-prohibition 2-warning 3-mandatory 4-guide 5-tourist
	int				supplementalType1;//refer to TSR_Enums-China.xlsx
	int				supplementalType2;//refer to TSR_Enums-China.xlsx
	float			supp1Confidence;
	float			supp2Confidence;
} METTSRstElement;

typedef struct METTSWarnElement
{
	float prevalue;
	float value;
	int valid;
	int missCnt;
	int warnNum;
	int warnflag;
} METTSWarnElement;

typedef struct METTSRstObj
{
	METTSWarnElement		HeightWarn;
	METTSWarnElement		HighspeedWarn;
	METTSWarnElement		LowspeedWarn;
	int						objCnt;
	METTSRstElement			objList[MET_TS_OUTPUT_OBJNUM];
	int						priorityIdx[MET_TS_OUTPUT_OBJNUM]; // from high to low, record the id of objList, invalid(-1)
} METTSRstObj;

typedef struct MET_SOC_TSRTrackResult
{
    unsigned int frameID;
	uint64_t	    picTimestamp;	  // camera exposure complete time
	uint64_t	    resultTimestamp;  // GO finished time

    /******Car Tracking  ********/
    METTSRstObj TSRRstObj;
} MET_SOC_TSRTrackResult;

#endif /* MET_IPC_TSR_TRACK_BUF_H_ */

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.00.00  2025-02-20  LML     Update TS result elements
*				
*  -------------------------------------------------------------------------------------------------------------------
* 
**********************************************************************************************************************/
