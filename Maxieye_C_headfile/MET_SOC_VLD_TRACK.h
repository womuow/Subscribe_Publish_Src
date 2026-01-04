/*
 *  version_idx:1
 *	topic: Trk/vld_track
 */

#ifndef MET_IPC_VLD_TRACK_BUF_H
#define MET_IPC_VLD_TRACK_BUF_H

#include <stdint.h>

/************************************************************/
/*					output macro define					    */
/************************************************************/

#define MET_SOC_VLD_TRACK_VERSION 1

#define MET_VLD_OUTPUT_MAXNUM		(80)
#define MET_ADB_OUTPUT_MAXNUM		(5)

/************************************************************/
/*					enum type definition				    */
/************************************************************/

typedef enum METVLDLampClass
{
	MET_VLD_LAMPCLASS_NONE = 0,
	MET_VLD_LAMPCLASS_HEAD,
	MET_VLD_LAMPCLASS_TAIL,
	MET_VLD_LAMPCLASS_HEADPAIR,
	MET_VLD_LAMPCLASS_TAILPAIR,
	MET_VLD_LAMPCLASS_TRUCKCABINTOP,
	MET_VLD_LAMPCLASS_WEAKONCOMING,
	MET_VLD_LAMPCLASS_CLUSTER,
	MET_VLD_LAMPCLASS_SIDELONG,
	MET_VLD_LAMPCLASS_VRU,

	/****** SIZE *******/
	MET_VLD_LAMPCLASS_SIZE,
}METVLDLampClass;

typedef enum METADBLampClass
{
	MET_ADB_LAMPCLASS_NONE = 0,
	MET_ADB_LAMPCLASS_ONCOMING,
	MET_ADB_LAMPCLASS_PRECEDING,
	MET_ADB_LAMPCLASS_CLUSTER,
	MET_ADB_LAMPCLASS_PEDESTRIAN,
	MET_ADB_LAMPCLASS_INIT,
	MET_ADB_LAMPCLASS_RESERVED1,
	MET_ADB_LAMPCLASS_RESERVED2,

	/****** SIZE *******/
	MET_ADB_LAMPCLASS_SIZE,
}METADBLampClass;

/************************************************************/
/*	   Vehicle Light Tracking API DataStructure define		*/
/************************************************************/
typedef struct METVLDTrkRstElement
{
	unsigned int	    id;
	unsigned int    age;
	int				status;			//0-invalid, 1-new, 2-update obj by measure, 3-missing
	float			leftAngle;
	float			rightAngle;
	float			topAngle;
	float			bottomAngle;
	float			longPos;
	float           latPos;
	float			x;		         // unit:pixel, horizontal coordinate
	float			y;		         // unit:pixel, vertical coordinate
	float			width;		     // unit:pixel, width
	float			height;		     // unit:pixel, height
	unsigned char   detSrc;           //0-only VLD, 1-only OBJ, 2-VLD and OBJ
	METVLDLampClass classification;
} METVLDTrkRstElement;

typedef struct METVLDTrkRstObj
{
	int					objNum;
	METVLDTrkRstElement objList[MET_VLD_OUTPUT_MAXNUM];
} METVLDTrkRstObj;

typedef struct METADBTrkRstElement
{
	unsigned int	    id;
	unsigned int    age;
	int				status;			//0-invalid, 1-new, 2-update obj by measure, 3-missing
	float			leftAngle;
	float			rightAngle;
	float			longPos;
	float           latPos;
	float			latRelVel;       // lateral relative velocity
	METADBLampClass classification;
} METADBTrkRstElement;

typedef struct METADBTrkRstObj
{
	int					objNum;
	METADBTrkRstElement objList[MET_ADB_OUTPUT_MAXNUM];
} METADBTrkRstObj;

typedef struct MET_SOC_VldTrackResult
{
	unsigned int	frameId;
	uint64_t	    picTimestamp;	  		// camera exposure complete time
	uint64_t	    resultTimestamp;  		// VLD finished time
	unsigned char	ahbcAvailable;		   // 0 = Day, 1 = Night under light, 2 = Night in dark
	unsigned char	beamRequest;		   // 0 = OFF, 1 = ON
	unsigned char	inTunnel;			   // 0 = UNKNOWN, 1 = FALSE, 2 = TRUE
	unsigned char   streetLampAvailable;   // 0 = Not Available, 1 = Street Lamp ON
	unsigned char   noValidVehFlag;        // 0 = exist valid vehicle, 1 = no valid vehicle
	unsigned char   noValidPdrFlag;        // 0 = exist valid VRU, 1 = no valid VRU
	METVLDTrkRstObj vldRstObj;
	METADBTrkRstObj adbRstObj;
} MET_SOC_VldTrackResult;

#endif /* MET_IPC_VLD_TRACK_BUF_H */

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.00.00  2025-02-20  
**********************************************************************************************************************/
