/*
 *    version: 4
 *    topic: Trk/object
 */

#ifndef MET_SOC_BEV_OBJ_TRACK_BUF_H_
#define MET_SOC_BEV_OBJ_TRACK_BUF_H_

/*===========================================================================*\
 * Include header files
\*===========================================================================*/
#include <stdint.h>

/*===========================================================================*\
 * Exported preprocessor
\*===========================================================================*/
#define MET_BEV_OBJ_TRACK_VERSION   4

#define BEV_OBJ_DETROI_NUM			(8)
#define BEV_OBJ_TRACK_LIST_NUM		(64)
#define BEV_OBJ_POS_PRED_NUM		(5)
#define BEV_OBJ_BEHAVIOR_NUM		(5)

/*===========================================================================*\
 * Exported type definitions
\*===========================================================================*/

typedef struct METBEVObjTrkRstElement
{
	int				status;			        // 0-invalid, 1-new, 2-update obj by both VPD and BEVObj detect, 3-missing，4-update by BEV Obj detect only, 5-update by VPD detect only
	float			confidence;		        // val[0-1], confidence of object
	int				trackID;                // id[1-64]
	int				age;                    

	/**** GOE related *******/
	float           goeMatchNum;
	float           goeMatchScore;          // val[0-1],exist_confidence judged by goeMatchNum

	uint8_t			objDetSrc;              // 0x10: VPD front, 0x01 : BEV
	uint8_t         objConfirmedFlag;       // 0-not confirmed,1-confirmed
	uint8_t			objConfirmedFlagV;      // 0-invalid ,1-valid
	
	uint8_t			potentialGhostFlag;     // 0-not ghost    1-potential ghost
	uint8_t	        coverType;				// Low->High ----bit0: f_bottom_covered, bit1: f_top_covered, bit2: f_right_covered, bit3: f_left_covered, bit4:f_bottom_partial_detect, bit5:f_top_partial_detect, bit6:f_right_partial_detect, bit7:f_left_partial_detect
	/*** other attribute ***/
	uint8_t         objClass;				// type: 0-ped, 1-bicycle, 2-car,3-van, 4-bus, 5-truck, 6-special car, 7- 3wheel
	uint8_t         BrakeLight;			    // flag of brake light: 0 - unknown, 1 - off, 2 - on
	uint8_t         TurnLight;			    // flag of turn light 0-unknown, 1-off, 2-Left on, 3-Right on, 4-all on
    uint8_t		 HazardLight;			// Reserved. flag of hazed light 0-unknown, 1 - off, 2 - on 
	
	int8_t			laneAssignment;			// The lane the target occupies:-2 = LEFT_NEXT_NEXT, - 1 = LEFT_NEXT, 0 = EGO_LANE, 1 = RIGHT_NEXT, 2 = RIGHT_NEXT_NEXT, 3 = UNKNOWN
	uint8_t			motionStatus;           // 0=Unknown, 1 = moving, 2 = Stopped, 3 = Stationary

	uint8_t         VelInitMod;             // TrkObj velocity init method 0:uninit 1:LS, 2:scale, 3:stationary, 4:overtaking, 5:crossing, 6:ego speed

	/*** attribute on vehicle coordinate ***/
	float			wWidth;		    // unit:m
	float			wHeight;        // unit:m
	float			wLength;        // unit:m

	float			longPos;		// unit:m; absolute longitudinal position; The coordinate system is based on the center of the vehicle's rear axle as the origin. 
	float			longPos_STD;	// unit:m, STD of longPos;
	float			latPos;			// unit:m, sybmol:left(+),right(-); absolute lateral position; The coordinate system is based on the center of the vehicle's rear axle as the origin. 
	float			latPos_STD;		// unit:m, STD of latPos;
	float			longSpeed;		// unit:m/s, absolute longitudinal speed; The coordinate system is based on the center of the vehicle's rear axle as the origin. 
	float			longSpeed_STD;  // unit:m/s, STD of longSpeed;
	float			latSpeed;		// unit:m/s, sybmol:left(+),right(-); absolute lateral speed; The coordinate system is based on the center of the vehicle's rear axle as the origin. 
	float			latSpeed_STD;   // unit:m/s, STD of latSpeed;
									   
	float			longAccel;		// unit:m/s^2, absolute longitudinal accel.The coordinate system is based on the center of the vehicle's rear axle as the origin. 
	float			longAccel_STD;  // unit:m/s, STD of longAccel;		                                       
    float            latAccel;        // Reserved. unit:m/s^2, absolute lateral accel.The coordinate system is based on the center of the vehicle's rear axle as the origin. 
    float            latAccel_STD;  // Reserved. unit:m/s, STD of latAccel;							   

    float			wz;				// unit:m, sybmol:up(+),down(-); The coordinate system is based on the center of the vehicle's rear axle as the origin.
									   
	float			heading;		// unit:rad, -pi~pi, N/A for ped objects
    float            headingRate;        // unit:rad, -pi~pi, N/A for ped objects


	/*** obj lane occupied percentage info ***/
	float			objLanePercentageLeft;  // The target left point percentage of lane occupied  Value: -1.5~-0.5 (LEFT_NEXT), -0.5~0.5(EGO_LANE), 0.5~1.5(RIGHT_NEXT) -100.0(UNKNOWN)
	float			objLanePercentageRight; // The target right point percentage of lane occupied  Value: -1.5~-0.5 (LEFT_NEXT), -0.5~0.5(EGO_LANE), 0.5~1.5(RIGHT_NEXT) 100.0(UNKNOWN)
	uint16_t	        objConfSets;            // low---high: bit0:pixPosConf, bit1:pixMoveConf , bit2:singleFrameConf , bit3:historyMeaConf , bit4:innoConf , bit5:blockConf , bit6:velInitConf 

	int				objBehavior[BEV_OBJ_BEHAVIOR_NUM];        // 1-lane keeping, 2-left lane change, 3-right lane change, 4-turn left, 5-turn right, 6-intersection straight, 7-left turn around, 8-right turn around, 9-stationary
	float			objBehaviorScore[BEV_OBJ_BEHAVIOR_NUM];   // val[0-1], confidence of object behavior

	int16_t			predWx[BEV_OBJ_POS_PRED_NUM];  // unit:cm, predict future longitudinal position offset relative to current frame in VCS(rear axle based)
	int16_t			predWy[BEV_OBJ_POS_PRED_NUM];  // unit:cm, predict future lateral position offset relative to current frame in VSC(rear axle based)

	uint8_t	        reserved_u8;
	uint16_t	    reserved_u16;
	float			reserved_f32;
}METBEVObjTrkRstElement;

typedef struct METBEVObjTrkRstObj
{
	int						objNum;
	METBEVObjTrkRstElement		objList[BEV_OBJ_TRACK_LIST_NUM];
}METBEVObjTrkRstObj;


/************************************************************/
/*	 BEV OBJ Track API DataStructure output define			*/
/************************************************************/
typedef struct MET_SOC_BEVOBJTrackResult
{
    unsigned int    frameId;
	uint64_t	    picTimestamp;	  // camera exposure complete time
	uint64_t	    resultTimestamp;  // GO finished time
    METBEVObjTrkRstObj trackRstObj;   // Track object list
} MET_SOC_BEVOBJTrackResult;


/*===========================================================================*\
 * File Revision History (top to bottom: first revision to last revision)
 *===========================================================================
 * Date          UserId	      Ver.No		Description
* 24.03.22      Feng.Jin	       1     	  Initial Version
* 24.05.06      LiTiankun	       2     	  change trajectory point data type from float to int16
* 24.05.22      Qin Lingguang      3           a. merge vru and veh object struct
                                               b. update object status enum definition
                                               c. update object class enum definitio
                                               d. update comments
                                               e. add reserved signals(lat accel, hazard light, etc..
                                               f. remove unused signals
* 24.12.20      Qin, Lingguang 	   4     	  remove long traj signals
\*===========================================================================*/


#endif  // MET_SOC_BEV_OBJ_TRACK_BUF_H_
