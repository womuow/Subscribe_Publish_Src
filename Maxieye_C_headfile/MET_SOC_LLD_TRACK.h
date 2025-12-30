#ifndef MET_SOC_LLD_TRACK_H_
#define MET_SOC_LLD_TRACK_H_
/*===========================================================================*\
 * FILE: MET_SOC_LLD_TRACK.h
 *===========================================================================
 * Copyright 2022 MAXIEYE Technologies, Inc., All Rights Reserved.
 * MAXIEYE Confidential
 *---------------------------------------------------------------------------
 * Configuration Management Keywords:
 * % version: v1.0
 * % topic: Trk/lld_trk
 * % date_created: 2025.09.02
 * % author: Zhang.Xianzhe
 *---------------------------------------------------------------------------
 * DESCRIPTION:
 *   This is a header file for road map result.
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

//"topic_name":"Trk/lld_trk"
#define MET_SOC_LLD_TRACK_VERSION 1

/*************************************************************/
/*				Macro Define							     */
/*************************************************************/
#define MET_SOC_LLD_TRANSPOINT_MAXNUM (8)
#define MET_SOC_LLD_LINE_MAXNUM (8)
#define MET_SOC_LLD_STOP_LANE_MAXNUM (4)
#define MET_SOC_LLD_CURB_LANE_MAXNUM (4)
#define MET_SOC_LLD_ZEBRA_MAXNUM (6)

/*************************************************************/
/*				DataStructure Define						 */
/*************************************************************/
/*TransitionPoints*/
typedef struct LLDTransPntBuffer_T
{
	float 			longPos;	// The longitudinal position of the transition point
	float 			latPos;	// The lateral position of the transition point
	float 			conf;		// Transition probability
	float 			floatPad[4];
	int 			trackId;
	unsigned char 	type;		// 0-Unknown, 1-SplitPoint, 2-MergePoint, 3-SCurvePoint
	unsigned char 	laneRole; // Indicate to which lane marker this point is related: bit 0 = left lane marker, bit 1 = right lane marker, bit 2 = left left lane marker, bit 3 = right right lane marker, bit 4-7: not used
	unsigned char 	uCharPad[6];
}LLDTransPntBuffer;

/*Multilane Output Lane*/
typedef struct LLDLineBuffer_T
{
	float			A[4];			// A[3]*(x^3)+A[2]*(x^2)+A[1]*x+A[0]
	float			laneWidth;		// the width of single lane;
	float			conf;
	float			btnDistTrk;		// bottom distance tracked in x direction
	float			topDistTrk;		// top
	float			btnDistDet;	// detect bottom distance in x direction
	float			topDistDet;	// top
	float			updateDist;		// lane has been updated for x m
	float			coastDist;		// lane has been coasted for x m
	float			floatPad[20];

	int				trackId;
	int				updateAge;	// lane has been udpated for x frames;
	int				coastAge;	// lane has been coasted for x frames;
	int 			intPad[20];

	unsigned char	state;			// 0-Unavailable, 1-PredictedLaneMarker, 2-DetectedLaneMarker
	unsigned char	lineType; 		// 0-Dashed 1-Solid 2-Undecided 3-DLM 4-Botts 5-Decel 6-Invalid
	unsigned char	dlmType;		// 0-NotDLM 1-SolidDashed 2-DashedSolid 3-SolidSolid 4-DashedDashed 5-Undecided_DLM
	unsigned char	decelType;		// 0-NotDecel 1-SolidDecel 2-DashedDecel 3-Undecided_Decel 4-Invalid_Decel 		
	unsigned char	f_SCurve;		// 0:not S curve 1:confirmed as S curve
	unsigned char	f_DashTurnSolid;// 1:means dash lane is about to turn Solid
	unsigned char	f_pendTurn;		// 1:means pend turn lane above stop lane
	unsigned char	f_straightLane;	// 0:the lane is not straight, 1: the lane is straight
	unsigned char	f_offsetMode; 	//0-closerMode, 1-innerMode.
	unsigned char	f_cutIn;		// 1 = cutIn
	unsigned char	f_interrupt;	// 1 = interrupt
	unsigned char	f_rapidCurve;	// 1 means far away Road is about to be curve road.
	unsigned char	uCharPad[15];
}LLDLineBuffer;

/*Stop Lane*/
typedef struct LLDStopBuffer_T
{
	float 			latPos;//the stop line form: y=tan(angle)*(x-lateralDistance)+longitudinalDistance.
	float 			longPos;
	float 			conf;
	float 			angle;
	float 			leftBoarder;//leftmost lateral corrdinate of the stop line.
	float 			rghtBoarder;//rightmost lateral corrdinate of the stop line.
	float 			floatPad[4];
	
	int 			updateAge;
	int 			coastAge;
	int 			trackId;
	int 			intPad[4];
	
	unsigned char	isRelevant;	// whether the stop line affects the host car.
	unsigned char 	state;		//UNKNOWN = 0,IN_IMAGE = 1,PREDICTED = 2
	unsigned char 	uCharPad[12];
}LLDStopBuffer;

/*Zebra Zone*/
typedef struct LLDZebraBuffer_T
{
	float       	refPointX;				//left bottom point
	float			refPointY;
	float           zoneHeading;        	//zebra zone slope
	float			segmentHeading;			//zebra segment slope
	float			width;					//zebra width
	float			height;                 //zebra height
	float			conf;	            	//zebra confidence
	float			floatPad[3];
	int            	trackId;				//zebra track id
	int				updateAge;
	int				coastAge;
	int				intPad[3];
	unsigned char	zebraType;				//0:zebra crossing, 1:solid crossing, 2:dashed crossing, 3:solid-or-dash crossing, 4:non-crosing
	unsigned char	uCharPad[3];
}LLDZebraBuffer;

/*Curb*/
typedef struct LLDCurbBuffer_T
{
	float			A[4];
	float			conf;			// 1.0 0.75 0.5 0.25 0.1	0
	float			btnDistTrk;		// trakced botton distance in x directio
	float			topDistTrk;		// trakced top distance in x direction
	float			btnDistDet;		// detected
	float			topDistDet;  	// detected
	float			floatPad[4];
	int				trackId;
	int				updateAge;		// curb has been udpated for x frames;
	int				coastAge;		// curb has been coasted for x frames;
	int				intPad[3];
	unsigned char	type;			//0 means normal curb, 1 means cone, 2 means shuima, 3 means yuantong, 4 means SIZE
	unsigned char	state;			// 0-Unavailable, 1-PredictedLaneMarker, 2-DetectedLaneMarker
	unsigned char	uCharPad[6];
}LLDCurbBuffer;

typedef struct LLDOtherInfoBuffer_T
{
	unsigned char 	f_laneChangeLeft;  // 1 means lane change left, 0 means no lane change left
	unsigned char 	f_laneChangeRight; // 1 means lane change right, 0 means no lane change right	
	unsigned char 	uCharPad[6];
}LLDOtherInfoBuffer;


typedef struct MET_SOC_LLD_Track_Result_T
{
	unsigned int			frameID;
	uint64_t				picTimestamp;		//camera exposure complete time
	uint64_t				resultTimestamp;	//LLD lane finished time
	LLDTransPntBuffer		transPntBuffer[MET_SOC_LLD_TRANSPOINT_MAXNUM];
	LLDLineBuffer			lineBuffer[MET_SOC_LLD_LINE_MAXNUM];
	LLDStopBuffer			stopBuffer[MET_SOC_LLD_STOP_LANE_MAXNUM];
	LLDCurbBuffer			curbBuffer[MET_SOC_LLD_CURB_LANE_MAXNUM];
	LLDZebraBuffer			zebraBuffer[MET_SOC_LLD_ZEBRA_MAXNUM];
	LLDOtherInfoBuffer		otherInfo;
}MET_SOC_LLD_Track_Result;

/*===========================================================================*\
 * File Revision History (top to bottom: first revision to last revision)
 *===========================================================================
 * Date        UserId       	Description
 * 25.09.02    Zhang.Xianzhe	Initial Version
\*===========================================================================*/
#endif
