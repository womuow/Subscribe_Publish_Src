/**********************************************************************************************************************
*  COPYRIGHT (c) 2021 by MaxieyeTech.                                              All rights reserved.
*  -------------------------------------------------------------------------------------------------------------------
*  FILE DESCRIPTION
*  -------------------------------------------------------------------------------------------------------------------
*         \file  MET_SOC_TL_TRACK_BUF.h
*        \brief  Interface of TL(Traffic Light) tracker output
*
*      \details  Crc module is used to calculate cyclic codes, which encode messages by adding a fixed-length check
*
 * version :4
 * topic : Trk/tl_track
*********************************************************************************************************************/

#ifndef MET_SOC_TL_TRACK_BUF_H_
#define MET_SOC_TL_TRACK_BUF_H_

/*********************************************************************************************************************
 *  LOCAL CONSTANT MACROS
 ********************************************************************************************************************/
#define MET_TL_OUTPUT_OBJNUM			32		//the same as trackList
#define MET_TL_SUB_PART_NUM				3
#define MET_SOC_TL_TRACK_VERSION		4
/*********************************************************************************************************************
 *  DATA STRUCTURE DEFINITIONS
 ********************************************************************************************************************/
typedef enum METTLShapeSrc
{
	TL_SOURCE_DETECT,
	TL_SOURCE_MAP,
	TL_SOURCE_MAP_AND_DETECT,
}METTLShapeSrc;

typedef enum METTLColor
{
	TL_COLOR_UNKNOWN=0,
	TL_COLOR_RED,
	TL_COLOR_YELLOW,
	TL_COLOR_REDandYELLOW,
	TL_COLOR_GREEN,
	TL_COLOR_REDandGREEN,
	TL_COLOR_YELLOWandGREEN,
	TL_COLOR_REDandYELLOWandGREEN,
	TL_COLOR_BLACK,
}METTLColor;

typedef enum METTLShape
{
	TL_SHAPE_UNKNOWN,
	TL_SHAPE_UP,
	TL_SHAPE_LEFT,
	TL_SHAPE_RIGHT,
	TL_SHAPE_U_TURN,
	TL_SHAPE_CIRCLE,
	TL_SHAPE_DOWN,
	TL_SHAPE_NUMBER,
	TL_SHAPE_INVALID
}METTLShape;

typedef struct METTLSubRstElement
{
	float xmin;
	float ymin;
	float xmax;
	float ymax;
	int subColor;//0: R; 1: Y; 2: Gl
	int status; // 0-not matched, 1-matched
	unsigned char f_isNumber; //0-not number, 1-number
	int part; // 0-left(up),1-middle,2-right(down)
	int countdown;
}METTLSubRstElement;

typedef struct METTLRstElement
{
	short			xmin;
	short			ymin;
	short			xmax;
	short			ymax;

	//30deg 
	float			xmin3, xmax3, ymin3, ymax3;			//unit:pixel, the coordinate of 2DBox on image
	unsigned char    camMask; // bit0: 120 deg front cam, bit1: 30 deg front cam

	//J6M add attribution
	METTLColor      preColor;
	unsigned char		isFlashing;
	METTLSubRstElement partRst[MET_TL_SUB_PART_NUM];
	int countdown;
	unsigned char f_CountValid;


	int			age;
	int			missCnt;
	METTLShapeSrc	shapeSrc; //0:detect 1:map 2:map and detect
	METTLColor		color;	
	METTLShape		shape;  // 0:unknown 1:up straight 2:left 3:right 4:Uturn 5:circle 6:down straight 7:invalid
	unsigned char		isCutIn;
	unsigned char		validFlag;
	float                conf;//TL type confidence

	float                   RangeAngle;
	float                   wx;
	float                   wy;
	float                   wz;
} METTLRstElement;

typedef struct METTLRstObj
{
	int				objCnt;
	METTLRstElement			objList[MET_TL_OUTPUT_OBJNUM];
	int				priorityIdx[MET_TL_OUTPUT_OBJNUM]; //from high to low, record the id of objList, invalid(-1)
} METTLRstObj;

typedef struct MET_SOC_TLTrackResult
{
	unsigned int 			frameID;

	/******Car Tracking  ********/
	METTLRstObj				TLRstObj;
} MET_SOC_TLTrackResult;

#endif /* MET_SOC_TL_TRACK_BUF_H_ */

/**********************************************************************************************************************
*  REVISION HISTORY
*  -------------------------------------------------------------------------------------------------------------------
*  Version   Date        Author  Require Id     Description
*  -------------------------------------------------------------------------------------------------------------------
*  01.05.00  2021-07-16  CYG      SNKP-500       Update TL result elements:
*						1. Change shape description
*						2. Add 'conf'
*                       3. Change color description
**********************************************************************************************************************/
