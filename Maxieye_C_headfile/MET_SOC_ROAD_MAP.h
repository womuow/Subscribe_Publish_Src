#ifndef MET_SOC_ROAD_MAP_H_
#define MET_SOC_ROAD_MAP_H_
/*===========================================================================*\
 * FILE: MET_SOC_ROAD_MAP.h
 *===========================================================================
 * Copyright 2022 MAXIEYE Technologies, Inc., All Rights Reserved.
 * MAXIEYE Confidential
 *---------------------------------------------------------------------------
 * Configuration Management Keywords:
 * % version: v6.0
 * % topic: Trk/road_map
 * % date_created: 2024.03.22
 * % author: Liu.Chang
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


#define MET_SOC_ROAD_MAP_VERSION 11
  /*===========================================================================*\
   * Exported preprocessor
   \*===========================================================================*/
#define MET_BEV_RST_MAX_POINT_NUM ((uint8_t)60u)
#define MET_BEV_RST_MAX_LINE_NUM ((uint8_t)44u)/* 16_lane_line/8_curb/16_center_line/4_stop_line*/
#define MET_BEV_RST_MAX_CONNECTIVITY_NUM ((uint8_t)4u)
#define MET_BEV_RST_MAX_ARROW_MARKER_NUM_IN_LANE ((uint8_t)6u)
#define MET_BEV_RST_MAX_ARROW_MARKER_NUM ((uint8_t)32u)
#define MET_BEV_RST_MAX_LANE_NUM ((uint8_t)10u)
#define MET_BEV_RST_MAX_LINK_NUM ((uint8_t)8u)
#define MET_BEV_RST_MAX_TRANSATION_POINT_NUM ((uint8_t)8u)
#define MET_BEV_RST_MAX_TYPE_NUM ((uint8_t)16u)
#define MET_BEV_RST_MAX_WIDTH_NUM ((uint8_t)4u)
#define MET_BEV_RST_MAX_POLY_NUM ((uint8_t)2u)
#define MET_BEV_RST_FORBIDDEN_ZONE_MAX_NUM ((uint8_t)4u)
#define MET_BEV_RST_MAX_FORK_POINT_NUM ((uint8_t)8u)
#define MET_BEV_RST_MAX_LINE_NUM_EACH_FORK ((uint8_t)8u)
#define MET_BEV_RST_MAX_ROAD_BLOCK_NUM ((uint8_t)100u)
#define MET_BEV_RST_MAX_ROAD_BLOCK_UNIT_POINT_NUM ((uint8_t)4u)
#define MET_BEV_RST_ZEBRA_ZONE_MAX_NUM ((uint8_t)8u)
#define MET_BEV_RST_MAX_ZEBRA_NUM_EACH_LINK ((uint8_t)2u)
#define MET_BEV_RST_MAX_SUB_LIGHT_NUM ((uint8_t)3u)
#define MET_BEV_RST_MAX_TRAFFIC_LIGHT_NUM ((uint8_t)16u)
/*===========================================================================*\
 * Exported type definitions
 \*===========================================================================*/
typedef enum BEVLineType
{
	IPC_LINE_TYPE_UNDETERMINED = 0,
	IPC_LINE_TYPE_SINGLE_DASH,
	IPC_LINE_TYPE_SINGLE_SOLID,
	IPC_LINE_TYPE_SINGLE_WIDE_DASH,
	IPC_LINE_TYPE_DOUBLE_DASH_SOLID,
	IPC_LINE_TYPE_DOUBLE_SOLID_DASH,
	IPC_LINE_TYPE_DOUBLE_DASH_DASH,
	IPC_LINE_TYPE_DOUBLE_SOLID_SOLID,
	IPC_LINE_TYPE_DECEL_DASH,
	IPC_LINE_TYPE_DECEL_SOLID,
	IPC_LINE_TYPE_ROAD_EDGE,
	IPC_LINE_TYPE_STOP_LINE,
	IPC_LINE_TYPE_CENTER_LINE,
	IPC_LINE_TYPE_CHEVRON,
	IPC_LINE_TYPE_GUIDE_LINE,
} BEVLineType;

typedef enum BEVTransPointType
{
	IPC_TRANS_POINT_TYPE_UNDETERMINED = 0,
	IPC_TRANS_POINT_TYPE_SPLIT,
	IPC_TRANS_POINT_TYPE_MERGE,
	IPC_TRANS_POINT_TYPE_TYPE_CHANGE,
	IPC_TRANS_POINT_TYPE_DISCONTINUITIES
}BEVTransPointType;

typedef enum BEVTLShapeSrc
{
	BEV_TL_SOURCE_DETECT = 0,
	BEV_TL_SOURCE_MAP,
	BEV_TL_SOURCE_MAP_AND_DETECT,
}BEVTLShapeSrc;

typedef enum BEVTLColor
{
	BEV_TL_COLOR_UNKNOWN = 0,
	BEV_TL_COLOR_RED,
	BEV_TL_COLOR_YELLOW,
	BEV_TL_COLOR_RED_AND_YELLOW,
	BEV_TL_COLOR_GREEN,
	BEV_TL_COLOR_GREEN_AND_RED,
	BEV_TL_COLOR_YELLOW_AND_GREEN,
	BEV_TL_COLOR_RED_AND_YELLOW_AND_GREEN,
}BEVTLColor;

typedef enum BEVTLShape
{
	BEV_TL_SHAPE_UNKNOWN = 0,
	BEV_TL_SHAPE_UP,
	BEV_TL_SHAPE_LEFT,
	BEV_TL_SHAPE_RIGHT,
	BEV_TL_SHAPE_U_TURN,
	BEV_TL_SHAPE_CIRCLE,
	BEV_TL_SHAPE_DOWN,
	BEV_TL_SHAPE_NUMBER,
	BEV_TL_SHAPE_INVALID,
}BEVTLShape;

typedef enum BEVHostPosState
{
	IPC_HOST_POS_STATE_UNDETERMINED = 0,
	IPC_HOST_POS_STATE_DETECTED,
	IPC_HOST_POS_STATE_PREDICTED
} BEVHostPosState;

typedef enum BEVLocationSourceType
{
	IPC_Location_Source_Type_UNDETERMINED = 0,
	IPC_Location_Source_Type_BEV_LLD,
	IPC_Location_Source_Type_LLD,
	IPC_Location_Source_Type_Forbidden_Zone
} BEVLocationSourceType;

typedef enum BEVLineState
{
	IPC_LINE_STATE_UNDETERMINED = 0,
	IPC_LINE_STATE_DETECTED,
	IPC_LINE_STATE_PREDICTED
} BEVLineState;

typedef enum BEVArrowState
{
	IPC_ARROW_STATE_UNDETERMINED = 0,
	IPC_ARROW_STATE_DETECTED,
	IPC_ARROW_STATE_PREDICTED
}BEVArrowState;

typedef enum BEVArrowType
{
	IPC_ARROW_TYPE_INVALID = 0,
	IPC_ARROW_TYPE_RIGHT_TURN,
	IPC_ARROW_TYPE_RIGHT_MERGE,
	IPC_ARROW_TYPE_LEFT_MERGE,
	IPC_ARROW_TYPE_LEFT_TURN,
	IPC_ARROW_TYPE_MERGE,
	IPC_ARROW_TYPE_STRAIGHT,
	IPC_ARROW_TYPE_U_TURN,
	IPC_ARROW_TYPE_FORBID_U_TURN,
} BEVArrowType;

typedef enum BEVCrossRoadState
{
	IPC_NOT_CROSS_ROAD = 0,
	IPC_BEFORE_CROSS_ROAD,
	IPC_IN_CROSS_ROAD,
	IPC_AFTER_CROSS_ROAD,
	IPC_SIMLR_CROSS_ROAD,
}BEVCrossRoadState;

typedef enum BEVForbiddenZoneType
{
	IPC_ZONE_NOT_SURE,
	IPC_SPLIT_ZONE,
	IPC_MERGE_ZONE,
}BEVForbiddenZoneType;

typedef enum BEVForkPointType
{
	IPC_FORK_UNDETERMINED = 0,
	IPC_FORK_BRANCH_POINT,
	IPC_FORK_SINK_POINT,
	IPC_FORBIDDEN_BRANCH_POINT,
	IPC_FORBIDDEN_SINK_POINT,
	IPC_STOP_LINE_POINT,
} BEVForkPointType;

typedef enum BEVForkPointState
{
	IPC_FORK_POINT_STATE_INVALID = 0,
	IPC_FORK_POINT_STATE_DETECTED,
	IPC_FORK_POINT_STATE_PREDICTED,
}BEVForkPointState;

typedef enum BEVRoadBlockState
{
	IPC_BLOCK_STATE_UNDETERMINED = 0,
	IPC_BLOCK_STATE_DETECTED,
	IPC_BLOCK_STATE_PREDICTED,
}BEVRoadBlockState;

typedef enum BEVLaneStableState
{
	IPC_BEV_LANE_INVALID_STATE = 0,
	IPC_BEV_LANE_STABLE,
	IPC_BEV_LANE_UNSTABLE,
}BEVLaneStableState;

typedef enum BEVRoadBlockType
{
	IPC_BLOCK_TYPE_UNDECIDED = 0,
	IPC_BLOCK_TYPE_CONE,
	IPC_BLOCK_TYPE_WATER_SAFETY_BARRIER,
	IPC_BLOCK_TYPE_ROAD_SAFETY_BARRIER,
	IPC_BLOCK_TYPE_CRUSH_CUSHION_CYLINDER,
	IPC_BLOCK_TYPE_WARNING_POLE,
	IPC_BLOCK_TYPE_EMERGENCY_REFLECTIVE_TRIANGLE,
	IPC_BLOCK_TYPE_NEAR_CHEVRON_CONE,
	IPC_BLOCK_TYPE_CYLINDRICAL_WARNING_POST,
} BEVRoadBlockType;

typedef enum BEVRoadBlockSourceType
{
	IPC_BLOCK_Source_Type_BEV,
	IPC_BLOCK_Source_Type_GOE,
	IPC_BLOCK_Source_Type_BEV_GOE,
} BEVRoadBlockSourceType;

typedef struct BEVArrowMarkerInfo
{
	BEVArrowType type; /* arrow type*/
	uint8_t trackID;	   /* arrow id*/
	int16_t wx; /* range in x direction in VCS, in centimeters */
	int16_t wy; /* range in y direction in VCS, in centimeters */
} BEVArrowMarkerInfo;

typedef struct BEVPoint3D
{
	int16_t wx; /* range in x direction in VCS, in centimeters */
	int16_t wy; /* range in y direction in VCS, in centimeters */
	int16_t wz; /* range in z direction in VCS, in centimeters */
} BEVPoint3D;

typedef struct BEVPoint2D
{
	int16_t wx; /* range in x direction in VCS, in centimeters */
	int16_t wy; /* range in y direction in VCS, in centimeters */
} BEVPoint2D;

typedef struct BEVLineTypeInfo
{
	int16_t btnDist; /* bottom distance of current type in VCS, in centimeters */
	int16_t topDist; /* top distance of current type in VCS, in centimeters */
	uint8_t conf; /* confidence of current type, [0 100] */
	uint8_t btnPtIdx; /* bottom point index of current type in line point list */
	uint8_t topPtIdx; /* top point index of current type in line point list */
	BEVLineType	type; /* line type information */
	bool f_covered;/*indicate whether the type segment is covered by vehicles*/
	BEVArrowMarkerInfo arrowMarkerList[MET_BEV_RST_MAX_ARROW_MARKER_NUM_IN_LANE];/*indicate the arrow information when line type is guide line*/
	uint8_t arrowMarkerNum;/*indicate the arrow number when line type is guide line*/
	bool f_btnFarAwayGuideLine;/*indicate whether the guide line is above of the cross road's start dist*/
}BEVLineTypeInfo;

typedef struct BEVLineWidthInfo
{
	int16_t	btnDist; /* bottom distance of current width in VCS, in centimeters */
	int16_t	topDist; /* top distance of current width in VCS, in centimeters */
	uint8_t	conf; /* confidence of current width, [0 100] */
	int16_t	width; /* width value, in centimeters */
	uint8_t		btnPtIdx; /* bottom point index of current width in line point list */
	uint8_t		topPtIdx; /* top point index of current width in line point list */
}BEVLineWidthInfo;

typedef struct BEVLinePolyInfo
{
	int16_t	btnDist; /* bottom distance of current polynomial in VCS, in centimeters */
	int16_t	topDist; /* top distance of current polynomial in VCS, in centimeters */
	uint8_t	conf; /* confidence of current polynomial, [0 100] */
	float	coeff[4]; /* polynomial coefficient in VCS, y=coeff[0]+coeff[1]*x+coeff[2]*x^2+coeff[3]*x^3 */
	uint8_t	btnPtIdx; /* bottom point index of current polynomial in line point list */
	uint8_t	topPtIdx; /* top point index of current polynomial in line point list */
}BEVLinePolyInfo;

typedef struct BEVLineInfo
{
	BEVPoint3D ptList[MET_BEV_RST_MAX_POINT_NUM]; /* point information of current line */
	BEVLinePolyInfo polyList[MET_BEV_RST_MAX_POLY_NUM]; /* poly information of current line's inner side, only use for LKA function */
	BEVLineTypeInfo typeList[MET_BEV_RST_MAX_TYPE_NUM];  /* type information of current line */
	BEVLineWidthInfo widthList[MET_BEV_RST_MAX_WIDTH_NUM]; /* width information of current line */
	int16_t btnDist; /* bottom distance of current line, in centimeters */
	int16_t topDist; /* top distance of current line, in centimeters */
	uint8_t conf; /* confidence of current line, [0 100] */
	int8_t id; /* track id of current line */
	uint8_t ptNum; /* point number in ptList */
	uint8_t polyNum; /* poly number in polyList */
	uint8_t typeNum; /* type number in typeList */
	uint8_t widthNum; /* width number in widthList */
	uint16_t coastAge; /* indicate how many frames current line has been coasted */
	uint16_t updateAge; /* indicate how many frames current line has been updated */
	BEVLineState state; /* current line state */
} BEVLineInfo;

typedef struct BEVLineBuffer
{
	BEVLineInfo lineList[MET_BEV_RST_MAX_LINE_NUM]; /* line information */
	uint8_t lineNum; /* line number */
}BEVLineBuffer;

typedef struct BEVTLSubInfo
{
	//Image Coordinates(2D-Box)
	uint16_t xMin;
	uint16_t yMin;
	uint16_t xMax;
	uint16_t yMax;
	uint8_t subColor; //color of sub traffic light 0: red, 1: yellow, 2: green
	uint8_t status;// the status of sub traffic light 0: not matched; 1: matched;
	bool f_isNumber; // whether the sub light is count-down number or not  0:not_number; 1:number
	uint8_t part; //which side of traffice light in TL group 0: left(up), 1: middle, 2: right(down)
	uint8_t countDown; // count down for sub traffic light
}BEVTLSubInfo;

typedef struct BEVTLInfo
{
	//Image Corrdinates(2D-Box)
	uint16_t xMin;
	uint16_t yMin;
	uint16_t xMax;
	uint16_t yMax;
	uint8_t conf;	//confidence of traffic light group;
	uint8_t trkIdx; // track id of traffic light group

	// BEV View position (VCS)
	float rangeAngle; // the angle from host vehichle to the traffic light
	int16_t wx;
	int16_t wy;
	int16_t wz;

	BEVTLSubInfo subLight[MET_BEV_RST_MAX_SUB_LIGHT_NUM]; //sub traffic light in TL group, each group has three sub lights;
	BEVTLColor preColor;// prev state of traffic light color 
	bool f_isFlashing;// whether the traffic light is flashing or not;
	bool f_countDownValid; // whether traffic light group is in count down state or not;
	uint8_t countDown;// count down of traffic light group

	BEVTLShapeSrc shapeSrc; // sorce of traffic light shape 0: detect 1: map 2: map and detect
	BEVTLColor color;	//color of current state of TL  0: unknown 1: red 2: yellow 3: red and yellow 4: green 5: red and green 6: yellow_and_green 7: red, yellow and green
	BEVTLShape shape; //tracffic light shape 0: unknown 1: up straight 2: left 3: right 4: u_turn 5: circle, 6: down straight 7:invalid
	bool f_isCutIn; //whether tracffic light is cut in by other object or not;
	bool f_isValid; // valid flag of traffic light group;
}BEVTLInfo;

typedef struct BEVLaneConnectInfo
{
    int8_t targetLinkID; /* link id */
    int8_t targetLaneID; /* lane id in target link */
} BEVLaneConnectInfo;

typedef struct BEVLaneWidthPropertyInfo
{
    bool f_isOverWideLane; /* parrellel OverWiden Lane*/
    bool f_isWidenLane; /* gradually Widen Lane */
    bool f_isMergeLane; /* gradually Merge Lane */
    int16_t overWidenLaneRange; /* when lane is recognized as over widen lane, output middle distance of over widen lanes in a link, in centimeters */
    int16_t widenLaneRange; /* when lane is recognized as gradually widen lane, output distance when the lane width gradually increased, in centimeters */
    int16_t mergeLaneRange;/* when lane is recognized as gradually merge lane, output distance when the lane width gradually decreased, in centimeters */
}BEVLaneWidthPropertyInfo;

typedef struct BEVLinkLaneInfo
{
    BEVLaneConnectInfo connectList[MET_BEV_RST_MAX_CONNECTIVITY_NUM]; /* indicate connection relationship between current link line to next link lane */
    BEVArrowMarkerInfo arrowMarkerList[MET_BEV_RST_MAX_ARROW_MARKER_NUM_IN_LANE]; /* arrow marker in current link lane */
    BEVLaneWidthPropertyInfo laneProperty; /* record lane type related to lane width: 1. Over Widen Lane, 2. gradually Widen Lane, 3. gradually Merge Lane*/
    BEVLaneStableState laneStableState;
    int16_t btnDist; /* bottom distance of current link lane in VCS, in centimeters */
    int16_t topDist; /* top distance of current link lane in VCS, in centimeters */
    int8_t id; /* id of current link lane, 0:host lane, 1:left lane, -1:right lane, 2:left left lane, -2:right right lane*/
    int8_t leftLineID; /* track id of lane left line */
    int8_t rightLineID; /* track id of lane right line */
    int8_t leftRoadEdgeID; /* track id of lane left road edge */
    int8_t rightRoadEdgeID; /* track id of lane right road edge */
    int8_t centerLineID;/* track id of center line */
    uint8_t leftLineBtnPtIdx; /* bottom point index of left line in current link lane */
    uint8_t leftLineTopPtIdx; /* top point index of left line in current link lane */
    uint8_t rightLineBtnPtIdx; /* bottom point index of right line in current link lane */
    uint8_t rightLineTopPtIdx; /* top point index of right line in current link lane */
    uint8_t leftRoadEdgeBtnPtIdx; /* bottom point index of left road edge in current link lane */
    uint8_t leftRoadEdgeTopPtIdx; /* top point index of left road edge in current link lane */
    uint8_t rightRoadEdgeBtnPtIdx; /* bottom point index of right road edge in current link lane */
    uint8_t rightRoadEdgeTopPtIdx; /* top point index of right road edge in current link lane */
    uint8_t arrowMarkerNum; /* arrow number in current link lane */
    uint8_t connectNum; /* indicate how many lanes in next link are connected with current link lane */
    bool f_isValid; /* indicate if current link lane is valid */
    bool f_waitingZone;/*indicate if lane is waiting zone*/
    bool f_oppositeLane;/*indicate if lane is opposite lane*/
} BEVLinkLaneInfo;

typedef struct BEVLinkInfo
{
    BEVLinkLaneInfo laneList[MET_BEV_RST_MAX_LANE_NUM]; /* lane information in current link */
    BEVCrossRoadState crossRoadState; /* indicate if the link is in cross road */
    int16_t btnDist; /* bottom distance of link in VCS, in centimeters */
    int16_t topDist; /* bottom distance of link in VCS, in centimeters */
    int8_t id; /* id of current link */
    uint8_t nextLinkNum; /* indicate how many links are connected with current link */
    uint8_t nextLinkID[MET_BEV_RST_MAX_LINK_NUM]; /* connected link ID list */
    uint8_t laneNum; /* indicate how many link lanes in current link */
    uint8_t zebraNum;/* indicate how many zebra zone in current link */
    uint8_t zebraID[MET_BEV_RST_MAX_ZEBRA_NUM_EACH_LINK];/*zebra id list*/
    bool f_isValid; /* indicate if current link is valid */
    bool f_isStable;/* indicate if after_cross_road link is stable */
} BEVLinkInfo;

typedef struct BEVLinkBuffer
{
    BEVLinkInfo linkInfo[MET_BEV_RST_MAX_LINK_NUM];
    uint8_t linkNum;
    int8_t currentLinkIdx; /* the link id that host is located */
} BEVLinkBuffer;

typedef struct BEVTransPointInfo
{
    int16_t range;
    int8_t trackID;
    uint8_t connectLineNum;
    BEVTransPointType type;
} BEVTransPointInfo;

typedef struct BEVTransPointBuffer
{
    BEVTransPointInfo transPointInfo[MET_BEV_RST_MAX_TRANSATION_POINT_NUM];
    uint8_t transPointNum;
} BEVTransPointBuffer;

typedef struct BEVHostPosition
{
    uint8_t conf; /* value range [0 100], indicate how much this position could be trusted */
    BEVHostPosState state; /* indicate this position is detected or predicted */
    uint8_t laneNum; /* indicate how many lanes at host left or right */
    uint16_t age; /* indicate how many frames this host position is detected or predicted */
    bool f_valid; /* indicate if this information is valid, base on road edge info */
	BEVLocationSourceType sourceType; /* indicate this host position's source type  */
} BEVHostPosition;

/*
 * record total lane number based on historical statistic
 * define the lane that host is in by counting how many lanes at left side and right side
 * for example, total 5 lanes, and host is in the second lane which counts from left to right
 */
typedef struct BEVLocationBuffer
{
    BEVHostPosition hostPosLeft; /* host position information, counting from left to right */
    BEVHostPosition hostPosRight; /* host position information, counting from right to left */
    uint8_t trkLaneNum; /* indicate how many lanes are detected and tracked */
} BEVLocationBuffer;

typedef struct BEVArrowInfo
{
    BEVArrowType type;
    BEVArrowState state;
    BEVPoint2D position;
    uint8_t trackID;
}BEVArrowInfo;

typedef struct BEVArrowBuffer
{
    BEVArrowInfo arrowList[MET_BEV_RST_MAX_ARROW_MARKER_NUM];
    uint8_t arrowNum;/* indicate how many arrows are tracked */
}BEVArrowBuffer;

typedef struct BEVZebraInfo 
{
    BEVPoint2D point1; /*four corner point of zebra zone*/
    BEVPoint2D point2;
    BEVPoint2D point3;
    BEVPoint2D point4;
    uint8_t trackID;
    float theta; /*indicate the angle between zebra main direction and wy positive direction*/
}BEVZebraInfo;

typedef struct BEVZebraCrossBuffer
{
    BEVZebraInfo zebraList[MET_BEV_RST_ZEBRA_ZONE_MAX_NUM];
    uint8_t zebraNum;
}BEVZebraCrossBuffer;

typedef struct BEVForbiddenZoneInfo 
{
    uint8_t track_id;
    int16_t wx;
    int16_t wy;
    int8_t leftLineIdx;
    int8_t rightLineIdx;
    BEVForbiddenZoneType type;
}BEVForbiddenZoneInfo;

typedef struct BEVForbiddenZoneBuffer 
{
    BEVForbiddenZoneInfo forbiddenZoneList[MET_BEV_RST_FORBIDDEN_ZONE_MAX_NUM];
    uint8_t forbiddenZoneNum;
}BEVForbiddenZoneBuffer;

typedef struct BEVForkPointInfo
{
	BEVForkPointState state;
	BEVForkPointType  type;
	int16_t wx;
	int16_t wy;
	uint8_t id;
	uint8_t aboveLineIdx[MET_BEV_RST_MAX_LINE_NUM_EACH_FORK]; /* the line above fork points with the order from left to right */
	uint8_t belowLineIdx[MET_BEV_RST_MAX_LINE_NUM_EACH_FORK]; /* the line below fork points with the order from left to right */
	uint8_t aboveNum; /* the number of the line above fork points */
	uint8_t belowNum; /* the number of the line below fork points */
}BEVForkPointInfo;

typedef struct BEVForkPointBuffer
{
	BEVForkPointInfo forkPointInfo[MET_BEV_RST_MAX_FORK_POINT_NUM];
	uint8_t forkPointNum;
}BEVForkPointBuffer;

typedef struct BEVRoadBlockInfo
{
    uint8_t id;
    uint8_t pointNum;
    BEVRoadBlockState state;
    BEVRoadBlockType type;
	BEVRoadBlockSourceType sourceType; /* indicate this block is verified by which source type  */
	BEVPoint2D pointList[MET_BEV_RST_MAX_ROAD_BLOCK_UNIT_POINT_NUM];
}BEVRoadBlockInfo;

typedef struct BEVRoadBlockBuffer
{
    uint8_t blockNum;
    BEVRoadBlockInfo blockList[MET_BEV_RST_MAX_ROAD_BLOCK_NUM];
}BEVRoadBlockBuffer;

typedef struct BEVTLBuffer
{
	BEVTLInfo tlList[MET_BEV_RST_MAX_TRAFFIC_LIGHT_NUM];
	uint8_t tlNum;
}BEVTLBuffer;

typedef struct MET_SOC_BEVRoadMapResult
{
	uint32_t frameID;
	uint64_t pic_time_stamp;
	uint64_t result_time_stamp;

	BEVLineBuffer lineBuffer;
	BEVLinkBuffer linkBuffer;
    BEVTransPointBuffer transPointBuffer;
    BEVLocationBuffer locationBuffer;
    BEVArrowBuffer arrowBuffer;
    BEVForbiddenZoneBuffer forbiddenZoneBuffer;
	BEVForkPointBuffer forkPointBuffer;
    BEVRoadBlockBuffer roadBlockBuffer;
    BEVZebraCrossBuffer zebraZoneBuffer;
	BEVTLBuffer trafficLightBuffer;
} MET_SOC_BEVRoadMapResult;

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
   * 24.03.22    Liu.Chang	  Initial Version
   * 24.08.21    Heng.Hao     Remove autofix buffer from this topic, and upgrade version to v6.0
   * 25.08.27    Heng.Hao     Remove redundant structure and tune member size, upgrade version from v10 to v11
   \*===========================================================================*/

#endif /* MET_SOC_ROAD_MAP_H_ */
