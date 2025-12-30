#ifndef MET_SOC_REFLINE_HPPINFO_H
#define MET_SOC_REFLINE_HPPINFO_H
#include <stdint.h>
#pragma pack(4)
#define REFLINE_HPP_MAJOR_VER 2
#define REFLINE_HPP_MINOR_VER 0

#define REFLINE_HPP_MAX_POLY_ORDER 8
#define REFLINE_HPP_MAX_POLY_NUM 5
#define REFLINE_HPP_MAXPOINTNUM_SINGLELINE 100
#define REFLINE_HPP_MAX_LANETURNTYPE_NUM 5
#define REFLINE_HPP_BEVLINEOBJECT_MAX_POINTNUM 120
#define REFLINE_HPP_MAX_ROADBLOCK_NUM 50
#define REFLINE_HPP_MAX_ROADBLOCK_POINTNUM 10


#ifndef ENV_SIMULINK_IMPORT

#ifndef ENUM_REFLINE_HPP_TRKSTS
#define ENUM_REFLINE_HPP_TRKSTS
typedef  uint64_t REFLINE_HPP_UINT64;
enum class REFLINE_HPP_TRKSTS : unsigned char {
	INVALID = 0,
	DETECTED = 1,
	COASTED = 2,
};
#endif

#ifndef ENUM_REFLINE_HPP_MERGESRC
#define ENUM_REFLINE_HPP_MERGESRC
enum class REFLINE_HPP_MERGESRC : unsigned char {
	INVALID = 0,
	PERCEPTION = 1,
	EHRMERGE = 2,
	CONEBASED = 3,
	EHRROADENTRY = 4,
};
#endif

#ifndef ENUM_REFLINE_HPP_MERGEDIR
#define ENUM_REFLINE_HPP_MERGEDIR
enum class REFLINE_HPP_MERGEDIR : signed char {
	LEFT_HOST = -5,//Left Merge to centerline itself
	LEFT_ADJ_4 = -4,
	LEFT_ADJ_3 = -3,
	LEFT_ADJ_2 = -2,
	LEFT_ADJ_1 = -1,
	INVALID = 0,
	RIGHT_ADJ_1 = 1,
	RIGHT_ADJ_2 = 2,
	RIGHT_ADJ_3 = 3,
	RIGHT_ADJ_4 = 4,
	RIGHT_HOST = 5,//Right Merge to centerline itself
	UNDECIDED = 127,
};
#endif

#ifndef ENUM_REFLINE_HPP_LANE_TURN_TYPE
#define ENUM_REFLINE_HPP_LANE_TURN_TYPE 
enum class REFLINE_HPP_LANETURNTYPE : unsigned char {
	INVALID = 0,
	STRAIGHT = 1,
	LEFT = 2,
	RIGHT = 3,
	LEFT_TURN_AROUND = 4,
	RIGHT_TURN_AROUND = 5,
	LEFT_TURN_AROUND_FORBIDDEN = 6,
	RIGHT_TURN_AROUND_FORBIDDEN = 7,
};
#endif

#ifndef ENUM_REFLINE_HPP_LINETYPE
#define ENUM_REFLINE_HPP_LINETYPE
enum class REFLINE_HPP_LINETYPE : unsigned char {
	INVALID = 0,
	SINGLE_DASH = 1,
	SINGLE_SOLID = 2,
	SINGLE_WIDE_DASH = 3,
	DOUBLE_DASH_SOLID = 4,
	DOUBLE_SOLID_DASH = 5,
	DOUBLE_DASH_DASH = 6,
	DOUBLE_SOLID_SOLID = 7,
	DECEL_DASH = 8,
	DECEL_SOLID = 9,
	ROAD_EDGE = 10,
	CHEVRON = 11,
	GUIDE_LINE = 12,
	RESERVERD13 = 13,
	RESERVERD14 = 14,
	RESERVERD15 = 15,
	RESERVERD16 = 16,
	UNDETERMINED = 255
};
#endif

#ifndef ENUM_REFLINE_HPP_ROADBLOCK_SRC_TYPE
#define ENUM_REFLINE_HPP_ROADBLOCK_SRC_TYPE 
enum class REFLINE_HPP_ROADBLOCKSRCTYPE : unsigned char {
	INVALID = 0,
	BEV = 1,
	GOE = 2,
	BEV_GOE = 3,
};
#endif

#else
typedef unsigned char REFLINE_HPP_LINETYPE;
typedef unsigned char REFLINE_HPP_MERGESRC;
typedef unsigned char REFLINE_HPP_TRKSTS;
typedef unsigned char REFLINE_HPP_ROADBLOCKSRCTYPE;
typedef unsigned char REFLINE_HPP_LANETURNTYPE;
typedef signed char   REFLINE_HPP_MERGEDIR;
typedef  double REFLINE_HPP_UINT64;
#endif

#ifndef REFLINE_HPP_SOFTWAREINFO_T
#define REFLINE_HPP_SOFTWAREINFO_T
typedef struct {
	uint8_t		RefLine_SW_Ver[2];//Reline的软件版本号
	uint8_t		RefLine_Platform;//Refline的硬件平台
	uint8_t		RefLine_ARC;//Refline的AliveRollingCnt，0-255滚动
	uint8_t		RefLine_ElapsedTimeMs;//Refline软件的耗时
	uint32_t	RefLine_FrameID;//Refline的FrameID
	uint32_t	RefLineStartTimeMs;//Refline开始运行时的时间戳
	uint32_t	RefLineEndTimeMs;//Refline运行结束时的时间戳
	uint32_t	RefLineOverallElapsedTimeMs;//从前视摄像头曝光到Refline结束的总耗时
	uint32_t	Reserved_1;
	uint32_t	Reserved_2;
}RefLine_HPP_SoftwareInfo_T;
#endif

#ifndef METTOOLDISPLAYPROPERTY_T
#define METTOOLDISPLAYPROPERTY_T
typedef struct {
	uint8_t MET_DispColor_R;
	uint8_t MET_DispColor_G;
	uint8_t MET_DispColor_B;
	uint8_t	MET_DispWidth;//cm
}MetToolDisplayProperty_T;//For MET TOOL Disp Only
#endif

#ifndef REFLINE_HPP_POINTINFO_T
#define REFLINE_HPP_POINTINFO_T
typedef struct {
	float	X;//中线X位置(m)，正方向以Maxieye标准坐标系为准
	float	Y;//中线Y位置(m) ，正方向以Maxieye标准坐标系为准
	int8_t	LeftBoundryId;//左侧安全边界的要素ID
	REFLINE_HPP_LINETYPE   LeftSafetyBoundryType;//左侧安全边界的线型
	uint8_t	LeftLineWidthCm;//左侧安全边界的线宽(cm)
	uint8_t	Reserved_1;
	float	LeftSafetyBoundry;//左侧安全边界与中线的距离(m),当左线在中线左侧，此值为正值
	int8_t	RightBoundryId;//右侧安全边界的要素ID
	REFLINE_HPP_LINETYPE	RightSafetyBoundryType;//右侧安全边界的线型
	uint8_t	RightLineWidthCm;//右侧安全边界的线宽(cm)
	uint8_t	Reserved_2;
	float	RightSafetyBoundry;//右侧安全边界与中线的距离(m),当右线在中线右侧，此值为正值
}RefLine_HPP_PointInfo_T;
#endif

#ifndef LINEPOLYINFO_T
#define LINEPOLYINFO_T
typedef struct {
	uint8_t PolyOrder;//多项式的阶次,3阶曲线此值为3，代表PolyCoeff中[0-3]有效
	uint8_t Reserved_1;
	uint8_t Reserved_2;
	uint8_t	Reserved_3;
	float	PolyStartRange;//StartRange of single poly(m)
	float	PolyEndRange;//EndRange of single poly(m)
	float	PolyCoeff[REFLINE_HPP_MAX_POLY_ORDER];//多项式系数，3阶为例 A0=PolyCoeff[0]...A3=PolyCoeff[3];Y=A0+A1*X+A2*X^2+A3*X^3
}LinePolyInfo_t;
#endif

#ifndef REFLINE_HPP_POLYINFO_T
#define REFLINE_HPP_POLYINFO_T
typedef struct {
	int HPP_PolyNum;
	LinePolyInfo_t	HPP_PolyInfo[REFLINE_HPP_MAX_POLY_NUM];//HPP中线多项式轨迹信息
	int LeftLine_PolyNum;
	LinePolyInfo_t	LeftLine_PolyInfo[REFLINE_HPP_MAX_POLY_NUM];//左线多项式轨迹信息
	int RightLine_PolyNum;
	LinePolyInfo_t	RightLine_PolyInfo[REFLINE_HPP_MAX_POLY_NUM];//左线多项式轨迹信息
}RefLine_HPP_PolyInfo_T;
#endif

#ifndef REFLINE_HPP_GEOTRACKINFO_T
#define REFLINE_HPP_GEOTRACKINFO_T
typedef struct {
	unsigned int frameID;//图像帧ID
	uint32_t	RefLineId;//此条Refline的ID
	uint8_t		RefLineConf;//此条Refline的置信度
	uint8_t		NumOfOriRefLineId;//若分裂/合并，分裂/合并前的ReflineID的数目
	uint32_t	OriRefLineId[4];//若分裂/合并，分裂/合并前的ReflineID
	REFLINE_HPP_TRKSTS trkState;//此条Refline追踪的状态
	float		TrkDist;//此条Refline被追踪的距离
	uint8_t		updateAge;//此条Refline被更新的Cnt
	uint8_t		coastAge;//[Reserved]此条refline被coast的Cnt
	uint8_t		SmoothStatus;//[Reserved]Refline的光滑状态, 0x0 Unsmoothed 0x1 FreshSmoothed 0x2 Previously Smoothed
	uint16_t	DiscreteDistanceCm;//离散点的采样间隔（cm）
	uint8_t		PointNum;//此条Refline的离散点数量
	RefLine_HPP_PointInfo_T	PointInfo[REFLINE_HPP_MAXPOINTNUM_SINGLELINE];//此条Refline的离散点信息
	RefLine_HPP_PolyInfo_T	PolyInfo;//[Reserved]此条Refline的多项式轨迹信息
	float		StartDistanceM;//此条Refline的起始距离(m)
	float		EndDistanceM;//此条Refline的结束距离(m)
	MetToolDisplayProperty_T	DisplayProperty;//[Debug]仅用于MET Tool显示
}RefLine_HPP_GeoTrackInfo_T;
#endif

#ifndef REFLINE_HPP_LANEPROPERTY_T
#define REFLINE_HPP_LANEPROPERTY_T
typedef struct {
	uint8_t	f_OpeningLane;//标注此车道前方为opening车道，即生长车道
	REFLINE_HPP_MERGESRC	f_MergeLane;//标注此车道前方为收窄车道并说明此收窄车道数据的来源
	REFLINE_HPP_MERGEDIR	MergeDirection;//收窄车道的类型与方向,正值表示车道向右合并，负值表示车道向左合并，
	int	NumOfLaneTurnType;//车道的转向类型数量，与箭头感知有关。非地图数据
	REFLINE_HPP_LANETURNTYPE LaneTurnType[REFLINE_HPP_MAX_LANETURNTYPE_NUM];//车道的转向类型
}RefLine_HPP_LaneProperty_T;
#endif

#ifndef REFLINE_HPP_ROADEDGE_T
#define REFLINE_HPP_ROADEDGE_T
typedef struct
{
	float		startRange;
	float		endRange;
	float		conf;
	uint32_t	id;
	uint8_t		coastAge;
	uint8_t		updateAge;
	float		trkDist;
	REFLINE_HPP_TRKSTS trkState;
	uint8_t   Reserved_1;
	uint8_t   Reserved_2;
	uint8_t   Reserved_3;
	uint8_t   PointNum;
	float	  X[REFLINE_HPP_BEVLINEOBJECT_MAX_POINTNUM];//正方向以Maxieye标准坐标系为准
	float	  Y[REFLINE_HPP_BEVLINEOBJECT_MAX_POINTNUM];//正方向以Maxieye标准坐标系为准
	float     LRoadEdgeWidth[REFLINE_HPP_BEVLINEOBJECT_MAX_POINTNUM];//[Reserved]
	float     RRoadEdgeWidth[REFLINE_HPP_BEVLINEOBJECT_MAX_POINTNUM];//[Reserved]
}RefLine_HPP_RoadEdge_T;
#endif

#ifndef REFLINE_HPP_ROADEDGEINFO_T
#define REFLINE_HPP_ROADEDGEINFO_T
typedef struct {
	RefLine_HPP_RoadEdge_T	LeftRoadEdge;//左路沿信息(透传感知的路沿信息)
	RefLine_HPP_RoadEdge_T	RightRoadEdge;//右路沿信息(透传感知的路沿信息)
	MetToolDisplayProperty_T	DisplayProperty;//[Debug]仅用于MET Tool显示
}RefLine_HPP_RoadEdgeInfo_T;
#endif

#ifndef REFLINE_HPP_ROADBLOCK_T
#define REFLINE_HPP_ROADBLOCK_T
typedef struct
{
	uint8_t RoadBlockId;
	uint8_t RoadBlockTrkStatus;
	uint8_t RoadBlockType;
	REFLINE_HPP_ROADBLOCKSRCTYPE RoadBlockSourceType;
	uint8_t RoadBlockRGB[3];
	uint8_t RoadBlockPointNum;
	float   RoadBlockBoundingBoxX[REFLINE_HPP_MAX_ROADBLOCK_POINTNUM];//正方向以Maxieye标准坐标系为准
	float   RoadBlockBoundingBoxY[REFLINE_HPP_MAX_ROADBLOCK_POINTNUM];//正方向以Maxieye标准坐标系为准
}RefLine_HPP_RoadBlock_T;
#endif

#ifndef REFLINE_HPP_ROADBLOCKINFO_T
#define REFLINE_HPP_ROADBLOCKINFO_T
typedef struct {
	uint8_t	RoadBlockNum;
	RefLine_HPP_RoadBlock_T RoadBlock[REFLINE_HPP_MAX_ROADBLOCK_NUM];//RefLine透传的感知锥桶信息
}RefLine_HPP_RoadBlockInfo_T;
#endif

#ifndef REFLINE_HPP_INTERSECTIONINFO_T
#define REFLINE_HPP_INTERSECTIONINFO_T
typedef struct {
	int		f_CrossRoad;//根据感知结果判定的路口场景Flag
	float	CrossRoadLinkRange[2];//根据感知结果判定的路口X方向范围(m),正方向以Maxieye标准坐标系为准
	uint8_t	StopLineType;//停止线的类型, 0x0 Invalid 0x1 Stopline
	int8_t	StopLineId;//停止线的ID, -1 Invalid
	float	StopLineDistM;//停止线的距离
	float	StopLineX[2];//停止线两端的X坐标,正方向以Maxieye标准坐标系为准
	float	StopLineY[2];//停止线的Y坐标,正方向以Maxieye标准坐标系为准
	uint8_t	TrafficLightType;//交通信号灯类型, 0x0 Invalid 0x1 Have Traffic light
	int8_t	TrafficLightId;//-1 Invalid
	int8_t	TrafficLightColor;//color of current state of TL, 0: unknown 1: red 2: yellow 3: red and yellow 4: green 5: red and green 6: yellow_and_green 7: red, yellow and green
	uint8_t	f_TrafficLightBlocked;//交通信号灯是否被遮挡
	uint8_t	TrafficLightShape;//交通信号灯形状, 0: unknown 1: up straight 2: left 3: right 4: u_turn 5: circle, 6: down straight 7:invalid
	uint8_t	TrafficLightRemianTimeSec;//If Invalid ,set as 255
	uint8_t	TrafficLightIsFlashing;//交通信号灯是否在闪烁
	float	TrafficLightConf;//交通信号灯置信度
	float	TrafficLightX;//交通信号灯X(m),正方向以Maxieye标准坐标系为准
	float	TrafficLightY;//交通信号灯Y(m),正方向以Maxieye标准坐标系为准
	float	TrafficLightZ;//交通信号灯Z(m),正方向以Maxieye标准坐标系为准
}RefLine_HPP_IntersectionInfo_T;
#endif

#ifndef REFLINE_HPP_INFO_T
#define REFLINE_HPP_INFO_T
typedef struct
{
	RefLine_HPP_SoftwareInfo_T		SoftwareInfo;//RefLine软件相关信息
	RefLine_HPP_GeoTrackInfo_T		GeoTrackInfo;//RefLine_HPP几何及跟踪信息
	RefLine_HPP_LaneProperty_T		LaneProperty;//RefLine_HPP车道信息
	RefLine_HPP_RoadEdgeInfo_T		RoadEdgeInfo;//RefLine_HPP左右路沿信息
	RefLine_HPP_RoadBlockInfo_T		RoadBlockInfo;//RefLine_HPP相关锥桶信息
	RefLine_HPP_IntersectionInfo_T	IntersectionInfo;//RefLine_HPP相关路口信息
}RefLine_HPP_Info_t;
#endif

#ifndef MET_REFLINE_HPP_INFO_T
#define MET_REFLINE_HPP_INFO_T
typedef struct
{
	unsigned int frameID;
	REFLINE_HPP_UINT64 start_time_stamp;
	REFLINE_HPP_UINT64 result_time_stamp;
	RefLine_HPP_Info_t RefLine_HPP_Info;
}MET_RefLine_HPP_Info_t;
#endif

#pragma pack()
#endif
