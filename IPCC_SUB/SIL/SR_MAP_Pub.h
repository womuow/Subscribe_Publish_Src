/*
Copyright (c) 2025 Magna Electronics. All rights reserved.
Version: 2.0.0
This software is proprietary and confidential to Magna Electronics.
Unauthorized copying, distribution, or use is strictly prohibited.
*/

#ifndef _VIH_HMI_SR_MAP_PUB_H_
#define _VIH_HMI_SR_MAP_PUB_H_

// topic name: Hmi_Visl/HmiVisl_HMI_Sr_Map

#include <cstdint>

#define VIH_HMI_SR_MAP_VERSION 1
#define VIH_HMI_SR_MAP_MAX_OBSTACLE_NUM 64
#define VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_NUM 16
#define VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_SUBLINE_NUM 3
#define VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_SUBLINE_POINT_NUM 60
#define VIH_HMI_SR_MAP_MAX_LANE_NUM 16
#define VIH_HMI_SR_MAP_MAX_LANE_POINT_NUM 60
#define VIH_HMI_SR_MAP_MAX_STOP_LANE_NUM 4
#define VIH_HMI_SR_MAP_MAX_LANE_MARKS_NUM 32
#define VIH_HMI_SR_MAP_MAX_LANE_EDGE_NUM 8
#define VIH_HMI_SR_MAP_MAX_CROSSWALK_NUM 8
#define VIH_HMI_SR_MAP_MAX_CROSSWALK_POINT_NUM 4
#define VIH_HMI_SR_MAP_MAX_PLANNER_PATH_NUM 60

namespace HMI_Visl
{
namespace SR_MAP
{
typedef struct {
    uint32_t seq_num;      // 序列号
    uint64_t timestamp_ns; // 时间，以纳秒为单位
}Header;

typedef struct{
    float x;
    float y;
    float z;
}Point3D;

typedef struct{
    enum TypeEnum : uint8_t {
        NO_DISPLAY = 0,
        CAR = 1,                 // 四轮轿车
        SUV = 2,                 // SUV
        VAN = 3,                 // 面包车;MPV
        POLICE_CAR = 4,          // 警车
        BOX_TRUCK = 5,           // 箱式卡车
        FLAT_TRUCK = 6,          // 板式货车
        BIGBUS = 7,              // 大巴
        ADULT = 8,               // 成人
        CYCLIST = 9,             // 人骑自行车
        MOTORCYCLIST = 10,       // 人骑摩托车
        TRICYCLE_BOX_RIDER = 11, // 人骑带箱三轮车
        NORMAL_CONE = 12,        // 锥桶
        PILLAR_CONE = 13,        // 防撞柱
        BUCKET = 14,             // 圆桩桶
        WATER_BARRIER = 15,      // 水马
        CONSTRUCTION_SIGN = 16,  // 施工牌
        TRICONE = 17,            // 三角牌
        BARRIER_GATE_OPEN = 18,  // 打开的收费杆
        BARRIER_GATE_CLOSE = 19, // 关闭的收费杆
        SQUARE_PILLAR = 20,      // 方柱（方柱圆柱）
        SPEEDBUMP = 21,          // 减速带
        FIRE_HYDRANT = 22,       // 消防箱
        LOCK_ON = 23,            // 地锁开
        LOCK_OFF = 24,           // 地锁关
        CHOCK = 25               // 轮档
    };

    typedef struct{
        bool brake_switch_on;           // 制动灯状态
        bool left_turn_switch_on;       // 左转灯状态
        bool right_turn_switch_on;      // 右转灯状态
        bool left_right_turn_switch_on; // 双闪灯状态
    }LightStatus;

    enum ColorEnum : uint8_t {
        DEFAULT = 0, // 默认
        BLUE = 1,    // 蓝
        YELLOW = 2,  // 黄
        RED = 3      // 红
    };

    typedef struct{
        float width;
        float length;
        float height;
    }Size;

    uint32_t id;              			// 障碍物ID
    TypeEnum type;            			// 障碍物类型
    Point3D position;                   // 坐标系下的xyz
    double angle;             			// 航向角(弧度)
    Point3D speed;                      // 车速(m/s)
    LightStatus light_status; 			// 车灯状态
    ColorEnum color;
    Size size;
}PerceptionObstacle;

typedef struct{
    PerceptionObstacle  obstacles[VIH_HMI_SR_MAP_MAX_OBSTACLE_NUM]; // 障碍物列表
    uint8_t             obstacle_num_veh;  // 障碍物数量_车
    uint8_t             obstacle_num_blck;  // 障碍物数量_障碍物
}PerceptionObstacles;

typedef struct  {
    enum TypeEnum : uint8_t {
        INVALID = 0,     // 无效
        SOLID = 1,       // 实线
        DASH = 2,        // 虚线
        SOLID_DASH = 3,  // 实-虚
        DASH_SOLID = 4,  // 虚-实
        SOLID_SOLID = 5, // 双实
        DASH_DASH = 6    // 双虚
};

enum ColorEnum : uint8_t {
    DEFAULT = 0, // 默认
    WHITE = 1,   // 白
    YELLOW = 2   // 黄
};

typedef struct  {
    uint32_t id;                 // 从1开始累加，单个LaneBoundary内局部唯一ID
    Point3D points[VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_SUBLINE_POINT_NUM];          // 点集
    uint8_t num_points;          // 点数量
    TypeEnum type;               // 类型
    ColorEnum color;             // 颜色
}BoundarySubLine;

    uint32_t id;                     // 车道线ID
    TypeEnum type;                   // 车道线类型
    ColorEnum color;                 // 车道线颜色
    BoundarySubLine sub_lines[VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_SUBLINE_NUM];   // 按照类型和颜色划分的子线段
    uint8_t sub_line_num;            // 子线段数量
}LaneBoundary;



typedef struct  {
    uint32_t id;                				// 车道ID
    uint32_t left_boundary_id;  				// 左侧laneboundary-id
    uint32_t right_boundary_id; 				// 右侧laneboundary-id
    Point3D centerline[VIH_HMI_SR_MAP_MAX_LANE_POINT_NUM];  // 车道中心线点集 60
    bool is_ego_lane;           				// 是否是自车所在的车道
}Lane;

typedef struct  {
    uint32_t id;         					// 停止线ID
    Point3D start_point; // 起点
    Point3D end_point;   // 终点
}Stopline;

typedef struct {
    enum TypeEnum : uint8_t {
        NONE = 0,               // 未知
        LEFT_FORWARD = 1,       // 左前方
        LEFT_RIGHT = 2,         // 左右
        RIGHT_FORWARD = 3,      // 右前方
        LEFT_RIGHT_FORWARD = 4, // 左右前方
        LEFT = 5,               // 左转
        RIGHT = 6,              // 右转
        FORWARD = 7,            // 直行
        U_TURN = 8,             // 掉头
        U_TURN_FORWARD = 9,     // 掉头并直行
        LEFT_U_TURN = 10        // 左转掉头
    };

    uint32_t id;         				// 道路箭头ID
    TypeEnum arrow_type; 				// 箭头类型
    Point3D position;// 位置
    float angle;         				// 朝向(弧度)
}LaneMark;

typedef struct {
    uint32_t id;          					              // 边缘线ID
    Point3D polyline[VIH_HMI_SR_MAP_MAX_LANE_POINT_NUM];  // 线段点集
}LaneEdge;


typedef struct {
    uint32_t id;         					// 斑马线ID
    Point3D polygon [VIH_HMI_SR_MAP_MAX_CROSSWALK_POINT_NUM];         // 多边形点集
}Crosswalk;

typedef struct{
    LaneBoundary lane_boundaries[VIH_HMI_SR_MAP_MAX_LANE_BOUNDARY_NUM]; // 车道线
    uint8_t lane_boundary_num;                                          // 车道线数量
    Lane lanes[VIH_HMI_SR_MAP_MAX_LANE_NUM];                           //车道中心线 16
    uint8_t lane_num;                                                  // 车道中心线数量
    Stopline stop_lines[VIH_HMI_SR_MAP_MAX_STOP_LANE_NUM];             // 停止线 4
    uint8_t stop_lines_num; 
    LaneMark lane_marks[VIH_HMI_SR_MAP_MAX_LANE_MARKS_NUM];            //箭头 32
    uint8_t lane_marks_num;
    LaneEdge lane_edges[VIH_HMI_SR_MAP_MAX_LANE_EDGE_NUM];             // 8
    uint8_t lane_edges_num;
    Crosswalk crosswalks[VIH_HMI_SR_MAP_MAX_CROSSWALK_NUM];             //斑马线 8
    uint8_t crosswalks_num;
}PerceptionRasmap;

typedef struct {
    enum TypeEnum : uint8_t {
        NONE = 0,
        DRIVING = 1,       // NOA&&LCC-PATH
        APA = 2,           // APA泊车轨迹线
        RADS = 3,          // 巡迹倒车轨迹
        ROUTING_GUIDE = 4, // 记忆泊车巡航引导线
        PRE_APA = 5,       // 预规划轨迹线
        P2P_GUIDE = 6      // 停车场全览导航路线
    };

    Point3D polylines[VIH_HMI_SR_MAP_MAX_PLANNER_PATH_NUM]; // 点集
    TypeEnum type;
}PlannerPath;

typedef struct {
    PlannerPath planner_path; // 规划轨迹线
}VehicleDynamicInfo;


typedef struct  
{
    Header header;      		                // 帧头，包含时间戳、序列号、模块名     
    PerceptionObstacles perception_obstacles;   // 感知障碍物
    PerceptionRasmap perception_rasmap;      	// Road And Semantic Map，感知在线地图
    VehicleDynamicInfo vehicle_dynamic_info;

}SrMap;

}  // namespace HMI 
}  // namespace HMI_Visl

#endif