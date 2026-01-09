#ifndef HMI_DATA_PUB_HPP
#define HMI_DATA_PUB_HPP

#include <cstdint>

#pragma pack(push, 4)               // 设置4字节对齐

namespace hmi_data {
namespace  hmi_data_hmiConfig{
/* Function switch enumeration */
typedef enum {
    FUNCTION_SWITCH_DISABLE = 0,    // 关闭，未激活
    FUNCTION_SWITCH_ENABLE = 1      // 打开，激活
} FunctionSwitchEnum;

typedef struct {
    /* Basic vehicle information */
    uint32_t total_odometer;                // 总里程数，单位km 
    float vehicle_speed;                    // 仪表车速，单位km/h
    
    /* Function switches (0: DISABLE, 1: ENABLE) */
    FunctionSwitchEnum  fcw_sw;             // FCW开关状态
    FunctionSwitchEnum  ldw_sw;             // LDW开关状态
    FunctionSwitchEnum  aeb_sw;             // AEB开关状态
    FunctionSwitchEnum  lka_sw;             // LKA开关状态
    FunctionSwitchEnum  acc_sw;             // ACC开关状态
    FunctionSwitchEnum  lcc_sw;             // LCC开关状态
    FunctionSwitchEnum  mai_sw;             // MAI开关状态
    FunctionSwitchEnum  slwf_sw;            // SLWF限速告警开关状态
    FunctionSwitchEnum  slif_sw;            // SLIF限速信息提示开关状态
    FunctionSwitchEnum  fvsa_sw;            // 前车起步提醒开关配置
    FunctionSwitchEnum  ihbc_active;        // IHBC开关状态
    FunctionSwitchEnum  disarm_alarm_sw;    // 解除脱手报警开关状态
    
    /* Configuration values */
    uint32_t fcw_sensitivity;               // FCW灵敏度 (0:正常 1:低 2:高)
    uint32_t acc_time_gap;                  // ACC车间时距 (0:正常 1:近 2:较远 3:远)
    uint32_t slwf_offset;                   // SLWF超速告警偏移量，km/h [-10,10]
    uint32_t ldp_sw;                        // LDP开关 (0:关闭 1:打开)
    uint32_t acc_target_speed_source;       // 仪表设定车速来源 (0:默认值 1:驾驶员调速 2:规控自动调速)
    
    /* Control signals */
    uint32_t func_active_signal;            // ADAS拨杆打开信号
    uint32_t swbl_roller_up_sts;            // 滚轮上拨信号 (0:no action 1:UP)
    uint32_t swbl_roller_dsts;              // 滚轮下拨信号 (0:no action 1:DOWN)
} HmiConfig;
}
}

#pragma pack(pop)

#endif
