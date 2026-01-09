#ifndef HMI_DATA_HANDLEIPC_A_H
#define HMI_DATA_HANDLEIPC_A_H

#include <stdint.h>

#pragma pack(push,4)

typedef uint32_t HmiConfig_total_odometer_T;
typedef float HmiConfig_vehicle_speed_T;
typedef uint8_t HmiConfig_func_sw_T;
typedef uint32_t HmiConfig_fcw_sensitivity_T;
typedef uint32_t HmiConfig_acc_time_gap_T;
typedef uint32_t HmiConfig_func_active_signal_T;
typedef uint32_t HmiConfig_slwf_offset_T;
typedef uint32_t HmiConfig_ldp_sw_T;
typedef uint32_t HmiConfig_acc_target_speed_source_T;
typedef uint32_t HmiConfig_swbl_roller_up_sts_T;
typedef uint32_t HmiConfig_swbl_roller_dsts_T;

typedef uint32_t ActiveSafetyInfo_aeb_warning_T;
typedef uint32_t ActiveSafetyInfo_mai_warning_T;
typedef uint32_t ActiveSafetyInfo_textinfo_index_T;
typedef uint8_t AutoDriveInfo_adas_status_T;
typedef uint8_t AutoDriveInfo_acc_status_T;
typedef uint32_t AutoDriveInfo_fault_adas_T;
typedef uint32_t hands_off_level_T;
typedef uint32_t hands_off_count_down_T;
typedef uint32_t hands_off_count_down_time_T;
typedef float AutoDriveInfo_adas_target_speed_set_T;
typedef float AutoDriveInfo_speed_set_limit_T;
typedef uint32_t WarningInfo_left_warning_level_T;
typedef uint32_t WarningInfo_left_warning_location_T;
typedef uint32_t WarningInfo_right_warning_level_T;
typedef uint32_t WarningInfo_right_warning_location_T;
typedef uint32_t FunModeInfo_warning_state_T;
typedef struct {
  WarningInfo_left_warning_level_T left_warning_level;
  WarningInfo_left_warning_location_T left_warning_location;
  WarningInfo_right_warning_level_T right_warning_level;
  WarningInfo_right_warning_location_T right_warning_location;
} FunModeInfo_warning_Info_T;
typedef struct {
  FunModeInfo_warning_Info_T warning_Info;
  FunModeInfo_warning_state_T warning_state;
} ActiveSafetyInfo_FunmodeInfo_T;
typedef struct {
  hands_off_level_T hands_off_level;
  hands_off_count_down_T hands_off_count_down;
  hands_off_count_down_time_T hands_off_count_down_time;
} AutoDriveInfo_hands_off_info_T;

typedef struct {
  HmiConfig_total_odometer_T HmiConfig_total_odometer;
  HmiConfig_vehicle_speed_T HmiConfig_vehicle_speed;
  HmiConfig_func_sw_T HmiConfig_fcw_sw;
  HmiConfig_fcw_sensitivity_T HmiConfig_fcw_sensitivity;
  HmiConfig_func_sw_T HmiConfig_ldw_sw;
  HmiConfig_func_sw_T HmiConfig_aeb_sw;
  HmiConfig_func_sw_T HmiConfig_lka_sw;
  HmiConfig_func_sw_T HmiConfig_acc_sw;
  HmiConfig_acc_time_gap_T HmiConfig_acc_time_gap;
  HmiConfig_func_sw_T HmiConfig_lcc_sw;
  HmiConfig_func_active_signal_T HmiConfig_func_active_signal;
  HmiConfig_func_sw_T HmiConfig_mai_sw;
  HmiConfig_func_sw_T HmiConfig_slwf_sw;
  HmiConfig_slwf_offset_T HmiConfig_slwf_offset;
  HmiConfig_func_sw_T HmiConfig_slif_sw;
  HmiConfig_func_sw_T HmiConfig_fvsa_sw;
  HmiConfig_func_sw_T HmiConfig_ihbc_active;
  HmiConfig_func_sw_T HmiConfig_disarm_alarm_sw;
  HmiConfig_ldp_sw_T HmiConfig_ldp_sw;
  HmiConfig_acc_target_speed_source_T HmiConfig_acc_target_speed_source;
  HmiConfig_swbl_roller_up_sts_T HmiConfig_swbl_roller_up_sts;
  HmiConfig_swbl_roller_dsts_T HmiConfig_swbl_roller_dsts;
} HmiConfig_FromDDS_T;


typedef struct {
  ActiveSafetyInfo_aeb_warning_T ActiveSafetyInfo_aeb_warning;
  ActiveSafetyInfo_mai_warning_T ActiveSafetyInfo_mai_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_lka_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_fdw_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_fcw_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_ldw_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_fvsa_warning;
  ActiveSafetyInfo_FunmodeInfo_T ActiveSafetyInfo_warning_info;
  ActiveSafetyInfo_textinfo_index_T ActiveSafetyInfo_textinfo_index;
} ActiveSafetyInfo_ToDDS_T;

typedef struct {
  AutoDriveInfo_adas_status_T AutoDriveInfo_adas_status;
  AutoDriveInfo_acc_status_T AutoDriveInfo_acc_status;
  AutoDriveInfo_fault_adas_T AutoDriveInfo_fault_adas;
  AutoDriveInfo_hands_off_info_T AutoDriveInfo_hands_off_info;
  AutoDriveInfo_adas_target_speed_set_T AutoDriveInfo_adas_target_speed_set;
  AutoDriveInfo_speed_set_limit_T AutoDriveInfo_speed_set_limit;
  ActiveSafetyInfo_textinfo_index_T AutoDriveInfo_textinfo_index;
} AutoDriveInfo_ToDDS_T;

#pragma pack(pop)

#endif