#ifndef MET_SOC_SWP1_H_
#define MET_SOC_SWP1_H_

#include <stdint.h>

/*topic name: Veh/swp1*/
/*****************************************************************************************************************************
**                                                  Revision Control History                                                                   **
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                                   */
/*  version:  1     2024-03-21     SH                      Initial version                                                  */
/*  version:  2     2024-10-24     SH                      add k_FrontCAM_x,k_FrontCAM_y,k_FrontCAM_z,Reserved20,k_FrontCAM_Yaw
	                                                       k_FrontCAM_Yaw,k_FrontCAM_Pitch,k_FrontCAM_Roll                  */ 
/*  version:  3     2025-08-19     SH                      add Platform_Type ,k_SideRadarType ,k_YawInertiaAdjustFac,k_RearAxleLoadRatio,
                                                           k_RearAxleLoadRatio,
       
                                               	     delete k_YawRateAvaliableFlag,k_Axis_Config,k_FrontRimOuterWidth,k_BSW_HostVehWidth,
                                                           k_Vehicle_Weight_FullLoad ,k_Vehicle_Rigidity,k_Roll_Compliance,
                                                           k_pitch_Compliance, k_VehBarycenterHeight,k_vehRearWheelWidth,k_DrivenMode
                                                          ,k_FrontAxleBrakingForceDistributRatio*/                                                                                                                    
/****************************************************************************************************************************/
#define MET_SWP1_HEADFILE_VERSION   (3u)

typedef struct MET_SOC_SWP1_Param{

	uint32_t frameId;
	uint64_t timestamps;
	uint8_t k_LongRangRadarType;
	uint8_t veh_config_type;
	uint8_t Platform_Type;
	uint8_t k_SideRadarType;
	uint16_t k_BSW_DistFrontToRearAxle;
	uint16_t k_WheelBase;
	uint16_t k_BSW_HostVehLength;
	uint16_t k_Vehicle_Weight;
	float k_front_cornering_compliance;
	float k_YawInertiaAdjustFac;
	float k_RearAxleLoadRatio;
	float k_rear_compliance;
	float k_SteeringGearRatio;
	uint16_t k_VehWidth_Min;
	uint16_t k_VehWidth_Max;
	uint16_t k_WheelRadius;
	uint16_t k_WheelWidthAve;
	uint8_t k_vehicle_speed_characteristic;
	uint16_t k_radar_VertPos_mm_MRR;
	uint16_t k_radar_LongPos_mm_MRR;
	int8_t k_radar_LatPos_mm_MRR;
	uint8_t k_Radar_VKI_Major_Version_MRR;
	uint8_t k_Radar_VKI_Minor_Version_MRR;
	uint16_t k_VKI_Request_dDxv_Rear_Bumper;
	uint16_t k_VKI_Request_dxv_Front_Bumper;
	uint8_t k_VKI_Request_CoverDamping_MRR;
	uint16_t k_VKI_Request_aXMountPosX;
	int8_t k_radar_azimuth_polarity_MRR;
	uint16_t k_radar_VertPos_mm_SRRFL;
	int16_t k_radar_LongPos_mm_SRRFL;
	int16_t k_radar_LatPos_mm_SRRFL;
	int8_t k_radar_azimuth_polarity_SRRFL;
	uint8_t Reserved11;
	float k_radar_boresight_angle_Pitch_deg_SRRFL;
	float k_radar_boresight_angle_Yaw_deg_SRRFL;
	uint16_t k_radar_VertPos_mm_SRRFR;
	int16_t k_radar_LongPos_mm_SRRFR;
	int16_t k_radar_LatPos_mm_SRRFR;
	int8_t k_radar_azimuth_polarity_SRRFR;
	uint8_t Reserved12;
	float k_radar_boresight_angle_Pitch_deg_SRRFR;
	float k_radar_boresight_angle_Yaw_deg_SRRFR;
	uint16_t k_radar_VertPos_mm_SRRRL;
	int16_t k_radar_LongPos_mm_SRRRL;
	int16_t k_radar_LatPos_mm_SRRRL;
	int8_t k_radar_azimuth_polarity_SRRRL;
	uint8_t Reserved13;
	float k_radar_boresight_angle_Pitch_deg_SRRRL;
	float k_radar_boresight_angle_Yaw_deg_SRRRL;
	uint16_t k_radar_VertPos_mm_SRRRR;
	int16_t k_radar_LongPos_mm_SRRRR;
	int16_t k_radar_LatPos_mm_SRRRR;
	int8_t k_radar_azimuth_polarity_SRRRR;
	uint8_t Reserved14;
	float k_radar_boresight_angle_Pitch_deg_SRRRR;
	float k_radar_boresight_angle_Yaw_deg_SRRRR;
	uint8_t k_origin_point_of_coordinate_SRR;
	uint8_t k_orientation_of_coordinate_SRR;
	int16_t k_AVM_FrontCAM_x;
	int16_t k_AVM_FrontCAM_y;
	uint16_t k_AVM_FrontCAM_z;
	uint16_t Reserved15;
	float k_AVM_FrontCAM_Yaw;
	float k_AVM_FrontCAM_Pitch;
	float k_AVM_FrontCAM_Roll;
	float k_AVM_FrontCAM_Pitch_Delta;
	float k_AVM_FrontCAM_Yaw_Delta;
	float k_AVM_FrontCAM_Roll_Delta;
	float k_AVM_FrontCAM_Pitch_Delta_Online;
	float k_AVM_FrontCAM_Yaw_Delta_Online;
	float k_AVM_FrontCAM_Roll_Delta_Online;
	int16_t k_AVM_LeftCAM_x;
	int16_t k_AVM_LeftCAM_y;
	uint16_t k_AVM_LeftCAM_z;
	uint16_t Reserved16;
	float k_AVM_LeftCAM_Yaw;
	float k_AVM_LeftCAM_Pitch;
	float k_AVM_LeftCAM_Roll;
	float k_AVM_LeftCAM_Pitch_Delta;
	float k_AVM_LeftCAM_Yaw_Delta;
	float k_AVM_LeftCAM_Roll_Delta;
	float k_AVM_LeftCAM_Pitch_Delta_Online;
	float k_AVM_LeftCAM_Yaw_Delta_Online;
	float k_AVM_LeftCAM_Roll_Delta_Online;
	int16_t k_AVM_RightCAM_x;
	int16_t k_AVM_RightCAM_y;
	uint16_t k_AVM_RightCAM_z;
	uint16_t Reserved17;
	float k_AVM_RightCAM_Yaw;
	float k_AVM_RightCAM_Pitch;
	float k_AVM_RightCAM_Roll;
	float k_AVM_RightCAM_Pitch_Delta;
	float k_AVM_RightCAM_Yaw_Delta;
	float k_AVM_RightCAM_Roll_Delta;
	float k_AVM_RightCAM_Pitch_Delta_Online;
	float k_AVM_RightCAM_Yaw_Delta_Online;
	float k_AVM_RightCAM_Roll_Delta_Online;
	int16_t k_AVM_RearCAM_x;
	int16_t k_AVM_RearCAM_y;
	uint16_t k_AVM_RearCAM_z;
	uint16_t Reserved18;
	float k_AVM_RearCAM_Yaw;
	float k_AVM_RearCAM_Pitch;
	float k_AVM_RearCAM_Roll;
	float k_AVM_RearCAM_Pitch_Delta;
	float k_AVM_RearCAM_Yaw_Delta;
	float k_AVM_RearCAM_Roll_Delta;
	float k_AVM_RearCAM_Pitch_Delta_Online;
	float k_AVM_RearCAM_Yaw_Delta_Online;
	float k_AVM_RearCAM_Roll_Delta_Online;
	int16_t k_BackCAM_x;
	int16_t k_BackCAM_y;
	uint16_t k_BackCAM_z;
	uint16_t Reserved19;
	float k_BackCAM_Yaw;
	float k_BackCAM_Pitch;
	float k_BackCAM_Roll;
	float k_AVM_BackCAM_Pitch_Delta;
	float k_AVM_BackCAM_Yaw_Delta;
	float k_AVM_BackCAM_Roll_Delta;
	float k_AVM_BackCAM_Pitch_Delta_Online;
	float k_AVM_BackCAM_Yaw_Delta_Online;
	float k_AVM_BackCAM_Roll_Delta_Online;
	int16_t k_FrontCAM_Narrow_x;
	int16_t k_FrontCAM_Narrow_y;
	uint16_t k_FrontCAM_Narrow_z;
	float k_FrontCAM_Narrow_Yaw;
	float k_FrontCAM_Narrow_Pitch;
	float k_FrontCAM_Narrow_Roll;
	float k_FrontCAM_Narrow_Pitch_Delta;
	float k_FrontCAM_Narrow_Yaw_Delta;
	float k_FrontCAM_Narrow_Roll_Delta;
	float k_FrontCAM_Narrow_Pitch_Delta_Online;
	float k_FrontCAM_Narrow_Yaw_Delta_Online;
	float k_FrontCAM_Narrow_Roll_Delta_Online;
	int16_t k_FrontCAM_Wide_x;
	int16_t k_FrontCAM_Wide_y;
	uint16_t k_FrontCAM_Wide_z;
	float k_FrontCAM_Wide_Yaw;
	float k_FrontCAM_Wide_Pitch;
	float k_FrontCAM_Wide_Roll;
	float k_FrontCAM_Wide_Pitch_Delta;
	float k_FrontCAM_Wide_Yaw_Delta;
	float k_FrontCAM_Wide_Roll_Delta;
	float k_FrontCAM_Wide_Pitch_Delta_Online;
	float k_FrontCAM_Wide_Yaw_Delta_Online;
	float k_FrontCAM_Wide_Roll_Delta_Online;
	int16_t k_IMU_x;
	int16_t k_IMU_y;
	int16_t k_IMU_z;
	float k_IMU_z_Rotaion;
	float k_IMU_x_Rotaion;
	float k_IMU_y_Rotaion;
	uint8_t k_GNSS_type;
}MET_SOC_SWP1_Param;

#endif
