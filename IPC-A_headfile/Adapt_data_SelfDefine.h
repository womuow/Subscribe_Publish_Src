#pragma once
#include <cstdint>
#include "edr_data_types_common.h"

struct	Intrinsic_Calibration_parameters{
	float	PAR_cameraProperties_EFL_X;
	float	PAR_cameraProperties_EFL_Y;
	float PAR_cameraProperties_COD_X;
	float PAR_cameraProperties_COD_Y;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK1;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK2;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsP1;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsP2;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK3;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK4;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK5;
	float	PAR_cameraProperties_cameraModelfloat32CoefficientsK6;
};



struct	Perception_Vehicle_parameters{
	uint64_t	timestamps;
	uint8_t	veh_config_type;
	uint8_t	Platform_Type;
	uint16_t	k_BSW_DistFrontToRearAxle;
	uint16_t	k_WheelBase;
	uint16_t	k_BSW_HostVehLength;
	float	k_front_cornering_compliance;
	float	k_YawInertiaAdjustFac;
	float	k_RearAxleLoadRatio;
	float	k_rear_compliance;
	uint16_t	k_VehWidth_Min;
	uint16_t	k_VehWidth_Max;
	uint16_t	k_WheelRadius;
	uint16_t	k_WheelWidthAve;
	int16_t	k_FrontCAM_Wide_x;
	int16_t	k_FrontCAM_Wide_y;
	uint16_t	k_FrontCAM_Wide_z;
	float	k_FrontCAM_Wide_Yaw;
	float	k_FrontCAM_Wide_Pitch;
	float	k_FrontCAM_Wide_Roll;
	float	k_FrontCAM_Wide_Pitch_Delta;
	float	k_FrontCAM_Wide_Yaw_Delta;
	float	k_FrontCAM_Wide_Roll_Delta;
	float	k_FrontCAM_Wide_Pitch_Delta_Online;
	float	k_FrontCAM_Wide_Yaw_Delta_Online;
	float	k_FrontCAM_Wide_Roll_Delta_Online;
};



struct EDR_Info
{
	uint8_t	HPCADAS_AEBDecCtrl;
	uint16_t	HPCADAS_LKA_SteeringWheelAngle;
	uint16_t	ADDC_ACCTargetTrq;
	uint16_t	ADDC_ACCTargetTrqBrk;
	uint8_t	HPCADAS_ACCMode;
	uint8_t	FSC_LCC_Mode;
	uint8_t	FSC_LCC_EscapeLevel;
	uint8_t	FSC_emergencyLightReq;
};



struct Logistics_data {
    uint8_t HW_Version[10];
    uint8_t SW_Version[10];
    uint8_t ECU_SerialNumber[30];
    uint8_t VIN_Code[17];
};



struct Rcore_reset_request
{
	bool R_reset_request;
};



struct SysState
{
	uint8_t Feature_errcode;
};