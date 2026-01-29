#pragma once
#include <cstdint>
#include "Platform_Types.h"


// typedef struct {
//    float PAR_cameraProperties_EFL_X;
//    float PAR_cameraProperties_EFL_Y;
//    float PAR_cameraProperties_COD_X;
//    float PAR_cameraProperties_COD_Y;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK1;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK2;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsP1;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsP2;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK3;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK4;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK5;
//    float PAR_cameraProperties_cameraModelFloatCoefficientsK6;
// } Intrinsic_Calibration_parameters;


typedef struct {
   uint64_t  timestamps;
   uint8_t   veh_config_type;
   uint8_t   Platform_Type;
   uint16_t  k_BSW_DistFrontToRearAxle;
   uint16_t  k_WheelBase;
   uint16_t  k_BSW_HostVehLength;
   float k_front_cornering_compliance;
   float k_YawInertiaAdjustFac;
   float k_RearAxleLoadRatio;
   float k_rear_compliance;
   uint16_t  k_VehWidth_Min;
   uint16_t  k_VehWidth_Max;
   uint16_t  k_WheelRadius;
   uint16_t  k_WheelWidthAve;
   sint16  k_FrontCAM_Wide_x;
   sint16  k_FrontCAM_Wide_y;
   uint16_t  k_FrontCAM_Wide_z;
   float k_FrontCAM_Wide_Yaw;
   float k_FrontCAM_Wide_Pitch;
   float k_FrontCAM_Wide_Roll;
   float k_FrontCAM_Wide_Pitch_Delta;
   float k_FrontCAM_Wide_Yaw_Delta;
   float k_FrontCAM_Wide_Roll_Delta;
   float k_FrontCAM_Wide_Pitch_Delta_Online;
   float k_FrontCAM_Wide_Yaw_Delta_Online;
   float k_FrontCAM_Wide_Roll_Delta_Online;
} Perception_Vehicle_parameters;


typedef struct {
   uint8_t  HPCADAS_AEBDecCtrl;
   uint8_t  FSC_LCC_Mode;
   uint8_t  FSC_LCC_EscapeLevel;
   uint8_t  FSC_emergencyLightReq;
   uint16_t HPCADAS_LKA_SteeringWheelAngle;
   uint16_t ADDC_ACCTargetTrq;
   uint16_t ADDC_ACCTargetTrqBrk;
   uint8_t  HPCADAS_ACCMode;
} EDR_Info;



typedef struct {
   uint8_t HW_Version[10];
   uint8_t SW_Version[10];
   uint8_t ECU_SerialNumber[30];
   uint8_t VIN_Code[17];
} Logistics_data;


typedef struct {
   bool R_reset_request;
} Rcore_reset_request;



typedef struct {
   uint8_t Feature_errcode;
} FeaState;


typedef struct {
 uint8_t addressing_format;
 uint16_t Doip_message_length;
 uint8_t Doip_message[256];
} UDS_ipc_req;

typedef struct {
 uint8_t addressing_format;
 uint16_t Doip_message_length;
 uint8_t Doip_message[256];
} UDS_ipc_response;
