#ifndef IPC_PUB_h
#define IPC_PUB_h
#include "Platform_Types.h"
#pragma pack(push,4)


/* AUTOSAR Implementation data types, specific to software component */
typedef uint8 AEBMon_State;
typedef uint8 AdpPath_ValidSamples;
typedef float32 AdpPath_EndDst;
typedef float32 AdpPath_Y;
typedef float32 AdpPath_Curvature;
typedef float32 AdpPath_DstAlongPah;
typedef float32 AdpPath_LocalOffsetCoGx;
typedef float32 AdpPath_LocalHeadingCoGx;
typedef float32 AdpPath_YawratePathCoGx;
typedef float32 AdpPath_YawratederivativePathCoGx;
typedef uint8 DynCalPrmForAccBrkLim;
typedef float32 DynCalPrmForAccSpdLoLim;
typedef float32 DynCalPrmForBicycleMdlAxleDistFrnt;
typedef uint8 DynCalPrmForSteerWhlPosn;
typedef float32 DynCalPrmForVehWhlBas;
typedef float32 RoadPath_LaneWidth;
typedef float32 RoadPath_OffsLat;
typedef float32 RoadPath_DstLgtToEndLaneMrk;
typedef float32 RoadPath_DstLgtToEndEhCrvt;
typedef float32 RoadPath_DstLgtToEndObj;
typedef float32 RoadPath_AgDir;
typedef float32 RoadPath_Crvt;
typedef float32 RoadPath_CrvtRate;
typedef float32 RoadPath_SegLen;
typedef float32 RoadPath_LowConfDist;
typedef float32 RoadPath_HighConfDist;
typedef uint64 RoadPath_Timestamp;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_A;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_Heading;
typedef uint8 TargetsSelectedForACC_AccTgtAdjLLeft_Idn;
typedef uint8 TargetsSelectedForACC_AccTgtAdjLLeft_Index;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_latPositionRoad;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_PosnLat;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_PosnLgt;
typedef float32 TargetsSelectedForACC_AccTgtAdjLLeft_Spd;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_A;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_Heading;
typedef uint8 TargetsSelectedForACC_AccTgtAdjLRight_Idn;
typedef uint8 TargetsSelectedForACC_AccTgtAdjLRight_Index;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_latPositionRoad;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_PosnLat;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_PosnLgt;
typedef float32 TargetsSelectedForACC_AccTgtAdjLRight_Spd;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_A;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_Heading;
typedef uint8 TargetsSelectedForACC_AccTgtCutIn_Idn;
typedef uint8 TargetsSelectedForACC_AccTgtCutIn_Index;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_latPositionRoad;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_PosnLat;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_PosnLgt;
typedef float32 TargetsSelectedForACC_AccTgtCutIn_Spd;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_A;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_Heading;
typedef uint8 TargetsSelectedForACC_AccTgtFrstClstLane_Idn;
typedef uint8 TargetsSelectedForACC_AccTgtFrstClstLane_Index;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_latPositionRoad;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_PosnLat;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_PosnLgt;
typedef float32 TargetsSelectedForACC_AccTgtFrstClstLane_Spd;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_A;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_Heading;
typedef uint8 TargetsSelectedForACC_AccTgtSecClstLane_Idn;
typedef uint8 TargetsSelectedForACC_AccTgtSecClstLane_Index;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_latPositionRoad;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_PosnLat;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_PosnLgt;
typedef float32 TargetsSelectedForACC_AccTgtSecClstLane_Spd;
typedef float32 TrafficFlow_LaneProperties_TrfcFlowAvrgDst;
typedef float32 TrafficFlow_LaneProperties_TrfcFlowAvrgSpd;
typedef uint32 TrafficFlow_SequenceID;
typedef uint32 WarningInfo_left_warning_level_T;
typedef uint32 WarningInfo_left_warning_location_T;
typedef uint32 WarningInfo_right_warning_level_T;
typedef uint32 WarningInfo_right_warning_location_T;
typedef uint32 FunModeInfo_warning_state_T;
typedef uint32 ActiveSafetyInfo_aeb_warning_T;
typedef uint32 ActiveSafetyInfo_mai_warning_T;
typedef uint32 ActiveSafetyInfo_textinfo_index_T;
typedef uint8 AutoDriveInfo_adas_status_T;
typedef uint8 AutoDriveInfo_acc_status_T;
typedef uint32 AutoDriveInfo_fault_adas_T;
typedef uint32 hands_off_level_T;
typedef uint32 hands_off_count_down_T;
typedef uint32 hands_off_count_down_time_T;
typedef float32 AutoDriveInfo_adas_target_speed_set_T;
typedef float32 AutoDriveInfo_speed_set_limit_T;
typedef uint8 HBC2CAN_FSC_IHBC_Mode_T;
typedef uint8 HBC2CAN_FSC_IHBC_High_Beam_Request_T;
typedef uint8 HBC2CAN_FSC_IHBC_NotAvailableReason_T;
typedef uint32 HmiConfig_total_odometer_T;
typedef float32 HmiConfig_vehicle_speed_T;
typedef uint8 HmiConfig_func_sw_T;
typedef uint32 HmiConfig_fcw_sensitivity_T;
typedef uint32 HmiConfig_acc_time_gap_T;
typedef uint32 HmiConfig_func_active_signal_T;
typedef uint32 HmiConfig_slwf_offset_T;
typedef uint32 HmiConfig_ldp_sw_T;
typedef uint32 HmiConfig_acc_target_speed_source_T;
typedef uint32 HmiConfig_swbl_roller_up_sts_T;
typedef uint32 HmiConfig_swbl_roller_dsts_T;
typedef float32 point_info_x_T;
typedef float32 point_info_y_T;
typedef float32 DynCalPrmForBicycleMdlCornrgStfnFrnt;
typedef float32 DynCalPrmForBicycleMdlCornrgStfnFrntByVehSpd;
typedef float32 DynCalPrmForBicycleMdlCornrgStfnRe;
typedef float32 DynCalPrmForBicycleMdlCornrgStfnReByVehSpd;
typedef float32 DynCalPrmForVehSteerWhlAgRat;
typedef float32 DynCalPrmForVehicleSpdForBicycleMdlCornrgStfn;
typedef float32 EgoVehicleState_VLgt;
typedef float32 EgoVehicleState_ALgt;
typedef float32 EgoVehicleState_ALgtRaw;
typedef float32 EgoVehicleState_ALatRaw;
typedef float32 EgoVehicleState_YawRate;
typedef float32 EgoVehicleState_YawRateRaw;
typedef uint32 EgoVehicleState_SequenceID;
typedef float32 FlcRoadCover_Snow_confidence;
typedef float32 FlcRoadCover_Gravel_confidence;
typedef float32 FlcRoadCover_Wet_confidence;
typedef float32 AdditionalTarSelnSignals_Signal1;
typedef float32 AdditionalTarSelnSignals_Signal10;
typedef float32 AdditionalTarSelnSignals_Signal2;
typedef float32 AdditionalTarSelnSignals_Signal3;
typedef float32 AdditionalTarSelnSignals_Signal4;
typedef float32 AdditionalTarSelnSignals_Signal5;
typedef float32 AdditionalTarSelnSignals_Signal6;
typedef float32 AdditionalTarSelnSignals_Signal7;
typedef float32 AdditionalTarSelnSignals_Signal8;
typedef float32 AdditionalTarSelnSignals_Signal9;
typedef float32 CrossingObject_EgoStates_latAcceleration;
typedef float32 CrossingObject_EgoStates_longAcceleration;
typedef float32 CrossingObject_EgoStates_latPosition;
typedef float32 CrossingObject_EgoStates_longPosition;
typedef float32 CrossingObject_EgoStates_latVelocity;
typedef float32 CrossingObject_EgoStates_longVelocity;
typedef uint16 CrossingObject_Properties_id;
typedef float32 CrossingObject_Properties_width;
typedef float32 CrossingObject_Properties_length;
typedef uint32 CrossingObject_SequenceID;
typedef float32 DynCalPrmForAxleDstReToVehFrnt;
typedef float32 DynCalPrmForBicycleMdlJ;
typedef float32 DynCalPrmForSteerGrdt;
typedef float32 DynCalPrmForVehLen;
typedef float32 DynCalPrmForVehM;
typedef uint8 DynCalPrmForVehTyp;
typedef float32 DynCalPrmForVehWidth;
typedef float32 DynCalPrmForWhlRadius;
typedef uint64 FusedFrontObj_timestamp;
typedef float32 FusedFrontObject_Properties_accelerationStdDev;
typedef float32 FusedFrontObject_Properties_classificationConfidence;
typedef float32 FusedFrontObject_Properties_distanceToLeftNearLaneMarking;
typedef float32 FusedFrontObject_Properties_distanceToRightNearLaneMarking;
typedef float32 FusedFrontObject_Properties_existenceConfidence;
typedef float32 FusedFrontObject_Properties_headingStdDev;
typedef uint8 FusedFrontObject_Properties_id;
typedef float32 FusedFrontObject_Properties_innovationFactor;
typedef float32 FusedFrontObject_Properties_latPositionStdDev;
typedef float32 FusedFrontObject_Properties_longPositionStdDev;
typedef uint8 FusedFrontObject_Properties_radarId;
typedef float32 FusedFrontObject_Properties_reserved;
typedef float32 FusedFrontObject_Properties_speedStdDev;
typedef uint8 FusedFrontObject_Properties_visionId;
typedef float32 FusedFrontObject_Properties_width;
typedef float32 FusedFrontObject_Properties_length;
typedef uint32 FusedFrontObject_SequenceID;
typedef float32 FusedFrontObject_States_acceleration;
typedef float32 FusedFrontObject_States_curvature;
typedef float32 FusedFrontObject_States_heading;
typedef float32 FusedFrontObject_States_latAcceleration;
typedef float32 FusedFrontObject_States_latPosition;
typedef float32 FusedFrontObject_States_latVelocity;
typedef float32 FusedFrontObject_States_longAcceleration;
typedef float32 FusedFrontObject_States_longPosition;
typedef float32 FusedFrontObject_States_longVelocity;
typedef float32 FusedFrontObject_States_speed;
typedef uint8 LaneAssignedFrontObjects_Properties_id;
typedef uint32 LaneAssignedFrontObjects_SequenceID;
typedef float32 LaneAssignedFrontObjects_States_latAccelerationRoad;
typedef float32 LaneAssignedFrontObjects_States_latPositionRoad;
typedef float32 LaneAssignedFrontObjects_States_headingRoad;
typedef float32 LaneAssignedFrontObjects_States_latVelocityRoad;
typedef float32 LaneAssignedFrontObjects_States_longAccelerationRoad;
typedef float32 LaneAssignedFrontObjects_States_longPositionRoad;
typedef float32 LaneAssignedFrontObjects_States_longVelocityRoad;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_a;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_b;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_c;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_d1;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_d2;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_longDistToEnd;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_longDistToStart;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_transitionDist;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_aVariance;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_bVariance;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_cVariance;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_d1Variance;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_d2Variance;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtOffset;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtHeading;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtCrvt;
typedef float32 LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtCrvtRate;
typedef uint8 LaneMarker_EgoLane_Left_Lane_id;
typedef float32 LaneMarker_EgoLane_Left_Lane_markingWidth;
typedef float32 LaneMarker_EgoLane_Left_Lane_measurementQuality;
typedef float32 LaneMarker_EgoLane_Left_Lane_modelError;
typedef float32 LaneMarker_EgoLane_Left_Lane_selectionConfidence;
typedef float32 LaneMarker_EgoLane_Left_Lane_totalMarkingWidth;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_a;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_b;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_c;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_d1;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_d2;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_longDistToEnd;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_longDistToStart;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_transitionDist;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_aVariance;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_bVariance;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_cVariance;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_d1Variance;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_d2Variance;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtOffset;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtHeading;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtCrvt;
typedef float32 LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtCrvtRate;
typedef uint8 LaneMarker_EgoLane_Right_Lane_id;
typedef float32 LaneMarker_EgoLane_Right_Lane_markingWidth;
typedef float32 LaneMarker_EgoLane_Right_Lane_measurementQuality;
typedef float32 LaneMarker_EgoLane_Right_Lane_modelError;
typedef float32 LaneMarker_EgoLane_Right_Lane_selectionConfidence;
typedef float32 LaneMarker_EgoLane_Right_Lane_totalMarkingWidth;
typedef float32 LaneMarker_EgoLane_parallelDistance;
typedef float32 LaneMarker_EgoLane_validProjectionDistance;
typedef float32 LaneMarker_DebugBus_debugFloat1;
typedef float32 LaneMarker_DebugBus_debugFloat2;
typedef float32 LaneMarker_DebugBus_debugFloat3;
typedef float32 LaneMarker_DebugBus_debugFloat4;
typedef float32 LaneMarker_DebugBus_debugFloat5;
typedef float32 LaneMarker_DebugBus_debugFloat6;
typedef float32 LaneMarker_DebugBus_debugFloat7;
typedef float32 LaneMarker_DebugBus_debugFloat8;
typedef uint8 LaneMarker_DebugBus_debugInteger1;
typedef uint8 LaneMarker_DebugBus_debugInteger2;
typedef uint8 LaneMarker_DebugBus_debugInteger3;
typedef uint8 LaneMarker_DebugBus_debugInteger4;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_a;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_b;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_c;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_d1;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_d2;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_longDistToEnd;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_longDistToStart;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_transitionDist;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_aVariance;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_bVariance;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_cVariance;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_d1Variance;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_d2Variance;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_vrtOffset;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_vrtHeading;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_vrtCrvt;
typedef float32 LaneMarker_SecClsLeft_Lane_DblClothoid_vrtCrvtRate;
typedef uint8 LaneMarker_SecClsLeft_Lane_id;
typedef float32 LaneMarker_SecClsLeft_Lane_markingWidth;
typedef float32 LaneMarker_SecClsLeft_Lane_measurementQuality;
typedef float32 LaneMarker_SecClsLeft_Lane_modelError;
typedef float32 LaneMarker_SecClsLeft_Lane_selectionConfidence;
typedef float32 LaneMarker_SecClsLeft_Lane_totalMarkingWidth;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_a;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_b;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_c;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_d1;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_d2;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_longDistToEnd;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_longDistToStart;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_transitionDist;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_aVariance;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_bVariance;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_cVariance;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_d1Variance;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_d2Variance;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_vrtOffset;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_vrtHeading;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_vrtCrvt;
typedef float32 LaneMarker_SecClsRight_Lane_DblClothoid_vrtCrvtRate;
typedef uint8 LaneMarker_SecClsRight_Lane_id;
typedef float32 LaneMarker_SecClsRight_Lane_markingWidth;
typedef float32 LaneMarker_SecClsRight_Lane_measurementQuality;
typedef float32 LaneMarker_SecClsRight_Lane_modelError;
typedef float32 LaneMarker_SecClsRight_Lane_selectionConfidence;
typedef float32 LaneMarker_SecClsRight_Lane_totalMarkingWidth;
typedef float32 LaneMarker_LaneEvent_distance;
typedef uint8 LaneMarker_LaneEvent_id;
typedef float32 LaneMarker_TemporaryMarking_longDistanceToStart;
typedef uint32 LaneMarker_SequenceID;
typedef float32 RearObject_EgoStates_heading;
typedef float32 RearObject_EgoStates_latAcceleration;
typedef float32 RearObject_EgoStates_latPosition;
typedef float32 RearObject_EgoStates_latPositionPath;
typedef float32 RearObject_EgoStates_latVelocity;
typedef float32 RearObject_EgoStates_longAcceleration;
typedef float32 RearObject_EgoStates_longPosition;
typedef float32 RearObject_EgoStates_longPositionPath;
typedef float32 RearObject_EgoStates_longVelocity;
typedef float32 RearObject_RoadStates_headingRoad;
typedef float32 RearObject_RoadStates_latAccelerationRoad;
typedef float32 RearObject_RoadStates_latPositionRoad;
typedef float32 RearObject_RoadStates_latVelocityRoad;
typedef float32 RearObject_RoadStates_longAccelerationRoad;
typedef float32 RearObject_RoadStates_longPositionRoad;
typedef float32 RearObject_RoadStates_longVelocityRoad;
typedef float32 RearObject_Properties_length;
typedef float32 RearObject_Properties_width;
typedef uint8 RearObject_Properties_id;
typedef uint32 RearObject_SequenceID;
typedef uint8 RoadEdge_LeftNonTrvsble_Edge_id;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_measurementQuality;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_modelError;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_a;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_aVariance;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_b;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_bVariance;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_c;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_cVariance;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d1Variance;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d2Variance;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_longDistToEnd;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_longDistToStart;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtOffset;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtHeading;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtCrvt;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtCrvtRate;
typedef float32 RoadEdge_LeftNonTrvsble_Edge_selectionConfidence;
typedef uint8 RoadEdge_RightNonTrvsble_Edge_id;
typedef float32 RoadEdge_RightNonTrvsble_Edge_measurementQuality;
typedef float32 RoadEdge_RightNonTrvsble_Edge_modelError;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_a;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_aVariance;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_b;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_bVariance;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_c;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_cVariance;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_d;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_d1Variance;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_d2Variance;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_longDistToEnd;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_longDistToStart;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtOffset;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtHeading;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtCrvt;
typedef float32 RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtCrvtRate;
typedef float32 RoadEdge_RightNonTrvsble_Edge_selectionConfidence;
typedef uint8 RoadEdge_LeftTrvsble_Edge_id;
typedef float32 RoadEdge_LeftTrvsble_Edge_measurementQuality;
typedef float32 RoadEdge_LeftTrvsble_Edge_modelError;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_a;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_aVariance;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_b;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_bVariance;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_c;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_cVariance;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_d;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_d1Variance;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_d2Variance;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_longDistToEnd;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_longDistToStart;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtOffset;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtHeading;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtCrvt;
typedef float32 RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtCrvtRate;
typedef float32 RoadEdge_LeftTrvsble_Edge_selectionConfidence;
typedef uint8 RoadEdge_RightTrvsble_Edge_id;
typedef float32 RoadEdge_RightTrvsble_Edge_measurementQuality;
typedef float32 RoadEdge_RightTrvsble_Edge_modelError;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_a;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_aVariance;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_b;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_bVariance;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_c;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_cVariance;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_d;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_d1Variance;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_d2Variance;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_longDistToEnd;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_longDistToStart;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_vrtOffset;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_vrtHeading;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_vrtCrvt;
typedef float32 RoadEdge_RightTrvsble_Edge_SingClothoid_vrtCrvtRate;
typedef float32 RoadEdge_RightTrvsble_Edge_selectionConfidence;
typedef uint32 RoadEdge_SequenceID;
typedef uint16 EPS_SteerWheelRotSpd;

/* AUTOSAR Enumeration Types */
typedef uint8 IDT_Message_QF;

#ifndef StatusOK
#define StatusOK                       (0)
#endif

#ifndef StatusNoNewData
#define StatusNoNewData                (1)
#endif

#ifndef StatusWrongCRC
#define StatusWrongCRC                 (2)
#endif

#ifndef StatusInit
#define StatusInit                     (4)
#endif

#ifndef StatusRepeat
#define StatusRepeat                   (8)
#endif

#ifndef StatusSync
#define StatusSync                     (16)
#endif

#ifndef StatusOKsomeLost
#define StatusOKsomeLost               (32)
#endif

#ifndef StatusWrongSeq
#define StatusWrongSeq                 (64)
#endif

#ifndef StatusLostComm
#define StatusLostComm                 (128)
#endif

typedef uint8 TrueFalse;

#ifndef TrueFalse_False
#define TrueFalse_False                (0)
#endif

#ifndef TrueFalse_True
#define TrueFalse_True                 (1)
#endif

typedef uint8 NoYes;

#ifndef NoYes_No
#define NoYes_No                       (0)
#endif

#ifndef NoYes_Yes
#define NoYes_Yes                      (1)
#endif

typedef uint8 TrfcAssiConf;

#ifndef TrfcAssiConf_NoConfidence
#define TrfcAssiConf_NoConfidence      (0)
#endif

#ifndef TrfcAssiConf_Reserved
#define TrfcAssiConf_Reserved          (1)
#endif

#ifndef TrfcAssiConf_LowConfidence
#define TrfcAssiConf_LowConfidence     (2)
#endif

#ifndef TrfcAssiConf_HighConfidence
#define TrfcAssiConf_HighConfidence    (3)
#endif

typedef uint8 LaneChangeType;

#ifndef LaneChangeType_Unchanged
#define LaneChangeType_Unchanged       (0)
#endif

#ifndef LaneChangeType_ChangedToLeft
#define LaneChangeType_ChangedToLeft   (1)
#endif

#ifndef LaneChangeType_ChangedToRight
#define LaneChangeType_ChangedToRight  (2)
#endif

typedef uint8 TrueFalse32;

#ifndef TrueFalse32_False
#define TrueFalse32_False              (0)
#endif

#ifndef TrueFalse32_True
#define TrueFalse32_True               (1)
#endif

typedef uint8 Reliable5;

#ifndef Reliable5_NotRelbl
#define Reliable5_NotRelbl             (0)
#endif

#ifndef Reliable5_Relbl1
#define Reliable5_Relbl1               (1)
#endif

#ifndef Reliable5_Relbl2
#define Reliable5_Relbl2               (2)
#endif

#ifndef Reliable5_Relbl3
#define Reliable5_Relbl3               (3)
#endif

typedef uint8 MarkerType;

#ifndef MarkerType_None
#define MarkerType_None                (0)
#endif

#ifndef MarkerType_Solid
#define MarkerType_Solid               (1)
#endif

#ifndef MarkerType_Dashed
#define MarkerType_Dashed              (2)
#endif

#ifndef MarkerType_Separation
#define MarkerType_Separation          (3)
#endif

#ifndef MarkerType_SolidDashed
#define MarkerType_SolidDashed         (4)
#endif

#ifndef MarkerType_DashedSolid
#define MarkerType_DashedSolid         (5)
#endif

#ifndef MarkerType_Reserved1
#define MarkerType_Reserved1           (6)
#endif

typedef uint8 DataConfidenceLvl3;

#ifndef DataConfidenceLvl3_NotRelbl
#define DataConfidenceLvl3_NotRelbl    (0)
#endif

#ifndef DataConfidenceLvl3_SoonNotRelbl
#define DataConfidenceLvl3_SoonNotRelbl (1)
#endif

#ifndef DataConfidenceLvl3_LowRelbl
#define DataConfidenceLvl3_LowRelbl    (2)
#endif

#ifndef DataConfidenceLvl3_HigherRelbl
#define DataConfidenceLvl3_HigherRelbl (3)
#endif

#ifndef DataConfidenceLvl3_HighestRelbl
#define DataConfidenceLvl3_HighestRelbl (4)
#endif

typedef uint8 MotionHistory;

#ifndef MotionHistory_NotSeenMoving
#define MotionHistory_NotSeenMoving    (0)
#endif

#ifndef MotionHistory_SeenReceding
#define MotionHistory_SeenReceding     (1)
#endif

#ifndef MotionHistory_SeenOncoming
#define MotionHistory_SeenOncoming     (2)
#endif

#ifndef MotionHistory_SeenCrossing
#define MotionHistory_SeenCrossing     (3)
#endif

typedef uint8 MotionPattern2;

#ifndef MotionPattern2_Unknown
#define MotionPattern2_Unknown         (0)
#endif

#ifndef MotionPattern2_Stationary
#define MotionPattern2_Stationary      (1)
#endif

#ifndef MotionPattern2_Receding
#define MotionPattern2_Receding        (2)
#endif

#ifndef MotionPattern2_Oncoming
#define MotionPattern2_Oncoming        (3)
#endif

#ifndef MotionPattern2_Movable
#define MotionPattern2_Movable         (4)
#endif

#ifndef MotionPattern2_Crossing
#define MotionPattern2_Crossing        (5)
#endif

typedef uint8 TargetLaneStatus;

#ifndef TargetLaneStatus_TgtKeepingLane
#define TargetLaneStatus_TgtKeepingLane (0)
#endif

#ifndef TargetLaneStatus_TgtChangingLaneToLeft
#define TargetLaneStatus_TgtChangingLaneToLeft (1)
#endif

#ifndef TargetLaneStatus_TgtChangingLaneToRight
#define TargetLaneStatus_TgtChangingLaneToRight (2)
#endif

typedef uint8 IndicatorStatus;

#ifndef IndicatorStatus_NoIndcn
#define IndicatorStatus_NoIndcn        (0)
#endif

#ifndef IndicatorStatus_Le
#define IndicatorStatus_Le             (1)
#endif

#ifndef IndicatorStatus_Ri
#define IndicatorStatus_Ri             (2)
#endif

#ifndef IndicatorStatus_Ukwn
#define IndicatorStatus_Ukwn           (3)
#endif

typedef uint8 ObjectClass7;

#ifndef ObjectClass7_UkwnClass
#define ObjectClass7_UkwnClass         (0)
#endif

#ifndef ObjectClass7_Car
#define ObjectClass7_Car               (1)
#endif

#ifndef ObjectClass7_Motorcycle
#define ObjectClass7_Motorcycle        (2)
#endif

#ifndef ObjectClass7_Truck
#define ObjectClass7_Truck             (3)
#endif

#ifndef ObjectClass7_Ped
#define ObjectClass7_Ped               (4)
#endif

#ifndef ObjectClass7_Anim
#define ObjectClass7_Anim              (7)
#endif

#ifndef ObjectClass7_ObjGen
#define ObjectClass7_ObjGen            (8)
#endif

#ifndef ObjectClass7_Bicycle
#define ObjectClass7_Bicycle           (9)
#endif

#ifndef ObjectClass7_VehOfUkwnClass
#define ObjectClass7_VehOfUkwnClass    (10)
#endif

typedef uint8 DataConfidenceLvl4;

#ifndef DataConfidenceLvl4_NotRelbl
#define DataConfidenceLvl4_NotRelbl    (0)
#endif

#ifndef DataConfidenceLvl4_Relbl
#define DataConfidenceLvl4_Relbl       (1)
#endif

#ifndef DataConfidenceLvl4_HighRelbl
#define DataConfidenceLvl4_HighRelbl   (2)
#endif

typedef uint8 TrafficFlowDir;

#ifndef TrafficFlowDir_Ukwn
#define TrafficFlowDir_Ukwn            (0)
#endif

#ifndef TrafficFlowDir_EquToSelfMovmt
#define TrafficFlowDir_EquToSelfMovmt  (1)
#endif

#ifndef TrafficFlowDir_NotEquToSelfMovmt
#define TrafficFlowDir_NotEquToSelfMovmt (2)
#endif

typedef uint8 DrivingSide5;

#ifndef DrivingSide5_Unknown
#define DrivingSide5_Unknown           (0)
#endif

#ifndef DrivingSide5_Left
#define DrivingSide5_Left              (1)
#endif

#ifndef DrivingSide5_Right
#define DrivingSide5_Right             (2)
#endif

typedef uint8 CmbbObjConfidence;

#ifndef CmbbObjConfidence_NotRelbl
#define CmbbObjConfidence_NotRelbl     (0)
#endif

#ifndef CmbbObjConfidence_CoastRelbl
#define CmbbObjConfidence_CoastRelbl   (1)
#endif

#ifndef CmbbObjConfidence_BrkSpprtRelbl
#define CmbbObjConfidence_BrkSpprtRelbl (2)
#endif

#ifndef CmbbObjConfidence_BrkgRelbl
#define CmbbObjConfidence_BrkgRelbl    (3)
#endif

typedef uint8 DataConfidenceLvl2;

#ifndef DataConfidenceLvl2_NotRelbl
#define DataConfidenceLvl2_NotRelbl    (0)
#endif

#ifndef DataConfidenceLvl2_LowRelbl
#define DataConfidenceLvl2_LowRelbl    (1)
#endif

#ifndef DataConfidenceLvl2_HighRelbl
#define DataConfidenceLvl2_HighRelbl   (2)
#endif

#ifndef DataConfidenceLvl2_HighestRelbl
#define DataConfidenceLvl2_HighestRelbl (3)
#endif

typedef uint8 EventTypeDef;

#ifndef EventTypeDef_NoEvent
#define EventTypeDef_NoEvent           (0)
#endif

#ifndef EventTypeDef_Opening
#define EventTypeDef_Opening           (1)
#endif

#ifndef EventTypeDef_Closing
#define EventTypeDef_Closing           (2)
#endif

#ifndef EventTypeDef_DashedToSolid
#define EventTypeDef_DashedToSolid     (3)
#endif

#ifndef EventTypeDef_SolidToDashed
#define EventTypeDef_SolidToDashed     (4)
#endif

#ifndef EventTypeDef_StartOfDashed
#define EventTypeDef_StartOfDashed     (5)
#endif

#ifndef EventTypeDef_StartOfSolid
#define EventTypeDef_StartOfSolid      (6)
#endif

typedef uint8 FusionSource;

#ifndef FusionSource_None
#define FusionSource_None              (0)
#endif

#ifndef FusionSource_VisionOnly
#define FusionSource_VisionOnly        (1)
#endif

#ifndef FusionSource_RadarOnly
#define FusionSource_RadarOnly         (2)
#endif

#ifndef FusionSource_FusedRadarVision
#define FusionSource_FusedRadarVision  (3)
#endif

typedef uint8 LaneChangeDirection2;

#ifndef LaneChangeDirection2_NoLaneChg
#define LaneChangeDirection2_NoLaneChg (0)
#endif

#ifndef LaneChangeDirection2_LaneChgLe
#define LaneChangeDirection2_LaneChgLe (1)
#endif

#ifndef LaneChangeDirection2_LaneChgRi
#define LaneChangeDirection2_LaneChgRi (2)
#endif

typedef uint8 LaneColor;

#ifndef LaneColor_White
#define LaneColor_White                (0)
#endif

#ifndef LaneColor_Yellow
#define LaneColor_Yellow               (1)
#endif

#ifndef LaneColor_Red
#define LaneColor_Red                  (2)
#endif

#ifndef LaneColor_Blue
#define LaneColor_Blue                 (3)
#endif

#ifndef LaneColor_Orange
#define LaneColor_Orange               (4)
#endif

#ifndef LaneColor_Green
#define LaneColor_Green                (5)
#endif

#ifndef LaneColor_Other
#define LaneColor_Other                (6)
#endif

typedef uint8 LaneMarkerStructure;

#ifndef LaneMarkerStructure_None
#define LaneMarkerStructure_None       (0)
#endif

#ifndef LaneMarkerStructure_Painted
#define LaneMarkerStructure_Painted    (1)
#endif

#ifndef LaneMarkerStructure_Reflector
#define LaneMarkerStructure_Reflector  (2)
#endif

typedef uint8 LaneTrackId;

#ifndef LaneTrackId_None
#define LaneTrackId_None               (0)
#endif

#ifndef LaneTrackId_ClosestLeft
#define LaneTrackId_ClosestLeft        (1)
#endif

#ifndef LaneTrackId_ClosestRight
#define LaneTrackId_ClosestRight       (2)
#endif

#ifndef LaneTrackId_SecondClosestLeft
#define LaneTrackId_SecondClosestLeft  (3)
#endif

#ifndef LaneTrackId_SecondClosestRight
#define LaneTrackId_SecondClosestRight (4)
#endif

typedef uint8 MarkingType;

#ifndef MarkingType_None
#define MarkingType_None               (0)
#endif

#ifndef MarkingType_Solid
#define MarkingType_Solid              (1)
#endif

#ifndef MarkingType_Dashed
#define MarkingType_Dashed             (2)
#endif

#ifndef MarkingType_SeparationMarking
#define MarkingType_SeparationMarking  (3)
#endif

typedef uint8 MotionModel3;

#ifndef MotionModel3_ConstantAcceleration
#define MotionModel3_ConstantAcceleration (0)
#endif

#ifndef MotionModel3_CoordinatedTurn
#define MotionModel3_CoordinatedTurn   (1)
#endif

typedef uint8 MotionPattern;

#ifndef MotionPattern_Ukwn
#define MotionPattern_Ukwn             (0)
#endif

#ifndef MotionPattern_Staty
#define MotionPattern_Staty            (1)
#endif

#ifndef MotionPattern_MovgFromSelf
#define MotionPattern_MovgFromSelf     (2)
#endif

#ifndef MotionPattern_MovgToSelf
#define MotionPattern_MovgToSelf       (3)
#endif

typedef uint8 ObjClassnTyp;

#ifndef ObjClassnTyp_Veh
#define ObjClassnTyp_Veh               (0)
#endif

#ifndef ObjClassnTyp_TwoWhl
#define ObjClassnTyp_TwoWhl            (1)
#endif

#ifndef ObjClassnTyp_Ped
#define ObjClassnTyp_Ped               (2)
#endif

#ifndef ObjClassnTyp_Ukwn
#define ObjClassnTyp_Ukwn              (3)
#endif

typedef uint8 ObjectSize2;

#ifndef ObjectSize2_Ukwn
#define ObjectSize2_Ukwn               (0)
#endif

#ifndef ObjectSize2_Sml
#define ObjectSize2_Sml                (1)
#endif

#ifndef ObjectSize2_Med
#define ObjectSize2_Med                (2)
#endif

#ifndef ObjectSize2_Lrg
#define ObjectSize2_Lrg                (3)
#endif

typedef uint8 OnOff2;

#ifndef OnOff2_Ukwn
#define OnOff2_Ukwn                    (0)
#endif

#ifndef OnOff2_Off
#define OnOff2_Off                     (1)
#endif

#ifndef OnOff2_On
#define OnOff2_On                      (2)
#endif

typedef uint8 ReferencePoint;

#ifndef ReferencePoint_refPointUnknown
#define ReferencePoint_refPointUnknown (0)
#endif

#ifndef ReferencePoint_refPointCenter
#define ReferencePoint_refPointCenter  (1)
#endif

#ifndef ReferencePoint_refPointFrontCenter
#define ReferencePoint_refPointFrontCenter (2)
#endif

#ifndef ReferencePoint_refPointCenterLeft
#define ReferencePoint_refPointCenterLeft (3)
#endif

#ifndef ReferencePoint_refPointCenterRight
#define ReferencePoint_refPointCenterRight (4)
#endif

#ifndef ReferencePoint_refPointRearCenter
#define ReferencePoint_refPointRearCenter (5)
#endif

typedef uint8 Reliable3;

#ifndef Reliable3_NotRelbl
#define Reliable3_NotRelbl             (0)
#endif

#ifndef Reliable3_Relbl
#define Reliable3_Relbl                (1)
#endif

typedef uint8 Reliable6;

#ifndef Reliable6_NotReliable
#define Reliable6_NotReliable          (0)
#endif

#ifndef Reliable6_Reliable
#define Reliable6_Reliable             (1)
#endif

typedef uint8 SecondMarkingType;

#ifndef SecondMarkingType_None
#define SecondMarkingType_None         (0)
#endif

#ifndef SecondMarkingType_Solid
#define SecondMarkingType_Solid        (1)
#endif

#ifndef SecondMarkingType_Dashed
#define SecondMarkingType_Dashed       (2)
#endif

#ifndef SecondMarkingType_SeperationMarking
#define SecondMarkingType_SeperationMarking (3)
#endif

#ifndef SecondMarkingType_MultipleMarkings
#define SecondMarkingType_MultipleMarkings (4)
#endif

typedef uint8 SensorUpdateStatus1;

#ifndef SensorUpdateStatus1_Invalid
#define SensorUpdateStatus1_Invalid    (0)
#endif

#ifndef SensorUpdateStatus1_New
#define SensorUpdateStatus1_New        (1)
#endif

#ifndef SensorUpdateStatus1_FlcUpdated
#define SensorUpdateStatus1_FlcUpdated (2)
#endif

#ifndef SensorUpdateStatus1_FlrUpdated
#define SensorUpdateStatus1_FlrUpdated (3)
#endif

#ifndef SensorUpdateStatus1_BothUpdated
#define SensorUpdateStatus1_BothUpdated (4)
#endif

#ifndef SensorUpdateStatus1_NoneUpdated
#define SensorUpdateStatus1_NoneUpdated (5)
#endif

typedef uint8 SideSuggestion;

#ifndef SideSuggestion_None
#define SideSuggestion_None            (0)
#endif

#ifndef SideSuggestion_UncoupledLeft
#define SideSuggestion_UncoupledLeft   (1)
#endif

#ifndef SideSuggestion_UncoupledRight
#define SideSuggestion_UncoupledRight  (2)
#endif

#ifndef SideSuggestion_BothNonParallel
#define SideSuggestion_BothNonParallel (3)
#endif

#ifndef SideSuggestion_BothParallel
#define SideSuggestion_BothParallel    (4)
#endif

#ifndef SideSuggestion_BothCoupled
#define SideSuggestion_BothCoupled     (5)
#endif

typedef uint8 TemporaryMarkingType;

#ifndef TemporaryMarkingType_NoTemporaryMarkings
#define TemporaryMarkingType_NoTemporaryMarkings (0)
#endif

#ifndef TemporaryMarkingType_AmbiguousMarkings
#define TemporaryMarkingType_AmbiguousMarkings (1)
#endif

#ifndef TemporaryMarkingType_Dynamic
#define TemporaryMarkingType_Dynamic   (2)
#endif

typedef uint8 TrackStatus4;

#ifndef TrackStatus4_Invld
#define TrackStatus4_Invld             (0)
#endif

#ifndef TrackStatus4_Fusn
#define TrackStatus4_Fusn              (1)
#endif

#ifndef TrackStatus4_New
#define TrackStatus4_New               (2)
#endif

#ifndef TrackStatus4_PredNew
#define TrackStatus4_PredNew           (3)
#endif

#ifndef TrackStatus4_UpdNew
#define TrackStatus4_UpdNew            (4)
#endif

#ifndef TrackStatus4_Upd
#define TrackStatus4_Upd               (5)
#endif

#ifndef TrackStatus4_Pred
#define TrackStatus4_Pred              (6)
#endif

#ifndef TrackStatus4_Resd
#define TrackStatus4_Resd              (7)
#endif

typedef uint8 TrackStatus6;

#ifndef TrackStatus6_Invld
#define TrackStatus6_Invld             (0)
#endif

#ifndef TrackStatus6_New
#define TrackStatus6_New               (1)
#endif

#ifndef TrackStatus6_Upd
#define TrackStatus6_Upd               (2)
#endif

#ifndef TrackStatus6_Coast
#define TrackStatus6_Coast             (3)
#endif

#ifndef TrackStatus6_CoastFusn
#define TrackStatus6_CoastFusn         (4)
#endif

#ifndef TrackStatus6_Fusn
#define TrackStatus6_Fusn              (5)
#endif

typedef uint8 TrackingStatus;

#ifndef TrackingStatus_Invalid
#define TrackingStatus_Invalid         (0)
#endif

#ifndef TrackingStatus_Tracked
#define TrackingStatus_Tracked         (1)
#endif

#ifndef TrackingStatus_Close
#define TrackingStatus_Close           (2)
#endif

#ifndef TrackingStatus_Far
#define TrackingStatus_Far             (3)
#endif

#ifndef TrackingStatus_Predicted
#define TrackingStatus_Predicted       (4)
#endif

typedef uint8 TrafficScenarioTyp;

#ifndef TrafficScenarioTyp_None
#define TrafficScenarioTyp_None        (0)
#endif

#ifndef TrafficScenarioTyp_TurnAcrssPah
#define TrafficScenarioTyp_TurnAcrssPah (1)
#endif

typedef uint8 VersionTrafficAssist;

#ifndef VersionTrafficAssist_Without
#define VersionTrafficAssist_Without   (0)
#endif

#ifndef VersionTrafficAssist_TrafficAssist
#define VersionTrafficAssist_TrafficAssist (1)
#endif

#ifndef VersionTrafficAssist_TrafficAssistPro
#define VersionTrafficAssist_TrafficAssistPro (2)
#endif

#ifndef VersionTrafficAssist_TrafficAssistReserved1
#define VersionTrafficAssist_TrafficAssistReserved1 (3)
#endif

#ifndef VersionTrafficAssist_TrafficAssistReserved2
#define VersionTrafficAssist_TrafficAssistReserved2 (4)
#endif

#ifndef VersionTrafficAssist_TrafficAssistReserved3
#define VersionTrafficAssist_TrafficAssistReserved3 (5)
#endif

/* AUTOSAR Structure Types */
#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ACU_2A3_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ACU_2A3_

typedef struct {
  uint8 ACU_CrashOutputSts;
  uint8 ACU_FLSeatBeltRSt;
  uint8 ACU_FRSeatBeltRSt;
  uint8 ACU_RLSeatBeltRSt;
  uint8 ACU_RMSeatBeltRSt;
  uint8 ACU_RRSeatBeltRSt;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_ACU_2A3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_1B4_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_1B4_

typedef struct {
  uint8 HPCBCM_challenge_A;
  uint8 HPCBCM_challenge_B;
  uint8 HPCBCM_challenge_C;
  uint8 HPCBCM_challenge_D;
} IDT_SM_SysSigGrp_HPCBCM_1B4;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_1B7_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_1B7_

typedef struct {
  uint8 HPCBCM_response_A;
  uint8 HPCBCM_response_B;
  uint8 HPCBCM_response_C;
  uint8 HPCBCM_response_D;
} IDT_SM_SysSigGrp_HPCBCM_1B7;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_290_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCBCM_290_

typedef struct {
  boolean BCM_BackDoorSts;
  uint8 BCM_BrakeFluidLevel;
  boolean BCM_DriverDoorSts;
  boolean BCM_DriverDoorSts_VD;
  boolean BCM_FLDoorSts;
  uint8 BCM_FLOC_PPD_Status;
  boolean BCM_FRDoorSts;
  uint8 BCM_FROC_PPD_Status;
  boolean BCM_FrontCoverSts;
  uint8 BCM_FrontWiperSts;
  boolean BCM_HazardSwtOpenSts;
  boolean BCM_HazardSwtSts;
  boolean BCM_HighBeamCtrl;
  boolean BCM_KeyPosition_ON1A;
  boolean BCM_KeyPosition_ON1B;
  boolean BCM_KeyPosition_ON2A;
  boolean BCM_KeyPosition_ON2B;
  boolean BCM_KeyPosition_ON3;
  boolean BCM_LowBeamCtrl;
  boolean BCM_ON2BRelaySts;
  boolean BCM_ON2CRelaySts;
  boolean BCM_RearFogCtrl;
  boolean BCM_RLDoorSts;
  uint8 BCM_RLOC_PPD_Status;
  uint8 BCM_RMOC_PPD_Status;
  boolean BCM_RRDoorSts;
  uint8 BCM_RROC_PPD_Status;
  boolean BCM_TurnLeftCtrl;
  boolean BCM_TurnLeftSwtSts;
  boolean BCM_TurnRightCtrl;
  boolean BCM_TurnRightSwtSts;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_HPCBCM_290;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_183_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_183_

typedef struct {
  boolean IBC_BrakePedalApplied;
  uint8 IBC_BrakePedalApplied_Q;
  uint16 IBC_HydraTorqTgtAct;
  uint8 IBC_sOutputRodDriverPer;
  uint8 IBC_sOutputRodDriverPer_Q;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_IBC_183;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_227_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_227_

typedef struct {
  boolean IBC_EPB_APAACCRequest_Available;
  uint8 IBC_EPB_FailStatus;
  uint8 IBC_EPB_Status;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_IBC_227;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_3CE_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_3CE_

typedef struct {
  boolean iTPMS_SystemStatus;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_IBC_3CE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_6F3_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_6F3_

typedef struct {
  uint8 IBC_UDS_SW1;
  uint8 IBC_UDS_SW10;
  uint8 IBC_UDS_SW2;
  uint8 IBC_UDS_SW3;
  uint8 IBC_UDS_SW4;
  uint8 IBC_UDS_SW5;
  uint8 IBC_UDS_SW6;
  uint8 IBC_UDS_SW7;
  uint8 IBC_UDS_SW8;
  uint8 IBC_UDS_SW9;
} IDT_SM_SysSigGrp_IBC_6F3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ICU_24D_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ICU_24D_

typedef struct {
  uint8 ICU_AccelerationMode;
  uint32 ICU_TotalOdometerkm;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_ICU_24D;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IVI_3E3_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IVI_3E3_

typedef struct {
  boolean IVI_GetRidAlarmCancelReq;
} IDT_SM_SysSigGrp_IVI_3E3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IVI_5CE_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IVI_5CE_

typedef struct {
  uint8 IVI_Data;
  uint8 IVI_Hour;
  uint8 IVI_Minute;
  uint8 IVI_Month;
  uint8 IVI_Second;
  uint16 IVI_Year;
} IDT_SM_SysSigGrp_IVI_5CE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_LMC_25A_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_LMC_25A_

typedef struct {
  uint8 LMC_LSFCActiveSt;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_LMC_25A;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_LMC_FD_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_LMC_FD_

typedef struct {
  boolean LMC_FTSC_ActiveSt;
  boolean LMC_HSPC_ActiveSt;
  uint8 LMC_WBDC_ActiveSt;
  IDT_Message_QF MessageQf;
} IDT_SM_SysSigGrp_LMC_FD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AEB_Flag_
#define DEFINED_TYPEDEF_FOR_AEB_Flag_

typedef struct {
  uint32 AEB_FunConfig;
  uint8 AEB_State;
  uint8 AEB_PrefillReq;
  uint8 AEB_Level;
  uint8 AEB_Type;
} AEB_Flag;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FCW_Flag_
#define DEFINED_TYPEDEF_FOR_FCW_Flag_

typedef struct {
  uint8 FCW_State;
  uint8 FCW_WarningState;
  uint8 FCW_LatentWarning;
  uint8 FCW_AcuteWarningState;
  uint8 FCW_TargetId;
} FCW_Flag;

#endif

#ifndef DEFINED_TYPEDEF_FOR_DRT_
#define DEFINED_TYPEDEF_FOR_DRT_

typedef struct {
  float32 DRT_TTC;
  uint8 DRT_ID;
  uint8 DRT_ObjectClass;
} DRT;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FCW_DEV_
#define DEFINED_TYPEDEF_FOR_FCW_DEV_

typedef struct {
  float32 FCW_ReactionTime;
  float32 FCW_TTCThres;
  float32 FCW_TTC;
} FCW_DEV;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CAH_ActiveSafety_
#define DEFINED_TYPEDEF_FOR_CAH_ActiveSafety_

typedef struct {
  FCW_Flag fcw_flag;
  AEB_Flag aeb_flag;
  DRT drt;
  FCW_DEV fcw_dev;
  AEBMon_State aebmon_state;
  float32 LgSf_AEBDecelReq;
  uint8 LgSf_AEBType;
  uint8 LgSf_AEBDecelReqFlag;
  uint8 LgSf_AEBBrkReqFlag;
} CAH_ActiveSafety;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_CATgtVisnID_
#define DEFINED_TYPEDEF_FOR_IDT_CATgtVisnID_

typedef struct {
  uint8 AEBTgtVisnID;
  uint8 FCWTgtVisnID;
} IDT_CATgtVisnID;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AdpPath_
#define DEFINED_TYPEDEF_FOR_AdpPath_

typedef struct {
  AdpPath_ValidSamples ValidSamples;
  AdpPath_EndDst EndDst;
  AdpPath_Y X[50];
  AdpPath_Y Y[50];
  AdpPath_Curvature Curvature[50];
  AdpPath_DstAlongPah DstAlongPah[50];
  AdpPath_LocalOffsetCoGx LocalOffsetCoGx;
  AdpPath_LocalHeadingCoGx LocalHeadingCoGx;
  AdpPath_YawratePathCoGx YawratePathCoGx;
  AdpPath_YawratederivativePathCoGx YawratederivativePathCoGx;
  TrueFalse Reset;
  TrueFalse LnChgInProgress;
} AdpPath;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadPath_ObjectInfo_
#define DEFINED_TYPEDEF_FOR_RoadPath_ObjectInfo_

typedef struct {
  TrueFalse32 ObjUseForUpdAgDir[32];
  TrueFalse32 ObjUseForUpdPosn[32];
  TrueFalse32 ObjVldForExtrpn[32];
  TrueFalse32 ObjVldForUpdPosn[32];
} RoadPath_ObjectInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadGeometry_LaneMarkerInfo_Left_
#define DEFINED_TYPEDEF_FOR_RoadGeometry_LaneMarkerInfo_Left_

typedef struct {
  MarkerType Type;
  NoYes IsUsedByRGF;
} RoadGeometry_LaneMarkerInfo_Left;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadGeometry_LaneMarkerInfo_Right_
#define DEFINED_TYPEDEF_FOR_RoadGeometry_LaneMarkerInfo_Right_

typedef struct {
  MarkerType Type;
  NoYes IsUsedByRGF;
} RoadGeometry_LaneMarkerInfo_Right;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadPath_LaneMarkerInfo_
#define DEFINED_TYPEDEF_FOR_RoadPath_LaneMarkerInfo_

typedef struct {
  RoadGeometry_LaneMarkerInfo_Left Left;
  RoadGeometry_LaneMarkerInfo_Right Right;
} RoadPath_LaneMarkerInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadPath_
#define DEFINED_TYPEDEF_FOR_RoadPath_

typedef struct {
  RoadPath_LaneWidth LaneWidth;
  RoadPath_OffsLat OffsLat;
  RoadPath_DstLgtToEndLaneMrk DstLgtToEndLaneMrk;
  RoadPath_DstLgtToEndEhCrvt DstLgtToEndEhCrvt;
  RoadPath_DstLgtToEndObj DstLgtToEndObj;
  RoadPath_AgDir AgDir;
  RoadPath_Crvt Crvt;
  RoadPath_CrvtRate CrvtRate[3];
  RoadPath_SegLen SegLen[3];
  NoYes Strtd;
  NoYes Vld;
  NoYes VldForELKA;
  TrfcAssiConf VldForTrfcAssi;
  LaneChangeType LaneChange;
  NoYes VldForCSA;
  NoYes VldForOncomingBraking;
  RoadPath_ObjectInfo ObjectInfo;
  RoadPath_LowConfDist LowConfDist;
  RoadPath_HighConfDist HighConfDist;
  Reliable5 CrvtQly;
  NoYes VldForACC;
  RoadPath_LaneMarkerInfo LaneMarkerInfo;
  RoadPath_Timestamp Timestamp;
} RoadPath;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtAdjLLeft_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtAdjLLeft_

typedef struct {
  TargetsSelectedForACC_AccTgtAdjLLeft_A A;
  TargetsSelectedForACC_AccTgtAdjLLeft_Heading Heading;
  TargetsSelectedForACC_AccTgtAdjLLeft_Idn Idn;
  TargetsSelectedForACC_AccTgtAdjLLeft_Index Index;
  TargetsSelectedForACC_AccTgtAdjLLeft_latPositionRoad latPositionRoad;
  DataConfidenceLvl3 latPositionRoadConfidence;
  MotionHistory motionHistory;
  MotionPattern2 motionPattern;
  TargetsSelectedForACC_AccTgtAdjLLeft_PosnLat PosnLat;
  TargetsSelectedForACC_AccTgtAdjLLeft_PosnLgt PosnLgt;
  TargetsSelectedForACC_AccTgtAdjLLeft_Spd Spd;
  TargetLaneStatus TgtLaneSts;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
} TargetsSelectedForACC_AccTgtAdjLLeft;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtAdjLRight_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtAdjLRight_

typedef struct {
  TargetsSelectedForACC_AccTgtAdjLRight_A A;
  TargetsSelectedForACC_AccTgtAdjLRight_Heading Heading;
  TargetsSelectedForACC_AccTgtAdjLRight_Idn Idn;
  TargetsSelectedForACC_AccTgtAdjLRight_Index Index;
  TargetsSelectedForACC_AccTgtAdjLRight_latPositionRoad latPositionRoad;
  DataConfidenceLvl3 latPositionRoadConfidence;
  MotionHistory motionHistory;
  MotionPattern2 motionPattern;
  TargetsSelectedForACC_AccTgtAdjLRight_PosnLat PosnLat;
  TargetsSelectedForACC_AccTgtAdjLRight_PosnLgt PosnLgt;
  TargetsSelectedForACC_AccTgtAdjLRight_Spd Spd;
  TargetLaneStatus TgtLaneSts;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
} TargetsSelectedForACC_AccTgtAdjLRight;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtCutIn_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtCutIn_

typedef struct {
  TargetsSelectedForACC_AccTgtCutIn_A A;
  TargetsSelectedForACC_AccTgtCutIn_Heading Heading;
  TargetsSelectedForACC_AccTgtCutIn_Idn Idn;
  TargetsSelectedForACC_AccTgtCutIn_Index Index;
  TargetsSelectedForACC_AccTgtCutIn_latPositionRoad latPositionRoad;
  DataConfidenceLvl3 latPositionRoadConfidence;
  MotionHistory motionHistory;
  MotionPattern2 motionPattern;
  TargetsSelectedForACC_AccTgtCutIn_PosnLat PosnLat;
  TargetsSelectedForACC_AccTgtCutIn_PosnLgt PosnLgt;
  TargetsSelectedForACC_AccTgtCutIn_Spd Spd;
  TargetLaneStatus TgtLaneSts;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
} TargetsSelectedForACC_AccTgtCutIn;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtFrstClstLane_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtFrstClstLane_

typedef struct {
  TargetsSelectedForACC_AccTgtFrstClstLane_A A;
  TargetsSelectedForACC_AccTgtFrstClstLane_Heading Heading;
  TargetsSelectedForACC_AccTgtFrstClstLane_Idn Idn;
  TargetsSelectedForACC_AccTgtFrstClstLane_Index Index;
  TargetsSelectedForACC_AccTgtFrstClstLane_latPositionRoad latPositionRoad;
  DataConfidenceLvl3 latPositionRoadConfidence;
  MotionHistory motionHistory;
  MotionPattern2 motionPattern;
  TargetsSelectedForACC_AccTgtFrstClstLane_PosnLat PosnLat;
  TargetsSelectedForACC_AccTgtFrstClstLane_PosnLgt PosnLgt;
  TargetsSelectedForACC_AccTgtFrstClstLane_Spd Spd;
  TargetLaneStatus TgtLaneSts;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
} TargetsSelectedForACC_AccTgtFrstClstLane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtSecClstLane_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_AccTgtSecClstLane_

typedef struct {
  TargetsSelectedForACC_AccTgtSecClstLane_A A;
  TargetsSelectedForACC_AccTgtSecClstLane_Heading Heading;
  TargetsSelectedForACC_AccTgtSecClstLane_Idn Idn;
  TargetsSelectedForACC_AccTgtSecClstLane_Index Index;
  TargetsSelectedForACC_AccTgtSecClstLane_latPositionRoad latPositionRoad;
  DataConfidenceLvl3 latPositionRoadConfidence;
  MotionHistory motionHistory;
  MotionPattern2 motionPattern;
  TargetsSelectedForACC_AccTgtSecClstLane_PosnLat PosnLat;
  TargetsSelectedForACC_AccTgtSecClstLane_PosnLgt PosnLgt;
  TargetsSelectedForACC_AccTgtSecClstLane_Spd Spd;
  TargetLaneStatus TgtLaneSts;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
} TargetsSelectedForACC_AccTgtSecClstLane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TrafficFlow_LaneProperties_
#define DEFINED_TYPEDEF_FOR_TrafficFlow_LaneProperties_

typedef struct {
  TrafficFlow_LaneProperties_TrfcFlowAvrgDst TrfcFlowAvrgDst;
  TrafficFlow_LaneProperties_TrfcFlowAvrgSpd TrfcFlowAvrgSpd;
  TrafficFlowDir TrfcFlowNormDir;
  DataConfidenceLvl4 TrfcFlowQly;
} TrafficFlow_LaneProperties;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TrafficFlow_
#define DEFINED_TYPEDEF_FOR_TrafficFlow_

typedef struct {
  DataConfidenceLvl4 DrvgSideQly;
  TrafficFlow_LaneProperties LaneProperties[3];
  TrafficFlow_SequenceID SequenceID;
  DrivingSide5 DrivingSide;
} TrafficFlow;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FunModeInfo_warning_Info_T_
#define DEFINED_TYPEDEF_FOR_FunModeInfo_warning_Info_T_

typedef struct {
  WarningInfo_left_warning_level_T left_warning_level;
  WarningInfo_left_warning_location_T left_warning_location;
  WarningInfo_right_warning_level_T right_warning_level;
  WarningInfo_right_warning_location_T right_warning_location;
} FunModeInfo_warning_Info_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AutoDriveInfo_hands_off_info_T_
#define DEFINED_TYPEDEF_FOR_AutoDriveInfo_hands_off_info_T_

typedef struct {
  hands_off_level_T hands_off_level;
  hands_off_count_down_T hands_off_count_down;
  hands_off_count_down_time_T hands_off_count_down_time;
} AutoDriveInfo_hands_off_info_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AutoDriveInfo_ToDDS_T_
#define DEFINED_TYPEDEF_FOR_AutoDriveInfo_ToDDS_T_

typedef struct {
  AutoDriveInfo_adas_status_T AutoDriveInfo_adas_status;
  AutoDriveInfo_acc_status_T AutoDriveInfo_acc_status;
  AutoDriveInfo_fault_adas_T AutoDriveInfo_fault_adas;
  AutoDriveInfo_hands_off_info_T AutoDriveInfo_hands_off_info;
  AutoDriveInfo_adas_target_speed_set_T AutoDriveInfo_adas_target_speed_set;
  AutoDriveInfo_speed_set_limit_T AutoDriveInfo_speed_set_limit;
  ActiveSafetyInfo_textinfo_index_T AutoDriveInfo_textinfo_index;
} AutoDriveInfo_ToDDS_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ActiveSafetyInfo_FunmodeInfo_T_
#define DEFINED_TYPEDEF_FOR_ActiveSafetyInfo_FunmodeInfo_T_

typedef struct {
  FunModeInfo_warning_Info_T warning_Info;
  FunModeInfo_warning_state_T warning_state;
} ActiveSafetyInfo_FunmodeInfo_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ActiveSafetyInfo_ToDDS_T_
#define DEFINED_TYPEDEF_FOR_ActiveSafetyInfo_ToDDS_T_

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

#endif

#ifndef DEFINED_TYPEDEF_FOR_HBC2CANBus_T_
#define DEFINED_TYPEDEF_FOR_HBC2CANBus_T_

typedef struct {
  HBC2CAN_FSC_IHBC_Mode_T HBC2CAN_FSC_IHBC_Mode;
  HBC2CAN_FSC_IHBC_High_Beam_Request_T HBC2CAN_FSC_IHBC_High_Beam_Request;
  HBC2CAN_FSC_IHBC_NotAvailableReason_T HBC2CAN_FSC_IHBC_NotAvailableReason;
} HBC2CANBus_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_HmiConfig_FromDDS_T_
#define DEFINED_TYPEDEF_FOR_HmiConfig_FromDDS_T_

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

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SpeedLimitWarning_
#define DEFINED_TYPEDEF_FOR_IDT_SpeedLimitWarning_

typedef struct {
  boolean is_limit_cancel;
  boolean is_speed_camera;
  uint8 func_mode;
  uint32 high_speed_limit_value;
  uint32 low_speed_limit_value;
  uint32 warning_type;
  float32 speed_camera_distance;
} IDT_SpeedLimitWarning;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_RoadSignInfo_
#define DEFINED_TYPEDEF_FOR_IDT_RoadSignInfo_

typedef struct {
  IDT_SpeedLimitWarning speed_warning;
  uint8 tsr_sign;
  uint32 textinfo_index;
} IDT_RoadSignInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Intersection_info_T_
#define DEFINED_TYPEDEF_FOR_Intersection_info_T_

typedef struct {
  boolean is_cross_road;
  float32 cross_road_link_range_a[2];
  boolean is_stop_line;
  float32 stop_line_distance;
} Intersection_info_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Lane_property_T_
#define DEFINED_TYPEDEF_FOR_Lane_property_T_

typedef struct {
  boolean is_opening_lane;
  boolean is_merge_lane;
  uint8 merge_direction;
  sint8 num_lane_turn_type;
  uint8 lane_turn_type_a[5];
} Lane_property_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_point_info_T_
#define DEFINED_TYPEDEF_FOR_point_info_T_

typedef struct {
  point_info_x_T point_info_x;
  point_info_y_T point_info_y;
} point_info_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Poly_info_clothoid_T_
#define DEFINED_TYPEDEF_FOR_Poly_info_clothoid_T_

typedef struct {
  float32 Poly_info_clothoid_a;
  float32 Poly_info_clothoid_b;
  float32 Poly_info_clothoid_c;
  float32 Poly_info_clothoid_d1;
  float32 Poly_info_clothoid_d2;
  float32 Poly_info_clothoid_detection_distance;
  float32 Poly_info_clothoid_transition_distance;
  float32 Poly_info_clothoid_use_second_poly;
} Poly_info_clothoid_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_poly_info_
#define DEFINED_TYPEDEF_FOR_poly_info_

typedef struct {
  boolean poly_info_valid;
  Poly_info_clothoid_T poly_info_clothoid;
} poly_info;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Refline_T_
#define DEFINED_TYPEDEF_FOR_Refline_T_

typedef struct {
  uint32 line_id;
  uint8 confidence;
  uint8 trk_state;
  uint8 smooth_status;
  uint8 point_num;
  point_info_T point_info_a[100];
  poly_info poly_info_;
  float32 start_distance;
  float32 end_distance;
  uint8 lane_change;
} Refline_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadInfo_T_
#define DEFINED_TYPEDEF_FOR_RoadInfo_T_

typedef struct {
  uint32 frame_id;
  uint64 timestamp;
  Intersection_info_T intersection_info;
  Lane_property_T lane_property;
  Refline_T refline;
} RoadInfo_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_EgoVehicleState_
#define DEFINED_TYPEDEF_FOR_EgoVehicleState_

typedef struct {
  EgoVehicleState_VLgt VLgt;
  EgoVehicleState_ALgt ALgt;
  EgoVehicleState_ALgtRaw ALgtRaw;
  EgoVehicleState_ALatRaw ALatRaw;
  EgoVehicleState_YawRate YawRate;
  EgoVehicleState_YawRateRaw YawRateRaw;
  EgoVehicleState_SequenceID SequenceID;
  TrueFalse valid;
} EgoVehicleState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FlcRoadCover_Snow_
#define DEFINED_TYPEDEF_FOR_FlcRoadCover_Snow_

typedef struct {
  TrueFalse isDetected;
  FlcRoadCover_Snow_confidence confidence;
} FlcRoadCover_Snow;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FlcRoadCover_Gravel_
#define DEFINED_TYPEDEF_FOR_FlcRoadCover_Gravel_

typedef struct {
  TrueFalse isDetected;
  FlcRoadCover_Gravel_confidence confidence;
} FlcRoadCover_Gravel;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FlcRoadCover_Wet_
#define DEFINED_TYPEDEF_FOR_FlcRoadCover_Wet_

typedef struct {
  TrueFalse isDetected;
  FlcRoadCover_Wet_confidence confidence;
} FlcRoadCover_Wet;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FlcRoadCover_
#define DEFINED_TYPEDEF_FOR_FlcRoadCover_

typedef struct {
  FlcRoadCover_Snow Snow;
  FlcRoadCover_Gravel Gravel;
  FlcRoadCover_Wet Wet;
} FlcRoadCover;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_LongTgtVisnID_
#define DEFINED_TYPEDEF_FOR_IDT_LongTgtVisnID_

typedef struct {
  uint8 ACCTgtVisnID;
  uint8 CutInTgtVisnID;
  uint8 FDWTgtVisnID;
} IDT_LongTgtVisnID;

#endif

#ifndef DEFINED_TYPEDEF_FOR_AdditionalTarSelnSignals_T_
#define DEFINED_TYPEDEF_FOR_AdditionalTarSelnSignals_T_

typedef struct {
  AdditionalTarSelnSignals_Signal1 Signal1;
  AdditionalTarSelnSignals_Signal2 Signal2;
  AdditionalTarSelnSignals_Signal3 Signal3;
  AdditionalTarSelnSignals_Signal4 Signal4;
  AdditionalTarSelnSignals_Signal5 Signal5;
  AdditionalTarSelnSignals_Signal6 Signal6;
  AdditionalTarSelnSignals_Signal7 Signal7;
  AdditionalTarSelnSignals_Signal8 Signal8;
  AdditionalTarSelnSignals_Signal9 Signal9;
  AdditionalTarSelnSignals_Signal10 Signal10;
} AdditionalTarSelnSignals_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CrossingObject_EgoStates_
#define DEFINED_TYPEDEF_FOR_CrossingObject_EgoStates_

typedef struct {
  CrossingObject_EgoStates_latAcceleration latAcceleration;
  CrossingObject_EgoStates_longAcceleration longAcceleration;
  CrossingObject_EgoStates_latPosition latPosition;
  CrossingObject_EgoStates_longPosition longPosition;
  CrossingObject_EgoStates_latVelocity latVelocity;
  CrossingObject_EgoStates_longVelocity longVelocity;
} CrossingObject_EgoStates;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CrossingObject_Properties_
#define DEFINED_TYPEDEF_FOR_CrossingObject_Properties_

typedef struct {
  CrossingObject_Properties_id id;
  MotionPattern motionPattern;
  ReferencePoint referencePoint;
  ObjClassnTyp type;
  CrossingObject_Properties_width width;
  CrossingObject_Properties_length length;
  CmbbObjConfidence cmbbQly;
} CrossingObject_Properties;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CrossingObject_
#define DEFINED_TYPEDEF_FOR_CrossingObject_

typedef struct {
  CrossingObject_EgoStates EgoStates[30];
  CrossingObject_Properties Properties[30];
  CrossingObject_SequenceID SequenceID;
} CrossingObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FusedFrontObject_Properties_
#define DEFINED_TYPEDEF_FOR_FusedFrontObject_Properties_

typedef struct {
  FusedFrontObject_Properties_accelerationStdDev accelerationStdDev;
  OnOff2 brakeLight;
  FusedFrontObject_Properties_classificationConfidence classificationConfidence;
  CmbbObjConfidence cmsConfidence;
  Reliable3 cmbbSecConfidence;
  FusedFrontObject_Properties_distanceToLeftNearLaneMarking distanceToLeftNearLaneMarking;
  FusedFrontObject_Properties_distanceToRightNearLaneMarking distanceToRightNearLaneMarking;
  DataConfidenceLvl4 elkaQly;
  FusedFrontObject_Properties_existenceConfidence existenceConfidence;
  Reliable3 fcwQly;
  FusionSource fusionSource;
  OnOff2 hazardLightStatus;
  FusedFrontObject_Properties_headingStdDev headingStdDev;
  FusedFrontObject_Properties_id id;
  FusedFrontObject_Properties_innovationFactor innovationFactor;
  FusedFrontObject_Properties_latPositionStdDev latPositionStdDev;
  Reliable6 leftNearLaneMarkingConfidence;
  FusedFrontObject_Properties_longPositionStdDev longPositionStdDev;
  MotionHistory motionHistory;
  MotionModel3 motionModel;
  MotionPattern2 motionPattern;
  FusedFrontObject_Properties_radarId radarId;
  ReferencePoint referencePoint;
  FusedFrontObject_Properties_reserved reserved;
  Reliable6 rightNearLaneMarkingConfidence;
  FusedFrontObject_Properties_speedStdDev speedStdDev;
  TrackStatus6 trackStatus;
  TrafficScenarioTyp trafficScenario;
  IndicatorStatus turnIndicator;
  ObjectClass7 type;
  FusedFrontObject_Properties_visionId visionId;
  FusedFrontObject_Properties_width width;
  FusedFrontObject_Properties_length length;
  SensorUpdateStatus1 SensorUpdateStatus;
} FusedFrontObject_Properties;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FusedFrontObject_States_
#define DEFINED_TYPEDEF_FOR_FusedFrontObject_States_

typedef struct {
  FusedFrontObject_States_acceleration acceleration;
  FusedFrontObject_States_curvature curvature;
  FusedFrontObject_States_heading heading;
  FusedFrontObject_States_latAcceleration latAcceleration;
  FusedFrontObject_States_latPosition latPosition;
  FusedFrontObject_States_latVelocity latVelocity;
  FusedFrontObject_States_longAcceleration longAcceleration;
  FusedFrontObject_States_longPosition longPosition;
  FusedFrontObject_States_longVelocity longVelocity;
  FusedFrontObject_States_speed speed;
} FusedFrontObject_States;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_Properties_
#define DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_Properties_

typedef struct {
  LaneAssignedFrontObjects_Properties_id id;
  DataConfidenceLvl2 latPositionOncomingConfidence;
  DataConfidenceLvl3 latPositionRoadConfidence;
} LaneAssignedFrontObjects_Properties;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_States_
#define DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_States_

typedef struct {
  LaneAssignedFrontObjects_States_latAccelerationRoad latAccelerationRoad;
  LaneAssignedFrontObjects_States_latPositionRoad latPositionRoad;
  LaneAssignedFrontObjects_States_headingRoad headingRoad;
  LaneAssignedFrontObjects_States_latVelocityRoad latVelocityRoad;
  LaneAssignedFrontObjects_States_longAccelerationRoad longAccelerationRoad;
  LaneAssignedFrontObjects_States_longPositionRoad longPositionRoad;
  LaneAssignedFrontObjects_States_longVelocityRoad longVelocityRoad;
} LaneAssignedFrontObjects_States;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_Lane_DblClothoid_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_Lane_DblClothoid_

typedef struct {
  LaneMarker_EgoLane_Left_Lane_DblClothoid_a a;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_b b;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_c c;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_d1 d1;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_d2 d2;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_longDistToEnd longDistToEnd;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_longDistToStart longDistToStart;
  TrueFalse secClothoidActive;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_transitionDist transitionDist;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_aVariance aVariance;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_bVariance bVariance;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_cVariance cVariance;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_d1Variance d1Variance;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_d2Variance d2Variance;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtOffset vrtOffset;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtHeading vrtHeading;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtCrvt vrtCrvt;
  LaneMarker_EgoLane_Left_Lane_DblClothoid_vrtCrvtRate vrtCrvtRate;
} LaneMarker_EgoLane_Left_Lane_DblClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_Lane_DblClothoid_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_Lane_DblClothoid_

typedef struct {
  LaneMarker_EgoLane_Right_Lane_DblClothoid_a a;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_b b;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_c c;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_d1 d1;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_d2 d2;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_longDistToEnd longDistToEnd;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_longDistToStart longDistToStart;
  TrueFalse secClothoidActive;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_transitionDist transitionDist;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_aVariance aVariance;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_bVariance bVariance;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_cVariance cVariance;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_d1Variance d1Variance;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_d2Variance d2Variance;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtOffset vrtOffset;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtHeading vrtHeading;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtCrvt vrtCrvt;
  LaneMarker_EgoLane_Right_Lane_DblClothoid_vrtCrvtRate vrtCrvtRate;
} LaneMarker_EgoLane_Right_Lane_DblClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_DebugBus_
#define DEFINED_TYPEDEF_FOR_LaneMarker_DebugBus_

typedef struct {
  LaneMarker_DebugBus_debugFloat1 debugFloat1;
  LaneMarker_DebugBus_debugFloat2 debugFloat2;
  LaneMarker_DebugBus_debugFloat3 debugFloat3;
  LaneMarker_DebugBus_debugFloat4 debugFloat4;
  LaneMarker_DebugBus_debugFloat5 debugFloat5;
  LaneMarker_DebugBus_debugFloat6 debugFloat6;
  LaneMarker_DebugBus_debugFloat7 debugFloat7;
  LaneMarker_DebugBus_debugFloat8 debugFloat8;
  LaneMarker_DebugBus_debugInteger1 debugInteger1;
  LaneMarker_DebugBus_debugInteger2 debugInteger2;
  LaneMarker_DebugBus_debugInteger3 debugInteger3;
  LaneMarker_DebugBus_debugInteger4 debugInteger4;
} LaneMarker_DebugBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_Lane_DblClothoid_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_Lane_DblClothoid_

typedef struct {
  LaneMarker_SecClsLeft_Lane_DblClothoid_a a;
  LaneMarker_SecClsLeft_Lane_DblClothoid_b b;
  LaneMarker_SecClsLeft_Lane_DblClothoid_c c;
  LaneMarker_SecClsLeft_Lane_DblClothoid_d1 d1;
  LaneMarker_SecClsLeft_Lane_DblClothoid_d2 d2;
  LaneMarker_SecClsLeft_Lane_DblClothoid_longDistToEnd longDistToEnd;
  LaneMarker_SecClsLeft_Lane_DblClothoid_longDistToStart longDistToStart;
  TrueFalse secClothoidActive;
  LaneMarker_SecClsLeft_Lane_DblClothoid_transitionDist transitionDist;
  LaneMarker_SecClsLeft_Lane_DblClothoid_aVariance aVariance;
  LaneMarker_SecClsLeft_Lane_DblClothoid_bVariance bVariance;
  LaneMarker_SecClsLeft_Lane_DblClothoid_cVariance cVariance;
  LaneMarker_SecClsLeft_Lane_DblClothoid_d1Variance d1Variance;
  LaneMarker_SecClsLeft_Lane_DblClothoid_d2Variance d2Variance;
  LaneMarker_SecClsLeft_Lane_DblClothoid_vrtOffset vrtOffset;
  LaneMarker_SecClsLeft_Lane_DblClothoid_vrtHeading vrtHeading;
  LaneMarker_SecClsLeft_Lane_DblClothoid_vrtCrvt vrtCrvt;
  LaneMarker_SecClsLeft_Lane_DblClothoid_vrtCrvtRate vrtCrvtRate;
} LaneMarker_SecClsLeft_Lane_DblClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_Lane_DblClothoid_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_Lane_DblClothoid_

typedef struct {
  LaneMarker_SecClsRight_Lane_DblClothoid_a a;
  LaneMarker_SecClsRight_Lane_DblClothoid_b b;
  LaneMarker_SecClsRight_Lane_DblClothoid_c c;
  LaneMarker_SecClsRight_Lane_DblClothoid_d1 d1;
  LaneMarker_SecClsRight_Lane_DblClothoid_d2 d2;
  LaneMarker_SecClsRight_Lane_DblClothoid_longDistToEnd longDistToEnd;
  LaneMarker_SecClsRight_Lane_DblClothoid_longDistToStart longDistToStart;
  TrueFalse secClothoidActive;
  LaneMarker_SecClsRight_Lane_DblClothoid_transitionDist transitionDist;
  LaneMarker_SecClsRight_Lane_DblClothoid_aVariance aVariance;
  LaneMarker_SecClsRight_Lane_DblClothoid_bVariance bVariance;
  LaneMarker_SecClsRight_Lane_DblClothoid_cVariance cVariance;
  LaneMarker_SecClsRight_Lane_DblClothoid_d1Variance d1Variance;
  LaneMarker_SecClsRight_Lane_DblClothoid_d2Variance d2Variance;
  LaneMarker_SecClsRight_Lane_DblClothoid_vrtOffset vrtOffset;
  LaneMarker_SecClsRight_Lane_DblClothoid_vrtHeading vrtHeading;
  LaneMarker_SecClsRight_Lane_DblClothoid_vrtCrvt vrtCrvt;
  LaneMarker_SecClsRight_Lane_DblClothoid_vrtCrvtRate vrtCrvtRate;
} LaneMarker_SecClsRight_Lane_DblClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_LaneEvent_
#define DEFINED_TYPEDEF_FOR_LaneMarker_LaneEvent_

typedef struct {
  LaneMarker_LaneEvent_distance distance;
  EventTypeDef eventType;
  LaneMarker_LaneEvent_id id;
  LaneTrackId laneTrack;
} LaneMarker_LaneEvent;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_TemporaryMarking_
#define DEFINED_TYPEDEF_FOR_LaneMarker_TemporaryMarking_

typedef struct {
  LaneMarker_TemporaryMarking_longDistanceToStart longDistanceToStart;
  TemporaryMarkingType type;
} LaneMarker_TemporaryMarking;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_Lane_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_Lane_

typedef struct {
  LaneColor color;
  LaneMarker_EgoLane_Left_Lane_DblClothoid DblClothoid;
  LaneMarker_EgoLane_Left_Lane_id id;
  MarkingType markingType;
  LaneMarker_EgoLane_Left_Lane_markingWidth markingWidth;
  LaneMarker_EgoLane_Left_Lane_measurementQuality measurementQuality;
  LaneMarker_EgoLane_Left_Lane_modelError modelError;
  SecondMarkingType secondMarkingType;
  LaneMarker_EgoLane_Left_Lane_selectionConfidence selectionConfidence;
  LaneMarkerStructure structure;
  LaneMarker_EgoLane_Left_Lane_totalMarkingWidth totalMarkingWidth;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
} LaneMarker_EgoLane_Left_Lane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Left_

typedef struct {
  LaneMarker_EgoLane_Left_Lane Lane;
} LaneMarker_EgoLane_Left;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_Lane_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_Lane_

typedef struct {
  LaneColor color;
  LaneMarker_EgoLane_Right_Lane_DblClothoid DblClothoid;
  LaneMarker_EgoLane_Right_Lane_id id;
  MarkingType markingType;
  LaneMarker_EgoLane_Right_Lane_markingWidth markingWidth;
  LaneMarker_EgoLane_Right_Lane_measurementQuality measurementQuality;
  LaneMarker_EgoLane_Right_Lane_modelError modelError;
  SecondMarkingType secondMarkingType;
  LaneMarker_EgoLane_Right_Lane_selectionConfidence selectionConfidence;
  LaneMarkerStructure structure;
  LaneMarker_EgoLane_Right_Lane_totalMarkingWidth totalMarkingWidth;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
} LaneMarker_EgoLane_Right_Lane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_Right_

typedef struct {
  LaneMarker_EgoLane_Right_Lane Lane;
} LaneMarker_EgoLane_Right;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_
#define DEFINED_TYPEDEF_FOR_LaneMarker_EgoLane_

typedef struct {
  LaneMarker_EgoLane_Left Left;
  LaneMarker_EgoLane_Right Right;
  LaneMarker_EgoLane_parallelDistance parallelDistance;
  LaneMarker_EgoLane_validProjectionDistance validProjectionDistance;
  TrueFalse attentionMarkerDetected;
  boolean isMergeLane;
  float32 MergeRange;
} LaneMarker_EgoLane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_Lane_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_Lane_

typedef struct {
  LaneColor color;
  LaneMarker_SecClsLeft_Lane_DblClothoid DblClothoid;
  LaneMarker_SecClsLeft_Lane_id id;
  MarkingType markingType;
  LaneMarker_SecClsLeft_Lane_markingWidth markingWidth;
  LaneMarker_SecClsLeft_Lane_measurementQuality measurementQuality;
  LaneMarker_SecClsLeft_Lane_modelError modelError;
  SecondMarkingType secondMarkingType;
  LaneMarker_SecClsLeft_Lane_selectionConfidence selectionConfidence;
  LaneMarkerStructure structure;
  LaneMarker_SecClsLeft_Lane_totalMarkingWidth totalMarkingWidth;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
} LaneMarker_SecClsLeft_Lane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsLeft_

typedef struct {
  LaneMarker_SecClsLeft_Lane Lane;
} LaneMarker_SecClsLeft;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_Lane_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_Lane_

typedef struct {
  LaneColor color;
  LaneMarker_SecClsRight_Lane_DblClothoid DblClothoid;
  LaneMarker_SecClsRight_Lane_id id;
  MarkingType markingType;
  LaneMarker_SecClsRight_Lane_markingWidth markingWidth;
  LaneMarker_SecClsRight_Lane_measurementQuality measurementQuality;
  LaneMarker_SecClsRight_Lane_modelError modelError;
  SecondMarkingType secondMarkingType;
  LaneMarker_SecClsRight_Lane_selectionConfidence selectionConfidence;
  LaneMarkerStructure structure;
  LaneMarker_SecClsRight_Lane_totalMarkingWidth totalMarkingWidth;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
} LaneMarker_SecClsRight_Lane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_
#define DEFINED_TYPEDEF_FOR_LaneMarker_SecClsRight_

typedef struct {
  LaneMarker_SecClsRight_Lane Lane;
} LaneMarker_SecClsRight;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RearObject_EgoStates_
#define DEFINED_TYPEDEF_FOR_RearObject_EgoStates_

typedef struct {
  RearObject_EgoStates_heading heading;
  RearObject_EgoStates_latAcceleration latAcceleration;
  RearObject_EgoStates_latPosition latPosition;
  RearObject_EgoStates_latPositionPath latPositionPath;
  RearObject_EgoStates_latVelocity latVelocity;
  RearObject_EgoStates_longAcceleration longAcceleration;
  RearObject_EgoStates_longPosition longPosition;
  RearObject_EgoStates_longPositionPath longPositionPath;
  RearObject_EgoStates_longVelocity longVelocity;
} RearObject_EgoStates;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RearObject_RoadStates_
#define DEFINED_TYPEDEF_FOR_RearObject_RoadStates_

typedef struct {
  RearObject_RoadStates_headingRoad headingRoad;
  RearObject_RoadStates_latAccelerationRoad latAccelerationRoad;
  RearObject_RoadStates_latPositionRoad latPositionRoad;
  RearObject_RoadStates_latVelocityRoad latVelocityRoad;
  RearObject_RoadStates_longAccelerationRoad longAccelerationRoad;
  RearObject_RoadStates_longPositionRoad longPositionRoad;
  RearObject_RoadStates_longVelocityRoad longVelocityRoad;
} RearObject_RoadStates;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RearObject_Properties_
#define DEFINED_TYPEDEF_FOR_RearObject_Properties_

typedef struct {
  Reliable3 latPositionConfidence;
  RearObject_Properties_length length;
  Reliable3 confidence;
  ObjectSize2 type;
  RearObject_Properties_width width;
  RearObject_Properties_id id;
  TrackStatus4 trackStatus;
} RearObject_Properties;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_Edge_SingClothoid_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_Edge_SingClothoid_

typedef struct {
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_a a;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_aVariance aVariance;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_b b;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_bVariance bVariance;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_c c;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_cVariance cVariance;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d d;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d1Variance d1Variance;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_d2Variance d2Variance;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_longDistToEnd longDistToEnd;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_longDistToStart longDistToStart;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtOffset vrtOffset;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtHeading vrtHeading;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtCrvt vrtCrvt;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid_vrtCrvtRate vrtCrvtRate;
} RoadEdge_LeftNonTrvsble_Edge_SingClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_Edge_SingClothoid_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_Edge_SingClothoid_

typedef struct {
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_a a;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_aVariance aVariance;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_b b;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_bVariance bVariance;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_c c;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_cVariance cVariance;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_d d;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_d1Variance d1Variance;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_d2Variance d2Variance;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_longDistToEnd longDistToEnd;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_longDistToStart longDistToStart;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtOffset vrtOffset;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtHeading vrtHeading;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtCrvt vrtCrvt;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid_vrtCrvtRate vrtCrvtRate;
} RoadEdge_RightNonTrvsble_Edge_SingClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_Edge_SingClothoid_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_Edge_SingClothoid_

typedef struct {
  RoadEdge_LeftTrvsble_Edge_SingClothoid_a a;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_aVariance aVariance;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_b b;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_bVariance bVariance;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_c c;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_cVariance cVariance;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_d d;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_d1Variance d1Variance;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_d2Variance d2Variance;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_longDistToEnd longDistToEnd;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_longDistToStart longDistToStart;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtOffset vrtOffset;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtHeading vrtHeading;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtCrvt vrtCrvt;
  RoadEdge_LeftTrvsble_Edge_SingClothoid_vrtCrvtRate vrtCrvtRate;
} RoadEdge_LeftTrvsble_Edge_SingClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_Edge_SingClothoid_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_Edge_SingClothoid_

typedef struct {
  RoadEdge_RightTrvsble_Edge_SingClothoid_a a;
  RoadEdge_RightTrvsble_Edge_SingClothoid_aVariance aVariance;
  RoadEdge_RightTrvsble_Edge_SingClothoid_b b;
  RoadEdge_RightTrvsble_Edge_SingClothoid_bVariance bVariance;
  RoadEdge_RightTrvsble_Edge_SingClothoid_c c;
  RoadEdge_RightTrvsble_Edge_SingClothoid_cVariance cVariance;
  RoadEdge_RightTrvsble_Edge_SingClothoid_d d;
  RoadEdge_RightTrvsble_Edge_SingClothoid_d1Variance d1Variance;
  RoadEdge_RightTrvsble_Edge_SingClothoid_d2Variance d2Variance;
  RoadEdge_RightTrvsble_Edge_SingClothoid_longDistToEnd longDistToEnd;
  RoadEdge_RightTrvsble_Edge_SingClothoid_longDistToStart longDistToStart;
  RoadEdge_RightTrvsble_Edge_SingClothoid_vrtOffset vrtOffset;
  RoadEdge_RightTrvsble_Edge_SingClothoid_vrtHeading vrtHeading;
  RoadEdge_RightTrvsble_Edge_SingClothoid_vrtCrvt vrtCrvt;
  RoadEdge_RightTrvsble_Edge_SingClothoid_vrtCrvtRate vrtCrvtRate;
} RoadEdge_RightTrvsble_Edge_SingClothoid;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_Edge_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_Edge_

typedef struct {
  RoadEdge_LeftNonTrvsble_Edge_id id;
  RoadEdge_LeftNonTrvsble_Edge_measurementQuality measurementQuality;
  RoadEdge_LeftNonTrvsble_Edge_modelError modelError;
  RoadEdge_LeftNonTrvsble_Edge_SingClothoid SingClothoid;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
  RoadEdge_LeftNonTrvsble_Edge_selectionConfidence selectionConfidence;
} RoadEdge_LeftNonTrvsble_Edge;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftNonTrvsble_

typedef struct {
  RoadEdge_LeftNonTrvsble_Edge Edge;
} RoadEdge_LeftNonTrvsble;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_Edge_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_Edge_

typedef struct {
  RoadEdge_LeftTrvsble_Edge_id id;
  RoadEdge_LeftTrvsble_Edge_measurementQuality measurementQuality;
  RoadEdge_LeftTrvsble_Edge_modelError modelError;
  RoadEdge_LeftTrvsble_Edge_SingClothoid SingClothoid;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
  RoadEdge_LeftTrvsble_Edge_selectionConfidence selectionConfidence;
} RoadEdge_LeftTrvsble_Edge;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_
#define DEFINED_TYPEDEF_FOR_RoadEdge_LeftTrvsble_

typedef struct {
  RoadEdge_LeftTrvsble_Edge Edge;
} RoadEdge_LeftTrvsble;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_Edge_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_Edge_

typedef struct {
  RoadEdge_RightNonTrvsble_Edge_id id;
  RoadEdge_RightNonTrvsble_Edge_measurementQuality measurementQuality;
  RoadEdge_RightNonTrvsble_Edge_modelError modelError;
  RoadEdge_RightNonTrvsble_Edge_SingClothoid SingClothoid;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
  RoadEdge_RightNonTrvsble_Edge_selectionConfidence selectionConfidence;
} RoadEdge_RightNonTrvsble_Edge;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightNonTrvsble_

typedef struct {
  RoadEdge_RightNonTrvsble_Edge Edge;
} RoadEdge_RightNonTrvsble;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_Edge_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_Edge_

typedef struct {
  RoadEdge_RightTrvsble_Edge_id id;
  RoadEdge_RightTrvsble_Edge_measurementQuality measurementQuality;
  RoadEdge_RightTrvsble_Edge_modelError modelError;
  RoadEdge_RightTrvsble_Edge_SingClothoid SingClothoid;
  TrackingStatus trackingStatus;
  TrueFalse valid;
  TrueFalse isVerified;
  RoadEdge_RightTrvsble_Edge_selectionConfidence selectionConfidence;
} RoadEdge_RightTrvsble_Edge;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_
#define DEFINED_TYPEDEF_FOR_RoadEdge_RightTrvsble_

typedef struct {
  RoadEdge_RightTrvsble_Edge Edge;
} RoadEdge_RightTrvsble;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RoadEdge_
#define DEFINED_TYPEDEF_FOR_RoadEdge_

typedef struct {
  RoadEdge_LeftNonTrvsble LeftNonTrvsble;
  RoadEdge_RightNonTrvsble RightNonTrvsble;
  RoadEdge_LeftTrvsble LeftTrvsble;
  RoadEdge_RightTrvsble RightTrvsble;
  RoadEdge_SequenceID SequenceID;
} RoadEdge;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_T_
#define DEFINED_TYPEDEF_FOR_TargetsSelectedForACC_T_

typedef struct {
  TargetsSelectedForACC_AccTgtAdjLLeft AccTgtAdjLLeft;
  TargetsSelectedForACC_AccTgtAdjLRight AccTgtAdjLRight;
  TargetsSelectedForACC_AccTgtCutIn AccTgtCutIn;
  TargetsSelectedForACC_AccTgtFrstClstLane AccTgtFrstClstLane;
  TargetsSelectedForACC_AccTgtSecClstLane AccTgtSecClstLane;
} TargetsSelectedForACC_T;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LongCtrlObjInfo_
#define DEFINED_TYPEDEF_FOR_LongCtrlObjInfo_

typedef struct {
  TargetsSelectedForACC_T TargetsSelectedForACC;
  AdditionalTarSelnSignals_T AdditionalTarSelnSignals;
} LongCtrlObjInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehParam_Tx_
#define DEFINED_TYPEDEF_FOR_VehParam_Tx_

typedef struct {
  DynCalPrmForAxleDstReToVehFrnt AxleDstReToVehFrnt;
  DynCalPrmForBicycleMdlAxleDistFrnt SingleTrackAxleDistFrnt;
  DynCalPrmForSteerWhlPosn SteerWhlPosn;
  DynCalPrmForVehLen Len;
  DynCalPrmForVehM Weight;
  DynCalPrmForVehWhlBas WhlBas;
  DynCalPrmForVehWidth Width;
  DynCalPrmForBicycleMdlCornrgStfnFrntByVehSpd SingleTrackCornrgStfnFrntByVehSpd[8];
  DynCalPrmForBicycleMdlCornrgStfnFrnt SingleTrackCornrgStfnFrnt;
  DynCalPrmForBicycleMdlCornrgStfnReByVehSpd SingleTrackCornrgStfnReByVehSpd[8];
  DynCalPrmForBicycleMdlCornrgStfnRe SingleTrackCornrgStfnRe;
  DynCalPrmForVehSteerWhlAgRat SteerWhlAgRat;
  DynCalPrmForVehicleSpdForBicycleMdlCornrgStfn SingleTrackCornrgStfnTable_Spd[8];
  TrueFalse BltFrntExist;
  TrueFalse OncomingBrk;
  DynCalPrmForSteerGrdt SelfStrGrdt;
  VersionTrafficAssist TrafficAssist;
  DynCalPrmForAccBrkLim LongCtrlBrkLim;
  TrueFalse LongCtrEco;
  DynCalPrmForAccSpdLoLim LongCtrSpdLoLim;
  TrueFalse LongCtrStopNGo;
  DynCalPrmForBicycleMdlJ SingleTrackMomentOfInertia;
  DynCalPrmForVehTyp VehTyp;
  DynCalPrmForWhlRadius WhlRadius;
} VehParam_Tx;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FusedFrontObject_
#define DEFINED_TYPEDEF_FOR_FusedFrontObject_

typedef struct {
  FusedFrontObject_Properties Properties[32];
  FusedFrontObject_SequenceID SequenceID;
  FusedFrontObject_States States[32];
  FusedFrontObj_timestamp Timestamp;
} FusedFrontObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_
#define DEFINED_TYPEDEF_FOR_LaneAssignedFrontObjects_

typedef struct {
  LaneAssignedFrontObjects_Properties Properties[32];
  LaneAssignedFrontObjects_SequenceID SequenceID;
  LaneAssignedFrontObjects_States States[32];
} LaneAssignedFrontObjects;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LaneMarker_
#define DEFINED_TYPEDEF_FOR_LaneMarker_

typedef struct {
  LaneMarker_EgoLane EgoLane;
  LaneMarker_DebugBus DebugBus;
  LaneMarker_SecClsLeft SecClsLeft;
  LaneMarker_SecClsRight SecClsRight;
  LaneMarker_LaneEvent LaneEvent[4];
  LaneMarker_TemporaryMarking TemporaryMarking;
  SideSuggestion sideSuggestion;
  LaneChangeDirection2 laneChange;
  LaneMarker_SequenceID SequenceID;
  uint64 Timestamp;
} LaneMarker;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RearObject_
#define DEFINED_TYPEDEF_FOR_RearObject_

typedef struct {
  RearObject_EgoStates EgoStates[10];
  RearObject_RoadStates RoadStates[10];
  RearObject_Properties Properties[10];
  RearObject_SequenceID SequenceID;
} RearObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_FuncTgtVisnID_
#define DEFINED_TYPEDEF_FOR_IDT_FuncTgtVisnID_

typedef struct {
  IDT_LongTgtVisnID LongTgtVisnID;
  IDT_CATgtVisnID CATgtVisnID;
} IDT_FuncTgtVisnID;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ACU_233_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ACU_233_APP_

typedef struct {
  float32 ACU_LateralAcce;
  float32 ACU_LongitAcce;
  uint8 ACU_LateralAcceValid;
  uint8 ACU_LongitAcceValid;
  float32 ACU_YawRate;
  uint8 ACU_YawRateValid;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_ACU_233_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_EPS_18D_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_EPS_18D_APP_

typedef struct {
  uint8 EPS_PulseActiveSts;
  float32 EPS_LateralCtrlHandTorq;
  float32 EPS_MotorTorque;
  uint8 EPS_LateralNotAvailableReason;
  uint8 EPS_LateralActive;
  uint8 EPS_LateralAvailable;
  uint8 EPS_SteeringModeStsFB;
  float32 EPS_SteeringTorque;
  EPS_SteerWheelRotSpd EPS_SteerWheelRotSpd_;
  float32 EPS_SteerWheelAngle;
  uint8 EPS_SteeringTorqueValidData;
  uint8 EPS_SteeringAngleSpeedValidData;
  uint8 EPS_SteeringAngleValidData;
  uint8 EPS_FailureSts;
  uint8 EPS_CalibrationSts;
  uint8 EPS_Fault;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_EPS_18D_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCGW_3B0_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCGW_3B0_APP_

typedef struct {
  float32 GW_TMS_OutsideTemp_Corrected;
  uint8 GW_TMS_OutsideTempVD;
  uint8 GW_TMS_FrontDefrostSts;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_HPCGW_3B0_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCGW_CB_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCGW_CB_APP_

typedef struct {
  float32 GW_MCUS_ActualTorqueFB;
  float32 GW_MCUM_ActualTorqueFB;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_HPCGW_CB_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCVCU_B6_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCVCU_B6_APP_

typedef struct {
  uint8 VCU_GearLeverValid;
  uint8 VCU_GearLevelPosSts;
  uint8 VCU_BrakePedalStsValid;
  uint8 VCU_BrakePedalSts;
  uint8 VCU_BrakeSysFault;
  uint8 VCU_AccPedalValid;
  float32 VCU_ThrottlePosition;
  float32 VCU_FDrvRequestTorque;
  float32 VCU_RDrvRequestTorque;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_HPCVCU_B6_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCVCU_B7_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_HPCVCU_B7_APP_

typedef struct {
  uint8 VCU_ADASPosTorqueCap_VD;
  uint16 VCU_ADASPosTorqueCap;
  uint8 VCU_AllMotWhlTrqFB_VD;
  sint16 VCU_AllMotWhlTrqFB;
  uint16 VCU_RecuBrakeTorqueTarget_Drag;
  uint16 VCU_RecuBrakeTorqueActDrag;
  uint16 VCU_HydraTorqTgt;
  uint8 VCU_TheoAccePedalValid;
  uint16 VCU_RecuBrakeTorqueAct;
  float32 VCU_TheoAccePedalPercent;
  uint8 VCU_DriverAssistProhibitSts;
  uint8 VCU_VlcHoldSts;
  uint8 VCU_ACCActiveSts;
  uint8 VCU_ACCAvailable;
  uint8 VCU_VehicleSts;
  uint8 VCU_Vehicle_Warning;
  uint8 VCU_MAI_Active;
  uint8 VCU_MAI_Available;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_HPCVCU_B7_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_182_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_182_APP_

typedef struct {
  uint8 IBC_VDCFailed;
  uint8 IBC_VDCActive;
  uint8 IBC_EBDFailed;
  uint8 IBC_Vehiclestandstill;
  uint8 IBC_ESCFunctionStatus;
  uint8 IBC_TCSFailed;
  uint8 IBC_TCSActive;
  uint8 IBC_ABSFailed;
  uint8 IBC_ABSActive;
  uint8 IBC_VehicleSpeedVD;
  float32 IBC_VehicleSpeed;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_IBC_182_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_184_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_184_APP_

typedef struct {
  uint16 IBC_RRWheelSpeedRC;
  uint16 IBC_RLWheelSpeedRC;
  uint16 IBC_FRWheelSpeedRC;
  uint16 IBC_FLWheelSpeedRC;
  uint8 IBC_RRWheelSpeedVD;
  uint8 IBC_RLWheelSpeedVD;
  uint8 IBC_FRWheelSpeedVD;
  uint8 IBC_FLWheelSpeedVD;
  uint8 IBC_RRWheelDirection;
  uint8 IBC_RLWheelDirection;
  uint8 IBC_FRWheelDirection;
  uint8 IBC_FLWheelDirection;
  float32 IBC_RRWheelSpeedKPH;
  float32 IBC_RLWheelSpeedKPH;
  float32 IBC_FRWheelSpeedKPH;
  float32 IBC_FLWheelSpeedKPH;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_IBC_184_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_185_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_IBC_185_APP_

typedef struct {
  uint8 IBC_VLC_Available_ACC;
  uint8 IBC_HDCStatus;
  uint8 IBC_QDCACC;
  uint8 IBC_CDD_Active_ACC;
  uint8 IBC_CDD_Available_ACC;
  uint8 IBC_QDCAEB;
  uint8 IBC_AVHAvailable;
  uint8 IBC_AVHStatus;
  uint8 IBC_AEBdecAvailable;
  uint8 IBC_AEB_active;
  uint8 IBC_LdmBLC;
  uint8 IBC_ABP_active;
  uint8 IBC_ABPAviliable;
  uint8 IBC_AWB_active;
  uint8 IBC_AWBAvaliable;
  uint8 IBC_ABA_active;
  uint8 IBC_ABAAvaliable;
  uint8 IBC_LongitAcceActValid;
  float32 IBC_LongitAcceAct;
  uint8 IBC_ABS_FailureLamp;
  uint8 IBC_ESP_FailureLamp;
  uint8 IBC_PlungerPressure_Q;
  float32 IBC_PlungerPressure;
  uint8 IBC_VehicleHoldStatus;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_IBC_185_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ICU_332_APP_
#define DEFINED_TYPEDEF_FOR_IDT_SM_SysSigGrp_ICU_332_APP_

typedef struct {
  uint8 ICU_LDW_shake_SW;
  uint8 ICU_LDW_levelSet;
  uint8 ICU_SLIF_SPEEDCHANGEVOICE_SW;
  uint8 ICU_IACC_Offset_Unit;
  uint8 ICU_IACC_Upper_Offset;
  sint8 ICU_IACC_Lower_Offset;
  uint8 ICU_LDW_Cloud_SW;
  uint8 ICU_SLWF_Cloud_SW;
  uint8 ICU_SLWF_SW;
  uint8 ICU_SLIF_SW;
  uint8 ICU_ISA_controlSW;
  uint8 ICU_ISA_SW;
  uint8 ICU_ELK_SW;
  uint8 ICU_LKA_SW;
  uint8 ICU_LDW_SW;
  uint8 ICU_LCC_SW;
  uint8 ICU_FCW_levelSet;
  uint8 ICU_IHBC_SW;
  uint8 ICU_FCW_SW;
  uint8 ICU_AEB_SW;
  uint8 ICU_RCTB_SW;
  uint8 ICU_RCW_SW;
  uint8 ICU_RCTA_SW;
  uint8 ICU_DOW_SW;
  uint8 ICU_BSD_SW;
  IDT_Message_QF Message_QF;
} IDT_SM_SysSigGrp_ICU_332_APP;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehBusIn_
#define DEFINED_TYPEDEF_FOR_VehBusIn_

typedef struct {
  IDT_SM_SysSigGrp_HPCGW_CB_APP SysSigGrp_HPCGW_CB_APP;
  IDT_SM_SysSigGrp_HPCGW_3B0_APP SysSigGrp_HPCGW_3B0_APP;
  IDT_SM_SysSigGrp_HPCVCU_B6_APP SysSigGrp_HPCVCU_B6_APP;
  IDT_SM_SysSigGrp_HPCVCU_B7_APP SysSigGrp_HPCVCU_B7_APP;
  IDT_SM_SysSigGrp_HPCBCM_290 SysSigGrp_HPCBCM_290;
  IDT_SM_SysSigGrp_ICU_24D SysSigGrp_ICU_24D;
  IDT_SM_SysSigGrp_ICU_332_APP SysSigGrp_ICU_332_APP;
  IDT_SM_SysSigGrp_IVI_5CE SysSigGrp_IVI_5CE;
  IDT_SM_SysSigGrp_ACU_2A3 SysSigGrp_ACU_2A3;
  IDT_SM_SysSigGrp_ACU_233_APP SysSigGrp_ACU_233_APP;
  IDT_SM_SysSigGrp_EPS_18D_APP SysSigGrp_EPS_18D_APP;
  IDT_SM_SysSigGrp_IBC_182_APP SysSigGrp_IBC_182_APP;
  IDT_SM_SysSigGrp_IBC_183 SysSigGrp_IBC_183;
  IDT_SM_SysSigGrp_IBC_184_APP SysSigGrp_IBC_184_APP;
  IDT_SM_SysSigGrp_IBC_185_APP SysSigGrp_IBC_185_APP;
  IDT_SM_SysSigGrp_IBC_227 SysSigGrp_IBC_227;
  IDT_SM_SysSigGrp_IBC_3CE SysSigGrp_IBC_3CE;
  IDT_SM_SysSigGrp_IVI_3E3 SysSigGrp_IVI_3E3;
  IDT_SM_SysSigGrp_HPCBCM_1B4 SysSigGrp_HPCBCM_1B4;
  IDT_SM_SysSigGrp_HPCBCM_1B7 SysSigGrp_HPCBCM_1B7;
  IDT_SM_SysSigGrp_IBC_6F3 SysSigGrp_IBC_6F3;
  IDT_SM_SysSigGrp_LMC_25A SysSigGrp_LMC_25A;
  IDT_SM_SysSigGrp_LMC_FD SysSigGrp_LMC_FD;
} VehBusIn;

#endif

/* AUTOSAR Array Types */
typedef RoadPath_CrvtRate RoadPath_CrvtRate_a[3];
typedef TrueFalse32 RoadPath_ObjectInfo_ObjUseForUpdAgDir_a[32];
typedef TrueFalse32 RoadPath_ObjectInfo_ObjUseForUpdPosn_a[32];
typedef TrueFalse32 RoadPath_ObjectInfo_ObjVldForExtrpn_a[32];
typedef TrueFalse32 RoadPath_ObjectInfo_ObjVldForUpdPosn_a[32];
typedef RoadPath_SegLen RoadPath_SegLen_a[3];
typedef TrafficFlow_LaneProperties TrafficFlow_LaneProperties_a[3];
typedef CrossingObject_EgoStates CrossingObject_EgoStates_a[30];
typedef CrossingObject_Properties CrossingObject_Properties_a[30];
typedef AdpPath_Curvature rt_Array_AdpPath_Curvature_50[50];
typedef AdpPath_DstAlongPah rt_Array_AdpPath_DstAlongPah_50[50];
typedef AdpPath_Y rt_Array_AdpPath_Y_50[50];
typedef DynCalPrmForBicycleMdlCornrgStfnFrntByVehSpd rt_Array_DynCalPrmForBicycleMdlCornrgStfnFrntByVehSpd_8[8];
typedef DynCalPrmForBicycleMdlCornrgStfnReByVehSpd rt_Array_DynCalPrmForBicycleMdlCornrgStfnReByVehSpd_8[8];
typedef DynCalPrmForVehicleSpdForBicycleMdlCornrgStfn rt_Array_DynCalPrmForVehicleSpdForBicycleMdlCornrgStfn_8[8];
typedef FusedFrontObject_Properties rt_Array_FusedFrontObject_Properties_32[32];
typedef FusedFrontObject_States rt_Array_FusedFrontObject_States_32[32];
typedef LaneAssignedFrontObjects_Properties rt_Array_LaneAssignedFrontObjects_Properties_32[32];
typedef LaneAssignedFrontObjects_States rt_Array_LaneAssignedFrontObjects_States_32[32];
typedef LaneMarker_LaneEvent rt_Array_LaneMarker_LaneEvent_4[4];
typedef RearObject_EgoStates rt_Array_RearObject_EgoStates_10[10];
typedef RearObject_Properties rt_Array_RearObject_Properties_10[10];
typedef RearObject_RoadStates rt_Array_RearObject_RoadStates_10[10];
typedef void* Rte_Instance;

#pragma pack(pop)

#endif