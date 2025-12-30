
#ifndef MET_SOC_ACTIVESAFETY_OUT_H_
#define MET_SOC_ACTIVESAFETY_OUT_H_

/*topic name :ACTIVESAFETY/activesafety*/
/*****************************************************************************************************************************
*****************************************************************************************************************************/
/*  <VERSION>       <DATE>        <AUTHOR>                 <REVISION LOG>                                                   */
/*  version:  1     2025-9-2      YY                       Initial version                                                  */       
                         
/****************************************************************************************************************************/
#define MET_ACTIVESAFETY_OUT_HEADFILE_VERSION   (1u)
#include <stdint.h>

#pragma pack(4)

typedef struct
{
    float FCW_ReactionTime;
    float FCW_TTCThres;
    float FCW_TTC;
    uint16_t FCW_SCPComponent;
    uint16_t FCW_GOComponent;
    uint8_t FCW_RLC;
    uint8_t FCW_State;
    uint8_t FCW_StateLastMoment;
    uint8_t FCW_AlrtPr;
    uint8_t FCW_ChimeSuppress;
    uint8_t FCW_BrakeSuppress;
    uint8_t FCW_StatComponent;
    uint8_t FCW_MovComponent;
} FCW_DEV_SOC_T;

typedef struct
{
    float FCW_DMRTRange;
    float FCW_DMRTRangeRate;
    float FCW_DMRTXolc;
    float FCW_DMRTXohp;
    float FCW_DMRTLatVel;
    float FCW_DMRTAngle;
    float FCW_DMRTRangeAccel;
    float FCW_DMRTTTC;
    float FCW_DMRTTTCThres;
    uint8_t FCW_DMRTMatchConf;
    uint8_t FCW_DMRTID;
    uint8_t FCW_DMRTClass;
    uint8_t FCW_DMRTGOEConfirm;
} FCW_DMRT_SOC_T;

typedef struct
{
    float FCW_DSRTRange;
    float FCW_DSRTRangeRate;
    float FCW_DSRTXolc;
    float FCW_DSRTXohp;
    float FCW_DSRTLatVel;
    float FCW_DSRTAngle;
    float FCW_DSRTRangeAccel;
    float FCW_DSRTTTC;
    float FCW_DSRTTTCThres;
    uint8_t FCW_DSRTMatchConf;
    uint8_t FCW_DSRTID;
    uint8_t FCW_DSRTClass;
    uint8_t FCW_DSRTGOEConfirm;
} FCW_DSRT_SOC_T;

typedef struct
{
    float FCW_GOLongPos;
    float FCW_GOLatPos;
    float FCW_GOXolcRef;
    float FCW_GOTTC;
    float FCW_GOTTCThres;
    uint8_t FCW_GOId;
    uint8_t FCW_GOClass;
    uint8_t ByteAlignedFill[2];
} FCW_GO_SOC_T;

typedef struct
{
    float FCW_TAPVRUHeading;
    float FCW_TAPVRULatPosCoG;
    float FCW_TAPVRULongPosCoG;
    float FCW_TAPVRULatVel;
    float FCW_TAPVRULongVel;
    float FCW_TAPVRULatAcc;
    float FCW_TAPVRULongAcc;
    float FCW_TAPVRUPathDisNear;
    float FCW_TAPVRUXolcRef1;
    float FCW_TAPVRUXolcRef2;
    float FCW_TAPVRUTTC;
    float FCW_TAPVRUTTCThres;
    uint16_t FCW_TAPVRUId;
    uint8_t ByteAlignedFill[2];
} FCW_TAP_VRU_SOC_T;

typedef struct
{
    float FCW_TAPVehHeading;
    float FCW_TAPVehLatPosCoG;
    float FCW_TAPVehLongPosCoG;
    float FCW_TAPVehLatVel;
    float FCW_TAPVehLongVel;
    float FCW_TAPVehLatAcc;
    float FCW_TAPVehLongAcc;
    float FCW_TAPVehPathDisNear;
    float FCW_TAPVehXolcRef1;
    float FCW_TAPVehXolcRef2;
    float FCW_TAPVehTTC;
    float FCW_TAPVehTTCThres;
    uint16_t FCW_TAPVehId;
    uint8_t ByteAlignedFill[2];
} FCW_TAP_Veh_SOC_T;

typedef struct
{
    uint8_t FCW_MajorVer;
    uint8_t FCW_MinorVer;
    uint8_t ByteAlignedFill[2];
} FCW_Version_SOC_T;

typedef struct
{
    float LgSf_DRTTTC;
    float LgSf_DRTRange;
    float LgSf_DRTRangeRate;
    float LgSf_DRTRangeAccel;
    float LgSf_DRTLatPosn;
    float LgSf_DRTLatVel;
    float LgSf_DRTXolc;
    float LgSf_VRULatEst;
    float LgSf_DRTVisLatAcc;
    float LgSf_DRTWidth;
    float LgSf_PrefillRangeThres;
    float LgSf_LowBrkRangeThres;
    float LgSf_HiBrkRangeThres;
    float LgSf_IBARangeThres;
    float LgSf_PrefillTTCThres;
    float LgSf_LowBrkTTCThres;
    float LgSf_HiBrkTTCThres;
    float LgSf_IBATTCThres;
    float LgSf_HostVehSpd_mps;
    float LgSf_HostLongAccel_mpss;
    float LgSf_BrkPdlPosRate;
    float LgSf_BrkPdlPrsRate;
    float LgSf_GasPedalPos;
    float LgSf_SingleRes01;
    float LgSf_SingleRes02;
    float LgSf_SingleRes03;
    uint32_t LgSf_DebugInfo1;
    uint32_t LgSf_DebugInfo2;
    uint32_t LgSf_VehEachTrkCntr;
    uint32_t LgSf_HiBrkCntr;
    uint32_t LgSf_LowBrkCntr;
    uint32_t LgSf_PrefillCntr;
    uint32_t LgSf_FLBACntr;
    uint32_t LgSf_MinStopCntr;
    uint32_t LgSf_IBACntr;
    uint32_t LgSf_DIDConfig1R;
    uint32_t LgSf_Uint32Res01;
    uint32_t LgSf_Uint32Res02;
    uint32_t LgSf_Uint32Res03;
    uint16_t LgSf_PrefillFlg;
    uint16_t LgSf_LowBrkFlg;
    uint16_t LgSf_HiBrkFlg;
    uint16_t LgSf_FullBrkFlg;
    uint16_t LgSf_IBAFlg;
    uint16_t LgSf_FLBAFlg;
    uint8_t LgSf_AEBRlsCmd;
    uint8_t LgSf_AEBBrkLvlReq;
    uint8_t LgSf_AEBSourceType;
    uint8_t LgSf_JAAEBActiveFlag;
    uint8_t LgSf_DRTID;
    uint8_t LgSf_DRTMovement;
    uint8_t LgSf_DRTMoveable;
    uint8_t LgSf_DRTMatchConf;
    uint8_t LgSf_DRTFusSource;
    uint8_t LgSf_DRTObjectClass;
    uint8_t LgSf_UseVRU;
    uint8_t LgSf_DRTGOEConfirm;
    uint8_t LgSf_DRTBeyondGuardrail;
    uint8_t LgSf_DrvrEng;
    uint8_t LgSf_GasPedalOvrd;
    uint8_t LgSf_SteeringOvrd;
    uint8_t LgSf_GasPedalRateCntr;
    uint8_t LgSf_BrkPosAct;
    uint8_t LgSf_BrkPressureAct;
    uint8_t LgSf_VehObjPrcCntr;
    uint8_t LgSf_VehToiDscrmntnCntr;
    uint8_t LgSf_VehBrkDscrmntnCntr;
    uint8_t LgSf_RLC;
    uint8_t LgSf_Uint8Res01;
    uint8_t LgSf_Uint8Res02;
    uint8_t LgSf_Uint8Res03;
    uint8_t ByteAlignedFill[2];
} LgSafe_DEV_SOC_T;

typedef struct
{
    uint8_t LgSf_MajorVer;
    uint8_t LgSf_MinorVer;
    uint8_t ByteAlignedFill[2];
} LgSafe_Version_SOC_T;

typedef struct
{
    uint32_t AEB_FunConfig;
    uint8_t AEB_State;
    uint8_t AEB_PrefillReq;
    uint8_t AEB_Level;
    uint8_t AEB_Type;
    uint8_t AEB_RLC;
    uint8_t AEB_MajorVer;
    uint8_t AEB_MinorVer;
    uint8_t AEB_Param_MajorVer;
    uint8_t AEB_Param_MinorVer;
    uint8_t AEB_Cal_MajorVer;
    uint8_t AEB_Cal_MinorVer;
    uint8_t reserved;
} SOC_AEB_Flag_SOC_T;

typedef struct
{
    float DRT_TTC;
    uint8_t DRT_ID;
    uint8_t DRT_ObjectClass;
    uint8_t reserved[2];
} SOC_DRT_SOC_T;

typedef struct
{
    uint8_t FCW_State;
    uint8_t FCW_WarningState;
    uint8_t FCW_LatentWarning;
    uint8_t FCW_AcuteWarningState;
    uint8_t FCW_RLC;
    uint8_t FCW_SWC_MajorVer;
    uint8_t FCW_SWC_MinorVer;
    uint8_t FCW_Param_MajorVer;
    uint8_t FCW_Param_MinorVer;
    uint8_t FCW_Cal_MajorVer;
    uint8_t FCW_Cal_MinorVer;
    uint8_t reserved;
} SOC_FCW_Flag_SOC_T;

typedef struct
{
    FCW_DEV_SOC_T FCW_DEV;
    FCW_DSRT_SOC_T FCW_DSRT;
    FCW_DMRT_SOC_T FCW_DMRT;
    FCW_TAP_Veh_SOC_T FCW_TAPVeh;
    FCW_TAP_VRU_SOC_T FCW_TAPVRU;
    FCW_GO_SOC_T FCW_GO;
    FCW_Version_SOC_T FCW_Version;
    float FCW_LatentWarning;
    uint8_t FCW_AcuteTargetId;
    uint8_t FCW_LatentWarningID;
    uint8_t FCW_WarningState;
    uint8_t FCW_TargetId;
    uint8_t FCW_AcuteWarningState;
    uint8_t ByteAlignedFill[3];
} FCW_Manager_SOC_T; // MCU2SOC

typedef struct
{
    LgSafe_DEV_SOC_T LgSafe_DEV;
    LgSafe_Version_SOC_T LgSafe_Version;
    float LgSf_AEBDecelReq;
    uint8_t LgSf_AEBType;
    uint8_t LgSf_PrefillReq;
    uint8_t LgSf_AEBDecelReqFlag;
    uint8_t LgSf_AEBBrkReqFlag;
    uint8_t LgSf_AEBAlrtReq;
    uint8_t LgSf_AEBVRUAlrtReq;
    uint8_t LgSf_FLBAReq;
    uint8_t LgSf_MinStopReq;
} LgSafe_SOC_T;

typedef struct
{
    SOC_FCW_Flag_SOC_T SOC_FCW_Flag;
    SOC_AEB_Flag_SOC_T SOC_AEB_Flag;
    SOC_DRT_SOC_T SOC_DRT;
} SOC_ActiveSafety_SOC_T;

typedef struct
{
    uint32_t frameID;
    uint64_t timestamp;
    SOC_ActiveSafety_SOC_T SOC_ActiveSafety_SOC;
    FCW_Manager_SOC_T FCW_Manager_SOC;
    LgSafe_SOC_T LgSafe_SOC;
} SOC_ActiveSafety_UDP_T_T;

#pragma pack()

#endif