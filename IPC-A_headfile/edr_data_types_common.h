
#pragma once
#include <cstdint>

#pragma pack(push, 4)

enum class EdrTriggerBit : uint8_t {
    None                     = 0,   // for peroid event data send (No event triggered)
    Bit1_LCC_Activated       = 1,
    Bit2_LCC_Deactivated     = 2,
    Bit3_HandsOff_Warn       = 3,
    Bit4_HandsOff_Cancel     = 4,
    Bit5_CollisionRisk       = 5,
    Bit6_Collision_Locked    = 6,
    Bit7_Collision_NonLocked = 7,
};

struct Logistics_data {
    uint8_t HW_Version[10];
    uint8_t SW_Version[10];
    uint8_t ECU_SerialNumber[30];
    uint8_t VIN_Code[17];
};

struct TimeStamp_Info {
    uint16_t year{0};
    uint8_t  month{0};   // 1~12
    uint8_t  day{0};     // 1~31
    uint8_t  hour{0};    // 0~23
    uint8_t  minute{0};  // 0~59
    uint8_t  second{0};  // 0~59
    uint16_t msec{0};    // 0~999
};

typedef struct {
    float ACU_LateralAcce;
    float ACU_LongitAcce;
    float ACU_YawRate;
} ACU_233;

typedef struct {
    uint8_t ACU_CrashOutputSts;
    uint8_t ACU_FLSeatBeltRSt;
    uint8_t ACU_FRSeatBeltRSt;
} ACU_2A3;

typedef struct {
    uint8_t IBC_AEBdecAvailable;
    uint8_t IBC_AEB_active;
    uint8_t IBC_ABP_active;
    uint8_t IBC_ABPAviliable;
} IBC_185;

typedef struct {
    uint8_t IBC_VDCFailed;
    uint8_t IBC_VDCActive;
    uint8_t IBC_EBDFailed;
    uint8_t IBC_Vehiclestandstill;
    uint8_t IBC_ESCFunctionStatus;
    uint8_t IBC_TCSFailed;
    uint8_t IBC_TCSActive;
    uint8_t IBC_ABSFailed;
    uint8_t IBC_VehicleSpeedVD;
    float IBC_VehicleSpeed;
} IBC_182;

typedef struct {
    uint8_t IBC_RRWheelDirection;
    uint8_t IBC_RLWheelDirection;
    uint8_t IBC_FRWheelDirection;
    uint8_t IBC_FLWheelDirection;
} IBC_184;

typedef struct {
    uint8_t IBC_sOutputRodDriverPer;
} IBC_183;

typedef struct {
    uint8_t VCU_GearLevelPosSts;
    float VCU_ThrottlePosition;
} HPCVCU_B6;

typedef struct {
    uint8_t VCU_VehicleSts;
} HPCVCU_B7;

typedef struct {
    bool BCM_BackDoorSts;
    bool BCM_FLDoorSts;
    uint8_t BCM_FLOC_PPD_Status;
    bool BCM_FRDoorSts;
    uint8_t BCM_FROC_PPD_Status;
    bool BCM_FrontCoverSts;
    bool BCM_KeyPosition_ON3;
    bool BCM_ON2CRelaySts;
    bool BCM_RLDoorSts;
    bool BCM_RRDoorSts;
} HPCBCM_290;

typedef struct {
    float       EPS_SteeringTorque;
    uint16_t    EPS_SteerWheelRotSpd;
    float       EPS_SteerWheelAngle;
} EPS_18D;

typedef struct {
    uint8_t LMC_LSFCActiveSt;
} LMC_25A;

// edr data from sheet F.1
struct EdrData_VehAdasBasicInfo {
    Logistics_data  logistics_snapshot{};
    uint8_t         event_code{0};
    TimeStamp_Info  timestamp{};
    uint32_t        ICU_TotalOdometerkm{0};
};

// edr data from sheet F.2
struct EdrData_VehStatusDynamicInfo {
    ACU_233     ACU_233_data;
    ACU_2A3     ACU_2A3_data;
    IBC_185     IBC_185_data;
    IBC_182     IBC_182_data;
    IBC_184     IBC_184_data;
    IBC_183     IBC_183_data;
    HPCVCU_B6   HPCVCU_B6_data;
    HPCVCU_B7   HPCVCU_B7_data;
    HPCBCM_290  HPCBCM_290_data;
    EPS_18D     EPS_18D_data;
    LMC_25A     LMC_25A_data;
};

// edr data from sheet F.3
struct EdrData_AdasOperationInfo_DriverInfo {
    // 都来自于 ehr_info
    uint8_t  FSC_LCC_EscapeLevel;
    uint8_t  FSC_emergencyLightReq;
    uint16_t HPCADAS_LKA_SteeringWheelAngle;
    uint16_t ADDC_ACCTargetTrq;
    uint16_t ADDC_ACCTargetTrqBrk;
    uint8_t  HPCADAS_ACCMode;
};

// edr data from sheet F.4
struct EdrData_Driving_EnvInfo {
    uint8_t	DRT_ID;
    uint8_t	DRT_ObjectClass;
    float	longPos;
    float	latPos;
    float	longSpeed;
    float	latSpeed;
    float	longAccel;
    float	latAccel;
    float	DRT_TTC;
};

struct EDR_Data_Period {
    EdrData_VehAdasBasicInfo                basicInfo;          // F.1 的数据
    EdrData_VehStatusDynamicInfo            veh_statusDynInfo;  // F.2 的数据
    EdrData_AdasOperationInfo_DriverInfo    adas_opInfo;        // F.3 的数据
    EdrData_Driving_EnvInfo                 drv_envInfo;        // F.4 的数据
};

#pragma pack(pop)