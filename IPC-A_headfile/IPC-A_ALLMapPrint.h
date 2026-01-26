#ifndef IPCC_ALL_
#define IPCC_ALL_
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <variant>
#include <iomanip>
#include <iostream>
#include <string>
#include <map>
#include <fcntl.h>
#include <pthread.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <atomic>
#include <queue>

#define def_Reset_A1 0x300C
#define def_edr_info 0x3012
#define def_Logistics_Data 0x3007
#define def_CS_Trigger_A1 0x3002
#define def_sysstate 0x3006
#define def_Parameter 0x3005
#define def_SW_fault 0x3003
#define def_InternalDID 0x3011
#define def_SOCStatusMon 0x3010
#define def_CS_Trigger_C1 0x3001
#define def_TimeSync 0x300E
#define def_PFM 0x300D
#define def_CameraBlockage 0x300F
#define def_HW_fault 0x3004
#define def_DoIP_response 0x3008
#define def_DoIP_req 0x3009
#define def_Cyc_monitor 0x300A
#define def_Reset_C1 0x300B
#include "Adapt_data_SelfDefine.h"


#ifdef IPC_MSG_DATA_SIZE_MAX_H
#include"wrapper/ipc_msg.h"
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_IPCC.h"
#endif

#ifdef IPC_A_H
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_IPC.h"
#endif




#ifdef RCORE_RESET_REQUEST_H
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_ResetA1.h"
#endif


#ifdef LOGISTICS_DATA_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_Logistics_Data.h"
#endif

#ifdef EDR_INFO_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_EDR_Info.h"
#endif

#ifdef SYSSTATE_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_SysState.h"
#endif

#if defined(PERCEPTION_VEHICLE_PARAMETERS_H) || defined(INTRINSIC_CALIBRATION_PARAMETERS_H)
//sint16 与 int16_t 等同，variant不能出现重复的类型
using VariableVariant = std::variant< uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_Parameter.h"
#endif


#if defined(MAGNAPARAMCSRES_H) || defined(MAGNACAMEXTPARAM_H) || defined(MAGNACAMINTPARAM_H) || defined(MAGNATACCALIBINPUT_H)

#include "MET_SOC_MAGNA_PARAM_CS.h"
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*,MagnaCalibResultCode*,MagnaStatusCode*>;
#include "Adapter_CS_Trigger_A1.h"

#endif



#include "ipc_msg.h"
extern IPC_MSG_DATA_SIZE_MAX ipc_msg_;
void print_memory(const void* ptr, size_t size);
void asyncInputThreadTTY() ;
// #ifndef IPC_A_H
void printVariableVariant(const std::string& name, VariableVariant var) ;
void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);
// #endif

#endif
