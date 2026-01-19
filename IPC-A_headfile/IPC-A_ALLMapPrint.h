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

#define Reset_A1 0x300C
#define edr_info 0x3012
#define Logistics_Data 0x3007
#define CS_Trigger_A1 0x3002
#define SysState 0x3006
#define Parameter 0x3005
#define SW_fault 0x3003
#define InternalDID 0x3011
#define SOCStatusMon 0x3010
#define CS_Trigger_C1 0x3001
#define TimeSync 0x300E
#define PFM 0x300D
#define CameraBlockage 0x300F
#define HW_fault 0x3004
#define DoIP_response 0x3008
#define DoIP_req 0x3009
#define Cyc_monitor 0x300A
#define Reset_C1 0x300B


#ifdef IPC_MSG_DATA_SIZE_MAX_H
#include"wrapper/ipc_msg.h"
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_IPCC.h"
#endif

#ifdef IPC_A_H
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_IPC.h"
#endif




#ifdef RESETA1_H
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,bool*>;
#include "Adapter_ResetA1.h"
#endif


#ifdef LOGISTICS_DATA_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_Logistics_Data.h"
#endif

#ifdef EDR_INFO_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_EDR_Info.h"
#endif



void print_memory(const void* ptr, size_t size);
void asyncInputThreadTTY() ;
// #ifndef IPC_A_H
void printVariableVariant(const std::string& name, VariableVariant var) ;
void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);
// #endif

#endif
