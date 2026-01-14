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

#ifdef IPC_MSG_DATA_SIZE_MAX_H
#include"wrapper/ipc_msg.h"
using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_IPCC.h"
#endif





#ifdef LOGISTICS_DATA_H

using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
#include "Adapter_Logistics_Data.h"
#endif




void print_memory(const void* ptr, size_t size);
void printVariableVariant(const std::string& name, VariableVariant var) ;
void asyncInputThreadTTY() ;
void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);


#endif
