#ifndef MET_SOC_ALL_
#define MET_SOC_ALL_
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




#ifdef MET_SOC_SWP1_PARAM_H

#include"MET_SOC_SWP1.h"
#endif



#endif




using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;
void print_memory(const void* ptr, size_t size);
void printVariableVariant(const std::string& name, VariableVariant var) ;
void asyncInputThreadTTY() ;
void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);

