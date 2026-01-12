#ifndef FEATURE_ALL
#define FEATURE_ALL
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
#include "IPC_Pub.h"




#ifdef CAH_ACTIVESAFETY_H
void setIntialValue_CAH_ActiveSafety(CAH_ActiveSafety& CAH_ActiveSafety_);
#endif


#ifdef CROSSINGOBJECT_H
void setIntialValue_CrossingObject(CrossingObject& CrossingObject_);
#endif


#ifdef EGOVEHICLESTATE_H
void setIntialValue_EgoVehicleState(EgoVehicleState& EgoVehicleState_);
#endif

#ifdef FLCROADCOVER_H
void setIntialValue_FlcRoadCover(FlcRoadCover& FlcRoadCover_);    
#endif

#ifdef FUSEDFRONTOBJECT_H
void setIntialValue_FusedFrontObject(FusedFrontObject& FusedFrontObject_);
#endif



#endif // FEATURE_ALL
void print_memory(const void* ptr, size_t size);
// void printVariableVariant(const std::string& name, VariableVariant var) ;
void asyncInputThreadTTY() ;
// void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);

