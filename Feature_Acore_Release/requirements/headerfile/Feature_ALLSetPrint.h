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

#ifdef LANEMARKER_H
void setIntialValue_LaneMarker(LaneMarker& LaneMarker_);
#endif


#ifdef LDP_OBJ_H
void setIntialValue_LDP_Obj(LDP_Obj& LDP_Obj_);
#endif

#ifdef LDPPATH_H
void setIntialValue_LDPPath(LDPPath& LDPPath_);
#endif

#ifdef LONGCTRLOBJINFO_H
void setIntialValue_LongCtrlObjInfo(LongCtrlObjInfo& LongCtrlObjInfo_);
#endif

#ifdef REAROBJECT_H
void setIntialValue_RearObject(RearObject& RearObject_);
#endif

#ifdef ROADEDGE_H
void setIntialValue_RoadEdge(RoadEdge& RoadEdge_);
#endif

#ifdef ROADPATH_H
void setIntialValue_RoadPath(RoadPath& RoadPath_);
#endif

#ifdef TRAFFICFLOW_H
void setIntialValue_TrafficFlow(TrafficFlow& TrafficFlow_);
#endif

#ifdef VEHPARAM_TX_H
using VariableVariant = std::variant<uint8*, uint16* ,uint32*,float32*,sint8*,sint16*,sint32*>;
#include "AdApter_VehParamTx_sub.h"

void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input);
void printVariableVariant(const std::string& name, VariableVariant var) ;
void asyncInputThreadTTY() ;
#endif




void print_memory(const void* ptr, size_t size);
#endif // FEATURE_ALL