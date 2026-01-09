#ifndef _IPCCONVERT_HPP_
#define _IPCCONVERT_HPP_

#include "mos-com/interface/publisher.hpp"
#include "mos-com/interface/subscriber.hpp"
#include "ipc_transport/ipc_msg.h"
#include "ipc_transport/ipc_transport.h"
#include <memory>
#include <thread>
#include <unordered_map>

extern std::atomic_bool stop;

namespace IPCC {

class IPCConvert {
public:
    static IPCConvert& getInstance()
    {
        static IPCConvert instance;
        return instance;
    }
    void run();

    void init();

private:
    IPCConvert()=default;
    ~IPCConvert()=default;

    // mos handle
    std::shared_ptr<MOS::communication::Subscriber> RoadPath_subscriber;
    void initSubscriberForRoadPath();
    std::shared_ptr<MOS::communication::Subscriber> LongCtrlObjInfo_subscriber;
    void initSubscriberForLongCtrlObjInfo();
    std::shared_ptr<MOS::communication::Subscriber> LaneMarker_subscriber;
    void initSubscriberForLaneMarker();
    std::shared_ptr<MOS::communication::Subscriber> EgoVehicleState_subscriber;
    void initSubscriberForEgoVehicleState();
    std::shared_ptr<MOS::communication::Subscriber> LDP_Obj_subscriber;
    void initSubscriberForLDP_Obj();
    std::shared_ptr<MOS::communication::Subscriber> ObjFrntCdnForSupp_subscriber;
    void initSubscriberForObjFrntCdnForSupp();
    std::shared_ptr<MOS::communication::Subscriber> LDPPath_subscriber;
    void initSubscriberForLDPPath();
    std::shared_ptr<MOS::communication::Subscriber> FlcRoadCover_subscriber;
    void initSubscriberForFlcRoadCover();
    std::shared_ptr<MOS::communication::Subscriber> RoadEdge_subscriber;
    void initSubscriberForRoadEdge();
    std::shared_ptr<MOS::communication::Publisher> VehParam_Tx_publisher;
    void initPublisherForVehParam_Tx();
    std::shared_ptr<MOS::communication::Subscriber> TrafficFlow_subscriber;
    void initSubscriberForTrafficFlow();
    std::shared_ptr<MOS::communication::Subscriber> FusedFrontObject_subscriber;
    void initSubscriberForFusedFrontObject();
    std::shared_ptr<MOS::communication::Subscriber> CrossingObject_subscriber;
    void initSubscriberForCrossingObject();
    std::shared_ptr<MOS::communication::Subscriber> RearObject_subscriber;
    void initSubscriberForRearObject();
    std::shared_ptr<MOS::communication::Publisher> VehBusIn_publisher;
    void initPublisherForVehBusIn();
    std::shared_ptr<MOS::communication::Subscriber> HBC2CANBus_T_subscriber;
    void initSubscriberForHBC2CANBus_T();
    std::shared_ptr<MOS::communication::Publisher> IDT_FuncTgtVisnID_publisher;
    void initPublisherForIDT_FuncTgtVisnID();

    // ipc handle
    static void static_handle_ipc_VehParam_Tx(const IPC_MSG *msg);
    void handle_ipc_VehParam_Tx(const IPC_MSG *msg);
    static void static_handle_ipc_VehBusIn(const IPC_MSG *msg);
    void handle_ipc_VehBusIn(const IPC_MSG *msg);
    static void static_handle_ipc_IDT_FuncTgtVisnID(const IPC_MSG *msg);
    void handle_ipc_IDT_FuncTgtVisnID(const IPC_MSG *msg);
};

} // end namespace IPCC

#endif // IPCCONVERT_HPP