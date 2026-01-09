#include "IPCConvert.hpp"
#include "IPC_Pub.h"
#include "CCITT-FALSE-16.hpp"

#include <iostream>
#include <signal.h>
#include <unordered_map>
#include <functional>

using namespace IPCC;


// Forward declaration for RoadPath
void handle_RoadPath(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for LongCtrlObjInfo
void handle_LongCtrlObjInfo(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for LaneMarker
void handle_LaneMarker(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for EgoVehicleState
void handle_EgoVehicleState(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for LDP_Obj
void handle_LDP_Obj(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for ObjFrntCdnForSupp
void handle_ObjFrntCdnForSupp(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for LDPPath
void handle_LDPPath(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for FlcRoadCover
void handle_FlcRoadCover(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for RoadEdge
void handle_RoadEdge(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for TrafficFlow
void handle_TrafficFlow(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for FusedFrontObject
void handle_FusedFrontObject(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for CrossingObject
void handle_CrossingObject(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for RearObject
void handle_RearObject(const std::shared_ptr<MOS::message::Message>& msg);

// Forward declaration for HBC2CANBus_T
void handle_HBC2CANBus_T(const std::shared_ptr<MOS::message::Message>& msg);


// IPC message handlers
// static Handle incoming IPC message for VehParam_Tx (MessageID: 4107)
void IPCConvert::static_handle_ipc_VehParam_Tx(const IPC_MSG *msg) {
    IPCC::IPCConvert::getInstance().handle_ipc_VehParam_Tx(msg);
}

// Handle incoming IPC message for VehParam_Tx (MessageID: 4107)
void IPCConvert::handle_ipc_VehParam_Tx(const IPC_MSG *msg) {

    if (msg->version != 3)
    {
        std::cout << "VehParam_Tx version err" <<std::endl;
    }
    uint16_t crc_src{0};
    memcpy(&crc_src, msg->data,4);
    uint16_t crc_calc{0};
    crc_calc = ccitt_false_16_calculate(msg->data+4, msg->data_size-4, 0, true);

    if (crc_calc == crc_src) {
        printf("Received IPC message for VehParam_Tx - ID: 4107, Size: %d\n", msg->data_size);

        // Convert IPC message to MOS message and publish to topic: IPCC/VehParam_Tx
        // Implementation depends on your data conversion logic
        auto data_ref = std::make_shared<MOS::message::DataRef>(msg->data, msg->data_size);
        auto mos_msg = std::make_shared<MOS::message::Message>();
        mos_msg->SetDataRef(data_ref);
        mos_msg->SetGenTimestamp(MOS::TimeUtils::NowNsec());
        
        // Publish to topic (publisher needs to be created in constructor)
        VehParam_Tx_publisher->Pub(mos_msg);
    } else {
        printf("Received IPC message for VehParam_Tx - ID: 4107 with CRC err. src_crc:%d, calc_crc:%d", crc_src, crc_calc);
    }
}

// static Handle incoming IPC message for VehBusIn (MessageID: 4112)
void IPCConvert::static_handle_ipc_VehBusIn(const IPC_MSG *msg) {
    IPCC::IPCConvert::getInstance().handle_ipc_VehBusIn(msg);
}

// Handle incoming IPC message for VehBusIn (MessageID: 4112)
void IPCConvert::handle_ipc_VehBusIn(const IPC_MSG *msg) {

    if (msg->version != 3)
    {
        std::cout << "VehBusIn version err" <<std::endl;
    }
    uint16_t crc_src{0};
    memcpy(&crc_src, msg->data,4);
    uint16_t crc_calc{0};
    crc_calc = ccitt_false_16_calculate(msg->data+4, msg->data_size-4, 0, true);

    if (crc_calc == crc_src) {
        printf("Received IPC message for VehBusIn - ID: 4112, Size: %d\n", msg->data_size);

        // Convert IPC message to MOS message and publish to topic: IPCC/VehBusIn
        // Implementation depends on your data conversion logic
        auto data_ref = std::make_shared<MOS::message::DataRef>(msg->data, msg->data_size);
        auto mos_msg = std::make_shared<MOS::message::Message>();
        mos_msg->SetDataRef(data_ref);
        mos_msg->SetGenTimestamp(MOS::TimeUtils::NowNsec());
        
        // Publish to topic (publisher needs to be created in constructor)
        VehBusIn_publisher->Pub(mos_msg);
    } else {
        printf("Received IPC message for VehBusIn - ID: 4112 with CRC err. src_crc:%d, calc_crc:%d", crc_src, crc_calc);
    }
}

// static Handle incoming IPC message for IDT_FuncTgtVisnID (MessageID: 4118)
void IPCConvert::static_handle_ipc_IDT_FuncTgtVisnID(const IPC_MSG *msg) {
    IPCC::IPCConvert::getInstance().handle_ipc_IDT_FuncTgtVisnID(msg);
}

// Handle incoming IPC message for IDT_FuncTgtVisnID (MessageID: 4118)
void IPCConvert::handle_ipc_IDT_FuncTgtVisnID(const IPC_MSG *msg) {

    if (msg->version != 1)
    {
        std::cout << "IDT_FuncTgtVisnID version err" <<std::endl;
    }
    uint16_t crc_src{0};
    memcpy(&crc_src, msg->data,4);
    uint16_t crc_calc{0};
    crc_calc = ccitt_false_16_calculate(msg->data+4, msg->data_size-4, 0, true);

    if (crc_calc == crc_src) {
        printf("Received IPC message for IDT_FuncTgtVisnID - ID: 4118, Size: %d\n", msg->data_size);

        // Convert IPC message to MOS message and publish to topic: IPCC/FuncTgtVisnID
        // Implementation depends on your data conversion logic
        auto data_ref = std::make_shared<MOS::message::DataRef>(msg->data, msg->data_size);
        auto mos_msg = std::make_shared<MOS::message::Message>();
        mos_msg->SetDataRef(data_ref);
        mos_msg->SetGenTimestamp(MOS::TimeUtils::NowNsec());
        
        // Publish to topic (publisher needs to be created in constructor)
        IDT_FuncTgtVisnID_publisher->Pub(mos_msg);
    } else {
        printf("Received IPC message for IDT_FuncTgtVisnID - ID: 4118 with CRC err. src_crc:%d, calc_crc:%d", crc_src, crc_calc);
    }
}


void IPCConvert::init() {
    // Initialize publishers for SOC-directed messages
    initPublisherForVehParam_Tx();
    initPublisherForVehBusIn();
    initPublisherForIDT_FuncTgtVisnID();
    
    // Register IPC message handlers
    register_recv_ipc_msg(0xC1, 4107, static_handle_ipc_VehParam_Tx);
    register_recv_ipc_msg(0xC1, 4112, static_handle_ipc_VehBusIn);
    register_recv_ipc_msg(0xC1, 4118, static_handle_ipc_IDT_FuncTgtVisnID);
    
    // Initialize subscribers for MCU-directed messages
    initSubscriberForRoadPath();
    initSubscriberForLongCtrlObjInfo();
    initSubscriberForLaneMarker();
    initSubscriberForEgoVehicleState();
    initSubscriberForLDP_Obj();
    initSubscriberForObjFrntCdnForSupp();
    initSubscriberForLDPPath();
    initSubscriberForFlcRoadCover();
    initSubscriberForRoadEdge();
    initSubscriberForTrafficFlow();
    initSubscriberForFusedFrontObject();
    initSubscriberForCrossingObject();
    initSubscriberForRearObject();
    initSubscriberForHBC2CANBus_T();
}

void IPCConvert::run() {
    std::cout << "IPCConvert started..." << std::endl;
    
    while (!stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "IPCConvert shutting down..." << std::endl;
}
void IPCConvert::initPublisherForVehParam_Tx() {
    MOS::communication::ProtocolInfo pub_proto_info;
    pub_proto_info.protocol_type = MOS::communication::kProtocolHybrid;
    pub_proto_info.shm_info.block_count = 10;
    pub_proto_info.shm_info.fast_mode = false;
    pub_proto_info.shm_info.block_size = sizeof(VehParam_Tx);
    
    VehParam_Tx_publisher = MOS::communication::Publisher::New(
        0,
        "IPCC/VehParam_Tx", 
        pub_proto_info
    );

    if (!VehParam_Tx_publisher) 
    {
        std::cout<<"VehParam_Tx_publisher err"<< std::endl;
    }
    
    std::cout << "Initialized publisher for VehParam_Tx on topic: IPCC/VehParam_Tx" << std::endl;
}
void IPCConvert::initPublisherForVehBusIn() {
    MOS::communication::ProtocolInfo pub_proto_info;
    pub_proto_info.protocol_type = MOS::communication::kProtocolHybrid;
    pub_proto_info.shm_info.block_count = 10;
    pub_proto_info.shm_info.fast_mode = false;
    pub_proto_info.shm_info.block_size = sizeof(VehBusIn);
    
    VehBusIn_publisher = MOS::communication::Publisher::New(
        0,
        "IPCC/VehBusIn", 
        pub_proto_info
    );

    if (!VehBusIn_publisher) 
    {
        std::cout<<"VehBusIn_publisher err"<< std::endl;
    }
    
    std::cout << "Initialized publisher for VehBusIn on topic: IPCC/VehBusIn" << std::endl;
}
void IPCConvert::initPublisherForIDT_FuncTgtVisnID() {
    MOS::communication::ProtocolInfo pub_proto_info;
    pub_proto_info.protocol_type = MOS::communication::kProtocolHybrid;
    pub_proto_info.shm_info.block_count = 10;
    pub_proto_info.shm_info.fast_mode = false;
    pub_proto_info.shm_info.block_size = sizeof(IDT_FuncTgtVisnID);
    
    IDT_FuncTgtVisnID_publisher = MOS::communication::Publisher::New(
        0,
        "IPCC/FuncTgtVisnID", 
        pub_proto_info
    );

    if (!IDT_FuncTgtVisnID_publisher) 
    {
        std::cout<<"IDT_FuncTgtVisnID_publisher err"<< std::endl;
    }
    
    std::cout << "Initialized publisher for IDT_FuncTgtVisnID on topic: IPCC/FuncTgtVisnID" << std::endl;
}
void IPCConvert::initSubscriberForRoadPath() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_RoadPath(msg);
    };
    
    RoadPath_subscriber = MOS::communication::Subscriber::New(
        0, 
        "PPF/RoadPath", 
        sub_proto_info, 
        sub_callback
    );

    if (!RoadPath_subscriber) 
    {
        std::cout<<"RoadPath_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for RoadPath on topic: PPF/RoadPath" << std::endl;
}

void handle_RoadPath(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_RoadPath called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4097;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent RoadPath via IPC - ID: 4097, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForLongCtrlObjInfo() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_LongCtrlObjInfo(msg);
    };
    
    LongCtrlObjInfo_subscriber = MOS::communication::Subscriber::New(
        0, 
        "TOS/LongCtrlObjInfo", 
        sub_proto_info, 
        sub_callback
    );

    if (!LongCtrlObjInfo_subscriber) 
    {
        std::cout<<"LongCtrlObjInfo_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for LongCtrlObjInfo on topic: TOS/LongCtrlObjInfo" << std::endl;
}

void handle_LongCtrlObjInfo(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_LongCtrlObjInfo called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4098;
            ipc_msg.header.version = 2;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent LongCtrlObjInfo via IPC - ID: 4098, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForLaneMarker() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_LaneMarker(msg);
    };
    
    LaneMarker_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/LaneMarker", 
        sub_proto_info, 
        sub_callback
    );

    if (!LaneMarker_subscriber) 
    {
        std::cout<<"LaneMarker_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for LaneMarker on topic: SF/LaneMarker" << std::endl;
}

void handle_LaneMarker(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_LaneMarker called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4099;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent LaneMarker via IPC - ID: 4099, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForEgoVehicleState() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_EgoVehicleState(msg);
    };
    
    EgoVehicleState_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/EgoVehicleState", 
        sub_proto_info, 
        sub_callback
    );

    if (!EgoVehicleState_subscriber) 
    {
        std::cout<<"EgoVehicleState_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for EgoVehicleState on topic: SF/EgoVehicleState" << std::endl;
}

void handle_EgoVehicleState(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_EgoVehicleState called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4100;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent EgoVehicleState via IPC - ID: 4100, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForLDP_Obj() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_LDP_Obj(msg);
    };
    
    LDP_Obj_subscriber = MOS::communication::Subscriber::New(
        0, 
        "ThrtAssemt/LDP_Obj", 
        sub_proto_info, 
        sub_callback
    );

    if (!LDP_Obj_subscriber) 
    {
        std::cout<<"LDP_Obj_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for LDP_Obj on topic: ThrtAssemt/LDP_Obj" << std::endl;
}

void handle_LDP_Obj(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_LDP_Obj called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4102;
            ipc_msg.header.version = 2;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent LDP_Obj via IPC - ID: 4102, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForObjFrntCdnForSupp() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_ObjFrntCdnForSupp(msg);
    };
    
    ObjFrntCdnForSupp_subscriber = MOS::communication::Subscriber::New(
        0, 
        "ThrtAssemt/ObjFrntCdnForSupp", 
        sub_proto_info, 
        sub_callback
    );

    if (!ObjFrntCdnForSupp_subscriber) 
    {
        std::cout<<"ObjFrntCdnForSupp_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for ObjFrntCdnForSupp on topic: ThrtAssemt/ObjFrntCdnForSupp" << std::endl;
}

void handle_ObjFrntCdnForSupp(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_ObjFrntCdnForSupp called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4103;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent ObjFrntCdnForSupp via IPC - ID: 4103, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForLDPPath() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_LDPPath(msg);
    };
    
    LDPPath_subscriber = MOS::communication::Subscriber::New(
        0, 
        "ThrtAssemt/LDPPath", 
        sub_proto_info, 
        sub_callback
    );

    if (!LDPPath_subscriber) 
    {
        std::cout<<"LDPPath_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for LDPPath on topic: ThrtAssemt/LDPPath" << std::endl;
}

void handle_LDPPath(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_LDPPath called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4104;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent LDPPath via IPC - ID: 4104, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForFlcRoadCover() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_FlcRoadCover(msg);
    };
    
    FlcRoadCover_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/FlcRoadCover", 
        sub_proto_info, 
        sub_callback
    );

    if (!FlcRoadCover_subscriber) 
    {
        std::cout<<"FlcRoadCover_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for FlcRoadCover on topic: SF/FlcRoadCover" << std::endl;
}

void handle_FlcRoadCover(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_FlcRoadCover called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4105;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent FlcRoadCover via IPC - ID: 4105, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForRoadEdge() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_RoadEdge(msg);
    };
    
    RoadEdge_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/RoadEdge", 
        sub_proto_info, 
        sub_callback
    );

    if (!RoadEdge_subscriber) 
    {
        std::cout<<"RoadEdge_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for RoadEdge on topic: SF/RoadEdge" << std::endl;
}

void handle_RoadEdge(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_RoadEdge called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4106;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent RoadEdge via IPC - ID: 4106, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForTrafficFlow() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_TrafficFlow(msg);
    };
    
    TrafficFlow_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/TrafficFlow", 
        sub_proto_info, 
        sub_callback
    );

    if (!TrafficFlow_subscriber) 
    {
        std::cout<<"TrafficFlow_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for TrafficFlow on topic: SF/TrafficFlow" << std::endl;
}

void handle_TrafficFlow(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_TrafficFlow called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4108;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent TrafficFlow via IPC - ID: 4108, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForFusedFrontObject() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_FusedFrontObject(msg);
    };
    
    FusedFrontObject_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/FrontObject", 
        sub_proto_info, 
        sub_callback
    );

    if (!FusedFrontObject_subscriber) 
    {
        std::cout<<"FusedFrontObject_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for FusedFrontObject on topic: SF/FrontObject" << std::endl;
}

void handle_FusedFrontObject(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_FusedFrontObject called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4109;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent FusedFrontObject via IPC - ID: 4109, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForCrossingObject() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_CrossingObject(msg);
    };
    
    CrossingObject_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/CrossingObject", 
        sub_proto_info, 
        sub_callback
    );

    if (!CrossingObject_subscriber) 
    {
        std::cout<<"CrossingObject_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for CrossingObject on topic: SF/CrossingObject" << std::endl;
}

void handle_CrossingObject(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_CrossingObject called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4110;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent CrossingObject via IPC - ID: 4110, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForRearObject() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_RearObject(msg);
    };
    
    RearObject_subscriber = MOS::communication::Subscriber::New(
        0, 
        "SF/RearObject", 
        sub_proto_info, 
        sub_callback
    );

    if (!RearObject_subscriber) 
    {
        std::cout<<"RearObject_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for RearObject on topic: SF/RearObject" << std::endl;
}

void handle_RearObject(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_RearObject called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4111;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent RearObject via IPC - ID: 4111, Size: %d\n", data_size_vec[i]);
        }
    }
}
void IPCConvert::initSubscriberForHBC2CANBus_T() {
    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    
    auto sub_callback = [this](const std::shared_ptr<MOS::message::Message>& msg) {
        handle_HBC2CANBus_T(msg);
    };
    
    HBC2CANBus_T_subscriber = MOS::communication::Subscriber::New(
        0, 
        "HBC/HBCInfo2CAN", 
        sub_proto_info, 
        sub_callback
    );

    if (!HBC2CANBus_T_subscriber) 
    {
        std::cout<<"HBC2CANBus_T_subscriber err"<< std::endl;
    }
    
    
    std::cout << "Initialized subscriber for HBC2CANBus_T on topic: HBC/HBCInfo2CAN" << std::endl;
}

void handle_HBC2CANBus_T(const std::shared_ptr<MOS::message::Message>& msg) {
    std::cout << "handle_HBC2CANBus_T called, msg use_count: " 
                << msg.use_count() << std::endl;
    if (msg) {
        auto data_str_vec = msg->GetDataRef()->GetDataVec();
        auto data_size_vec = msg->GetDataRef()->GetDataSizeVec();

        // Convert to IPC message and send
        for (size_t i=0; i<data_size_vec.size(); ++i)
        {
            IPC_MSG_DATA_SIZE_MAX ipc_msg;
            ipc_msg.header.target = 0xC1; // MCU target
            ipc_msg.header.id = 4117;
            ipc_msg.header.version = 1;
            ipc_msg.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            ipc_msg.header.data_size = data_size_vec[i];
            
            // add CRC check
            uint16_t crc_res = ccitt_false_16_calculate(static_cast<uint8_t*>(data_str_vec[i]),data_size_vec[i],0, true);
            memcpy(ipc_msg.data, &crc_res, sizeof(crc_res));

            // Copy data (adjust based on actual data structure)
            memcpy(ipc_msg.data+sizeof(crc_res), data_str_vec[i], data_size_vec[i]);

            bool res = ipc_send_msg(&(ipc_msg.header));
            if (!res) 
            {
                std::cout<<"ipc_send_msg interface err"<< std::endl;
            }
            printf("Sent HBC2CANBus_T via IPC - ID: 4117, Size: %d\n", data_size_vec[i]);
        }
    }
}