#include "SIL/hmi_data_topics_internal_Rcore.h"
#include "ipc_msg.h"
#include "ipc_transport.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mos-com/utils/register.h>
#include <mos-com/utils/time_utils.h>
#include <mos-com/utils/error.h>
#include "mos-com/interface/client.hpp"
#include "mos-com/interface/getter.hpp"
#include "mos-com/interface/publisher.hpp"
#include "mos-com/interface/service.hpp"
#include "mos-com/interface/subscriber.hpp"
#include "mos-com/message/message.h"
#include "mos-com/utils/debug_log.h"
#include "ipc_transport/ipc_msg.h"
#include "ipc_transport/ipc_transport.h"
#include <filesystem> 

// #include "IPCConvert.hpp" 
// 声明全局停止标志
std::atomic_bool stop{false};

// Global variable to track the test index for ActiveSafetyInfo
static uint32_t active_safety_test_index = 0;

// 在全局作用域添加ProtocolInfo和Publisher的声明
static MOS::communication::ProtocolInfo protoc_info;
static std::shared_ptr<MOS::communication::Publisher> pub_ipc_;
static std::shared_ptr<MOS::communication::Subscriber> sub_ipc_;

// 添加初始化函数
void InitPublisher() {
    std::filesystem::path current_path = std::filesystem::current_path();
    std::string config_path = current_path.string() + "/config/discovery_config.json";
    std::cout << "配置文件路径: " << config_path << std::endl;
    MOS::communication::Init(config_path.c_str()); 
    MOS::utils::Register::get().register_version("ipcc", "1.1.0");

    protoc_info.protocol_type = MOS::communication::kProtocolHybrid;
    protoc_info.shm_info.block_count = 256;
    protoc_info.shm_info.block_size = 1024*6;
    protoc_info.shm_info.fast_mode = false;
    std::cout << "开始注册Publisher" << std::endl;
    pub_ipc_ = MOS::communication::Publisher::New(80, "ipc_services/msg/subscriber", protoc_info);
    std::cout << "注册Publisher 完成" << std::endl;
    // sub_ipc_ = MOS::communication::Subscriber::New(80, 
    //                                             "ipc_services/msg/subscriber", 
    //                                             protoc_info, 
    //                                             [](const MOS::message::spMsg msg){printf("Subscriber is called\n");});
    // std::cout << "注册Subscriber 完成" << std::endl;
}

void Send_ActiveSafetyInfo_R() {  
    // Prepare the IPC message
    IPC_MSG_DATA_SIZE_MAX ipc_msg_ActiveSafetyInfo;
    ipc_msg_ActiveSafetyInfo.header.target = 0xC1; // TARGET_ID_R_CORE
    ipc_msg_ActiveSafetyInfo.header.id = 8211; // MSG_ID_IPC_TO_DDS_ActiveSafetyInfo
    ipc_msg_ActiveSafetyInfo.header.version = 1;
    ipc_msg_ActiveSafetyInfo.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ipc_msg_ActiveSafetyInfo.header.data_size = sizeof(ActiveSafetyInfo_ToDDS_T);
    auto pub_msg = std::make_shared<MOS::message::Message>();

    // Create and initialize the ActiveSafetyInfo_ToDDS_T struct
    ActiveSafetyInfo_ToDDS_T activeSafetyInfo = {};
    
    activeSafetyInfo.ActiveSafetyInfo_textinfo_index = active_safety_test_index % 401; // Test value: 0 to 400

    // Copy the struct data to the message
    memcpy(ipc_msg_ActiveSafetyInfo.data, &activeSafetyInfo, sizeof(ActiveSafetyInfo_ToDDS_T));

    const auto now_time = MOS::TimeUtils::NowNsec();
    const auto data_ref = std::make_shared<MOS::message::DataRef>(
        &ipc_msg_ActiveSafetyInfo,
        sizeof(IPC_MSG)+sizeof(ActiveSafetyInfo_ToDDS_T)
    );
    pub_msg->SetDataRef(data_ref);
    pub_msg->SetGenTimestamp(now_time);
    if (pub_ipc_) {
        pub_ipc_->Pub(pub_msg);
        printf("Sent ActiveSafetyInfo via MOS Publisher - ID: 8211, Size: %zu, textinfo_index: %u\n",
               sizeof(ActiveSafetyInfo_ToDDS_T), activeSafetyInfo.ActiveSafetyInfo_textinfo_index);
    } else {
        std::cout << "ERROR: Publisher not initialized" << std::endl;
    }
    
    active_safety_test_index = (active_safety_test_index + 1) % 401;
}

// Global variable to track the test index for AutoDriveInfo
static uint32_t auto_drive_test_index = 200;

void Send_AutoDriveInfo_R() {
    // Prepare the IPC message
    IPC_MSG_DATA_SIZE_MAX ipc_msg_AutoDriveInfo;
    ipc_msg_AutoDriveInfo.header.target = 0xC1; // TARGET_ID_R_CORE
    ipc_msg_AutoDriveInfo.header.id = 8212; // MSG_ID_IPC_TO_DDS_AutoDriveInfo
    // ipc_msg_AutoDriveInfo.header.version = 3;
    ipc_msg_AutoDriveInfo.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ipc_msg_AutoDriveInfo.header.data_size = sizeof(AutoDriveInfo_ToDDS_T);

    AutoDriveInfo_ToDDS_T autoDriveInfo = {};
    autoDriveInfo.AutoDriveInfo_textinfo_index = auto_drive_test_index % 401; // Test value: 0 to 400
    memcpy(ipc_msg_AutoDriveInfo.data, &autoDriveInfo, sizeof(AutoDriveInfo_ToDDS_T));

    const auto now_time = MOS::TimeUtils::NowNsec();
    const auto data_ref = std::make_shared<MOS::message::DataRef>(
        &ipc_msg_AutoDriveInfo, 
        sizeof(IPC_MSG)+sizeof(AutoDriveInfo_ToDDS_T)
    );
    auto pub_msg = std::make_shared<MOS::message::Message>();
    pub_msg->SetDataRef(data_ref);
    pub_msg->SetGenTimestamp(now_time);
    if (pub_ipc_) {
        auto ret = pub_ipc_->Pub(pub_msg);
        printf("Sent autoDriveInfo via MOS Publisher - ID: 8212, Result: %d, textinfo_index: %u\n",
               ret, autoDriveInfo.AutoDriveInfo_textinfo_index);
    } else {
        std::cout << "ERROR: Publisher not initialized" << std::endl;
    }
    // Increment the test index for next call and wrap around at 401
    auto_drive_test_index = (auto_drive_test_index + 1) % 401;
}

int main() {
    std::cout << "Starting IPCC Simulator..." << std::endl;

    // Initialize communication
    InitPublisher();
    // register_recv_ipc_msg(0xC1,8212,[](const IPC_MSG *data) { printf("=====\n");});
    while (true) {
        // Send ActiveSafetyInfo message
        Send_ActiveSafetyInfo_R();

        // Sleep for 50ms before sending the next message
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Send AutoDriveInfo message
        Send_AutoDriveInfo_R();

        // Sleep for 50ms to complete the 100ms cycle
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        
        // Check for stop condition if needed
        if(stop.load()) {
            break;
        }
    }

    std::cout << "Shutting down IPCC Simulator..." << std::endl;
    return 0;
}