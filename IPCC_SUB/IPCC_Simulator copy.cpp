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

void Send_ActiveSafetyInfo_R() {
    auto pub_msg = std::make_shared<MOS::message::Message>();

    MOS::communication::ProtocolInfo sub_proto_info;
    sub_proto_info.protocol_type = MOS::communication::kProtocolShm;
    // Create and initialize the ActiveSafetyInfo_ToDDS_T struct
    ActiveSafetyInfo_ToDDS_T activeSafetyInfo = {};

    // Assign values to the struct fields
    activeSafetyInfo.ActiveSafetyInfo_aeb_warning = 1;
    activeSafetyInfo.ActiveSafetyInfo_mai_warning = 0;
    activeSafetyInfo.ActiveSafetyInfo_textinfo_index = active_safety_test_index % 401; // Test value: 0 to 400

    // Initialize the FunmodeInfo components
    activeSafetyInfo.ActiveSafetyInfo_lka_warning.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_lka_warning.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_lka_warning.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_lka_warning.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_lka_warning.warning_state = 0;

    activeSafetyInfo.ActiveSafetyInfo_fdw_warning.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fdw_warning.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fdw_warning.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fdw_warning.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fdw_warning.warning_state = 0;

    activeSafetyInfo.ActiveSafetyInfo_fcw_warning.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fcw_warning.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fcw_warning.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fcw_warning.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fcw_warning.warning_state = 0;

    activeSafetyInfo.ActiveSafetyInfo_ldw_warning.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_ldw_warning.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_ldw_warning.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_ldw_warning.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_ldw_warning.warning_state = 0;

    activeSafetyInfo.ActiveSafetyInfo_fvsa_warning.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fvsa_warning.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fvsa_warning.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_fvsa_warning.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_fvsa_warning.warning_state = 0;

    activeSafetyInfo.ActiveSafetyInfo_warning_info.warning_Info.left_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_warning_info.warning_Info.left_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_warning_info.warning_Info.right_warning_level = 0;
    activeSafetyInfo.ActiveSafetyInfo_warning_info.warning_Info.right_warning_location = 0;
    activeSafetyInfo.ActiveSafetyInfo_warning_info.warning_state = 0;
    
    MOS::communication::ProtocolInfo protoc_info;
    protoc_info.protocol_type = MOS::communication::kProtocolHybrid;
    protoc_info.shm_info.block_count = 256;
    protoc_info.shm_info.block_size = 1024*6;
    protoc_info.shm_info.fast_mode = false;
    auto pub_ipc_ = MOS::communication::Publisher::New(0, "ipc_services/msg/subscriber", protoc_info);


    // // Prepare the IPC message
    IPC_MSG_DATA_SIZE_MAX ipc_msg_ActiveSafetyInfo;
    ipc_msg_ActiveSafetyInfo.header.target = 0xC1; // TARGET_ID_R_CORE
    ipc_msg_ActiveSafetyInfo.header.id = 8211; // MSG_ID_IPC_TO_DDS_ActiveSafetyInfo
    ipc_msg_ActiveSafetyInfo.header.version = 1;
    ipc_msg_ActiveSafetyInfo.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ipc_msg_ActiveSafetyInfo.header.data_size = sizeof(ActiveSafetyInfo_ToDDS_T);

    // Copy the struct data to the message
    memcpy(ipc_msg_ActiveSafetyInfo.data, &activeSafetyInfo, sizeof(ActiveSafetyInfo_ToDDS_T));

    const auto now_time = MOS::TimeUtils::NowNsec();
    const auto data_ref = std::make_shared<MOS::message::DataRef>(ipc_msg_ActiveSafetyInfo, sizeof(ActiveSafetyInfo_ToDDS_T));
    pub_msg->SetDataRef(data_ref);
    pub_msg->SetGenTimestamp(now_time);
    pub_ipc_->Pub(pub_msg);
    
    active_safety_test_index = (active_safety_test_index + 1) % 401;
}

// Global variable to track the test index for AutoDriveInfo
static uint32_t auto_drive_test_index = 0;

void Send_AutoDriveInfo_R() {
    // Create and initialize the AutoDriveInfo_ToDDS_T struct
    AutoDriveInfo_ToDDS_T autoDriveInfo = {};

    // Assign values to the struct fields
    autoDriveInfo.AutoDriveInfo_adas_status = 1; // Example value (uint8)
    autoDriveInfo.AutoDriveInfo_acc_status = 1; // Example value (uint8)
    autoDriveInfo.AutoDriveInfo_fault_adas = 0; // Example value (uint32)
    autoDriveInfo.AutoDriveInfo_adas_target_speed_set = 60.0f; // Example value (float32)
    autoDriveInfo.AutoDriveInfo_speed_set_limit = 120.0f; // Example value (float32)
    autoDriveInfo.AutoDriveInfo_textinfo_index = auto_drive_test_index % 401; // Test value: 0 to 400

    // Initialize the hands_off_info component
    autoDriveInfo.AutoDriveInfo_hands_off_info.hands_off_level = 0; // Example value (uint32)
    autoDriveInfo.AutoDriveInfo_hands_off_info.hands_off_count_down = 0; // Example value (uint32)
    autoDriveInfo.AutoDriveInfo_hands_off_info.hands_off_count_down_time = 0; // Example value (uint32)

    // Prepare the IPC message
    IPC_MSG_DATA_SIZE_MAX ipc_msg_AutoDriveInfo;
    ipc_msg_AutoDriveInfo.header.target = 0xC1; // TARGET_ID_R_CORE
    ipc_msg_AutoDriveInfo.header.id = 8212; // MSG_ID_IPC_TO_DDS_AutoDriveInfo
    ipc_msg_AutoDriveInfo.header.version = 1;
    ipc_msg_AutoDriveInfo.header.timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    ipc_msg_AutoDriveInfo.header.data_size = sizeof(AutoDriveInfo_ToDDS_T);

    // Copy the struct data to the message
    memcpy(ipc_msg_AutoDriveInfo.data, &autoDriveInfo, sizeof(AutoDriveInfo_ToDDS_T));

    bool res = ipc_send_msg(&(ipc_msg_AutoDriveInfo.header));
    if (!res)
    {
        std::cout << "ERROR: Send ipc_msg_AutoDriveInfo fail" << std::endl;
    }
    printf("Sent AutoDriveInfo via IPC - ID: 8212, Size: %zu, textinfo_index: %u\n",
           sizeof(AutoDriveInfo_ToDDS_T), autoDriveInfo.AutoDriveInfo_textinfo_index);

    // Increment the test index for next call and wrap around at 401
    auto_drive_test_index = (auto_drive_test_index + 1) % 401;
}

int main() {
    std::cout << "Starting IPCC Simulator..." << std::endl;

    // Initialize communication
    std::filesystem::path current_path = std::filesystem::current_path();
    std::string config_path = current_path.string() + "/config/discovery_config.json";
    std::cout << "配置文件路径: " << config_path << std::endl;
    MOS::communication::Init(config_path.c_str()); 
    MOS::utils::Register::get().register_version("ipcc", "1.1.0");

    // Initialize IPC converter if needed
    // IPCC::IPCConvert& ipc_converter = IPCC::IPCConvert::getInstance();
    // ipc_converter.init(); // Uncomment if initialization is required

    while (true) {
        // Send ActiveSafetyInfo message
        Send_ActiveSafetyInfo_R();

        // Sleep for 50ms before sending the next message
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // // Send AutoDriveInfo message
        // Send_AutoDriveInfo_R();

        // // Sleep for 50ms to complete the 100ms cycle
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Check for stop condition if needed
        if(stop.load()) {
            break;
        }
    }

    std::cout << "Shutting down IPCC Simulator..." << std::endl;
    return 0;
}