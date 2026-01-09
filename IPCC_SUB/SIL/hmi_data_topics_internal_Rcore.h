#ifndef HMI_DATA_INTERNAL_TOPICS_R_CORE_H_
#define HMI_DATA_INTERNAL_TOPICS_R_CORE_H_

#include <string>

#include "hmi_data_HandleIPC_A.h"

namespace hmi_data {
namespace internal {
namespace R_core {

// Topic names for internal communication
namespace topics {
    // target id in IPC_MSG header
    const unsigned short TARGET_ID_R_CORE = 0xC1;
    
    // message id in IPC_MSG header
    const unsigned short MSG_ID_DDS_TO_IPC_HmiConfig        = 8210;
    const unsigned short MSG_ID_IPC_TO_DDS_ActiveSafetyInfo = 8211;
    const unsigned short MSG_ID_IPC_TO_DDS_AutoDriveInfo    = 8212;
    }

    // Message types for internal communication
namespace msg_types {
    using HmiConfig         = HmiConfig_FromDDS_T;
    using ActiveSafetyInfo  = ActiveSafetyInfo_ToDDS_T;
    using AutoDriveInfo     = AutoDriveInfo_ToDDS_T;
}

}  // namespace R_core
}  // namespace internal
}  // namespace hmi_data

#endif  // HMI_DATA_INTERNAL_TOPICS_R_CORE_H_
