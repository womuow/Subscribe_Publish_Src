#ifndef HMI_DATA_INTERNAL_TOPICS_A_CORE_H_
#define HMI_DATA_INTERNAL_TOPICS_A_CORE_H_

#include <string>

#include "HmiConfig.hpp"
#include "SR_MAP_Pub.h"
#include "RSI_PUB.h"

namespace hmi_data {
namespace internal {
namespace A_core {

// Topic names for internal communication
namespace topics {
    const std::string HMI_hmiConfig             = "hmi_data/hmiConfig";
    const std::string HmiVisl_HMI_Sr_Map        = "Hmi_Visl/HmiVisl_HMI_Sr_Map";
    const std::string RSI_HMI_ActiveSafetyInfo  = "RSI/RSI_HMI_ActiveSafetyInfo";
    }

    // Message types for internal communication
namespace msg_types {
    // A核通信的消息类型
    // using HmiConfig = hmi_data::hmi_data_pub::HmiConfig;
    using SrMap = HMI_Visl::SR_MAP::SrMap;
    using RoadSignInfo = RSI_RoadSignInfo;
}

}  // namespace A_core
}  // namespace internal
}  // namespace hmi_data

#endif  // HMI_DATA_INTERNAL_TOPICS_A_CORE_H_
