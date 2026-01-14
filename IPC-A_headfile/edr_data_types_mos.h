
#pragma once
#include <cstdint>
#include "edr_data_types_common.h"

#pragma pack(push, 4)

// Structure for pub to MOS
// message for timestamp event
// topic: /EDR_Data/edr_event/trigger_timestamp
struct EdrEventTimestampMsg {
    EdrTriggerBit               trigger_bit;
    EdrData_VehAdasBasicInfo    edr_data;
};

// message for period event
// topic: /EDR_Data/edr_event/trigger_period
struct EdrEventPeriodMsg {
    EdrTriggerBit   trigger_bit;   // None/Bit5/Bit6/Bit7
    uint32_t        period_seq = 0; // 每100ms自增（事件期间）
    EDR_Data_Period edr_data;
};

// topic: /EDR_Data/edr_event/lock_override_flag
bool override_flag;

#pragma pack(pop)