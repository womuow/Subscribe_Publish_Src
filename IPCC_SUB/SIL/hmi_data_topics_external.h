#ifndef HMI_DATA_TOPIC_H
#define HMI_DATA_TOPIC_H
//      defined topic                          dds topic                       period                       datatype
// ------------------------------------------------------------------------------------------------------------------
// upstream 
#define TOPIC_MGN_HMI_SR_MAP                 "/mgn/hmi/sr_map"              // 100ms-150ms SrMap 64*1024    proto
#define TOPIC_MGN_HMI_AUTO_DRIVE_INFO        "/mgn/hmi/auto_driveInfo"      // 100ms AutoDriveInfo          proto
#define TOPIC_MGN_HMI_ACTIVE_SAFETY_INFO     "/mgn/hmi/active_safetyInfo"   // 100ms ActiveSafetyInfo       struct

// downstream
#define TOPIC_LP_HMI_CONFIG_INFO             "/leap/adas/hmi_config"        // 200ms Hmi_Config             struct

#endif