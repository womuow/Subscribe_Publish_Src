#include"AdApter_LDP_Obj_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_LDP_ObjPUB::AdApter_LDP_ObjPUB()
{
}
AdApter_LDP_ObjPUB::~AdApter_LDP_ObjPUB()
{
}
auto AdApter_LDP_ObjPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_LDP_ObjPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LDP_Obj));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libLDP_ObjPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    LDP_Obj LDP_Obj_;
    setIntialValue_LDP_Obj(LDP_Obj_);

    while (true) {        
        data_in.resize(sizeof(LDP_Obj_));
        std::memcpy(&data_in[0], &LDP_Obj_, sizeof(LDP_Obj_));
        auto data_ref = std::make_shared<MOS::message::DataRef>(const_cast<char*>(data_in.data()), data_in.size());
        mos_msg->SetDataRef(data_ref);
        auto now_time = MOS::TimeUtils::NowNsec();
        mos_msg->SetGenTimestamp(now_time);
        pub->Pub(mos_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms));//40ms
    }
}
int main()
{
#ifdef _WIN32
    char* path = nullptr;
    _get_pgmptr(&path);
    std::stringstream ss;
    ss << path;
    std::string fullPath(path);
    std::cout << "Running on Windows" << std::endl;
#endif
#ifdef __linux__
    std::string fullPath = std::filesystem::read_symlink("/proc/self/exe");
    std::cout << "Running LDP_Obj_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_LDP_ObjPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}





/* Set and Print struct LDP_Obj initial value */
void setIntialValue_LDP_Obj(LDP_Obj& LDP_Obj_){
    std::cout << "Set struct LDP_Obj variable and Publish:" << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType = 1;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.objectType = 2;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.objectType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.objectType) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint = 3;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.length = 1.10;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.length(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.length << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.width = 2.20;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.width(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.width << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.heading = 3.30;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.heading(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.heading << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision = 4.40;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach = 5.50;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane = 6.60;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision = 7.70;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition = 8.80;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition = 9.90;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity = 11.00;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity = 12.10;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane = 4;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf = 13.20;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance = 14.30;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.exists = 5;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.exists(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.exists) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInEgoLane = 6;
    std::cout << "LDP_Obj_.secondaryObstacleInEgoLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInEgoLane) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInLeftLane = 7;
    std::cout << "LDP_Obj_.secondaryObstacleInLeftLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInLeftLane) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInRightLane = 8;
    std::cout << "LDP_Obj_.secondaryObstacleInRightLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInRightLane) << std::dec  << std::endl;
}









