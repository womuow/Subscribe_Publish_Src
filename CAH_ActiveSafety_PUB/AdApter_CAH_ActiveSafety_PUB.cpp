#include"AdApter_CAH_ActiveSafety_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}



AdApter_CAH_ActiveSafetyPUB::AdApter_CAH_ActiveSafetyPUB()
{
}
AdApter_CAH_ActiveSafetyPUB::~AdApter_CAH_ActiveSafetyPUB()
{
}
auto AdApter_CAH_ActiveSafetyPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_CAH_ActiveSafetyPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(CAH_ActiveSafety));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libCAHActiveSafetyPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    CAH_ActiveSafety CAH_ActiveSafety_;
    setIntialValue_CAH_ActiveSafety(CAH_ActiveSafety_);

    while (true) {        
        data_in.resize(sizeof(CAH_ActiveSafety_));
        std::memcpy(&data_in[0], &CAH_ActiveSafety_, sizeof(CAH_ActiveSafety_));
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
    std::cout << "Running CAH_ActiveSafety_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_CAH_ActiveSafetyPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}









/* Set and Print struct CAH_ActiveSafety initial value */
void setIntialValue_CAH_ActiveSafety(CAH_ActiveSafety& CAH_ActiveSafety_){
    std::cout << "Set struct CAH_ActiveSafety variable and Publish:" << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_State = 1;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_State(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_State) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_WarningState = 2;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_WarningState(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_WarningState) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning = 3;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState = 4;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_FunConfig = 1;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_FunConfig(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_FunConfig) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_State = 5;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_State(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_State) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq = 6;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_Level = 7;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_Level(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_Level) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_Type = 8;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_Type) << std::dec  << std::endl;
    CAH_ActiveSafety_.drt.DRT_TTC = 1.10;
    std::cout << "CAH_ActiveSafety_.drt.DRT_TTC(float32): " << CAH_ActiveSafety_.drt.DRT_TTC << std::endl;
    CAH_ActiveSafety_.drt.DRT_ID = 9;
    std::cout << "CAH_ActiveSafety_.drt.DRT_ID(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.drt.DRT_ID) << std::dec  << std::endl;
    CAH_ActiveSafety_.drt.DRT_ObjectClass = 10;
    std::cout << "CAH_ActiveSafety_.drt.DRT_ObjectClass(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.drt.DRT_ObjectClass) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime = 2.20;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_TTCThres = 3.30;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_TTCThres(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_TTCThres << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_TTC = 4.40;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_TTC(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_TTC << std::endl;
}




