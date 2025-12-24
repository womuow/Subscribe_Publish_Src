#include"AdApter_EgoVehicleState_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}



AdApter_EgoVehicleStatePUB::AdApter_EgoVehicleStatePUB()
{
}
AdApter_EgoVehicleStatePUB::~AdApter_EgoVehicleStatePUB()
{
}
auto AdApter_EgoVehicleStatePUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_EgoVehicleStatePUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(EgoVehicleState));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libEgoVehicleStatePUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    EgoVehicleState EgoVehicleState_;
    setIntialValue_EgoVehicleState(EgoVehicleState_);

    while (true) {        
        data_in.resize(sizeof(EgoVehicleState_));
        std::memcpy(&data_in[0], &EgoVehicleState_, sizeof(EgoVehicleState_));
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
    std::cout << "Running EgoVehicleState_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_EgoVehicleStatePUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}







/* Set and Print struct EgoVehicleState initial value */
void setIntialValue_EgoVehicleState(EgoVehicleState& EgoVehicleState_){
    std::cout << "Set struct EgoVehicleState variable and Publish:" << std::endl;
    EgoVehicleState_.VLgt = 1.10;
    std::cout << "EgoVehicleState_.VLgt(float32): " << EgoVehicleState_.VLgt << std::endl;
    EgoVehicleState_.ALgt = 2.20;
    std::cout << "EgoVehicleState_.ALgt(float32): " << EgoVehicleState_.ALgt << std::endl;
    EgoVehicleState_.ALgtRaw = 3.30;
    std::cout << "EgoVehicleState_.ALgtRaw(float32): " << EgoVehicleState_.ALgtRaw << std::endl;
    EgoVehicleState_.ALatRaw = 4.40;
    std::cout << "EgoVehicleState_.ALatRaw(float32): " << EgoVehicleState_.ALatRaw << std::endl;
    EgoVehicleState_.YawRate = 5.50;
    std::cout << "EgoVehicleState_.YawRate(float32): " << EgoVehicleState_.YawRate << std::endl;
    EgoVehicleState_.YawRateRaw = 6.60;
    std::cout << "EgoVehicleState_.YawRateRaw(float32): " << EgoVehicleState_.YawRateRaw << std::endl;
    EgoVehicleState_.SequenceID = 1;
    std::cout << "EgoVehicleState_.SequenceID(uint32): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(EgoVehicleState_.SequenceID) << std::dec  << std::endl;
    EgoVehicleState_.valid = 1;
    std::cout << "EgoVehicleState_.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EgoVehicleState_.valid) << std::dec  << std::endl;
}




