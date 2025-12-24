#include"AdApter_TrafficFlow_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_TrafficFlowPUB::AdApter_TrafficFlowPUB()
{
}
AdApter_TrafficFlowPUB::~AdApter_TrafficFlowPUB()
{
}
auto AdApter_TrafficFlowPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_TrafficFlowPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(TrafficFlow));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libTrafficFlowPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    TrafficFlow TrafficFlow_;
    setIntialValue_TrafficFlow(TrafficFlow_);

    while (true) {        
        data_in.resize(sizeof(TrafficFlow_));
        std::memcpy(&data_in[0], &TrafficFlow_, sizeof(TrafficFlow_));
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
    std::cout << "Running TrafficFlow_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_TrafficFlowPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}




/* Set and Print struct TrafficFlow initial value */
void setIntialValue_TrafficFlow(TrafficFlow& TrafficFlow_){
    std::cout << "Set struct TrafficFlow variable and Publish:" << std::endl;
    TrafficFlow_.DrvgSideQly = 1;
    std::cout << "TrafficFlow_.DrvgSideQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.DrvgSideQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst = 1.10;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd = 2.20;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowNormDir = 2;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[0].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowQly = 3;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[0].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst = 3.30;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd = 4.40;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowNormDir = 4;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[1].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowQly = 5;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[1].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst = 5.50;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd = 6.60;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowNormDir = 6;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[2].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowQly = 7;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[2].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.SequenceID = 1;
    std::cout << "TrafficFlow_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(TrafficFlow_.SequenceID) << std::dec  << std::endl;
    TrafficFlow_.DrivingSide = 8;
    std::cout << "TrafficFlow_.DrivingSide(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.DrivingSide) << std::dec  << std::endl;
}








