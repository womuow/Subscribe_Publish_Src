#include"AdApter_ObjFrntCdnForSupp_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_ObjFrntCdnForSuppPUB::AdApter_ObjFrntCdnForSuppPUB()
{
}
AdApter_ObjFrntCdnForSuppPUB::~AdApter_ObjFrntCdnForSuppPUB()
{
}
auto AdApter_ObjFrntCdnForSuppPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_ObjFrntCdnForSuppPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(ObjFrntCdnForSupp));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libObjFrntCdnForSuppPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    ObjFrntCdnForSupp ObjFrntCdnForSupp_;
    setIntialValue_ObjFrntCdnForSupp(ObjFrntCdnForSupp_);

    while (true) {        
        data_in.resize(sizeof(ObjFrntCdnForSupp_));
        std::memcpy(&data_in[0], &ObjFrntCdnForSupp_, sizeof(ObjFrntCdnForSupp_));
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
    std::cout << "Running ObjFrntCdnForSupp_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_ObjFrntCdnForSuppPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}





/* Set and Print struct ObjFrntCdnForSupp initial value */
void setIntialValue_ObjFrntCdnForSupp(ObjFrntCdnForSupp& ObjFrntCdnForSupp_){
    std::cout << "Set struct ObjFrntCdnForSupp variable and Publish:" << std::endl;
    ObjFrntCdnForSupp_.ObjIdx = 1;
    std::cout << "ObjFrntCdnForSupp_.ObjIdx(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.ObjIdx) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.ObjPosnLat = 1.10;
    std::cout << "ObjFrntCdnForSupp_.ObjPosnLat(float32): " << ObjFrntCdnForSupp_.ObjPosnLat << std::endl;
    ObjFrntCdnForSupp_.ObjPosnLgt = 2.20;
    std::cout << "ObjFrntCdnForSupp_.ObjPosnLgt(float32): " << ObjFrntCdnForSupp_.ObjPosnLgt << std::endl;
    ObjFrntCdnForSupp_.ObjSpdLgt = 3.30;
    std::cout << "ObjFrntCdnForSupp_.ObjSpdLgt(float32): " << ObjFrntCdnForSupp_.ObjSpdLgt << std::endl;
    ObjFrntCdnForSupp_.ObjTyp = 2;
    std::cout << "ObjFrntCdnForSupp_.ObjTyp(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.ObjTyp) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.ObjWidth = 4.40;
    std::cout << "ObjFrntCdnForSupp_.ObjWidth(float32): " << ObjFrntCdnForSupp_.ObjWidth << std::endl;
    ObjFrntCdnForSupp_.TiToCllsn = 5.50;
    std::cout << "ObjFrntCdnForSupp_.TiToCllsn(float32): " << ObjFrntCdnForSupp_.TiToCllsn << std::endl;
    ObjFrntCdnForSupp_.VisnIdx = 3;
    std::cout << "ObjFrntCdnForSupp_.VisnIdx(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.VisnIdx) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.SuppressSideRoadEdge = 4;
    std::cout << "ObjFrntCdnForSupp_.SuppressSideRoadEdge(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.SuppressSideRoadEdge) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.SuppressSideSolidLKA = 5;
    std::cout << "ObjFrntCdnForSupp_.SuppressSideSolidLKA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.SuppressSideSolidLKA) << std::dec  << std::endl;
}








