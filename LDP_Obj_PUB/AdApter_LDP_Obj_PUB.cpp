#include"AdApter_LDP_Obj_PUB.h"



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











