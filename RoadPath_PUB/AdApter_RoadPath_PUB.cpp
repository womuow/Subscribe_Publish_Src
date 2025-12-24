#include"AdApter_RoadPath_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}



AdApter_RoadPathPUB::AdApter_RoadPathPUB()
{
}
AdApter_RoadPathPUB::~AdApter_RoadPathPUB()
{
}
auto AdApter_RoadPathPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_RoadPathPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(RoadPath));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libRoadPathPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();


    RoadPath RoadPath_;
    setIntialValue_RoadPath(RoadPath_);

    while (true) {        
        data_in.resize(sizeof(RoadPath_));
        std::memcpy(&data_in[0], &RoadPath_, sizeof(RoadPath_));
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
    std::cout << "Running RoadPath_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_RoadPathPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}


/* Set and Print struct RoadPath initial value */
void setIntialValue_RoadPath(RoadPath& RoadPath_){
    std::cout << "Set struct RoadPath variable and Publish:" << std::endl;
    RoadPath_.LaneWidth = 1.10;
    std::cout << "RoadPath_.LaneWidth(float32): " << RoadPath_.LaneWidth << std::endl;
    RoadPath_.OffsLat = 2.20;
    std::cout << "RoadPath_.OffsLat(float32): " << RoadPath_.OffsLat << std::endl;
    RoadPath_.DstLgtToEndLaneMrk = 3.30;
    std::cout << "RoadPath_.DstLgtToEndLaneMrk(float32): " << RoadPath_.DstLgtToEndLaneMrk << std::endl;
    RoadPath_.DstLgtToEndEhCrvt = 4.40;
    std::cout << "RoadPath_.DstLgtToEndEhCrvt(float32): " << RoadPath_.DstLgtToEndEhCrvt << std::endl;
    RoadPath_.DstLgtToEndObj = 5.50;
    std::cout << "RoadPath_.DstLgtToEndObj(float32): " << RoadPath_.DstLgtToEndObj << std::endl;
    RoadPath_.AgDir = 6.60;
    std::cout << "RoadPath_.AgDir(float32): " << RoadPath_.AgDir << std::endl;
    RoadPath_.Crvt = 7.70;
    std::cout << "RoadPath_.Crvt(float32): " << RoadPath_.Crvt << std::endl;
    RoadPath_.CrvtRate[0] = 8.80;
    std::cout << "RoadPath_.CrvtRate[0](float32): " << RoadPath_.CrvtRate[0] << std::endl;
    RoadPath_.CrvtRate[1] = 9.90;
    std::cout << "RoadPath_.CrvtRate[1](float32): " << RoadPath_.CrvtRate[1] << std::endl;
    RoadPath_.CrvtRate[2] = 11.00;
    std::cout << "RoadPath_.CrvtRate[2](float32): " << RoadPath_.CrvtRate[2] << std::endl;
    RoadPath_.SegLen[0] = 12.10;
    std::cout << "RoadPath_.SegLen[0](float32): " << RoadPath_.SegLen[0] << std::endl;
    RoadPath_.SegLen[1] = 13.20;
    std::cout << "RoadPath_.SegLen[1](float32): " << RoadPath_.SegLen[1] << std::endl;
    RoadPath_.SegLen[2] = 14.30;
    std::cout << "RoadPath_.SegLen[2](float32): " << RoadPath_.SegLen[2] << std::endl;
    RoadPath_.Strtd = 1;
    std::cout << "RoadPath_.Strtd(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.Strtd) << std::dec  << std::endl;
    RoadPath_.Vld = 2;
    std::cout << "RoadPath_.Vld(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.Vld) << std::dec  << std::endl;
    RoadPath_.VldForELKA = 3;
    std::cout << "RoadPath_.VldForELKA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForELKA) << std::dec  << std::endl;
    RoadPath_.VldForTrfcAssi = 4;
    std::cout << "RoadPath_.VldForTrfcAssi(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForTrfcAssi) << std::dec  << std::endl;
    RoadPath_.LaneChange = 5;
    std::cout << "RoadPath_.LaneChange(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneChange) << std::dec  << std::endl;
    RoadPath_.VldForCSA = 6;
    std::cout << "RoadPath_.VldForCSA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForCSA) << std::dec  << std::endl;
    RoadPath_.VldForOncomingBraking = 7;
    std::cout << "RoadPath_.VldForOncomingBraking(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForOncomingBraking) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0] = 8;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1] = 9;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2] = 10;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3] = 11;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4] = 12;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5] = 13;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6] = 14;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7] = 15;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8] = 16;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9] = 17;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10] = 18;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11] = 19;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12] = 20;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13] = 21;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14] = 22;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15] = 23;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16] = 24;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17] = 25;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18] = 26;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19] = 27;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20] = 28;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21] = 29;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22] = 30;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23] = 31;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24] = 32;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25] = 33;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26] = 34;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27] = 35;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28] = 36;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29] = 37;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30] = 38;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31] = 39;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[0] = 40;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[1] = 41;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[2] = 42;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[3] = 43;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[4] = 44;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[5] = 45;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[6] = 46;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[7] = 47;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[8] = 48;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[9] = 49;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[10] = 50;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[11] = 51;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[12] = 52;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[13] = 53;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[14] = 54;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[15] = 55;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[16] = 56;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[17] = 57;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[18] = 58;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[19] = 59;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[20] = 60;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[21] = 61;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[22] = 62;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[23] = 63;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[24] = 64;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[25] = 65;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[26] = 66;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[27] = 67;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[28] = 68;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[29] = 69;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[30] = 70;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[31] = 71;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[0] = 72;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[1] = 73;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[2] = 74;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[3] = 75;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[4] = 76;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[5] = 77;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[6] = 78;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[7] = 79;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[8] = 80;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[9] = 81;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[10] = 82;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[11] = 83;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[12] = 84;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[13] = 85;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[14] = 86;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[15] = 87;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[16] = 88;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[17] = 89;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[18] = 90;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[19] = 91;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[20] = 92;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[21] = 93;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[22] = 94;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[23] = 95;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[24] = 96;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[25] = 97;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[26] = 98;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[27] = 99;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[28] = 100;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[29] = 101;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[30] = 102;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[31] = 103;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[0] = 104;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[1] = 105;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[2] = 106;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[3] = 107;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[4] = 108;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[5] = 109;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[6] = 110;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[7] = 111;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[8] = 112;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[9] = 113;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[10] = 114;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[11] = 115;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[12] = 116;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[13] = 117;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[14] = 118;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[15] = 119;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[16] = 120;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[17] = 121;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[18] = 122;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[19] = 123;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[20] = 124;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[21] = 125;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[22] = 126;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[23] = 127;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[24] = 128;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[25] = 129;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[26] = 130;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[27] = 131;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[28] = 132;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[29] = 133;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[30] = 134;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[31] = 135;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[31]) << std::dec  << std::endl;
    RoadPath_.LowConfDist = 15.40;
    std::cout << "RoadPath_.LowConfDist(float32): " << RoadPath_.LowConfDist << std::endl;
    RoadPath_.HighConfDist = 16.50;
    std::cout << "RoadPath_.HighConfDist(float32): " << RoadPath_.HighConfDist << std::endl;
    RoadPath_.CrvtQly = 136;
    std::cout << "RoadPath_.CrvtQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.CrvtQly) << std::dec  << std::endl;
    RoadPath_.VldForACC = 137;
    std::cout << "RoadPath_.VldForACC(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForACC) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Left.Type = 138;
    std::cout << "RoadPath_.LaneMarkerInfo.Left.Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Left.Type) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF = 139;
    std::cout << "RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Right.Type = 140;
    std::cout << "RoadPath_.LaneMarkerInfo.Right.Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Right.Type) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF = 141;
    std::cout << "RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF) << std::dec  << std::endl;
}

