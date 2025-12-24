#include"AdApter_LDPPath_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_LDPPathPUB::AdApter_LDPPathPUB()
{
}
AdApter_LDPPathPUB::~AdApter_LDPPathPUB()
{
}
auto AdApter_LDPPathPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_LDPPathPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LDPPath));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libLDPPathPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    LDPPath LDPPath_;
    setIntialValue_LDPPath(LDPPath_);

    while (true) {        
        data_in.resize(sizeof(LDPPath_));
        std::memcpy(&data_in[0], &LDPPath_, sizeof(LDPPath_));
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
    std::cout << "Running LDPPath_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_LDPPathPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}







/* Set and Print struct LDPPath initial value */
void setIntialValue_LDPPath(LDPPath& LDPPath_){
    std::cout << "Set struct LDPPath variable and Publish:" << std::endl;
    LDPPath_.ITC.nrOfSegments = 1;
    std::cout << "LDPPath_.ITC.nrOfSegments(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDPPath_.ITC.nrOfSegments) << std::dec  << std::endl;
    LDPPath_.ITC.coefficientVector[0] = 1.10;
    std::cout << "LDPPath_.ITC.coefficientVector[0](float32): " << LDPPath_.ITC.coefficientVector[0] << std::endl;
    LDPPath_.ITC.coefficientVector[1] = 2.20;
    std::cout << "LDPPath_.ITC.coefficientVector[1](float32): " << LDPPath_.ITC.coefficientVector[1] << std::endl;
    LDPPath_.ITC.coefficientVector[2] = 3.30;
    std::cout << "LDPPath_.ITC.coefficientVector[2](float32): " << LDPPath_.ITC.coefficientVector[2] << std::endl;
    LDPPath_.ITC.coefficientVector[3] = 4.40;
    std::cout << "LDPPath_.ITC.coefficientVector[3](float32): " << LDPPath_.ITC.coefficientVector[3] << std::endl;
    LDPPath_.ITC.coefficientVector[4] = 5.50;
    std::cout << "LDPPath_.ITC.coefficientVector[4](float32): " << LDPPath_.ITC.coefficientVector[4] << std::endl;
    LDPPath_.ITC.coefficientVector[5] = 6.60;
    std::cout << "LDPPath_.ITC.coefficientVector[5](float32): " << LDPPath_.ITC.coefficientVector[5] << std::endl;
    LDPPath_.ITC.coefficientVector[6] = 7.70;
    std::cout << "LDPPath_.ITC.coefficientVector[6](float32): " << LDPPath_.ITC.coefficientVector[6] << std::endl;
    LDPPath_.ITC.coefficientVector[7] = 8.80;
    std::cout << "LDPPath_.ITC.coefficientVector[7](float32): " << LDPPath_.ITC.coefficientVector[7] << std::endl;
    LDPPath_.ITC.coefficientVector[8] = 9.90;
    std::cout << "LDPPath_.ITC.coefficientVector[8](float32): " << LDPPath_.ITC.coefficientVector[8] << std::endl;
    LDPPath_.ITC.coefficientVector[9] = 11.00;
    std::cout << "LDPPath_.ITC.coefficientVector[9](float32): " << LDPPath_.ITC.coefficientVector[9] << std::endl;
    LDPPath_.ITC.coefficientVector[10] = 12.10;
    std::cout << "LDPPath_.ITC.coefficientVector[10](float32): " << LDPPath_.ITC.coefficientVector[10] << std::endl;
    LDPPath_.ITC.coefficientVector[11] = 13.20;
    std::cout << "LDPPath_.ITC.coefficientVector[11](float32): " << LDPPath_.ITC.coefficientVector[11] << std::endl;
    LDPPath_.ITC.coefficientVector[12] = 14.30;
    std::cout << "LDPPath_.ITC.coefficientVector[12](float32): " << LDPPath_.ITC.coefficientVector[12] << std::endl;
    LDPPath_.ITC.coefficientVector[13] = 15.40;
    std::cout << "LDPPath_.ITC.coefficientVector[13](float32): " << LDPPath_.ITC.coefficientVector[13] << std::endl;
    LDPPath_.ITC.coefficientVector[14] = 16.50;
    std::cout << "LDPPath_.ITC.coefficientVector[14](float32): " << LDPPath_.ITC.coefficientVector[14] << std::endl;
    LDPPath_.ITC.coefficientVector[15] = 17.60;
    std::cout << "LDPPath_.ITC.coefficientVector[15](float32): " << LDPPath_.ITC.coefficientVector[15] << std::endl;
    LDPPath_.ITC.coefficientVector[16] = 18.70;
    std::cout << "LDPPath_.ITC.coefficientVector[16](float32): " << LDPPath_.ITC.coefficientVector[16] << std::endl;
    LDPPath_.ITC.coefficientVector[17] = 19.80;
    std::cout << "LDPPath_.ITC.coefficientVector[17](float32): " << LDPPath_.ITC.coefficientVector[17] << std::endl;
    LDPPath_.ITC.coefficientVector[18] = 20.90;
    std::cout << "LDPPath_.ITC.coefficientVector[18](float32): " << LDPPath_.ITC.coefficientVector[18] << std::endl;
    LDPPath_.ITC.coefficientVector[19] = 22.00;
    std::cout << "LDPPath_.ITC.coefficientVector[19](float32): " << LDPPath_.ITC.coefficientVector[19] << std::endl;
    LDPPath_.ITC.coefficientVector[20] = 23.10;
    std::cout << "LDPPath_.ITC.coefficientVector[20](float32): " << LDPPath_.ITC.coefficientVector[20] << std::endl;
    LDPPath_.ITC.coefficientVector[21] = 24.20;
    std::cout << "LDPPath_.ITC.coefficientVector[21](float32): " << LDPPath_.ITC.coefficientVector[21] << std::endl;
    LDPPath_.ITC.coefficientVector[22] = 25.30;
    std::cout << "LDPPath_.ITC.coefficientVector[22](float32): " << LDPPath_.ITC.coefficientVector[22] << std::endl;
    LDPPath_.ITC.coefficientVector[23] = 26.40;
    std::cout << "LDPPath_.ITC.coefficientVector[23](float32): " << LDPPath_.ITC.coefficientVector[23] << std::endl;
    LDPPath_.ITC.timeVector[0] = 27.50;
    std::cout << "LDPPath_.ITC.timeVector[0](float32): " << LDPPath_.ITC.timeVector[0] << std::endl;
    LDPPath_.ITC.timeVector[1] = 28.60;
    std::cout << "LDPPath_.ITC.timeVector[1](float32): " << LDPPath_.ITC.timeVector[1] << std::endl;
    LDPPath_.ITC.timeVector[2] = 29.70;
    std::cout << "LDPPath_.ITC.timeVector[2](float32): " << LDPPath_.ITC.timeVector[2] << std::endl;
    LDPPath_.ITC.timeVector[3] = 30.80;
    std::cout << "LDPPath_.ITC.timeVector[3](float32): " << LDPPath_.ITC.timeVector[3] << std::endl;
    LDPPath_.ITC.timeVector[4] = 31.90;
    std::cout << "LDPPath_.ITC.timeVector[4](float32): " << LDPPath_.ITC.timeVector[4] << std::endl;
    LDPPath_.ITC.timeVector[5] = 33.00;
    std::cout << "LDPPath_.ITC.timeVector[5](float32): " << LDPPath_.ITC.timeVector[5] << std::endl;
    LDPPath_.initialLatPosition = 34.10;
    std::cout << "LDPPath_.initialLatPosition(float32): " << LDPPath_.initialLatPosition << std::endl;
    LDPPath_.initialLatVelocity = 35.20;
    std::cout << "LDPPath_.initialLatVelocity(float32): " << LDPPath_.initialLatVelocity << std::endl;
    LDPPath_.initialLatAcceleration = 36.30;
    std::cout << "LDPPath_.initialLatAcceleration(float32): " << LDPPath_.initialLatAcceleration << std::endl;
    LDPPath_.initialLongVelocity = 37.40;
    std::cout << "LDPPath_.initialLongVelocity(float32): " << LDPPath_.initialLongVelocity << std::endl;
    LDPPath_.initialLongAcceleration = 38.50;
    std::cout << "LDPPath_.initialLongAcceleration(float32): " << LDPPath_.initialLongAcceleration << std::endl;
    LDPPath_.latAccRequiredForAvoidance = 39.60;
    std::cout << "LDPPath_.latAccRequiredForAvoidance(float32): " << LDPPath_.latAccRequiredForAvoidance << std::endl;
    LDPPath_.latAccRequiredForAlignment = 40.70;
    std::cout << "LDPPath_.latAccRequiredForAlignment(float32): " << LDPPath_.latAccRequiredForAlignment << std::endl;
    LDPPath_.pathInfo = 1;
    std::cout << "LDPPath_.pathInfo(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(LDPPath_.pathInfo) << std::dec  << std::endl;
    LDPPath_.nrOfSegments = 2;
    std::cout << "LDPPath_.nrOfSegments(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDPPath_.nrOfSegments) << std::dec  << std::endl;
    LDPPath_.timeVector[0] = 41.80;
    std::cout << "LDPPath_.timeVector[0](float32): " << LDPPath_.timeVector[0] << std::endl;
    LDPPath_.timeVector[1] = 42.90;
    std::cout << "LDPPath_.timeVector[1](float32): " << LDPPath_.timeVector[1] << std::endl;
    LDPPath_.timeVector[2] = 44.00;
    std::cout << "LDPPath_.timeVector[2](float32): " << LDPPath_.timeVector[2] << std::endl;
    LDPPath_.timeVector[3] = 45.10;
    std::cout << "LDPPath_.timeVector[3](float32): " << LDPPath_.timeVector[3] << std::endl;
    LDPPath_.timeVector[4] = 46.20;
    std::cout << "LDPPath_.timeVector[4](float32): " << LDPPath_.timeVector[4] << std::endl;
    LDPPath_.timeVector[5] = 47.30;
    std::cout << "LDPPath_.timeVector[5](float32): " << LDPPath_.timeVector[5] << std::endl;
    LDPPath_.timeVector[6] = 48.40;
    std::cout << "LDPPath_.timeVector[6](float32): " << LDPPath_.timeVector[6] << std::endl;
    LDPPath_.timeVector[7] = 49.50;
    std::cout << "LDPPath_.timeVector[7](float32): " << LDPPath_.timeVector[7] << std::endl;
    LDPPath_.coefficientVector[0] = 50.60;
    std::cout << "LDPPath_.coefficientVector[0](float32): " << LDPPath_.coefficientVector[0] << std::endl;
    LDPPath_.coefficientVector[1] = 51.70;
    std::cout << "LDPPath_.coefficientVector[1](float32): " << LDPPath_.coefficientVector[1] << std::endl;
    LDPPath_.coefficientVector[2] = 52.80;
    std::cout << "LDPPath_.coefficientVector[2](float32): " << LDPPath_.coefficientVector[2] << std::endl;
    LDPPath_.coefficientVector[3] = 53.90;
    std::cout << "LDPPath_.coefficientVector[3](float32): " << LDPPath_.coefficientVector[3] << std::endl;
    LDPPath_.coefficientVector[4] = 55.00;
    std::cout << "LDPPath_.coefficientVector[4](float32): " << LDPPath_.coefficientVector[4] << std::endl;
    LDPPath_.coefficientVector[5] = 56.10;
    std::cout << "LDPPath_.coefficientVector[5](float32): " << LDPPath_.coefficientVector[5] << std::endl;
    LDPPath_.coefficientVector[6] = 57.20;
    std::cout << "LDPPath_.coefficientVector[6](float32): " << LDPPath_.coefficientVector[6] << std::endl;
    LDPPath_.coefficientVector[7] = 58.30;
    std::cout << "LDPPath_.coefficientVector[7](float32): " << LDPPath_.coefficientVector[7] << std::endl;
    LDPPath_.coefficientVector[8] = 59.40;
    std::cout << "LDPPath_.coefficientVector[8](float32): " << LDPPath_.coefficientVector[8] << std::endl;
    LDPPath_.coefficientVector[9] = 60.50;
    std::cout << "LDPPath_.coefficientVector[9](float32): " << LDPPath_.coefficientVector[9] << std::endl;
    LDPPath_.coefficientVector[10] = 61.60;
    std::cout << "LDPPath_.coefficientVector[10](float32): " << LDPPath_.coefficientVector[10] << std::endl;
    LDPPath_.coefficientVector[11] = 62.70;
    std::cout << "LDPPath_.coefficientVector[11](float32): " << LDPPath_.coefficientVector[11] << std::endl;
    LDPPath_.coefficientVector[12] = 63.80;
    std::cout << "LDPPath_.coefficientVector[12](float32): " << LDPPath_.coefficientVector[12] << std::endl;
    LDPPath_.coefficientVector[13] = 64.90;
    std::cout << "LDPPath_.coefficientVector[13](float32): " << LDPPath_.coefficientVector[13] << std::endl;
    LDPPath_.coefficientVector[14] = 66.00;
    std::cout << "LDPPath_.coefficientVector[14](float32): " << LDPPath_.coefficientVector[14] << std::endl;
    LDPPath_.coefficientVector[15] = 67.10;
    std::cout << "LDPPath_.coefficientVector[15](float32): " << LDPPath_.coefficientVector[15] << std::endl;
    LDPPath_.coefficientVector[16] = 68.20;
    std::cout << "LDPPath_.coefficientVector[16](float32): " << LDPPath_.coefficientVector[16] << std::endl;
    LDPPath_.coefficientVector[17] = 69.30;
    std::cout << "LDPPath_.coefficientVector[17](float32): " << LDPPath_.coefficientVector[17] << std::endl;
    LDPPath_.coefficientVector[18] = 70.40;
    std::cout << "LDPPath_.coefficientVector[18](float32): " << LDPPath_.coefficientVector[18] << std::endl;
    LDPPath_.coefficientVector[19] = 71.50;
    std::cout << "LDPPath_.coefficientVector[19](float32): " << LDPPath_.coefficientVector[19] << std::endl;
    LDPPath_.coefficientVector[20] = 72.60;
    std::cout << "LDPPath_.coefficientVector[20](float32): " << LDPPath_.coefficientVector[20] << std::endl;
    LDPPath_.coefficientVector[21] = 73.70;
    std::cout << "LDPPath_.coefficientVector[21](float32): " << LDPPath_.coefficientVector[21] << std::endl;
    LDPPath_.coefficientVector[22] = 74.80;
    std::cout << "LDPPath_.coefficientVector[22](float32): " << LDPPath_.coefficientVector[22] << std::endl;
    LDPPath_.coefficientVector[23] = 75.90;
    std::cout << "LDPPath_.coefficientVector[23](float32): " << LDPPath_.coefficientVector[23] << std::endl;
    LDPPath_.coefficientVector[24] = 77.00;
    std::cout << "LDPPath_.coefficientVector[24](float32): " << LDPPath_.coefficientVector[24] << std::endl;
    LDPPath_.coefficientVector[25] = 78.10;
    std::cout << "LDPPath_.coefficientVector[25](float32): " << LDPPath_.coefficientVector[25] << std::endl;
    LDPPath_.coefficientVector[26] = 79.20;
    std::cout << "LDPPath_.coefficientVector[26](float32): " << LDPPath_.coefficientVector[26] << std::endl;
    LDPPath_.coefficientVector[27] = 80.30;
    std::cout << "LDPPath_.coefficientVector[27](float32): " << LDPPath_.coefficientVector[27] << std::endl;
    LDPPath_.coefficientVector[28] = 81.40;
    std::cout << "LDPPath_.coefficientVector[28](float32): " << LDPPath_.coefficientVector[28] << std::endl;
    LDPPath_.coefficientVector[29] = 82.50;
    std::cout << "LDPPath_.coefficientVector[29](float32): " << LDPPath_.coefficientVector[29] << std::endl;
    LDPPath_.coefficientVector[30] = 83.60;
    std::cout << "LDPPath_.coefficientVector[30](float32): " << LDPPath_.coefficientVector[30] << std::endl;
    LDPPath_.coefficientVector[31] = 84.70;
    std::cout << "LDPPath_.coefficientVector[31](float32): " << LDPPath_.coefficientVector[31] << std::endl;
    LDPPath_.coefficientVector[32] = 85.80;
    std::cout << "LDPPath_.coefficientVector[32](float32): " << LDPPath_.coefficientVector[32] << std::endl;
    LDPPath_.coefficientVector[33] = 86.90;
    std::cout << "LDPPath_.coefficientVector[33](float32): " << LDPPath_.coefficientVector[33] << std::endl;
    LDPPath_.coefficientVector[34] = 88.00;
    std::cout << "LDPPath_.coefficientVector[34](float32): " << LDPPath_.coefficientVector[34] << std::endl;
    LDPPath_.coefficientVector[35] = 89.10;
    std::cout << "LDPPath_.coefficientVector[35](float32): " << LDPPath_.coefficientVector[35] << std::endl;
    LDPPath_.coefficientVector[36] = 90.20;
    std::cout << "LDPPath_.coefficientVector[36](float32): " << LDPPath_.coefficientVector[36] << std::endl;
    LDPPath_.coefficientVector[37] = 91.30;
    std::cout << "LDPPath_.coefficientVector[37](float32): " << LDPPath_.coefficientVector[37] << std::endl;
    LDPPath_.coefficientVector[38] = 92.40;
    std::cout << "LDPPath_.coefficientVector[38](float32): " << LDPPath_.coefficientVector[38] << std::endl;
    LDPPath_.coefficientVector[39] = 93.50;
    std::cout << "LDPPath_.coefficientVector[39](float32): " << LDPPath_.coefficientVector[39] << std::endl;
    LDPPath_.coefficientVector[40] = 94.60;
    std::cout << "LDPPath_.coefficientVector[40](float32): " << LDPPath_.coefficientVector[40] << std::endl;
    LDPPath_.coefficientVector[41] = 95.70;
    std::cout << "LDPPath_.coefficientVector[41](float32): " << LDPPath_.coefficientVector[41] << std::endl;
}







