#include"AdApter_LongCtrlObjInfo_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_LongCtrlObjInfoPUB::AdApter_LongCtrlObjInfoPUB()
{
}
AdApter_LongCtrlObjInfoPUB::~AdApter_LongCtrlObjInfoPUB()
{
}
auto AdApter_LongCtrlObjInfoPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_LongCtrlObjInfoPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LongCtrlObjInfo));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libLongCtrlObjInfoPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    LongCtrlObjInfo LongCtrlObjInfo_;
    setIntialValue_LongCtrlObjInfo(LongCtrlObjInfo_);

    while (true) {        
        data_in.resize(sizeof(LongCtrlObjInfo_));
        std::memcpy(&data_in[0], &LongCtrlObjInfo_, sizeof(LongCtrlObjInfo_));
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
    std::cout << "Running LongCtrlObjInfo_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_LongCtrlObjInfoPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}






/* Set and Print struct LongCtrlObjInfo initial value */
void setIntialValue_LongCtrlObjInfo(LongCtrlObjInfo& LongCtrlObjInfo_){
    std::cout << "Set struct LongCtrlObjInfo variable and Publish:" << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A = 1.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading = 2.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn = 1;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index = 2;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad = 3.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence = 3;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory = 4;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern = 5;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat = 4.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt = 5.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd = 6.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts = 6;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator = 7;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type = 8;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A = 7.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading = 8.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn = 9;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index = 10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad = 9.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence = 11;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory = 12;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern = 13;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat = 11.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt = 12.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd = 13.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts = 14;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator = 15;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type = 16;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A = 14.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading = 15.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn = 17;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index = 18;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad = 16.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence = 19;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory = 20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern = 21;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat = 17.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt = 18.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd = 19.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts = 22;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator = 23;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type = 24;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A = 20.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading = 22.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn = 25;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index = 26;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad = 23.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence = 27;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory = 28;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern = 29;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat = 24.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt = 25.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd = 26.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts = 30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator = 31;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type = 32;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A = 27.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading = 28.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn = 33;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index = 34;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad = 29.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence = 35;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory = 36;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern = 37;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat = 30.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt = 31.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd = 33.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts = 38;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator = 39;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type = 40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1 = 34.10;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2 = 35.20;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3 = 36.30;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4 = 37.40;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5 = 38.50;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6 = 39.60;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7 = 40.70;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8 = 41.80;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9 = 42.90;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10 = 44.00;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10 << std::endl;
}








