#include"AdApter_LongCtrlObjInfo.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<LongCtrlObjInfoModule::LongCtrlObjInfo> AdApter_LongCtrlObjInfo::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<LongCtrlObjInfoModule::LongCtrlObjInfo> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<LongCtrlObjInfoModule::LongCtrlObjInfo> writer(publisher, topic);
    return writer;
}
void AdApter_LongCtrlObjInfo::update_objInfo(std::string data_str)
{
    LongCtrlObjInfo LongCtrlObjInfo_;
    std::memcpy(&LongCtrlObjInfo_, data_str.data(), sizeof(LongCtrlObjInfo));
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().A() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().Heading() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().Idn() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().Index() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().latPositionRoad() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().latPositionRoadConfidence() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().motionHistory() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().motionPattern() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().PosnLat() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().PosnLgt() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().Spd() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().TgtLaneSts() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().turnIndicator() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator;
    topic_out.TargetsSelectedForACC().AccTgtAdjLLeft().type() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().A() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().Heading() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().Idn() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().Index() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().latPositionRoad() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().latPositionRoadConfidence() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().motionHistory() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().motionPattern() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().PosnLat() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().PosnLgt() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().Spd() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().TgtLaneSts() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().turnIndicator() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator;
    topic_out.TargetsSelectedForACC().AccTgtAdjLRight().type() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().A() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().Heading() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().Idn() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().Index() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().latPositionRoad() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().latPositionRoadConfidence() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().motionHistory() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().motionPattern() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().PosnLat() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().PosnLgt() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().Spd() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().TgtLaneSts() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().turnIndicator() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator;
    topic_out.TargetsSelectedForACC().AccTgtCutIn().type() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().A() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().Heading() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().Idn() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().Index() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().latPositionRoad() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().latPositionRoadConfidence() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().motionHistory() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().motionPattern() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().PosnLat() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().PosnLgt() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().Spd() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().TgtLaneSts() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().turnIndicator() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator;
    topic_out.TargetsSelectedForACC().AccTgtFrstClstLane().type() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().A() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().Heading() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().Idn() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().Index() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().latPositionRoad() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().latPositionRoadConfidence() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().motionHistory() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().motionPattern() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().PosnLat() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().PosnLgt() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().Spd() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().TgtLaneSts() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().turnIndicator() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator;
    topic_out.TargetsSelectedForACC().AccTgtSecClstLane().type() = LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type;
    topic_out.AdditionalTarSelnSignals().Signal1() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1;
    topic_out.AdditionalTarSelnSignals().Signal2() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2;
    topic_out.AdditionalTarSelnSignals().Signal3() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3;
    topic_out.AdditionalTarSelnSignals().Signal4() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4;
    topic_out.AdditionalTarSelnSignals().Signal5() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5;
    topic_out.AdditionalTarSelnSignals().Signal6() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6;
    topic_out.AdditionalTarSelnSignals().Signal7() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7;
    topic_out.AdditionalTarSelnSignals().Signal8() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8;
    topic_out.AdditionalTarSelnSignals().Signal9() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9;
    topic_out.AdditionalTarSelnSignals().Signal10() = LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10;
    data_in.resize(sizeof(LongCtrlObjInfo));
    std::memcpy(&data_in[0], &LongCtrlObjInfo_, sizeof(LongCtrlObjInfo));
}
void AdApter_LongCtrlObjInfo::maxeye_data_init()
{
}
AdApter_LongCtrlObjInfo::AdApter_LongCtrlObjInfo()
{
}
AdApter_LongCtrlObjInfo::~AdApter_LongCtrlObjInfo()
{
}
auto AdApter_LongCtrlObjInfo::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LongCtrlObjInfo));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libLongCtrlObjInfo", "1.1.0");
MOS::communication::ProtocolInfo proto_info;
if (type) {
    proto_info.protocol_type = MOS::communication::kProtocolNet;
    if (!MOS::communication::IsDynamic()) {
        proto_info.net_info.remote_addr_vec.emplace_back(ip, port);
    }
}
else {
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
}
return proto_info;
}
int config_async_sub(std::string json_file) {
    AdApter_LongCtrlObjInfo AdApter_LongCtrlObjInfo_;
    AdApter_LongCtrlObjInfo_.json_file = json_file;
   dds::pub::DataWriter<LongCtrlObjInfoModule::LongCtrlObjInfo> writer = AdApter_LongCtrlObjInfo_.topic_writer_init(AdApter_LongCtrlObjInfo_.topicName);
    int domain_id = AdApter_LongCtrlObjInfo_.domiain_id;
    std::string topic = AdApter_LongCtrlObjInfo_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libLongCtrlObjInfo", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_LongCtrlObjInfo_.domiain_id,AdApter_LongCtrlObjInfo_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
    auto data_vec = tmp->GetDataRef()->GetDataVec();
    auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
    auto size = data_size_vec.size();
    for (int i = 0; i < size; i++) {
    auto vec_size = data_size_vec[i];
    uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
    data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
    }
  });
    while (!stop.load()) {
        AdApter_LongCtrlObjInfo_.update_objInfo(data_in);
        writer.write(AdApter_LongCtrlObjInfo_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_LongCtrlObjInfo_.cycle_ms));
    }
    return 0;
    }
void AdApter_LongCtrlObjInfo::run()
{
    config_async_sub(json_file);
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
    std::cout << "Running on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_LongCtrlObjInfo objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
