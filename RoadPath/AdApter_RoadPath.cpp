#include"AdApter_RoadPath.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<RoadPathModule::RoadPath> AdApter_RoadPath::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<RoadPathModule::RoadPath> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<RoadPathModule::RoadPath> writer(publisher, topic);
    return writer;
}
void AdApter_RoadPath::update_objInfo(std::string data_str)
{
    RoadPath RoadPath_;
    std::memcpy(&RoadPath_, data_str.data(), sizeof(RoadPath));
    topic_out.LaneWidth() = RoadPath_.LaneWidth;
    topic_out.OffsLat() = RoadPath_.OffsLat;
    topic_out.DstLgtToEndLaneMrk() = RoadPath_.DstLgtToEndLaneMrk;
    topic_out.DstLgtToEndEhCrvt() = RoadPath_.DstLgtToEndEhCrvt;
    topic_out.DstLgtToEndObj() = RoadPath_.DstLgtToEndObj;
    topic_out.AgDir() = RoadPath_.AgDir;
    topic_out.Crvt() = RoadPath_.Crvt;
    for (int i = 0; i < 3; i++)
    {
    topic_out.CrvtRate()[i] = RoadPath_.CrvtRate[i];
    }
    for (int i = 0; i < 3; i++)
    {
    topic_out.SegLen()[i] = RoadPath_.SegLen[i];
    }
    topic_out.Strtd() = RoadPath_.Strtd;
    topic_out.Vld() = RoadPath_.Vld;
    topic_out.VldForELKA() = RoadPath_.VldForELKA;
    topic_out.VldForTrfcAssi() = RoadPath_.VldForTrfcAssi;
    topic_out.LaneChange() = RoadPath_.LaneChange;
    topic_out.VldForCSA() = RoadPath_.VldForCSA;
    topic_out.VldForOncomingBraking() = RoadPath_.VldForOncomingBraking;
    for (int i = 0; i < 32; i++)
    {
    }
    for (int i = 0; i < 32; i++)
    {
    }
    for (int i = 0; i < 32; i++)
    {
    }
    for (int i = 0; i < 32; i++)
    {
    }
    topic_out.LowConfDist() = RoadPath_.LowConfDist;
    topic_out.HighConfDist() = RoadPath_.HighConfDist;
    topic_out.CrvtQly() = RoadPath_.CrvtQly;
    topic_out.VldForACC() = RoadPath_.VldForACC;
    topic_out.LaneMarkerInfo().Left().Type() = RoadPath_.LaneMarkerInfo.Left.Type;
    topic_out.LaneMarkerInfo().Left().IsUsedByRGF() = RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF;
    topic_out.LaneMarkerInfo().Right().Type() = RoadPath_.LaneMarkerInfo.Right.Type;
    topic_out.LaneMarkerInfo().Right().IsUsedByRGF() = RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF;
    data_in.resize(sizeof(RoadPath));
    std::memcpy(&data_in[0], &RoadPath_, sizeof(RoadPath));
}
void AdApter_RoadPath::maxeye_data_init()
{
}
AdApter_RoadPath::AdApter_RoadPath()
{
}
AdApter_RoadPath::~AdApter_RoadPath()
{
}
auto AdApter_RoadPath::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(RoadPath));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libRoadPath", "1.1.0");
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
    AdApter_RoadPath AdApter_RoadPath_;
    AdApter_RoadPath_.json_file = json_file;
   dds::pub::DataWriter<RoadPathModule::RoadPath> writer = AdApter_RoadPath_.topic_writer_init(AdApter_RoadPath_.topicName);
    int domain_id = AdApter_RoadPath_.domiain_id;
    std::string topic = AdApter_RoadPath_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libRoadPath", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_RoadPath_.domiain_id,AdApter_RoadPath_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_RoadPath_.update_objInfo(data_in);
        writer.write(AdApter_RoadPath_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_RoadPath_.cycle_ms));
    }
    return 0;
    }
void AdApter_RoadPath::run()
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
    AdApter_RoadPath objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
