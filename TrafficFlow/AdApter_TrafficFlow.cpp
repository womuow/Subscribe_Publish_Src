#include"AdApter_TrafficFlow.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<TrafficFlowModule::TrafficFlow> AdApter_TrafficFlow::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<TrafficFlowModule::TrafficFlow> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<TrafficFlowModule::TrafficFlow> writer(publisher, topic);
    return writer;
}
void AdApter_TrafficFlow::update_objInfo(std::string data_str)
{
    TrafficFlow TrafficFlow_;
    std::memcpy(&TrafficFlow_, data_str.data(), sizeof(TrafficFlow));
    topic_out.DrvgSideQly() = TrafficFlow_.DrvgSideQly;
    for (int i = 0; i < 3; i++)
    {
    topic_out.LaneProperties()[i].TrfcFlowAvrgDst() = TrafficFlow_.LaneProperties[i].TrfcFlowAvrgDst;
    topic_out.LaneProperties()[i].TrfcFlowAvrgSpd() = TrafficFlow_.LaneProperties[i].TrfcFlowAvrgSpd;
    topic_out.LaneProperties()[i].TrfcFlowNormDir() = TrafficFlow_.LaneProperties[i].TrfcFlowNormDir;
    topic_out.LaneProperties()[i].TrfcFlowQly() = TrafficFlow_.LaneProperties[i].TrfcFlowQly;
    }
    topic_out.SequenceID() = TrafficFlow_.SequenceID;
    topic_out.DrivingSide() = TrafficFlow_.DrivingSide;
    data_in.resize(sizeof(TrafficFlow));
    std::memcpy(&data_in[0], &TrafficFlow_, sizeof(TrafficFlow));
}
void AdApter_TrafficFlow::maxeye_data_init()
{
}
AdApter_TrafficFlow::AdApter_TrafficFlow()
{
}
AdApter_TrafficFlow::~AdApter_TrafficFlow()
{
}
auto AdApter_TrafficFlow::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(TrafficFlow));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libTrafficFlow", "1.1.0");
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
    AdApter_TrafficFlow AdApter_TrafficFlow_;
    AdApter_TrafficFlow_.json_file = json_file;
   dds::pub::DataWriter<TrafficFlowModule::TrafficFlow> writer = AdApter_TrafficFlow_.topic_writer_init(AdApter_TrafficFlow_.topicName);
    int domain_id = AdApter_TrafficFlow_.domiain_id;
    std::string topic = AdApter_TrafficFlow_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libTrafficFlow", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_TrafficFlow_.domiain_id,AdApter_TrafficFlow_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_TrafficFlow_.update_objInfo(data_in);
        writer.write(AdApter_TrafficFlow_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_TrafficFlow_.cycle_ms));
    }
    return 0;
    }
void AdApter_TrafficFlow::run()
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
    AdApter_TrafficFlow objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
