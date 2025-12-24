#include"AdApter_FlcRoadCover.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<FlcRoadCoverModule::FlcRoadCover> AdApter_FlcRoadCover::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<FlcRoadCoverModule::FlcRoadCover> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<FlcRoadCoverModule::FlcRoadCover> writer(publisher, topic);
    return writer;
}
void AdApter_FlcRoadCover::update_objInfo(std::string data_str)
{
    FlcRoadCover FlcRoadCover_;
    std::memcpy(&FlcRoadCover_, data_str.data(), sizeof(FlcRoadCover));
    topic_out.Snow().isDetected() = FlcRoadCover_.Snow.isDetected;
    topic_out.Snow().confidence() = FlcRoadCover_.Snow.confidence;
    topic_out.Gravel().isDetected() = FlcRoadCover_.Gravel.isDetected;
    topic_out.Gravel().confidence() = FlcRoadCover_.Gravel.confidence;
    topic_out.Wet().isDetected() = FlcRoadCover_.Wet.isDetected;
    topic_out.Wet().confidence() = FlcRoadCover_.Wet.confidence;
    data_in.resize(sizeof(FlcRoadCover));
    std::memcpy(&data_in[0], &FlcRoadCover_, sizeof(FlcRoadCover));
}
void AdApter_FlcRoadCover::maxeye_data_init()
{
}
AdApter_FlcRoadCover::AdApter_FlcRoadCover()
{
}
AdApter_FlcRoadCover::~AdApter_FlcRoadCover()
{
}
auto AdApter_FlcRoadCover::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(FlcRoadCover));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libFlcRoadCover", "1.1.0");
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
    AdApter_FlcRoadCover AdApter_FlcRoadCover_;
    AdApter_FlcRoadCover_.json_file = json_file;
   dds::pub::DataWriter<FlcRoadCoverModule::FlcRoadCover> writer = AdApter_FlcRoadCover_.topic_writer_init(AdApter_FlcRoadCover_.topicName);
    int domain_id = AdApter_FlcRoadCover_.domiain_id;
    std::string topic = AdApter_FlcRoadCover_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libFlcRoadCover", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_FlcRoadCover_.domiain_id,AdApter_FlcRoadCover_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_FlcRoadCover_.update_objInfo(data_in);
        writer.write(AdApter_FlcRoadCover_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_FlcRoadCover_.cycle_ms));
    }
    return 0;
    }
void AdApter_FlcRoadCover::run()
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
    AdApter_FlcRoadCover objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
