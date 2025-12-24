#include"AdApter_LDPPath.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<LDPPathModule::LDPPath> AdApter_LDPPath::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<LDPPathModule::LDPPath> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<LDPPathModule::LDPPath> writer(publisher, topic);
    return writer;
}
void AdApter_LDPPath::update_objInfo(std::string data_str)
{
    LDPPath LDPPath_;
    std::memcpy(&LDPPath_, data_str.data(), sizeof(LDPPath));
    topic_out.ITC().nrOfSegments() = LDPPath_.ITC.nrOfSegments;
    for (int i = 0; i < 24; i++)
    {
    topic_out.ITC().coefficientVector()[i] = LDPPath_.ITC.coefficientVector[i];
    }
    for (int i = 0; i < 6; i++)
    {
    topic_out.ITC().timeVector()[i] = LDPPath_.ITC.timeVector[i];
    }
    topic_out.initialLatPosition() = LDPPath_.initialLatPosition;
    topic_out.initialLatVelocity() = LDPPath_.initialLatVelocity;
    topic_out.initialLatAcceleration() = LDPPath_.initialLatAcceleration;
    topic_out.initialLongVelocity() = LDPPath_.initialLongVelocity;
    topic_out.initialLongAcceleration() = LDPPath_.initialLongAcceleration;
    topic_out.latAccRequiredForAvoidance() = LDPPath_.latAccRequiredForAvoidance;
    topic_out.latAccRequiredForAlignment() = LDPPath_.latAccRequiredForAlignment;
    topic_out.pathInfo() = LDPPath_.pathInfo;
    topic_out.nrOfSegments() = LDPPath_.nrOfSegments;
    for (int i = 0; i < 8; i++)
    {
    topic_out.timeVector()[i] = LDPPath_.timeVector[i];
    }
    for (int i = 0; i < 42; i++)
    {
    topic_out.coefficientVector()[i] = LDPPath_.coefficientVector[i];
    }
    data_in.resize(sizeof(LDPPath));
    std::memcpy(&data_in[0], &LDPPath_, sizeof(LDPPath));
}
void AdApter_LDPPath::maxeye_data_init()
{
}
AdApter_LDPPath::AdApter_LDPPath()
{
}
AdApter_LDPPath::~AdApter_LDPPath()
{
}
auto AdApter_LDPPath::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LDPPath));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libLDPPath", "1.1.0");
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
    AdApter_LDPPath AdApter_LDPPath_;
    AdApter_LDPPath_.json_file = json_file;
   dds::pub::DataWriter<LDPPathModule::LDPPath> writer = AdApter_LDPPath_.topic_writer_init(AdApter_LDPPath_.topicName);
    int domain_id = AdApter_LDPPath_.domiain_id;
    std::string topic = AdApter_LDPPath_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libLDPPath", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_LDPPath_.domiain_id,AdApter_LDPPath_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_LDPPath_.update_objInfo(data_in);
        writer.write(AdApter_LDPPath_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_LDPPath_.cycle_ms));
    }
    return 0;
    }
void AdApter_LDPPath::run()
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
    AdApter_LDPPath objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
