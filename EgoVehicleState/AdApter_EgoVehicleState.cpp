#include"AdApter_EgoVehicleState.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<EgoVehicleStateModule::EgoVehicleState> AdApter_EgoVehicleState::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<EgoVehicleStateModule::EgoVehicleState> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<EgoVehicleStateModule::EgoVehicleState> writer(publisher, topic);
    return writer;
}
void AdApter_EgoVehicleState::update_objInfo(std::string data_str)
{
    EgoVehicleState EgoVehicleState_;
    std::memcpy(&EgoVehicleState_, data_str.data(), sizeof(EgoVehicleState));
    topic_out.VLgt() = EgoVehicleState_.VLgt;
    topic_out.ALgt() = EgoVehicleState_.ALgt;
    topic_out.ALgtRaw() = EgoVehicleState_.ALgtRaw;
    topic_out.ALatRaw() = EgoVehicleState_.ALatRaw;
    topic_out.YawRate() = EgoVehicleState_.YawRate;
    topic_out.YawRateRaw() = EgoVehicleState_.YawRateRaw;
    topic_out.SequenceID() = EgoVehicleState_.SequenceID;
    topic_out.valid() = EgoVehicleState_.valid;
    data_in.resize(sizeof(EgoVehicleState));
    std::memcpy(&data_in[0], &EgoVehicleState_, sizeof(EgoVehicleState));
}
void AdApter_EgoVehicleState::maxeye_data_init()
{
}
AdApter_EgoVehicleState::AdApter_EgoVehicleState()
{
}
AdApter_EgoVehicleState::~AdApter_EgoVehicleState()
{
}
auto AdApter_EgoVehicleState::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(EgoVehicleState));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libEgoVehicleState", "1.1.0");
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
    AdApter_EgoVehicleState AdApter_EgoVehicleState_;
    AdApter_EgoVehicleState_.json_file = json_file;
   dds::pub::DataWriter<EgoVehicleStateModule::EgoVehicleState> writer = AdApter_EgoVehicleState_.topic_writer_init(AdApter_EgoVehicleState_.topicName);
    int domain_id = AdApter_EgoVehicleState_.domiain_id;
    std::string topic = AdApter_EgoVehicleState_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libEgoVehicleState", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(
        AdApter_EgoVehicleState_.domiain_id,
        AdApter_EgoVehicleState_.topic, 
        proto_info, 
        [&data_in](MOS::message::spMsg tmp) {
    auto data_vec = tmp->GetDataRef()->GetDataVec();
    auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
    auto size = data_size_vec.size();
    for (int i = 0; i < size; i++) {
    auto vec_size = data_size_vec[i];
    uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
    data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
    }
  }
);
    while (!stop.load()) {
        AdApter_EgoVehicleState_.update_objInfo(data_in);
        writer.write(AdApter_EgoVehicleState_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_EgoVehicleState_.cycle_ms));
    }
    return 0;
    }
void AdApter_EgoVehicleState::run()
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
    AdApter_EgoVehicleState objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
