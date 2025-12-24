#include"AdApter_RearObject.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<RearObjectModule::RearObject> AdApter_RearObject::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<RearObjectModule::RearObject> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<RearObjectModule::RearObject> writer(publisher, topic);
    return writer;
}
void AdApter_RearObject::update_objInfo(std::string data_str)
{
    RearObject RearObject_;
    std::memcpy(&RearObject_, data_str.data(), sizeof(RearObject));
    for (int i = 0; i < 10; i++)
    {
    topic_out.EgoStates()[i].heading() = RearObject_.EgoStates[i].heading;
    topic_out.EgoStates()[i].latAcceleration() = RearObject_.EgoStates[i].latAcceleration;
    topic_out.EgoStates()[i].latPosition() = RearObject_.EgoStates[i].latPosition;
    topic_out.EgoStates()[i].latPositionPath() = RearObject_.EgoStates[i].latPositionPath;
    topic_out.EgoStates()[i].latVelocity() = RearObject_.EgoStates[i].latVelocity;
    topic_out.EgoStates()[i].longAcceleration() = RearObject_.EgoStates[i].longAcceleration;
    topic_out.EgoStates()[i].longPosition() = RearObject_.EgoStates[i].longPosition;
    topic_out.EgoStates()[i].longPositionPath() = RearObject_.EgoStates[i].longPositionPath;
    topic_out.EgoStates()[i].longVelocity() = RearObject_.EgoStates[i].longVelocity;
    }
    for (int i = 0; i < 10; i++)
    {
    topic_out.RoadStates()[i].headingRoad() = RearObject_.RoadStates[i].headingRoad;
    topic_out.RoadStates()[i].latAccelerationRoad() = RearObject_.RoadStates[i].latAccelerationRoad;
    topic_out.RoadStates()[i].latPositionRoad() = RearObject_.RoadStates[i].latPositionRoad;
    topic_out.RoadStates()[i].latVelocityRoad() = RearObject_.RoadStates[i].latVelocityRoad;
    topic_out.RoadStates()[i].longAccelerationRoad() = RearObject_.RoadStates[i].longAccelerationRoad;
    topic_out.RoadStates()[i].longPositionRoad() = RearObject_.RoadStates[i].longPositionRoad;
    topic_out.RoadStates()[i].longVelocityRoad() = RearObject_.RoadStates[i].longVelocityRoad;
    }
    for (int i = 0; i < 10; i++)
    {
    topic_out.Properties()[i].latPositionConfidence() = RearObject_.Properties[i].latPositionConfidence;
    topic_out.Properties()[i].length() = RearObject_.Properties[i].length;
    topic_out.Properties()[i].confidence() = RearObject_.Properties[i].confidence;
    topic_out.Properties()[i].type() = RearObject_.Properties[i].type;
    topic_out.Properties()[i].width() = RearObject_.Properties[i].width;
    topic_out.Properties()[i].id() = RearObject_.Properties[i].id;
    topic_out.Properties()[i].trackStatus() = RearObject_.Properties[i].trackStatus;
    }
    topic_out.SequenceID() = RearObject_.SequenceID;
    data_in.resize(sizeof(RearObject));
    std::memcpy(&data_in[0], &RearObject_, sizeof(RearObject));
}
void AdApter_RearObject::maxeye_data_init()
{
}
AdApter_RearObject::AdApter_RearObject()
{
}
AdApter_RearObject::~AdApter_RearObject()
{
}
auto AdApter_RearObject::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(RearObject));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libRearObject", "1.1.0");
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
    AdApter_RearObject AdApter_RearObject_;
    AdApter_RearObject_.json_file = json_file;
   dds::pub::DataWriter<RearObjectModule::RearObject> writer = AdApter_RearObject_.topic_writer_init(AdApter_RearObject_.topicName);
    int domain_id = AdApter_RearObject_.domiain_id;
    std::string topic = AdApter_RearObject_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libRearObject", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_RearObject_.domiain_id,AdApter_RearObject_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_RearObject_.update_objInfo(data_in);
        writer.write(AdApter_RearObject_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_RearObject_.cycle_ms));
    }
    return 0;
    }
void AdApter_RearObject::run()
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
    AdApter_RearObject objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
