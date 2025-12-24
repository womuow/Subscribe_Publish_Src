#include"AdApter_CrossingObject.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<CrossingObjectModule::CrossingObject> AdApter_CrossingObject::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<CrossingObjectModule::CrossingObject> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<CrossingObjectModule::CrossingObject> writer(publisher, topic);
    return writer;
}
void AdApter_CrossingObject::update_objInfo(std::string data_str)
{
    CrossingObject CrossingObject_;
    std::memcpy(&CrossingObject_, data_str.data(), sizeof(CrossingObject));
    for (int i = 0; i < 30; i++)
    {
    topic_out.EgoStates()[i].latAcceleration() = CrossingObject_.EgoStates[i].latAcceleration;
    topic_out.EgoStates()[i].longAcceleration() = CrossingObject_.EgoStates[i].longAcceleration;
    topic_out.EgoStates()[i].latPosition() = CrossingObject_.EgoStates[i].latPosition;
    topic_out.EgoStates()[i].longPosition() = CrossingObject_.EgoStates[i].longPosition;
    topic_out.EgoStates()[i].latVelocity() = CrossingObject_.EgoStates[i].latVelocity;
    topic_out.EgoStates()[i].longVelocity() = CrossingObject_.EgoStates[i].longVelocity;
    }
    for (int i = 0; i < 30; i++)
    {
    topic_out.Properties()[i].id() = CrossingObject_.Properties[i].id;
    topic_out.Properties()[i].motionPattern() = CrossingObject_.Properties[i].motionPattern;
    topic_out.Properties()[i].referencePoint() = CrossingObject_.Properties[i].referencePoint;
    topic_out.Properties()[i].type() = CrossingObject_.Properties[i].type;
    topic_out.Properties()[i].width() = CrossingObject_.Properties[i].width;
    topic_out.Properties()[i].length() = CrossingObject_.Properties[i].length;
    topic_out.Properties()[i].cmbbQly() = CrossingObject_.Properties[i].cmbbQly;
    }
    topic_out.SequenceID() = CrossingObject_.SequenceID;
    data_in.resize(sizeof(CrossingObject));
    std::memcpy(&data_in[0], &CrossingObject_, sizeof(CrossingObject));
}
void AdApter_CrossingObject::maxeye_data_init()
{
}
AdApter_CrossingObject::AdApter_CrossingObject()
{
}
AdApter_CrossingObject::~AdApter_CrossingObject()
{
}
auto AdApter_CrossingObject::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(CrossingObject));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libCrossingObject", "1.1.0");
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
    AdApter_CrossingObject AdApter_CrossingObject_;
    AdApter_CrossingObject_.json_file = json_file;
   dds::pub::DataWriter<CrossingObjectModule::CrossingObject> writer = AdApter_CrossingObject_.topic_writer_init(AdApter_CrossingObject_.topicName);
    int domain_id = AdApter_CrossingObject_.domiain_id;
    std::string topic = AdApter_CrossingObject_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libCrossingObject", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_CrossingObject_.domiain_id,AdApter_CrossingObject_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_CrossingObject_.update_objInfo(data_in);
        writer.write(AdApter_CrossingObject_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_CrossingObject_.cycle_ms));
    }
    return 0;
    }
void AdApter_CrossingObject::run()
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
    AdApter_CrossingObject objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
