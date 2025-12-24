#include"AdApter_LDP_Obj.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<LDP_ObjModule::LDP_Obj> AdApter_LDP_Obj::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<LDP_ObjModule::LDP_Obj> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<LDP_ObjModule::LDP_Obj> writer(publisher, topic);
    return writer;
}
void AdApter_LDP_Obj::update_objInfo(std::string data_str)
{
    LDP_Obj LDP_Obj_;
    std::memcpy(&LDP_Obj_, data_str.data(), sizeof(LDP_Obj));
    topic_out.CritLatCdn_PrimaryTarget().interventionType() = LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType;
    topic_out.CritLatCdn_PrimaryTarget().objectType() = LDP_Obj_.CritLatCdn_PrimaryTarget.objectType;
    topic_out.CritLatCdn_PrimaryTarget().referencePoint() = LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint;
    topic_out.CritLatCdn_PrimaryTarget().length() = LDP_Obj_.CritLatCdn_PrimaryTarget.length;
    topic_out.CritLatCdn_PrimaryTarget().width() = LDP_Obj_.CritLatCdn_PrimaryTarget.width;
    topic_out.CritLatCdn_PrimaryTarget().heading() = LDP_Obj_.CritLatCdn_PrimaryTarget.heading;
    topic_out.CritLatCdn_PrimaryTarget().timeToCollision() = LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision;
    topic_out.CritLatCdn_PrimaryTarget().timeToReach() = LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach;
    topic_out.CritLatCdn_PrimaryTarget().timeOutsideEgoLane() = LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane;
    topic_out.CritLatCdn_PrimaryTarget().latPositionAtTimeToCollision() = LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision;
    topic_out.CritLatCdn_PrimaryTarget().lgtPosition() = LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition;
    topic_out.CritLatCdn_PrimaryTarget().latPosition() = LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition;
    topic_out.CritLatCdn_PrimaryTarget().lgtVelocity() = LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity;
    topic_out.CritLatCdn_PrimaryTarget().latVelocity() = LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity;
    topic_out.CritLatCdn_PrimaryTarget().targetInEgoLane() = LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane;
    topic_out.CritLatCdn_PrimaryTarget().lgtAccRqrdForPrimTarToAvdSelf() = LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf;
    topic_out.CritLatCdn_PrimaryTarget().egoLatAccRequiredForAvoidance() = LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance;
    topic_out.CritLatCdn_PrimaryTarget().exists() = LDP_Obj_.CritLatCdn_PrimaryTarget.exists;
    topic_out.secondaryObstacleInEgoLane() = LDP_Obj_.secondaryObstacleInEgoLane;
    topic_out.secondaryObstacleInLeftLane() = LDP_Obj_.secondaryObstacleInLeftLane;
    topic_out.secondaryObstacleInRightLane() = LDP_Obj_.secondaryObstacleInRightLane;
    data_in.resize(sizeof(LDP_Obj));
    std::memcpy(&data_in[0], &LDP_Obj_, sizeof(LDP_Obj));
}
void AdApter_LDP_Obj::maxeye_data_init()
{
}
AdApter_LDP_Obj::AdApter_LDP_Obj()
{
}
AdApter_LDP_Obj::~AdApter_LDP_Obj()
{
}
auto AdApter_LDP_Obj::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LDP_Obj));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libLDP_Obj", "1.1.0");
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
    AdApter_LDP_Obj AdApter_LDP_Obj_;
    AdApter_LDP_Obj_.json_file = json_file;
   dds::pub::DataWriter<LDP_ObjModule::LDP_Obj> writer = AdApter_LDP_Obj_.topic_writer_init(AdApter_LDP_Obj_.topicName);
    int domain_id = AdApter_LDP_Obj_.domiain_id;
    std::string topic = AdApter_LDP_Obj_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libLDP_Obj", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_LDP_Obj_.domiain_id,AdApter_LDP_Obj_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_LDP_Obj_.update_objInfo(data_in);
        writer.write(AdApter_LDP_Obj_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_LDP_Obj_.cycle_ms));
    }
    return 0;
    }
void AdApter_LDP_Obj::run()
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
    AdApter_LDP_Obj objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
