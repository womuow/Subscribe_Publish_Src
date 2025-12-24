#include"AdApter_FusedFrontObject.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<FusedFrontObjectModule::FusedFrontObject> AdApter_FusedFrontObject::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<FusedFrontObjectModule::FusedFrontObject> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<FusedFrontObjectModule::FusedFrontObject> writer(publisher, topic);
    return writer;
}
void AdApter_FusedFrontObject::update_objInfo(std::string data_str)
{
    FusedFrontObject FusedFrontObject_;
    std::memcpy(&FusedFrontObject_, data_str.data(), sizeof(FusedFrontObject));
    for (int i = 0; i < 32; i++)
    {
    topic_out.Properties()[i].accelerationStdDev() = FusedFrontObject_.Properties[i].accelerationStdDev;
    topic_out.Properties()[i].brakeLight() = FusedFrontObject_.Properties[i].brakeLight;
    topic_out.Properties()[i].classificationConfidence() = FusedFrontObject_.Properties[i].classificationConfidence;
    topic_out.Properties()[i].cmsConfidence() = FusedFrontObject_.Properties[i].cmsConfidence;
    topic_out.Properties()[i].cmbbSecConfidence() = FusedFrontObject_.Properties[i].cmbbSecConfidence;
    topic_out.Properties()[i].distanceToLeftNearLaneMarking() = FusedFrontObject_.Properties[i].distanceToLeftNearLaneMarking;
    topic_out.Properties()[i].distanceToRightNearLaneMarking() = FusedFrontObject_.Properties[i].distanceToRightNearLaneMarking;
    topic_out.Properties()[i].elkaQly() = FusedFrontObject_.Properties[i].elkaQly;
    topic_out.Properties()[i].existenceConfidence() = FusedFrontObject_.Properties[i].existenceConfidence;
    topic_out.Properties()[i].fcwQly() = FusedFrontObject_.Properties[i].fcwQly;
    topic_out.Properties()[i].fusionSource() = FusedFrontObject_.Properties[i].fusionSource;
    topic_out.Properties()[i].hazardLightStatus() = FusedFrontObject_.Properties[i].hazardLightStatus;
    topic_out.Properties()[i].headingStdDev() = FusedFrontObject_.Properties[i].headingStdDev;
    topic_out.Properties()[i].id() = FusedFrontObject_.Properties[i].id;
    topic_out.Properties()[i].innovationFactor() = FusedFrontObject_.Properties[i].innovationFactor;
    topic_out.Properties()[i].latPositionStdDev() = FusedFrontObject_.Properties[i].latPositionStdDev;
    topic_out.Properties()[i].leftNearLaneMarkingConfidence() = FusedFrontObject_.Properties[i].leftNearLaneMarkingConfidence;
    topic_out.Properties()[i].longPositionStdDev() = FusedFrontObject_.Properties[i].longPositionStdDev;
    topic_out.Properties()[i].motionHistory() = FusedFrontObject_.Properties[i].motionHistory;
    topic_out.Properties()[i].motionModel() = FusedFrontObject_.Properties[i].motionModel;
    topic_out.Properties()[i].motionPattern() = FusedFrontObject_.Properties[i].motionPattern;
    topic_out.Properties()[i].radarId() = FusedFrontObject_.Properties[i].radarId;
    topic_out.Properties()[i].referencePoint() = FusedFrontObject_.Properties[i].referencePoint;
    topic_out.Properties()[i].reserved() = FusedFrontObject_.Properties[i].reserved;
    topic_out.Properties()[i].rightNearLaneMarkingConfidence() = FusedFrontObject_.Properties[i].rightNearLaneMarkingConfidence;
    topic_out.Properties()[i].speedStdDev() = FusedFrontObject_.Properties[i].speedStdDev;
    topic_out.Properties()[i].trackStatus() = FusedFrontObject_.Properties[i].trackStatus;
    topic_out.Properties()[i].trafficScenario() = FusedFrontObject_.Properties[i].trafficScenario;
    topic_out.Properties()[i].turnIndicator() = FusedFrontObject_.Properties[i].turnIndicator;
    topic_out.Properties()[i].type() = FusedFrontObject_.Properties[i].type;
    topic_out.Properties()[i].visionId() = FusedFrontObject_.Properties[i].visionId;
    topic_out.Properties()[i].width() = FusedFrontObject_.Properties[i].width;
    topic_out.Properties()[i].length() = FusedFrontObject_.Properties[i].length;
    topic_out.Properties()[i].SensorUpdateStatus() = FusedFrontObject_.Properties[i].SensorUpdateStatus;
    }
    topic_out.SequenceID() = FusedFrontObject_.SequenceID;
    for (int i = 0; i < 32; i++)
    {
    topic_out.States()[i].acceleration() = FusedFrontObject_.States[i].acceleration;
    topic_out.States()[i].curvature() = FusedFrontObject_.States[i].curvature;
    topic_out.States()[i].heading() = FusedFrontObject_.States[i].heading;
    topic_out.States()[i].latAcceleration() = FusedFrontObject_.States[i].latAcceleration;
    topic_out.States()[i].latPosition() = FusedFrontObject_.States[i].latPosition;
    topic_out.States()[i].latVelocity() = FusedFrontObject_.States[i].latVelocity;
    topic_out.States()[i].longAcceleration() = FusedFrontObject_.States[i].longAcceleration;
    topic_out.States()[i].longPosition() = FusedFrontObject_.States[i].longPosition;
    topic_out.States()[i].longVelocity() = FusedFrontObject_.States[i].longVelocity;
    topic_out.States()[i].speed() = FusedFrontObject_.States[i].speed;
    }
    data_in.resize(sizeof(FusedFrontObject));
    std::memcpy(&data_in[0], &FusedFrontObject_, sizeof(FusedFrontObject));
}
void AdApter_FusedFrontObject::maxeye_data_init()
{
}
AdApter_FusedFrontObject::AdApter_FusedFrontObject()
{
}
AdApter_FusedFrontObject::~AdApter_FusedFrontObject()
{
}
auto AdApter_FusedFrontObject::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(FusedFrontObject));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libFusedFrontObject", "1.1.0");
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
    AdApter_FusedFrontObject AdApter_FusedFrontObject_;
    AdApter_FusedFrontObject_.json_file = json_file;
   dds::pub::DataWriter<FusedFrontObjectModule::FusedFrontObject> writer = AdApter_FusedFrontObject_.topic_writer_init(AdApter_FusedFrontObject_.topicName);
    int domain_id = AdApter_FusedFrontObject_.domiain_id;
    std::string topic = AdApter_FusedFrontObject_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libFusedFrontObject", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_FusedFrontObject_.domiain_id,AdApter_FusedFrontObject_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_FusedFrontObject_.update_objInfo(data_in);
        writer.write(AdApter_FusedFrontObject_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_FusedFrontObject_.cycle_ms));
    }
    return 0;
    }
void AdApter_FusedFrontObject::run()
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
    AdApter_FusedFrontObject objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
