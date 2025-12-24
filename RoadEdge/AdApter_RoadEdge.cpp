#include"AdApter_RoadEdge.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<RoadEdgeModule::RoadEdge> AdApter_RoadEdge::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<RoadEdgeModule::RoadEdge> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<RoadEdgeModule::RoadEdge> writer(publisher, topic);
    return writer;
}
void AdApter_RoadEdge::update_objInfo(std::string data_str)
{
    RoadEdge RoadEdge_;
    std::memcpy(&RoadEdge_, data_str.data(), sizeof(RoadEdge));
    topic_out.LeftNonTrvsble().Edge().id() = RoadEdge_.LeftNonTrvsble.Edge.id;
    topic_out.LeftNonTrvsble().Edge().measurementQuality() = RoadEdge_.LeftNonTrvsble.Edge.measurementQuality;
    topic_out.LeftNonTrvsble().Edge().modelError() = RoadEdge_.LeftNonTrvsble.Edge.modelError;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().a() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().aVariance() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().b() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().bVariance() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().c() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().cVariance() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().d() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().d1Variance() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().d2Variance() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().longDistToEnd() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().longDistToStart() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().vrtOffset() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().vrtHeading() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().vrtCrvt() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt;
    topic_out.LeftNonTrvsble().Edge().SingClothoid().vrtCrvtRate() = RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate;
    topic_out.LeftNonTrvsble().Edge().trackingStatus() = RoadEdge_.LeftNonTrvsble.Edge.trackingStatus;
    topic_out.LeftNonTrvsble().Edge().valid() = RoadEdge_.LeftNonTrvsble.Edge.valid;
    topic_out.LeftNonTrvsble().Edge().isVerified() = RoadEdge_.LeftNonTrvsble.Edge.isVerified;
    topic_out.LeftNonTrvsble().Edge().selectionConfidence() = RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence;
    topic_out.RightNonTrvsble().Edge().id() = RoadEdge_.RightNonTrvsble.Edge.id;
    topic_out.RightNonTrvsble().Edge().measurementQuality() = RoadEdge_.RightNonTrvsble.Edge.measurementQuality;
    topic_out.RightNonTrvsble().Edge().modelError() = RoadEdge_.RightNonTrvsble.Edge.modelError;
    topic_out.RightNonTrvsble().Edge().SingClothoid().a() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a;
    topic_out.RightNonTrvsble().Edge().SingClothoid().aVariance() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance;
    topic_out.RightNonTrvsble().Edge().SingClothoid().b() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b;
    topic_out.RightNonTrvsble().Edge().SingClothoid().bVariance() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance;
    topic_out.RightNonTrvsble().Edge().SingClothoid().c() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c;
    topic_out.RightNonTrvsble().Edge().SingClothoid().cVariance() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance;
    topic_out.RightNonTrvsble().Edge().SingClothoid().d() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d;
    topic_out.RightNonTrvsble().Edge().SingClothoid().d1Variance() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance;
    topic_out.RightNonTrvsble().Edge().SingClothoid().d2Variance() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance;
    topic_out.RightNonTrvsble().Edge().SingClothoid().longDistToEnd() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd;
    topic_out.RightNonTrvsble().Edge().SingClothoid().longDistToStart() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart;
    topic_out.RightNonTrvsble().Edge().SingClothoid().vrtOffset() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset;
    topic_out.RightNonTrvsble().Edge().SingClothoid().vrtHeading() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading;
    topic_out.RightNonTrvsble().Edge().SingClothoid().vrtCrvt() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt;
    topic_out.RightNonTrvsble().Edge().SingClothoid().vrtCrvtRate() = RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate;
    topic_out.RightNonTrvsble().Edge().trackingStatus() = RoadEdge_.RightNonTrvsble.Edge.trackingStatus;
    topic_out.RightNonTrvsble().Edge().valid() = RoadEdge_.RightNonTrvsble.Edge.valid;
    topic_out.RightNonTrvsble().Edge().isVerified() = RoadEdge_.RightNonTrvsble.Edge.isVerified;
    topic_out.RightNonTrvsble().Edge().selectionConfidence() = RoadEdge_.RightNonTrvsble.Edge.selectionConfidence;
    topic_out.LeftTrvsble().Edge().id() = RoadEdge_.LeftTrvsble.Edge.id;
    topic_out.LeftTrvsble().Edge().measurementQuality() = RoadEdge_.LeftTrvsble.Edge.measurementQuality;
    topic_out.LeftTrvsble().Edge().modelError() = RoadEdge_.LeftTrvsble.Edge.modelError;
    topic_out.LeftTrvsble().Edge().SingClothoid().a() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.a;
    topic_out.LeftTrvsble().Edge().SingClothoid().aVariance() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance;
    topic_out.LeftTrvsble().Edge().SingClothoid().b() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.b;
    topic_out.LeftTrvsble().Edge().SingClothoid().bVariance() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance;
    topic_out.LeftTrvsble().Edge().SingClothoid().c() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.c;
    topic_out.LeftTrvsble().Edge().SingClothoid().cVariance() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance;
    topic_out.LeftTrvsble().Edge().SingClothoid().d() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.d;
    topic_out.LeftTrvsble().Edge().SingClothoid().d1Variance() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance;
    topic_out.LeftTrvsble().Edge().SingClothoid().d2Variance() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance;
    topic_out.LeftTrvsble().Edge().SingClothoid().longDistToEnd() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd;
    topic_out.LeftTrvsble().Edge().SingClothoid().longDistToStart() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart;
    topic_out.LeftTrvsble().Edge().SingClothoid().vrtOffset() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset;
    topic_out.LeftTrvsble().Edge().SingClothoid().vrtHeading() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading;
    topic_out.LeftTrvsble().Edge().SingClothoid().vrtCrvt() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt;
    topic_out.LeftTrvsble().Edge().SingClothoid().vrtCrvtRate() = RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate;
    topic_out.LeftTrvsble().Edge().trackingStatus() = RoadEdge_.LeftTrvsble.Edge.trackingStatus;
    topic_out.LeftTrvsble().Edge().valid() = RoadEdge_.LeftTrvsble.Edge.valid;
    topic_out.LeftTrvsble().Edge().isVerified() = RoadEdge_.LeftTrvsble.Edge.isVerified;
    topic_out.LeftTrvsble().Edge().selectionConfidence() = RoadEdge_.LeftTrvsble.Edge.selectionConfidence;
    topic_out.RightTrvsble().Edge().id() = RoadEdge_.RightTrvsble.Edge.id;
    topic_out.RightTrvsble().Edge().measurementQuality() = RoadEdge_.RightTrvsble.Edge.measurementQuality;
    topic_out.RightTrvsble().Edge().modelError() = RoadEdge_.RightTrvsble.Edge.modelError;
    topic_out.RightTrvsble().Edge().SingClothoid().a() = RoadEdge_.RightTrvsble.Edge.SingClothoid.a;
    topic_out.RightTrvsble().Edge().SingClothoid().aVariance() = RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance;
    topic_out.RightTrvsble().Edge().SingClothoid().b() = RoadEdge_.RightTrvsble.Edge.SingClothoid.b;
    topic_out.RightTrvsble().Edge().SingClothoid().bVariance() = RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance;
    topic_out.RightTrvsble().Edge().SingClothoid().c() = RoadEdge_.RightTrvsble.Edge.SingClothoid.c;
    topic_out.RightTrvsble().Edge().SingClothoid().cVariance() = RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance;
    topic_out.RightTrvsble().Edge().SingClothoid().d() = RoadEdge_.RightTrvsble.Edge.SingClothoid.d;
    topic_out.RightTrvsble().Edge().SingClothoid().d1Variance() = RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance;
    topic_out.RightTrvsble().Edge().SingClothoid().d2Variance() = RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance;
    topic_out.RightTrvsble().Edge().SingClothoid().longDistToEnd() = RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd;
    topic_out.RightTrvsble().Edge().SingClothoid().longDistToStart() = RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart;
    topic_out.RightTrvsble().Edge().SingClothoid().vrtOffset() = RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset;
    topic_out.RightTrvsble().Edge().SingClothoid().vrtHeading() = RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading;
    topic_out.RightTrvsble().Edge().SingClothoid().vrtCrvt() = RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt;
    topic_out.RightTrvsble().Edge().SingClothoid().vrtCrvtRate() = RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate;
    topic_out.RightTrvsble().Edge().trackingStatus() = RoadEdge_.RightTrvsble.Edge.trackingStatus;
    topic_out.RightTrvsble().Edge().valid() = RoadEdge_.RightTrvsble.Edge.valid;
    topic_out.RightTrvsble().Edge().isVerified() = RoadEdge_.RightTrvsble.Edge.isVerified;
    topic_out.RightTrvsble().Edge().selectionConfidence() = RoadEdge_.RightTrvsble.Edge.selectionConfidence;
    topic_out.SequenceID() = RoadEdge_.SequenceID;
    data_in.resize(sizeof(RoadEdge));
    std::memcpy(&data_in[0], &RoadEdge_, sizeof(RoadEdge));
}
void AdApter_RoadEdge::maxeye_data_init()
{
}
AdApter_RoadEdge::AdApter_RoadEdge()
{
}
AdApter_RoadEdge::~AdApter_RoadEdge()
{
}
auto AdApter_RoadEdge::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(RoadEdge));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libRoadEdge", "1.1.0");
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
    AdApter_RoadEdge AdApter_RoadEdge_;
    AdApter_RoadEdge_.json_file = json_file;
   dds::pub::DataWriter<RoadEdgeModule::RoadEdge> writer = AdApter_RoadEdge_.topic_writer_init(AdApter_RoadEdge_.topicName);
    int domain_id = AdApter_RoadEdge_.domiain_id;
    std::string topic = AdApter_RoadEdge_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libRoadEdge", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_RoadEdge_.domiain_id,AdApter_RoadEdge_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_RoadEdge_.update_objInfo(data_in);
        writer.write(AdApter_RoadEdge_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_RoadEdge_.cycle_ms));
    }
    return 0;
    }
void AdApter_RoadEdge::run()
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
    AdApter_RoadEdge objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
