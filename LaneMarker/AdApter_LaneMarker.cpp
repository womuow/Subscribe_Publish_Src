#include"AdApter_LaneMarker.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<LaneMarkerModule::LaneMarker> AdApter_LaneMarker::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<LaneMarkerModule::LaneMarker> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<LaneMarkerModule::LaneMarker> writer(publisher, topic);
    return writer;
}
void AdApter_LaneMarker::update_objInfo(std::string data_str)
{
    LaneMarker LaneMarker_;
    std::memcpy(&LaneMarker_, data_str.data(), sizeof(LaneMarker));
    topic_out.EgoLane().Left().Lane().color() = LaneMarker_.EgoLane.Left.Lane.color;
    topic_out.EgoLane().Left().Lane().DblClothoid().a() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.a;
    topic_out.EgoLane().Left().Lane().DblClothoid().b() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.b;
    topic_out.EgoLane().Left().Lane().DblClothoid().c() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.c;
    topic_out.EgoLane().Left().Lane().DblClothoid().d1() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1;
    topic_out.EgoLane().Left().Lane().DblClothoid().d2() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2;
    topic_out.EgoLane().Left().Lane().DblClothoid().longDistToEnd() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd;
    topic_out.EgoLane().Left().Lane().DblClothoid().longDistToStart() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart;
    topic_out.EgoLane().Left().Lane().DblClothoid().secClothoidActive() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive;
    topic_out.EgoLane().Left().Lane().DblClothoid().transitionDist() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist;
    topic_out.EgoLane().Left().Lane().DblClothoid().aVariance() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance;
    topic_out.EgoLane().Left().Lane().DblClothoid().bVariance() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance;
    topic_out.EgoLane().Left().Lane().DblClothoid().cVariance() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance;
    topic_out.EgoLane().Left().Lane().DblClothoid().d1Variance() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance;
    topic_out.EgoLane().Left().Lane().DblClothoid().d2Variance() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance;
    topic_out.EgoLane().Left().Lane().DblClothoid().vrtOffset() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset;
    topic_out.EgoLane().Left().Lane().DblClothoid().vrtHeading() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading;
    topic_out.EgoLane().Left().Lane().DblClothoid().vrtCrvt() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt;
    topic_out.EgoLane().Left().Lane().DblClothoid().vrtCrvtRate() = LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate;
    topic_out.EgoLane().Left().Lane().id() = LaneMarker_.EgoLane.Left.Lane.id;
    topic_out.EgoLane().Left().Lane().markingType() = LaneMarker_.EgoLane.Left.Lane.markingType;
    topic_out.EgoLane().Left().Lane().markingWidth() = LaneMarker_.EgoLane.Left.Lane.markingWidth;
    topic_out.EgoLane().Left().Lane().measurementQuality() = LaneMarker_.EgoLane.Left.Lane.measurementQuality;
    topic_out.EgoLane().Left().Lane().modelError() = LaneMarker_.EgoLane.Left.Lane.modelError;
    topic_out.EgoLane().Left().Lane().secondMarkingType() = LaneMarker_.EgoLane.Left.Lane.secondMarkingType;
    topic_out.EgoLane().Left().Lane().selectionConfidence() = LaneMarker_.EgoLane.Left.Lane.selectionConfidence;
    topic_out.EgoLane().Left().Lane().structure() = LaneMarker_.EgoLane.Left.Lane.structure;
    topic_out.EgoLane().Left().Lane().totalMarkingWidth() = LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth;
    topic_out.EgoLane().Left().Lane().trackingStatus() = LaneMarker_.EgoLane.Left.Lane.trackingStatus;
    topic_out.EgoLane().Left().Lane().valid() = LaneMarker_.EgoLane.Left.Lane.valid;
    topic_out.EgoLane().Left().Lane().isVerified() = LaneMarker_.EgoLane.Left.Lane.isVerified;
    topic_out.EgoLane().Right().Lane().color() = LaneMarker_.EgoLane.Right.Lane.color;
    topic_out.EgoLane().Right().Lane().DblClothoid().a() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.a;
    topic_out.EgoLane().Right().Lane().DblClothoid().b() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.b;
    topic_out.EgoLane().Right().Lane().DblClothoid().c() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.c;
    topic_out.EgoLane().Right().Lane().DblClothoid().d1() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1;
    topic_out.EgoLane().Right().Lane().DblClothoid().d2() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2;
    topic_out.EgoLane().Right().Lane().DblClothoid().longDistToEnd() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd;
    topic_out.EgoLane().Right().Lane().DblClothoid().longDistToStart() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart;
    topic_out.EgoLane().Right().Lane().DblClothoid().secClothoidActive() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive;
    topic_out.EgoLane().Right().Lane().DblClothoid().transitionDist() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist;
    topic_out.EgoLane().Right().Lane().DblClothoid().aVariance() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance;
    topic_out.EgoLane().Right().Lane().DblClothoid().bVariance() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance;
    topic_out.EgoLane().Right().Lane().DblClothoid().cVariance() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance;
    topic_out.EgoLane().Right().Lane().DblClothoid().d1Variance() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance;
    topic_out.EgoLane().Right().Lane().DblClothoid().d2Variance() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance;
    topic_out.EgoLane().Right().Lane().DblClothoid().vrtOffset() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset;
    topic_out.EgoLane().Right().Lane().DblClothoid().vrtHeading() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading;
    topic_out.EgoLane().Right().Lane().DblClothoid().vrtCrvt() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt;
    topic_out.EgoLane().Right().Lane().DblClothoid().vrtCrvtRate() = LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate;
    topic_out.EgoLane().Right().Lane().id() = LaneMarker_.EgoLane.Right.Lane.id;
    topic_out.EgoLane().Right().Lane().markingType() = LaneMarker_.EgoLane.Right.Lane.markingType;
    topic_out.EgoLane().Right().Lane().markingWidth() = LaneMarker_.EgoLane.Right.Lane.markingWidth;
    topic_out.EgoLane().Right().Lane().measurementQuality() = LaneMarker_.EgoLane.Right.Lane.measurementQuality;
    topic_out.EgoLane().Right().Lane().modelError() = LaneMarker_.EgoLane.Right.Lane.modelError;
    topic_out.EgoLane().Right().Lane().secondMarkingType() = LaneMarker_.EgoLane.Right.Lane.secondMarkingType;
    topic_out.EgoLane().Right().Lane().selectionConfidence() = LaneMarker_.EgoLane.Right.Lane.selectionConfidence;
    topic_out.EgoLane().Right().Lane().structure() = LaneMarker_.EgoLane.Right.Lane.structure;
    topic_out.EgoLane().Right().Lane().totalMarkingWidth() = LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth;
    topic_out.EgoLane().Right().Lane().trackingStatus() = LaneMarker_.EgoLane.Right.Lane.trackingStatus;
    topic_out.EgoLane().Right().Lane().valid() = LaneMarker_.EgoLane.Right.Lane.valid;
    topic_out.EgoLane().Right().Lane().isVerified() = LaneMarker_.EgoLane.Right.Lane.isVerified;
    topic_out.EgoLane().parallelDistance() = LaneMarker_.EgoLane.parallelDistance;
    topic_out.EgoLane().validProjectionDistance() = LaneMarker_.EgoLane.validProjectionDistance;
    topic_out.EgoLane().attentionMarkerDetected() = LaneMarker_.EgoLane.attentionMarkerDetected;
    topic_out.DebugBus().debugFloat1() = LaneMarker_.DebugBus.debugFloat1;
    topic_out.DebugBus().debugFloat2() = LaneMarker_.DebugBus.debugFloat2;
    topic_out.DebugBus().debugFloat3() = LaneMarker_.DebugBus.debugFloat3;
    topic_out.DebugBus().debugFloat4() = LaneMarker_.DebugBus.debugFloat4;
    topic_out.DebugBus().debugFloat5() = LaneMarker_.DebugBus.debugFloat5;
    topic_out.DebugBus().debugFloat6() = LaneMarker_.DebugBus.debugFloat6;
    topic_out.DebugBus().debugFloat7() = LaneMarker_.DebugBus.debugFloat7;
    topic_out.DebugBus().debugFloat8() = LaneMarker_.DebugBus.debugFloat8;
    topic_out.DebugBus().debugInteger1() = LaneMarker_.DebugBus.debugInteger1;
    topic_out.DebugBus().debugInteger2() = LaneMarker_.DebugBus.debugInteger2;
    topic_out.DebugBus().debugInteger3() = LaneMarker_.DebugBus.debugInteger3;
    topic_out.DebugBus().debugInteger4() = LaneMarker_.DebugBus.debugInteger4;
    topic_out.SecClsLeft().Lane().color() = LaneMarker_.SecClsLeft.Lane.color;
    topic_out.SecClsLeft().Lane().DblClothoid().a() = LaneMarker_.SecClsLeft.Lane.DblClothoid.a;
    topic_out.SecClsLeft().Lane().DblClothoid().b() = LaneMarker_.SecClsLeft.Lane.DblClothoid.b;
    topic_out.SecClsLeft().Lane().DblClothoid().c() = LaneMarker_.SecClsLeft.Lane.DblClothoid.c;
    topic_out.SecClsLeft().Lane().DblClothoid().d1() = LaneMarker_.SecClsLeft.Lane.DblClothoid.d1;
    topic_out.SecClsLeft().Lane().DblClothoid().d2() = LaneMarker_.SecClsLeft.Lane.DblClothoid.d2;
    topic_out.SecClsLeft().Lane().DblClothoid().longDistToEnd() = LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd;
    topic_out.SecClsLeft().Lane().DblClothoid().longDistToStart() = LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart;
    topic_out.SecClsLeft().Lane().DblClothoid().secClothoidActive() = LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive;
    topic_out.SecClsLeft().Lane().DblClothoid().transitionDist() = LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist;
    topic_out.SecClsLeft().Lane().DblClothoid().aVariance() = LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance;
    topic_out.SecClsLeft().Lane().DblClothoid().bVariance() = LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance;
    topic_out.SecClsLeft().Lane().DblClothoid().cVariance() = LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance;
    topic_out.SecClsLeft().Lane().DblClothoid().d1Variance() = LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance;
    topic_out.SecClsLeft().Lane().DblClothoid().d2Variance() = LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance;
    topic_out.SecClsLeft().Lane().DblClothoid().vrtOffset() = LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset;
    topic_out.SecClsLeft().Lane().DblClothoid().vrtHeading() = LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading;
    topic_out.SecClsLeft().Lane().DblClothoid().vrtCrvt() = LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt;
    topic_out.SecClsLeft().Lane().DblClothoid().vrtCrvtRate() = LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate;
    topic_out.SecClsLeft().Lane().id() = LaneMarker_.SecClsLeft.Lane.id;
    topic_out.SecClsLeft().Lane().markingType() = LaneMarker_.SecClsLeft.Lane.markingType;
    topic_out.SecClsLeft().Lane().markingWidth() = LaneMarker_.SecClsLeft.Lane.markingWidth;
    topic_out.SecClsLeft().Lane().measurementQuality() = LaneMarker_.SecClsLeft.Lane.measurementQuality;
    topic_out.SecClsLeft().Lane().modelError() = LaneMarker_.SecClsLeft.Lane.modelError;
    topic_out.SecClsLeft().Lane().secondMarkingType() = LaneMarker_.SecClsLeft.Lane.secondMarkingType;
    topic_out.SecClsLeft().Lane().selectionConfidence() = LaneMarker_.SecClsLeft.Lane.selectionConfidence;
    topic_out.SecClsLeft().Lane().structure() = LaneMarker_.SecClsLeft.Lane.structure;
    topic_out.SecClsLeft().Lane().totalMarkingWidth() = LaneMarker_.SecClsLeft.Lane.totalMarkingWidth;
    topic_out.SecClsLeft().Lane().trackingStatus() = LaneMarker_.SecClsLeft.Lane.trackingStatus;
    topic_out.SecClsLeft().Lane().valid() = LaneMarker_.SecClsLeft.Lane.valid;
    topic_out.SecClsLeft().Lane().isVerified() = LaneMarker_.SecClsLeft.Lane.isVerified;
    topic_out.SecClsRight().Lane().color() = LaneMarker_.SecClsRight.Lane.color;
    topic_out.SecClsRight().Lane().DblClothoid().a() = LaneMarker_.SecClsRight.Lane.DblClothoid.a;
    topic_out.SecClsRight().Lane().DblClothoid().b() = LaneMarker_.SecClsRight.Lane.DblClothoid.b;
    topic_out.SecClsRight().Lane().DblClothoid().c() = LaneMarker_.SecClsRight.Lane.DblClothoid.c;
    topic_out.SecClsRight().Lane().DblClothoid().d1() = LaneMarker_.SecClsRight.Lane.DblClothoid.d1;
    topic_out.SecClsRight().Lane().DblClothoid().d2() = LaneMarker_.SecClsRight.Lane.DblClothoid.d2;
    topic_out.SecClsRight().Lane().DblClothoid().longDistToEnd() = LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd;
    topic_out.SecClsRight().Lane().DblClothoid().longDistToStart() = LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart;
    topic_out.SecClsRight().Lane().DblClothoid().secClothoidActive() = LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive;
    topic_out.SecClsRight().Lane().DblClothoid().transitionDist() = LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist;
    topic_out.SecClsRight().Lane().DblClothoid().aVariance() = LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance;
    topic_out.SecClsRight().Lane().DblClothoid().bVariance() = LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance;
    topic_out.SecClsRight().Lane().DblClothoid().cVariance() = LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance;
    topic_out.SecClsRight().Lane().DblClothoid().d1Variance() = LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance;
    topic_out.SecClsRight().Lane().DblClothoid().d2Variance() = LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance;
    topic_out.SecClsRight().Lane().DblClothoid().vrtOffset() = LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset;
    topic_out.SecClsRight().Lane().DblClothoid().vrtHeading() = LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading;
    topic_out.SecClsRight().Lane().DblClothoid().vrtCrvt() = LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt;
    topic_out.SecClsRight().Lane().DblClothoid().vrtCrvtRate() = LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate;
    topic_out.SecClsRight().Lane().id() = LaneMarker_.SecClsRight.Lane.id;
    topic_out.SecClsRight().Lane().markingType() = LaneMarker_.SecClsRight.Lane.markingType;
    topic_out.SecClsRight().Lane().markingWidth() = LaneMarker_.SecClsRight.Lane.markingWidth;
    topic_out.SecClsRight().Lane().measurementQuality() = LaneMarker_.SecClsRight.Lane.measurementQuality;
    topic_out.SecClsRight().Lane().modelError() = LaneMarker_.SecClsRight.Lane.modelError;
    topic_out.SecClsRight().Lane().secondMarkingType() = LaneMarker_.SecClsRight.Lane.secondMarkingType;
    topic_out.SecClsRight().Lane().selectionConfidence() = LaneMarker_.SecClsRight.Lane.selectionConfidence;
    topic_out.SecClsRight().Lane().structure() = LaneMarker_.SecClsRight.Lane.structure;
    topic_out.SecClsRight().Lane().totalMarkingWidth() = LaneMarker_.SecClsRight.Lane.totalMarkingWidth;
    topic_out.SecClsRight().Lane().trackingStatus() = LaneMarker_.SecClsRight.Lane.trackingStatus;
    topic_out.SecClsRight().Lane().valid() = LaneMarker_.SecClsRight.Lane.valid;
    topic_out.SecClsRight().Lane().isVerified() = LaneMarker_.SecClsRight.Lane.isVerified;
    for (int i = 0; i < 4; i++)
    {
    topic_out.LaneEvent()[i].distance() = LaneMarker_.LaneEvent[i].distance;
    topic_out.LaneEvent()[i].eventType() = LaneMarker_.LaneEvent[i].eventType;
    topic_out.LaneEvent()[i].id() = LaneMarker_.LaneEvent[i].id;
    topic_out.LaneEvent()[i].laneTrack() = LaneMarker_.LaneEvent[i].laneTrack;
    }
    topic_out.TemporaryMarking().longDistanceToStart() = LaneMarker_.TemporaryMarking.longDistanceToStart;
    topic_out.TemporaryMarking().type() = LaneMarker_.TemporaryMarking.type;
    topic_out.sideSuggestion() = LaneMarker_.sideSuggestion;
    topic_out.laneChange() = LaneMarker_.laneChange;
    topic_out.SequenceID() = LaneMarker_.SequenceID;
    data_in.resize(sizeof(LaneMarker));
    std::memcpy(&data_in[0], &LaneMarker_, sizeof(LaneMarker));
}
void AdApter_LaneMarker::maxeye_data_init()
{
}
AdApter_LaneMarker::AdApter_LaneMarker()
{
}
AdApter_LaneMarker::~AdApter_LaneMarker()
{
}
auto AdApter_LaneMarker::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LaneMarker));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libLaneMarker", "1.1.0");
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
    AdApter_LaneMarker AdApter_LaneMarker_;
    AdApter_LaneMarker_.json_file = json_file;
   dds::pub::DataWriter<LaneMarkerModule::LaneMarker> writer = AdApter_LaneMarker_.topic_writer_init(AdApter_LaneMarker_.topicName);
    int domain_id = AdApter_LaneMarker_.domiain_id;
    std::string topic = AdApter_LaneMarker_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libLaneMarker", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_LaneMarker_.domiain_id,AdApter_LaneMarker_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_LaneMarker_.update_objInfo(data_in);
        writer.write(AdApter_LaneMarker_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_LaneMarker_.cycle_ms));
    }
    return 0;
    }
void AdApter_LaneMarker::run()
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
    AdApter_LaneMarker objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
