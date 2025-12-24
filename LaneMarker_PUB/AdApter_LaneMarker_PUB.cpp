#include"AdApter_LaneMarker_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}



AdApter_LaneMarkerPUB::AdApter_LaneMarkerPUB()
{
}
AdApter_LaneMarkerPUB::~AdApter_LaneMarkerPUB()
{
}
auto AdApter_LaneMarkerPUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_LaneMarkerPUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(LaneMarker));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libLaneMarkerPUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    LaneMarker LaneMarker_;
    setIntialValue_LaneMarker(LaneMarker_);

    while (true) {        
        data_in.resize(sizeof(LaneMarker_));
        std::memcpy(&data_in[0], &LaneMarker_, sizeof(LaneMarker_));
        auto data_ref = std::make_shared<MOS::message::DataRef>(const_cast<char*>(data_in.data()), data_in.size());
        mos_msg->SetDataRef(data_ref);
        auto now_time = MOS::TimeUtils::NowNsec();
        mos_msg->SetGenTimestamp(now_time);
        pub->Pub(mos_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms));//40ms
    }
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
    std::cout << "Running LaneMarker_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_LaneMarkerPUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}



/* Set and Print struct LaneMarker initial value */
void setIntialValue_LaneMarker(LaneMarker& LaneMarker_){
    std::cout << "Set struct LaneMarker variable and Publish:" << std::endl;
    LaneMarker_.EgoLane.Left.Lane.color = 1;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.color) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.a = 1.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.a(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.a << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.b = 2.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.b(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.b << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.c = 3.30;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.c(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.c << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1 = 4.40;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2 = 5.50;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd = 6.60;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart = 7.70;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive = 2;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist = 8.80;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance = 9.90;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance = 11.00;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance = 12.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance = 13.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance = 14.30;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset = 15.40;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading = 16.50;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt = 17.60;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate = 18.70;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.EgoLane.Left.Lane.id = 3;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.id) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.markingType = 4;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.markingWidth = 19.80;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.markingWidth(float32): " << LaneMarker_.EgoLane.Left.Lane.markingWidth << std::endl;
    LaneMarker_.EgoLane.Left.Lane.measurementQuality = 20.90;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.measurementQuality(float32): " << LaneMarker_.EgoLane.Left.Lane.measurementQuality << std::endl;
    LaneMarker_.EgoLane.Left.Lane.modelError = 22.00;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.modelError(float32): " << LaneMarker_.EgoLane.Left.Lane.modelError << std::endl;
    LaneMarker_.EgoLane.Left.Lane.secondMarkingType = 5;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.selectionConfidence = 23.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.selectionConfidence(float32): " << LaneMarker_.EgoLane.Left.Lane.selectionConfidence << std::endl;
    LaneMarker_.EgoLane.Left.Lane.structure = 6;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth = 24.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth(float32): " << LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.EgoLane.Left.Lane.trackingStatus = 7;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.valid = 8;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.isVerified = 9;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.color = 10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.color) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.a = 25.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.a(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.a << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.b = 26.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.b(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.b << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.c = 27.50;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.c(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.c << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1 = 28.60;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2 = 29.70;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd = 30.80;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart = 31.90;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive = 11;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist = 33.00;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance = 34.10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance = 35.20;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance = 36.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance = 37.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance = 38.50;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset = 39.60;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading = 40.70;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt = 41.80;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate = 42.90;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.EgoLane.Right.Lane.id = 12;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.id) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.markingType = 13;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.markingWidth = 44.00;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.markingWidth(float32): " << LaneMarker_.EgoLane.Right.Lane.markingWidth << std::endl;
    LaneMarker_.EgoLane.Right.Lane.measurementQuality = 45.10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.measurementQuality(float32): " << LaneMarker_.EgoLane.Right.Lane.measurementQuality << std::endl;
    LaneMarker_.EgoLane.Right.Lane.modelError = 46.20;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.modelError(float32): " << LaneMarker_.EgoLane.Right.Lane.modelError << std::endl;
    LaneMarker_.EgoLane.Right.Lane.secondMarkingType = 14;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.selectionConfidence = 47.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.selectionConfidence(float32): " << LaneMarker_.EgoLane.Right.Lane.selectionConfidence << std::endl;
    LaneMarker_.EgoLane.Right.Lane.structure = 15;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth = 48.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth(float32): " << LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.EgoLane.Right.Lane.trackingStatus = 16;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.valid = 17;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.isVerified = 18;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.EgoLane.parallelDistance = 49.50;
    std::cout << "LaneMarker_.EgoLane.parallelDistance(float32): " << LaneMarker_.EgoLane.parallelDistance << std::endl;
    LaneMarker_.EgoLane.validProjectionDistance = 50.60;
    std::cout << "LaneMarker_.EgoLane.validProjectionDistance(float32): " << LaneMarker_.EgoLane.validProjectionDistance << std::endl;
    LaneMarker_.EgoLane.attentionMarkerDetected = 19;
    std::cout << "LaneMarker_.EgoLane.attentionMarkerDetected(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.attentionMarkerDetected) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugFloat1 = 51.70;
    std::cout << "LaneMarker_.DebugBus.debugFloat1(float32): " << LaneMarker_.DebugBus.debugFloat1 << std::endl;
    LaneMarker_.DebugBus.debugFloat2 = 52.80;
    std::cout << "LaneMarker_.DebugBus.debugFloat2(float32): " << LaneMarker_.DebugBus.debugFloat2 << std::endl;
    LaneMarker_.DebugBus.debugFloat3 = 53.90;
    std::cout << "LaneMarker_.DebugBus.debugFloat3(float32): " << LaneMarker_.DebugBus.debugFloat3 << std::endl;
    LaneMarker_.DebugBus.debugFloat4 = 55.00;
    std::cout << "LaneMarker_.DebugBus.debugFloat4(float32): " << LaneMarker_.DebugBus.debugFloat4 << std::endl;
    LaneMarker_.DebugBus.debugFloat5 = 56.10;
    std::cout << "LaneMarker_.DebugBus.debugFloat5(float32): " << LaneMarker_.DebugBus.debugFloat5 << std::endl;
    LaneMarker_.DebugBus.debugFloat6 = 57.20;
    std::cout << "LaneMarker_.DebugBus.debugFloat6(float32): " << LaneMarker_.DebugBus.debugFloat6 << std::endl;
    LaneMarker_.DebugBus.debugFloat7 = 58.30;
    std::cout << "LaneMarker_.DebugBus.debugFloat7(float32): " << LaneMarker_.DebugBus.debugFloat7 << std::endl;
    LaneMarker_.DebugBus.debugFloat8 = 59.40;
    std::cout << "LaneMarker_.DebugBus.debugFloat8(float32): " << LaneMarker_.DebugBus.debugFloat8 << std::endl;
    LaneMarker_.DebugBus.debugInteger1 = 20;
    std::cout << "LaneMarker_.DebugBus.debugInteger1(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger1) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger2 = 21;
    std::cout << "LaneMarker_.DebugBus.debugInteger2(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger2) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger3 = 22;
    std::cout << "LaneMarker_.DebugBus.debugInteger3(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger3) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger4 = 23;
    std::cout << "LaneMarker_.DebugBus.debugInteger4(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger4) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.color = 24;
    std::cout << "LaneMarker_.SecClsLeft.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.color) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.a = 60.50;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.a(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.a << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.b = 61.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.b(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.b << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.c = 62.70;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.c(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.c << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d1 = 63.80;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d1(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d2 = 64.90;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d2(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd = 66.00;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart = 67.10;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive = 25;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist = 68.20;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance = 69.30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance = 70.40;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance = 71.50;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance = 72.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance = 73.70;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset = 74.80;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading = 75.90;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt = 77.00;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate = 78.10;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.SecClsLeft.Lane.id = 26;
    std::cout << "LaneMarker_.SecClsLeft.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.id) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.markingType = 27;
    std::cout << "LaneMarker_.SecClsLeft.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.markingWidth = 79.20;
    std::cout << "LaneMarker_.SecClsLeft.Lane.markingWidth(float32): " << LaneMarker_.SecClsLeft.Lane.markingWidth << std::endl;
    LaneMarker_.SecClsLeft.Lane.measurementQuality = 80.30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.measurementQuality(float32): " << LaneMarker_.SecClsLeft.Lane.measurementQuality << std::endl;
    LaneMarker_.SecClsLeft.Lane.modelError = 81.40;
    std::cout << "LaneMarker_.SecClsLeft.Lane.modelError(float32): " << LaneMarker_.SecClsLeft.Lane.modelError << std::endl;
    LaneMarker_.SecClsLeft.Lane.secondMarkingType = 28;
    std::cout << "LaneMarker_.SecClsLeft.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.selectionConfidence = 82.50;
    std::cout << "LaneMarker_.SecClsLeft.Lane.selectionConfidence(float32): " << LaneMarker_.SecClsLeft.Lane.selectionConfidence << std::endl;
    LaneMarker_.SecClsLeft.Lane.structure = 29;
    std::cout << "LaneMarker_.SecClsLeft.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.totalMarkingWidth = 83.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.totalMarkingWidth(float32): " << LaneMarker_.SecClsLeft.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.SecClsLeft.Lane.trackingStatus = 30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.valid = 31;
    std::cout << "LaneMarker_.SecClsLeft.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.isVerified = 32;
    std::cout << "LaneMarker_.SecClsLeft.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.color = 33;
    std::cout << "LaneMarker_.SecClsRight.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.color) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.a = 84.70;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.a(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.a << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.b = 85.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.b(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.b << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.c = 86.90;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.c(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.c << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d1 = 88.00;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d1(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d2 = 89.10;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d2(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd = 90.20;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart = 91.30;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive = 34;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist = 92.40;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance = 93.50;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance = 94.60;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance = 95.70;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance = 96.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance = 97.90;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset = 99.00;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading = 100.10;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt = 101.20;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate = 102.30;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.SecClsRight.Lane.id = 35;
    std::cout << "LaneMarker_.SecClsRight.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.id) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.markingType = 36;
    std::cout << "LaneMarker_.SecClsRight.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.markingWidth = 103.40;
    std::cout << "LaneMarker_.SecClsRight.Lane.markingWidth(float32): " << LaneMarker_.SecClsRight.Lane.markingWidth << std::endl;
    LaneMarker_.SecClsRight.Lane.measurementQuality = 104.50;
    std::cout << "LaneMarker_.SecClsRight.Lane.measurementQuality(float32): " << LaneMarker_.SecClsRight.Lane.measurementQuality << std::endl;
    LaneMarker_.SecClsRight.Lane.modelError = 105.60;
    std::cout << "LaneMarker_.SecClsRight.Lane.modelError(float32): " << LaneMarker_.SecClsRight.Lane.modelError << std::endl;
    LaneMarker_.SecClsRight.Lane.secondMarkingType = 37;
    std::cout << "LaneMarker_.SecClsRight.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.selectionConfidence = 106.70;
    std::cout << "LaneMarker_.SecClsRight.Lane.selectionConfidence(float32): " << LaneMarker_.SecClsRight.Lane.selectionConfidence << std::endl;
    LaneMarker_.SecClsRight.Lane.structure = 38;
    std::cout << "LaneMarker_.SecClsRight.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.totalMarkingWidth = 107.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.totalMarkingWidth(float32): " << LaneMarker_.SecClsRight.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.SecClsRight.Lane.trackingStatus = 39;
    std::cout << "LaneMarker_.SecClsRight.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.valid = 40;
    std::cout << "LaneMarker_.SecClsRight.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.isVerified = 41;
    std::cout << "LaneMarker_.SecClsRight.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].distance = 108.90;
    std::cout << "LaneMarker_.LaneEvent[0].distance(float32): " << LaneMarker_.LaneEvent[0].distance << std::endl;
    LaneMarker_.LaneEvent[0].eventType = 42;
    std::cout << "LaneMarker_.LaneEvent[0].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].id = 43;
    std::cout << "LaneMarker_.LaneEvent[0].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].laneTrack = 44;
    std::cout << "LaneMarker_.LaneEvent[0].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].distance = 110.00;
    std::cout << "LaneMarker_.LaneEvent[1].distance(float32): " << LaneMarker_.LaneEvent[1].distance << std::endl;
    LaneMarker_.LaneEvent[1].eventType = 45;
    std::cout << "LaneMarker_.LaneEvent[1].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].id = 46;
    std::cout << "LaneMarker_.LaneEvent[1].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].laneTrack = 47;
    std::cout << "LaneMarker_.LaneEvent[1].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].distance = 111.10;
    std::cout << "LaneMarker_.LaneEvent[2].distance(float32): " << LaneMarker_.LaneEvent[2].distance << std::endl;
    LaneMarker_.LaneEvent[2].eventType = 48;
    std::cout << "LaneMarker_.LaneEvent[2].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].id = 49;
    std::cout << "LaneMarker_.LaneEvent[2].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].laneTrack = 50;
    std::cout << "LaneMarker_.LaneEvent[2].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].distance = 112.20;
    std::cout << "LaneMarker_.LaneEvent[3].distance(float32): " << LaneMarker_.LaneEvent[3].distance << std::endl;
    LaneMarker_.LaneEvent[3].eventType = 51;
    std::cout << "LaneMarker_.LaneEvent[3].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].id = 52;
    std::cout << "LaneMarker_.LaneEvent[3].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].laneTrack = 53;
    std::cout << "LaneMarker_.LaneEvent[3].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].laneTrack) << std::dec  << std::endl;
    LaneMarker_.TemporaryMarking.longDistanceToStart = 113.30;
    std::cout << "LaneMarker_.TemporaryMarking.longDistanceToStart(float32): " << LaneMarker_.TemporaryMarking.longDistanceToStart << std::endl;
    LaneMarker_.TemporaryMarking.type = 54;
    std::cout << "LaneMarker_.TemporaryMarking.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.TemporaryMarking.type) << std::dec  << std::endl;
    LaneMarker_.sideSuggestion = 55;
    std::cout << "LaneMarker_.sideSuggestion(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.sideSuggestion) << std::dec  << std::endl;
    LaneMarker_.laneChange = 56;
    std::cout << "LaneMarker_.laneChange(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.laneChange) << std::dec  << std::endl;
    LaneMarker_.SequenceID = 1;
    std::cout << "LaneMarker_.SequenceID(uint32): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(LaneMarker_.SequenceID) << std::dec  << std::endl;
}



