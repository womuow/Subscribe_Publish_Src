#include"AdApter_RoadEdge_PUB.h"

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}






AdApter_RoadEdgePUB::AdApter_RoadEdgePUB()
{
}
AdApter_RoadEdgePUB::~AdApter_RoadEdgePUB()
{
}
auto AdApter_RoadEdgePUB::maxeye_midware_init()
{

return proto_info;
}


void AdApter_RoadEdgePUB::run()
{
    int i=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(RoadEdge));
    MOS::communication::Init(json_file.c_str());
    MOS::utils::Register::get().register_version("libRoadEdgePUB", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;//kProtocolHybrid
    proto_info.shm_info.block_count = 10;
    proto_info.shm_info.block_size = data_in.size();
    proto_info.shm_info.fast_mode = false;
    auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms*3));//40ms


    RoadEdge RoadEdge_;
    setIntialValue_RoadEdge(RoadEdge_);

    while (true) {        
        data_in.resize(sizeof(RoadEdge_));
        std::memcpy(&data_in[0], &RoadEdge_, sizeof(RoadEdge_));
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
    std::cout << "Running RoadEdge_PUB on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_RoadEdgePUB objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}




/* Set and Print struct RoadEdge initial value */
void setIntialValue_RoadEdge(RoadEdge& RoadEdge_){
    std::cout << "Set struct RoadEdge variable and Publish:" << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.id = 1;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.measurementQuality = 1.10;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.LeftNonTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.modelError = 2.20;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.modelError(float32): " << RoadEdge_.LeftNonTrvsble.Edge.modelError << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a = 3.30;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance = 4.40;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b = 5.50;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance = 6.60;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c = 7.70;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance = 8.80;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d = 9.90;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance = 11.00;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance = 12.10;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd = 13.20;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart = 14.30;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset = 15.40;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading = 16.50;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt = 17.60;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate = 18.70;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.trackingStatus = 2;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.valid = 3;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.isVerified = 4;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence = 19.80;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.id = 5;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.measurementQuality = 20.90;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.RightNonTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.modelError = 22.00;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.modelError(float32): " << RoadEdge_.RightNonTrvsble.Edge.modelError << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a = 23.10;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance = 24.20;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b = 25.30;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance = 26.40;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c = 27.50;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance = 28.60;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d = 29.70;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance = 30.80;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance = 31.90;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd = 33.00;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart = 34.10;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset = 35.20;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading = 36.30;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt = 37.40;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate = 38.50;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.trackingStatus = 6;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.valid = 7;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.isVerified = 8;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.selectionConfidence = 39.60;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.RightNonTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.LeftTrvsble.Edge.id = 9;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.measurementQuality = 40.70;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.LeftTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.LeftTrvsble.Edge.modelError = 41.80;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.modelError(float32): " << RoadEdge_.LeftTrvsble.Edge.modelError << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.a = 42.90;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance = 44.00;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.b = 45.10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance = 46.20;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.c = 47.30;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance = 48.40;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d = 49.50;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance = 50.60;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance = 51.70;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd = 52.80;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart = 53.90;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset = 55.00;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading = 56.10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt = 57.20;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate = 58.30;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.LeftTrvsble.Edge.trackingStatus = 10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.valid = 11;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.isVerified = 12;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.selectionConfidence = 59.40;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.LeftTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.RightTrvsble.Edge.id = 13;
    std::cout << "RoadEdge_.RightTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.measurementQuality = 60.50;
    std::cout << "RoadEdge_.RightTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.RightTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.RightTrvsble.Edge.modelError = 61.60;
    std::cout << "RoadEdge_.RightTrvsble.Edge.modelError(float32): " << RoadEdge_.RightTrvsble.Edge.modelError << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.a = 62.70;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance = 63.80;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.b = 64.90;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance = 66.00;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.c = 67.10;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance = 68.20;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d = 69.30;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance = 70.40;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance = 71.50;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd = 72.60;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart = 73.70;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset = 74.80;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading = 75.90;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt = 77.00;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate = 78.10;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.RightTrvsble.Edge.trackingStatus = 14;
    std::cout << "RoadEdge_.RightTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.valid = 15;
    std::cout << "RoadEdge_.RightTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.isVerified = 16;
    std::cout << "RoadEdge_.RightTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.selectionConfidence = 79.20;
    std::cout << "RoadEdge_.RightTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.RightTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.SequenceID = 1;
    std::cout << "RoadEdge_.SequenceID(uint32): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(RoadEdge_.SequenceID) << std::dec  << std::endl;
}







