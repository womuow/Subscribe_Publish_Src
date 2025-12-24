#include"AdApter_ObjFrntCdnForSupp.h"
std::atomic_bool stop{ false };
dds::pub::DataWriter<ObjFrntCdnForSuppModule::ObjFrntCdnForSupp> AdApter_ObjFrntCdnForSupp::topic_writer_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<ObjFrntCdnForSuppModule::ObjFrntCdnForSupp> topic(participant, topicName);
    dds::pub::Publisher publisher(participant);
    dds::pub::DataWriter<ObjFrntCdnForSuppModule::ObjFrntCdnForSupp> writer(publisher, topic);
    return writer;
}
void AdApter_ObjFrntCdnForSupp::update_objInfo(std::string data_str)
{
    ObjFrntCdnForSupp ObjFrntCdnForSupp_;
    std::memcpy(&ObjFrntCdnForSupp_, data_str.data(), sizeof(ObjFrntCdnForSupp));
    topic_out.ObjIdx() = ObjFrntCdnForSupp_.ObjIdx;
    topic_out.ObjPosnLat() = ObjFrntCdnForSupp_.ObjPosnLat;
    topic_out.ObjPosnLgt() = ObjFrntCdnForSupp_.ObjPosnLgt;
    topic_out.ObjSpdLgt() = ObjFrntCdnForSupp_.ObjSpdLgt;
    topic_out.ObjTyp() = ObjFrntCdnForSupp_.ObjTyp;
    topic_out.ObjWidth() = ObjFrntCdnForSupp_.ObjWidth;
    topic_out.TiToCllsn() = ObjFrntCdnForSupp_.TiToCllsn;
    topic_out.VisnIdx() = ObjFrntCdnForSupp_.VisnIdx;
    topic_out.SuppressSideRoadEdge() = ObjFrntCdnForSupp_.SuppressSideRoadEdge;
    topic_out.SuppressSideSolidLKA() = ObjFrntCdnForSupp_.SuppressSideSolidLKA;
    data_in.resize(sizeof(ObjFrntCdnForSupp));
    std::memcpy(&data_in[0], &ObjFrntCdnForSupp_, sizeof(ObjFrntCdnForSupp));
}
void AdApter_ObjFrntCdnForSupp::maxeye_data_init()
{
}
AdApter_ObjFrntCdnForSupp::AdApter_ObjFrntCdnForSupp()
{
}
AdApter_ObjFrntCdnForSupp::~AdApter_ObjFrntCdnForSupp()
{
}
auto AdApter_ObjFrntCdnForSupp::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(ObjFrntCdnForSupp));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libObjFrntCdnForSupp", "1.1.0");
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
    AdApter_ObjFrntCdnForSupp AdApter_ObjFrntCdnForSupp_;
    AdApter_ObjFrntCdnForSupp_.json_file = json_file;
   dds::pub::DataWriter<ObjFrntCdnForSuppModule::ObjFrntCdnForSupp> writer = AdApter_ObjFrntCdnForSupp_.topic_writer_init(AdApter_ObjFrntCdnForSupp_.topicName);
    int domain_id = AdApter_ObjFrntCdnForSupp_.domiain_id;
    std::string topic = AdApter_ObjFrntCdnForSupp_.topic;
    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libObjFrntCdnForSupp", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    auto sub = MOS::communication::Subscriber::New(AdApter_ObjFrntCdnForSupp_.domiain_id,AdApter_ObjFrntCdnForSupp_.topic, proto_info, [&data_in](MOS::message::spMsg tmp) {
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
        AdApter_ObjFrntCdnForSupp_.update_objInfo(data_in);
        writer.write(AdApter_ObjFrntCdnForSupp_.topic_out);
        std::this_thread::sleep_for(std::chrono::milliseconds(AdApter_ObjFrntCdnForSupp_.cycle_ms));
    }
    return 0;
    }
void AdApter_ObjFrntCdnForSupp::run()
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
    AdApter_ObjFrntCdnForSupp objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
