#include"AdApter_VehParam_Tx.h"
dds::sub::DataReader<VehParam_TxModule::VehParam_Tx> AdApter_VehParam_Tx::topic_read_init(std::string topicName)
{
    dds::domain::DomainParticipant participant(domain::default_id());
    dds::topic::Topic<VehParam_TxModule::VehParam_Tx> topic(participant, topicName);
    dds::sub::Subscriber subscriber(participant);
    dds::sub::DataReader<VehParam_TxModule::VehParam_Tx> reader(subscriber, topic);
    return reader;
}
void AdApter_VehParam_Tx::update_objInfo(const VehParam_TxModule::VehParam_Tx msg, std::string& data_str)
{
    VehParam_Tx VehParam_Tx_;
    VehParam_Tx_.AxleDstReToVehFrnt = msg.AxleDstReToVehFrnt();
    VehParam_Tx_.SingleTrackAxleDistFrnt = msg.SingleTrackAxleDistFrnt();
    VehParam_Tx_.SteerWhlPosn = msg.SteerWhlPosn();
    VehParam_Tx_.Len = msg.Len();
    VehParam_Tx_.Weight = msg.Weight();
    VehParam_Tx_.WhlBas = msg.WhlBas();
    VehParam_Tx_.Width = msg.Width();
    for (int i = 0; i < 8; i++)
    {
    VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[i] = msg.SingleTrackCornrgStfnFrntByVehSpd()[i];
    }
    VehParam_Tx_.SingleTrackCornrgStfnFrnt = msg.SingleTrackCornrgStfnFrnt();
    for (int i = 0; i < 8; i++)
    {
    VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[i] = msg.SingleTrackCornrgStfnReByVehSpd()[i];
    }
    VehParam_Tx_.SingleTrackCornrgStfnRe = msg.SingleTrackCornrgStfnRe();
    VehParam_Tx_.SteerWhlAgRat = msg.SteerWhlAgRat();
    for (int i = 0; i < 8; i++)
    {
    VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[i] = msg.SingleTrackCornrgStfnTable_Spd()[i];
    }
    VehParam_Tx_.LDWLatInnerOffsetInhibit = msg.LDWLatInnerOffsetInhibit();
    VehParam_Tx_.LDWLatOffsetTrigLKAOn = msg.LDWLatOffsetTrigLKAOn();
    VehParam_Tx_.LDWLatOffsetTrig = msg.LDWLatOffsetTrig();
    VehParam_Tx_.LDWTimeToLaneCrossMin = msg.LDWTimeToLaneCrossMin();
    VehParam_Tx_.BltFrntExist = msg.BltFrntExist();
    VehParam_Tx_.OncomingBrk = msg.OncomingBrk();
    VehParam_Tx_.SelfStrGrdt = msg.SelfStrGrdt();
    VehParam_Tx_.TrafficAssist = msg.TrafficAssist();
    VehParam_Tx_.LongCtrlBrkLim = msg.LongCtrlBrkLim();
    VehParam_Tx_.LongCtrEco = msg.LongCtrEco();
    VehParam_Tx_.LongCtrSpdLoLim = msg.LongCtrSpdLoLim();
    VehParam_Tx_.LongCtrStopNGo = msg.LongCtrStopNGo();
    VehParam_Tx_.SingleTrackMomentOfInertia = msg.SingleTrackMomentOfInertia();
    VehParam_Tx_.VehTyp = msg.VehTyp();
    data_in.resize(sizeof(VehParam_Tx));
    std::memcpy(&data_in[0], &VehParam_Tx_, sizeof(VehParam_Tx));
}
void AdApter_VehParam_Tx::maxeye_data_init()
{
}
AdApter_VehParam_Tx::AdApter_VehParam_Tx()
{
}
AdApter_VehParam_Tx::~AdApter_VehParam_Tx()
{
}
auto AdApter_VehParam_Tx::maxeye_midware_init()
{
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    data_in.resize(sizeof(VehParam_Tx));
MOS::communication::Init(json_file.c_str());
MOS::utils::Register::get().register_version("libVehParam_Tx", "1.1.0");
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
void AdApter_VehParam_Tx::run()
{
    dds::sub::DataReader<VehParam_TxModule::VehParam_Tx> reader = topic_read_init(AdApter_VehParam_Tx::topicName);
dds::sub::LoanedSamples<VehParam_TxModule::VehParam_Tx> samples;
    auto proto_info = maxeye_midware_init();
auto pub = MOS::communication::Publisher::New(domiain_id, topic, proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();
    while (true) {
        maxeye_data_init();
        samples = reader.take();
        if (samples.length() > 0) {
            for (dds::sub::LoanedSamples<VehParam_TxModule::VehParam_Tx>::const_iterator sample_iter = samples.begin();
                sample_iter < samples.end();
                ++sample_iter) {
                const VehParam_TxModule::VehParam_Tx& msg = sample_iter->data();
                const dds::sub::SampleInfo& info = sample_iter->info();
                if (info.valid()) {
                    update_objInfo(msg,data_in);   
                }
            }
        }
        auto data_ref = std::make_shared<MOS::message::DataRef>(const_cast<char*>(data_in.data()), data_in.size());
        mos_msg->SetDataRef(data_ref);
        auto now_time = MOS::TimeUtils::NowNsec();
        mos_msg->SetGenTimestamp(now_time);
        pub->Pub(mos_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(cycle_ms));
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
    std::cout << "Running on Linux"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_VehParam_Tx objtest;
    objtest.json_file = configPath;
    objtest.run();
return 0;
}
