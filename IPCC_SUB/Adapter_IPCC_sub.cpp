#include"Adapter_IPCC.h"
// typedef bool uint8_t;



IPC_MSG_DATA_SIZE_MAX IPC_MSG_DATA_SIZE_MAX_;
IPC_MSG_DATA_SIZE_MAX IPC_MSG_DATA_SIZE_MAX_old;

void config_async_pub(std::string json_file){
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));    

    uint8_t count=0;
    std::cout << "config file path: " << json_file.c_str() << std::endl;
    std::string data_pub="";
    
    std::cout << "initial Publisher " << std::endl;
    MOS::communication::Init(json_file);
    std::cout << "get register_version " << std::endl;
    bool a = MOS::utils::Register::get().register_version("ipcc_pub", "1.1.0");
    std::cout << "get register_version result :"<<a << std::endl;
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.shm_info.block_count = 256;
    proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    std::cout << "new Publisher " << std::endl;
    auto pub = MOS::communication::Publisher::New(80, "ipc_services/msg/subscriber", proto_info);
    auto mos_msg = std::make_shared<MOS::message::Message>();

    IPC_MSG_DATA_SIZE_MAX IPC_MSG_DATA_SIZE_MAX_PUB;


    while (true) {
        //count++;        
        
        std::cout << "set variable " << std::endl;
        IPC_MSG_DATA_SIZE_MAX_PUB.header.id = 8211;
        IPC_MSG_DATA_SIZE_MAX_PUB.header.version =1;
        IPC_MSG_DATA_SIZE_MAX_PUB.header.data_size =count++;
        IPC_MSG_DATA_SIZE_MAX_PUB.header.target =0xC1;
        IPC_MSG_DATA_SIZE_MAX_PUB.header.timestamp =count++;
        IPC_MSG_DATA_SIZE_MAX_PUB.data[0] =count++;
        IPC_MSG_DATA_SIZE_MAX_PUB.data[1] =count++;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        data_pub.resize(sizeof(IPC_MSG_DATA_SIZE_MAX_PUB));
        // std::cout << "memcpy " << std::endl;
        std::memcpy(&data_pub[0], &IPC_MSG_DATA_SIZE_MAX_PUB, sizeof(IPC_MSG_DATA_SIZE_MAX_PUB));
        auto data_ref = std::make_shared<MOS::message::DataRef>(const_cast<char*>(data_pub.data()), data_pub.size());
        mos_msg->SetDataRef(data_ref);
        auto now_time = MOS::TimeUtils::NowNsec();
        mos_msg->SetGenTimestamp(now_time);
        pub->Pub(mos_msg);
        
    }
}



int config_async_sub(std::string json_file) {
    

        std::this_thread::sleep_for(std::chrono::milliseconds(53000));

    Adapter_IPCC Adapter_IPCC_;
    Adapter_IPCC_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    // MOS::communication::Init(json_file);
    // MOS::utils::Register::get().register_version("ipcc_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolHybrid;
    proto_info.shm_info.block_count = 256;
    proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_IPCC_.data_in;
    data_in = "";


std::map<std::string, VariableVariant > variableMap = IPC_MSG_DATA_SIZE_MAX_Map;
            
                



        stop.store(true);     


    auto sub = MOS::communication::Subscriber::New(
        Adapter_IPCC_.domain_id,
        Adapter_IPCC_.topic, 
        proto_info, 
        [](MOS::message::spMsg tmp) {

        auto data_vec = tmp->GetDataRef()->GetDataVec();
        auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
        auto size = data_size_vec.size();

        for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);

        }
        stop.store(true);        
    }
);

    


    // std::thread inputThread2(asyncInputThreadTTY);
    

    while (true) {//while (!stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        std::cout<<"while sub "<<std::endl;
        if(stop.load())
        {
        std::memcpy(&IPC_MSG_DATA_SIZE_MAX_, data_in.data(), data_in.length());


        print_IPC_MSG_DATA_SIZE_MAX(IPC_MSG_DATA_SIZE_MAX_,IPC_MSG_DATA_SIZE_MAX_old);
        if (flag)
        {
            std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            flag=false;
        }

        IPC_MSG_DATA_SIZE_MAX_old = IPC_MSG_DATA_SIZE_MAX_;


        }
        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&IPCC_old, data_in.data(), sizeof(IPCC));
    }
    return 0;
}

void Adapter_IPCC::run()
{
    // (config_async_pub);

    std::thread inputThread3(config_async_pub,json_file);
    // config_async_pub(json_file);
    config_async_sub(json_file);

}
Adapter_IPCC::Adapter_IPCC()
{
}
Adapter_IPCC::~Adapter_IPCC()
{
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
    std::cout << "Running on Linux(IPCC_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_IPCC objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














