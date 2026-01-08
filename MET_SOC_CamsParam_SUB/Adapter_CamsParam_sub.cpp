#include"Adapter_CamsParam.h"
// typedef bool uint8_t;



MET_SOC_CamsParam MET_SOC_CamsParam_;
MET_SOC_CamsParam MET_SOC_CamsParam_old;


int config_async_sub(std::string json_file) {
    

    Adapter_CamsParam Adapter_CamsParam_;
    Adapter_CamsParam_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libCamsParam", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // std::string data_in = Adapter_CamsParam_.data_in;
    data_in = "";


std::map<std::string, VariableVariant > variableMap = MET_SOC_CamsParam_Map;
            
                





    auto sub = MOS::communication::Subscriber::New(
        Adapter_CamsParam_.domain_id,
        Adapter_CamsParam_.topic, 
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

    


    std::thread inputThread2(asyncInputThreadTTY);
    

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if(stop.load())
        {
        std::memcpy(&MET_SOC_CamsParam_, data_in.data(), data_in.length());


        print_MET_SOC_CamsParam(MET_SOC_CamsParam_,MET_SOC_CamsParam_old);
        if (flag)
        {
            std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            flag=false;
        }

        MET_SOC_CamsParam_old = MET_SOC_CamsParam_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&CamsParam_old, data_in.data(), sizeof(CamsParam));
    }
    return 0;
}

void Adapter_CamsParam::run()
{
    config_async_sub(json_file);
}
Adapter_CamsParam::Adapter_CamsParam()
{
}
Adapter_CamsParam::~Adapter_CamsParam()
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
    std::cout << "Running on Linux(CamsParam_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_CamsParam objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














