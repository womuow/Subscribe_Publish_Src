#include"Adapter_SWP1.h"



MET_SOC_SWP1_Param MET_SOC_SWP1_Param_;

MET_SOC_SWP1_Param MET_SOC_SWP1_Param_old;





int config_async_sub(std::string json_file) {
    

    Adapter_SWP1 Adapter_SWP1_;
    Adapter_SWP1_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libSWP1", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;

        std::map<std::string, VariableVariant > variableMap = MET_SOC_SWP1_Param_Map;
        




        
    auto sub = MOS::communication::Subscriber::New(
        Adapter_SWP1_.domain_id,
        Adapter_SWP1_.topic, 
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


    // std::this_thread::sleep_for(std::chrono::milliseconds(3));
    // std::thread inputThread(asyncInputThread);
    std::thread inputThread2(asyncInputThreadTTY);
    memset(&MET_SOC_SWP1_Param_old, 0, sizeof(MET_SOC_SWP1_Param));

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if(stop.load())
        {
        std::memcpy(&MET_SOC_SWP1_Param_, data_in.data(), sizeof(MET_SOC_SWP1_Param));

        print_MET_SOC_SWP1_Param(MET_SOC_SWP1_Param_,MET_SOC_SWP1_Param_old);
        if (flag)
        {
            std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            flag=false;
        }



        MET_SOC_SWP1_Param_old = MET_SOC_SWP1_Param_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&SWP1_old, data_in.data(), sizeof(SWP1));
    }
    return 0;
}

void Adapter_SWP1::run()
{
    config_async_sub(json_file);
}
Adapter_SWP1::Adapter_SWP1()
{
}
Adapter_SWP1::~Adapter_SWP1()
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
    std::cout << "Running on Linux(SWP1_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_SWP1 objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}










