#include"Adapter_ResetA1.h"
// typedef bool uint8_t;



Rcore_reset_request ResetA1_;
Rcore_reset_request ResetA1_old;


int config_async_sub(std::string json_file) {
    

    Adapter_ResetA1 Adapter_ResetA1_;
    Adapter_ResetA1_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("ResetA1_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_ResetA1_.data_in;



    std::map<std::string, VariableVariant > variableMap = ResetA1_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_ResetA1_.domain_id,
        Adapter_ResetA1_.topic, 
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

    


    while (true) {

        if(stop.load())
        {            
            stop.store(false);  
            std::memcpy(&ipc_msg_, data_in.data(), data_in.length());
            

            if(ipc_msg_.header.id == Reset_A1 )
            {
                if (flag)
                {
                    print_memory(data_in.data(),data_in.size());  
                    std::cout << "data_in.data() size="<< static_cast<int>(data_in.size())<< std::endl;
                    flag=false;

                    std::cout << "ipc_msg_.header id=0x"
                    <<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.id)<<std::endl
                    <<"target=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.target)<<std::endl
                    <<"timestamp=0x"<<std::hex << std::setw(16)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.timestamp)<<std::endl;
                }

                std::memcpy(&ResetA1_, ipc_msg_.data, sizeof(ResetA1_));
                print_ResetA1(ResetA1_, ResetA1_old);                
                ResetA1_old = ResetA1_;
            }


            if (!inputQueue.empty())
            {                                
                getVariableValue(variableMap,inputQueue.front());
                
                inputQueue.pop();
            }
        }
        
    }
    return 0;
}

void Adapter_ResetA1::run()
{

    std::thread inputThread(asyncInputThreadTTY);

    config_async_sub(json_file);

}
Adapter_ResetA1::Adapter_ResetA1()
{
}
Adapter_ResetA1::~Adapter_ResetA1()
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
    std::cout << "Running on Linux(ResetA1_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_ResetA1 objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














