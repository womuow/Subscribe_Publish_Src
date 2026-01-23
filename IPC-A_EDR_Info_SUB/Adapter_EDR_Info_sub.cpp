#include"Adapter_EDR_Info.h"




EDR_Info EDR_Info_;
EDR_Info EDR_Info_old;


int config_async_sub(std::string json_file) {
    

    Adapter_EDR_Info Adapter_EDR_Info_;
    Adapter_EDR_Info_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("EDR_Info_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_EDR_Info_.data_in;



    std::map<std::string, VariableVariant > variableMap = EDR_Info_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_EDR_Info_.domain_id,
        Adapter_EDR_Info_.topic, 
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
        // if(stop.load())
        // {
            // stop.store(false);  
            const char* byte_array = data_in.data();
            if ((byte_array[0] == (def_edr_info&0xFF))  && (byte_array[1] == ((def_edr_info&0xFF00)>>8 )))
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

                std::memcpy(&ipc_msg_, data_in.data(), data_in.length());
                std::memcpy(&EDR_Info_, ipc_msg_.data, sizeof(EDR_Info));
                
                print_EDR_Info(EDR_Info_, EDR_Info_old);    
                
                EDR_Info_old = EDR_Info_;
            }
        // }
        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            
            inputQueue.pop();
        }
    }
    return 0;
}

void Adapter_EDR_Info::run()
{

    std::thread inputThread(asyncInputThreadTTY);

    config_async_sub(json_file);

}
Adapter_EDR_Info::Adapter_EDR_Info()
{
}
Adapter_EDR_Info::~Adapter_EDR_Info()
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
    std::cout << "Running on Linux(EDR_Info_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_EDR_Info objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














