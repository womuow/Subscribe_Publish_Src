#include"Adapter_Parameter.h"
// typedef bool uint8_t;


Perception_Vehicle_parameters Perception_Vehicle_parameters_;
Perception_Vehicle_parameters Perception_Vehicle_parameters_old;
Intrinsic_Calibration_parameters Intrinsic_Calibration_parameters_;
Intrinsic_Calibration_parameters Intrinsic_Calibration_parameters_old;


int config_async_sub(std::string json_file) {
    

    Adapter_Parameter Adapter_Parameter_;
    Adapter_Parameter_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("Parameter_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_Parameter_.data_in;



    std::map<std::string, VariableVariant > variableMap = Parameter_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_Parameter_.domain_id,
        Adapter_Parameter_.topic, 
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
            

            const char* byte_array = data_in.data();
            if ((byte_array[0] == (def_Parameter&0xFF))  && (byte_array[1] == ((def_Parameter&0xFF00)>>8 )))
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
                if(data_in.length() == sizeof(ipc_msg_.header) + sizeof(Perception_Vehicle_parameters))
                {
                    std::memcpy(&Perception_Vehicle_parameters_, ipc_msg_.data, sizeof(Perception_Vehicle_parameters));
                    print_Perception_Vehicle_parameters(Perception_Vehicle_parameters_, Perception_Vehicle_parameters_old);                
                    Perception_Vehicle_parameters_old = Perception_Vehicle_parameters_;
                }
                else if(data_in.length() == sizeof(ipc_msg_.header) + sizeof(Intrinsic_Calibration_parameters))
                {
                    std::memcpy(&Intrinsic_Calibration_parameters_, ipc_msg_.data, sizeof(Intrinsic_Calibration_parameters_));
                    print_Intrinsic_Calibration_parameters(Intrinsic_Calibration_parameters_, Intrinsic_Calibration_parameters_old);                
                    Intrinsic_Calibration_parameters_old = Intrinsic_Calibration_parameters_;
                }
                else
                {
                    print_memory(data_in.data(),data_in.size());  
                    std::cout << "data_in.data() size="<< static_cast<int>(data_in.size())<< std::endl;
                    flag=false;

                    std::cout << "ipc_msg_.header id=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.id)<<std::endl                    
                    <<"target=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.target)<<std::endl
                    <<"timestamp=0x"<<std::hex << std::setw(16)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.timestamp)<<std::endl;
                    std::cout << "Intrinsic_Calibration_parameters size=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< sizeof(Intrinsic_Calibration_parameters)<<std::endl ;
                    std::cout << "Perception_Vehicle_parameters size=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< sizeof(Perception_Vehicle_parameters)<<std::endl ;
                }

                
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

void Adapter_Parameter::run()
{

    std::thread inputThread(asyncInputThreadTTY);

    config_async_sub(json_file);

}
Adapter_Parameter::Adapter_Parameter()
{
}
Adapter_Parameter::~Adapter_Parameter()
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
    std::cout << "Running on Linux(Parameter_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_Parameter objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














