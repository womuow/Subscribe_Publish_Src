#include"Adapter_CS_Trigger_A1.h"
// typedef bool uint8_t;


MagnaParamCSRes MagnaParamCSRes_;
MagnaParamCSRes MagnaParamCSRes_old;
MagnaCamExtParam MagnaCamExtParam_;
MagnaCamExtParam MagnaCamExtParam_old;
MagnaCamIntParam MagnaCamIntParam_;
MagnaCamIntParam MagnaCamIntParam_old;
MagnaTACCalibInput MagnaTACCalibInput_;
MagnaTACCalibInput MagnaTACCalibInput_old;


int config_async_sub(std::string json_file) {
    

    Adapter_CSTriggerA1 Adapter_CSTriggerA1_;
    Adapter_CSTriggerA1_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("CSTriggerA1_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_CSTriggerA1_.data_in;



    std::map<std::string, VariableVariant > variableMap = CSTriggerA1_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_CSTriggerA1_.domain_id,
        Adapter_CSTriggerA1_.topic, 
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
            if ((byte_array[0] == (CS_Trigger_A1&0xFF))  && (byte_array[1] == ((CS_Trigger_A1&0xFF00)>>8 )))
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
                
               
                std::memcpy(&MagnaParamCSRes_, ipc_msg_.data, sizeof(MagnaParamCSRes));
                print_MagnaParamCSRes(MagnaParamCSRes_, MagnaParamCSRes_old);                
                MagnaParamCSRes_old = MagnaParamCSRes_;

                if(data_in.length() == sizeof(ipc_msg_.header) + sizeof(MagnaCamExtParam) + 2)
                {
                    std::memcpy(&MagnaCamExtParam_, MagnaParamCSRes_.meta_res_data, sizeof(MagnaCamExtParam));
                    print_MagnaCamExtParam(MagnaCamExtParam_,MagnaCamExtParam_old);
                    MagnaCamExtParam_old = MagnaCamExtParam_;
                }
                else if(data_in.length() == sizeof(ipc_msg_.header) + sizeof(MagnaCamIntParam) + 2)
                {
                    std::memcpy(&MagnaCamIntParam_, MagnaParamCSRes_.meta_res_data, sizeof(MagnaCamIntParam));
                    print_MagnaCamIntParam(MagnaCamIntParam_,MagnaCamIntParam_old);
                    MagnaCamIntParam_old = MagnaCamIntParam_;
                }
                else if(data_in.length() == sizeof(ipc_msg_.header) + sizeof(MagnaTACCalibInput) + 2)
                {
                    std::memcpy(&MagnaTACCalibInput_, MagnaParamCSRes_.meta_res_data, sizeof(MagnaTACCalibInput));
                    print_MagnaTACCalibInput(MagnaTACCalibInput_,MagnaTACCalibInput_old);
                    MagnaTACCalibInput_old = MagnaTACCalibInput_;
                }
                
                
                
                else
                {
                    print_memory(data_in.data(),data_in.size());  
                    std::cout << "data_in.data() size="<< static_cast<int>(data_in.size())<< std::endl;

                    std::cout << "ipc_msg_.header id=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.id)<<std::endl                    
                    <<"target=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.target)<<std::endl
                    <<"timestamp=0x"<<std::hex << std::setw(16)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.timestamp)<<std::endl;
                    std::cout << "MagnaCamExtParam size=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< sizeof(MagnaCamExtParam)<<std::endl ;
                    std::cout << "MagnaCamIntParam size=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< sizeof(MagnaCamIntParam)<<std::endl ;
                    std::cout << "MagnaTACCalibInput size=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< sizeof(MagnaTACCalibInput)<<std::endl ;
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

void Adapter_CSTriggerA1::run()
{

    std::thread inputThread(asyncInputThreadTTY);

    config_async_sub(json_file);

}
Adapter_CSTriggerA1::Adapter_CSTriggerA1()
{
}
Adapter_CSTriggerA1::~Adapter_CSTriggerA1()
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
    std::cout << "Running on Linux(CSTriggerA1_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_CSTriggerA1 objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














