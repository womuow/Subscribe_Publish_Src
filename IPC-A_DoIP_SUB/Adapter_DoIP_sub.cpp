#include"Adapter_DoIP.h"
// typedef bool uint8_t;


UDS_ipc_req UDS_ipc_req_;
UDS_ipc_response UDS_ipc_response_;

int config_async_Response_sub(std::string json_file) {
    

    Adapter_DoIP Adapter_DoIP_;
    Adapter_DoIP_.json_file = json_file;

    std::cout << "config_async_Response_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("DoIP_Response_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_DoIP_.data_in;



    //std::map<std::string, VariableVariant > variableMap = DoIP_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_DoIP_.domain_id,
        Adapter_DoIP_.topic, 
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
            if ((byte_array[0] == (def_DoIP_response&0xFF))  && (byte_array[1] == ((def_DoIP_response&0xFF00)>>8 )))
            {
                // if (flag)
                // {
                //     print_memory(data_in.data(),data_in.size());  
                //     std::cout << "data_in.data() size="<< static_cast<int>(data_in.size())<< std::endl;
                //     flag=false;

                //     std::cout << "ipc_msg_.header id=0x"
                //     <<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.id)<<std::endl
                //     <<"target=0x"<<std::hex << std::setw(4)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.target)<<std::endl
                //     <<"timestamp=0x"<<std::hex << std::setw(16)<<std::setfill('0')<< static_cast<int>(ipc_msg_.header.timestamp)<<std::endl;
                // }

                std::memcpy(&ipc_msg_, data_in.data(), data_in.length());
                
               
                std::memcpy(&UDS_ipc_response_, ipc_msg_.data, sizeof(UDS_ipc_response));
                
                    std::cout << "UDS_ipc_response_.addressing_format(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.addressing_format) << std::dec  << std::endl;
                    
                    std::cout << "UDS_ipc_response_.Doip_message_length(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << UDS_ipc_response_.Doip_message_length << std::dec  << std::endl;
                    

                    std::cout << "UDS_ipc_response_.Doip_message[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[0]) << std::dec  << std::endl;

                    std::cout << "UDS_ipc_response_.Doip_message: 0x" ;
                    //print_memory(UDS_ipc_response_.Doip_message,10);
                    for (size_t i = 0; i < UDS_ipc_response_.Doip_message_length; ++i) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                        << static_cast<int>(UDS_ipc_response_.Doip_message[i]) ;
                    }
                    std::cout << std::endl;
            }
        }
            // if (!inputQueue.empty())
            // {                                
            //     getVariableValue(variableMap,inputQueue.front());
                
            //     inputQueue.pop();
            // }
    }
    return 0;
}






int config_async_Req_sub(std::string json_file) {
    

    Adapter_DoIP Adapter_DoIP_;
    Adapter_DoIP_.json_file = json_file;

    std::cout << "config_async_Req_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("DoIP_Req_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_DoIP_.data_in;



    //std::map<std::string, VariableVariant > variableMap = DoIP_Map;

    
    auto sub2 = MOS::communication::Subscriber::New(
        Adapter_DoIP_.domain_id,
        Adapter_DoIP_.topic2, 
        proto_info, 
        [](MOS::message::spMsg tmp) {

        auto data_vec = tmp->GetDataRef()->GetDataVec();
        auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
        auto size = data_size_vec.size();

        for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in2 = std::string(reinterpret_cast<const char*>(vec_data), vec_size);

        }
        stop2.store(true);        
        }
    );


    while (true) {
        if(stop2.load())
        {            
            stop2.store(false);  
            

            const char* byte_array = data_in.data();
            if ((byte_array[0] == (def_DoIP_req&0xFF))  && (byte_array[1] == ((def_DoIP_req&0xFF00)>>8 )))
            {
                std::memcpy(&ipc_msg_, data_in.data(), data_in.length());
                
               
                std::memcpy(&UDS_ipc_req_, ipc_msg_.data, sizeof(UDS_ipc_req));
                
                
                    std::cout << "UDS_ipc_req_.addressing_format(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.addressing_format) << std::dec  << std::endl;
                    
                    std::cout << "UDS_ipc_req_.Doip_message_length(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << UDS_ipc_req_.Doip_message_length << std::dec  << std::endl;
                    
                    std::cout << "UDS_ipc_req_.Doip_message[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[0]) << std::dec  << std::endl;
                    
                    std::cout << "UDS_ipc_req_.Doip_message: 0x" ;
                    print_memory(UDS_ipc_req_.Doip_message,10);
                    std::cout << std::endl;
            }


            // if (!inputQueue.empty())
            // {                                
            //     getVariableValue(variableMap,inputQueue.front());
                
            //     inputQueue.pop();
            // }
        }

    }
    return 0;
}

void Adapter_DoIP::run()
{

    std::thread inputThread(config_async_Req_sub,json_file);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    config_async_Response_sub(json_file);

}
Adapter_DoIP::Adapter_DoIP()
{
}
Adapter_DoIP::~Adapter_DoIP()
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
    std::cout << "Running on Linux(DoIP_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_DoIP objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














