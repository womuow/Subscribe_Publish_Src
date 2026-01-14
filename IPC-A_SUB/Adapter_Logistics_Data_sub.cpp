#include"Adapter_Logistics_Data.h"
// typedef bool uint8_t;



Logistics_data Logistics_data_;
Logistics_data Logistics_data_old;


int config_async_sub(std::string json_file) {
    

    Adapter_Logistics_Data Adapter_Logistics_Data_;
    Adapter_Logistics_Data_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    // MOS::communication::Init(json_file);
    // MOS::utils::Register::get().register_version("Logistics_Data_sub", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    // // proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // proto_info.shm_info.block_count = 256;
    // proto_info.shm_info.block_size = 1024*6;
    proto_info.shm_info.fast_mode = false;
    // std::string data_in = Adapter_Logistics_Data_.data_in;



    std::map<std::string, VariableVariant > variableMap = Logistics_data_Map;
            
                




    auto sub = MOS::communication::Subscriber::New(
        Adapter_Logistics_Data_.domain_id,
        Adapter_Logistics_Data_.topic, 
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
    

    while (true) {
        
        
        if(stop.load())
        {
        std::memcpy(&Logistics_data_, data_in.data(), data_in.length());


        print_Logistics_data(Logistics_data_, Logistics_data_old);
        
        if (flag)
        {
            std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            std::cout << "Logistics_data_ size="<<sizeof(Logistics_data)<< std::endl;
            flag=false;
        }

        Logistics_data_old = Logistics_data_;


        }
        if (!inputQueue.empty())
        {
            if (inputQueue.front() == "Logistics_data_.HW_Version")
            {
                std::cout<<"Logistics_data_.HW_Version: "<< std::hex << std::setw(2);
                for(int i =0;i<sizeof(Logistics_data_.HW_Version);i++)
                {
                    std::cout<<"0x"<<std::hex << std::setw(2)<<std::setfill('0')<< static_cast<int>(Logistics_data_.HW_Version[i])<<" ";
                }
                std::cout<<std::dec<<std::endl;
            }
            else if (inputQueue.front() == "Logistics_data_.SW_Version")
            {
                std::cout<<"Logistics_data_.SW_Version: "<< std::hex << std::setw(2);
                for(int i =0;i<sizeof(Logistics_data_.SW_Version);i++)
                {
                    std::cout<<"0x"<<std::hex << std::setw(2)<<std::setfill('0')<< static_cast<int>(Logistics_data_.SW_Version[i])<<" ";
                }
                std::cout<<std::dec<<std::endl;
            }
            else if (inputQueue.front() == "Logistics_data_.ECU_SerialNumber")
            {
                std::cout<<"Logistics_data_.ECU_SerialNumber: "<< std::hex << std::setw(2);
                for(int i =0;i<sizeof(Logistics_data_.ECU_SerialNumber);i++)
                {
                    std::cout<<"0x"<<std::hex << std::setw(2)<<std::setfill('0')<< static_cast<int>(Logistics_data_.ECU_SerialNumber[i])<<" ";
                }
                std::cout<<std::dec<<std::endl;
            }
            else if (inputQueue.front() == "Logistics_data_.VIN_Code")
            {
                std::cout<<"Logistics_data_.VIN_Code: "<< std::hex << std::setw(2);
                for(int i =0;i<sizeof(Logistics_data_.VIN_Code);i++)
                {
                    std::cout<<"0x"<<std::hex << std::setw(2)<<std::setfill('0')<< static_cast<int>(Logistics_data_.VIN_Code[i])<<" ";
                }
                std::cout<<std::dec<<std::endl;
            }

            else
            {
                getVariableValue(variableMap,inputQueue.front());
            }
            


            inputQueue.pop();
        }
        
    }
    return 0;
}

void Adapter_Logistics_Data::run()
{

    std::thread inputThread(asyncInputThreadTTY);

    config_async_sub(json_file);

}
Adapter_Logistics_Data::Adapter_Logistics_Data()
{
}
Adapter_Logistics_Data::~Adapter_Logistics_Data()
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
    std::cout << "Running on Linux(Logistics_Data_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_Logistics_Data objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}














