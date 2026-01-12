#include "AdApter_FuncTgtVisnID_sub.h"

IDT_FuncTgtVisnID IDT_FuncTgtVisnID_;
IDT_FuncTgtVisnID IDT_FuncTgtVisnID_old;

int config_async_sub(std::string json_file) {
    

    AdApter_FuncTgtVisnID AdApter_FuncTgtVisnID_;
    AdApter_FuncTgtVisnID_.json_file = json_file;
    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;

    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libFuncTgtVisnID", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;

        std::map<std::string, VariableVariant > variableMap = IDT_FuncTgtVisnID_Map;

    auto sub = MOS::communication::Subscriber::New(
        AdApter_FuncTgtVisnID_.domain_id,
        AdApter_FuncTgtVisnID_.topic, 
        proto_info, 
        [](MOS::message::spMsg tmp) {

        auto data_vec = tmp->GetDataRef()->GetDataVec();
        auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
        auto size = data_size_vec.size();
    



        for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        

        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
        if (flag)
        {
            std::cout << "data_in.data() hex: flag="  <<flag<< "size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            flag=false;
        }
        }
    }
);


    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::memcpy(&IDT_FuncTgtVisnID_, data_in.data(), sizeof(IDT_FuncTgtVisnID));
    print_IDT_FuncTgtVisnID(IDT_FuncTgtVisnID_,IDT_FuncTgtVisnID_old);
    IDT_FuncTgtVisnID_old = IDT_FuncTgtVisnID_;
    // std::thread inputThread(asyncInputThread);
    std::thread inputThread2(asyncInputThreadTTY);

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::memcpy(&IDT_FuncTgtVisnID_, data_in.data(), sizeof(IDT_FuncTgtVisnID));

        // std::cout<< "Print FuncTgtVisnID changed value"<< std::endl;
        //print_FuncTgtVisnID(IDT_FuncTgtVisnID_,FuncTgtVisnID_old);
        // std::cout<< "----------------------------------End"<< std::endl;
        print_IDT_FuncTgtVisnID(IDT_FuncTgtVisnID_,IDT_FuncTgtVisnID_old);


        IDT_FuncTgtVisnID_old = IDT_FuncTgtVisnID_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&FuncTgtVisnID_old, data_in.data(), sizeof(FuncTgtVisnID));
    }
    return 0;
}

void AdApter_FuncTgtVisnID::run()
{
    config_async_sub(json_file);
}
AdApter_FuncTgtVisnID::AdApter_FuncTgtVisnID()
{
}
AdApter_FuncTgtVisnID::~AdApter_FuncTgtVisnID()
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
    std::cout << "Running on Linux(FuncTgtVisnID_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_FuncTgtVisnID objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
return 0;
}










