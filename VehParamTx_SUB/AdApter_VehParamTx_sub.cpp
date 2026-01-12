#include "AdApter_VehParamTx_sub.h"


using namespace std::chrono_literals;



VehParam_Tx VehParam_Tx_;
VehParam_Tx VehParam_Tx_old;





int config_async_sub(std::string json_file) {
    AdApter_VehParam_Tx AdApter_VehParam_Tx_;
    AdApter_VehParam_Tx_.json_file = json_file;
    

    int domain_id=0;
    std::string topic ="IPCC/VehParam_Tx";
    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libVehParam_Tx", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;


    std::map<std::string, VariableVariant > variableMap = VehParam_Tx_Map;

    
    std::thread inputThread2(asyncInputThreadTTY);
    auto sub = MOS::communication::Subscriber::New(
        domain_id,
        topic, 
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

    // int i=0;
    //std::string log;
    // char templog[100];
    // char log[1000000];
    // log.clear();
    // snprintf(log,sizeof(log),"");

    while (true) {//while (!stop.load()) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::memcpy(&VehParam_Tx_, data_in.data(), sizeof(VehParam_Tx));
        // std::string temptime=getCurrentTime();
        // snprintf(templog,sizeof(templog),"%s VehParam_Tx_.SteerWhlPosn(uint8)::: 0x%02X;\n",temptime.c_str(),VehParam_Tx_.SteerWhlPosn);
        
        // strncat(log,templog,sizeof(log)-strlen(log)-1);
        // snprintf(templog,sizeof(templog),"");
        // //std::this_thread::sleep_for(std::chrono::microseconds(20));
        // i++;
        // if(i==1000)
        // {
        //     std::cout <<"save log" << std::endl;
        //     std::cout <<log << std::endl;
        //     i=0;
        //     snprintf(log,sizeof(log),"");
        // }
        print_VehParam_Tx(VehParam_Tx_,VehParam_Tx_old);

        VehParam_Tx_old = VehParam_Tx_;




        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }        
    }
    return 0;
}

void AdApter_VehParam_Tx::run()
{
    config_async_sub(json_file);
}
AdApter_VehParam_Tx::AdApter_VehParam_Tx()
{
}
AdApter_VehParam_Tx::~AdApter_VehParam_Tx()
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
    std::cout << "Running on Linux(VehParamTx_Sub)"<< fullPath << std::endl;
#endif
	// std::ofstream logFile("./logfile.log");
    // if (!logFile.is_open()) {
    //     std::cerr << "could not create log" << std::endl;
    //     return -1;
    // }
    //save count orign buffer to log
    // std::streambuf* originalCoutBuffer = std::cout.rdbuf();
    // std::cout.rdbuf(logFile.rdbuf()); //redirect to log file


    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_VehParam_Tx objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    
    // Restore cout to the console
    // std::cout.rdbuf(originalCoutBuffer);
    // logFile.close();

return 0;
}




