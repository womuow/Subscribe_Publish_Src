#include "AdApter_VehBusIn_sub.h"

using namespace std::chrono_literals;


VehBusIn VehBusIn_;
VehBusIn VehBusIn_old;  

int config_async_sub(std::string json_file) {
    AdApter_VehBusIn AdApter_VehBusIn_;
    AdApter_VehBusIn_.json_file = json_file;

    int domain_id=0;
    std::string topic ="IPCC/VehBusIn";

    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libVehBusIn", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;

    auto sub = MOS::communication::Subscriber::New(
        domain_id,
        topic, 
        proto_info, 
        [](MOS::message::spMsg tmp) {

    auto data_vec = tmp->GetDataRef()->GetDataVec();
    auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
    auto size = data_size_vec.size();
    
    // std::cout << "size: " << size << std::endl;

    // for (int i = 0; i < size; i++) {
    // std::cout << "data_vec_"<<i<<": " << data_vec[i] << std::endl;
    // }


    for (int i = 0; i < size; i++) {
    auto vec_size = data_size_vec[i];
    
    // std::cout << "vec_size: " << vec_size << std::endl;

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
    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::memcpy(&VehBusIn_, data_in.data(), sizeof(VehBusIn));

        // std::cout<< "Print VehBusIn changed value"<< std::endl;
        
        print_VehBusIn(VehBusIn_,VehBusIn_old);
        // std::cout<< "----------------------------------End"<< std::endl;




        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        VehBusIn_old = VehBusIn_;// no pointer
        //std::memcpy(&VehBusIn_old, data_in.data(), sizeof(VehBusIn));
    }
    return 0;
}

void AdApter_VehBusIn::run()
{
    config_async_sub(json_file);
}
AdApter_VehBusIn::AdApter_VehBusIn()
{
}
AdApter_VehBusIn::~AdApter_VehBusIn()
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
    std::cout << "Running on Linux(Sub)"<< fullPath << std::endl;
#endif

    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_VehBusIn objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
return 0;
}
















