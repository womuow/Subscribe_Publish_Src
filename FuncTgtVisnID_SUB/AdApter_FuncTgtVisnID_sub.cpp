#include "AdApter_FuncTgtVisnID_sub.h"


using namespace std::chrono_literals;
using VariableVariant = std::variant<uint8*, uint16* ,uint32*,float32*,sint8*,sint16*,sint32*>;
enum VarType {UINT8=0,UINT16,UINT32,FLOAT32,SINT8,SINT16,SINT32};
struct VarInfo{
    VariableVariant  var;
    VarType type;
};

void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}
void asyncInputThread() {
    std::string input;
    while (keepRunning) {
        std::getline(std::cin, input);
        
        std::lock_guard<std::mutex> lock(inputMutex);
        if (!input.empty()) {
            inputQueue.push(input);
        }
    }
}
void printVariableVariant(const std::string& name, VariableVariant var) {
    std::visit([&name](auto&& ptr) {
        using T = std::decay_t<decltype(*ptr)>;
        
        // 根据类型决定打印格式
        if constexpr (std::is_same_v<T, uint8> ) {
            // 8位类型：宽度2
            std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr (std::is_same_v<T, uint16> ) {
            // 16位类型：宽度4
            std::cout << name << ": 0x" << std::hex << std::setw(4) << std::setfill('0') 
                      << *ptr << std::dec << std::endl;
        }
        else if constexpr (std::is_same_v<T, uint32> ) {
            // 32位类型：宽度8
            std::cout << name << ": 0x" << std::hex << std::setw(8) << std::setfill('0') 
                      << *ptr << std::dec << std::endl;
        }
        
        else if constexpr (std::is_same_v<T, sint8>) {
            // 8位类型：宽度2
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr ( std::is_same_v<T, sint16>) {
            // 16位类型：宽度4
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        }
        else if constexpr ( std::is_same_v<T, sint32>) {
            // 32位类型：宽度8
            std::cout << name << ": " << std::dec<< std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        }

        else if constexpr (std::is_same_v<T, float32>) {
            // 浮点类型：十进制
            std::cout << name << ": " << std::fixed << std::setprecision(2) << *ptr << std::endl;
        }
    }, var);
}
void getVariableValue(std::map<std::string, VariableVariant> VarMap,std::string input)
{
    

    if (input.length()<=6)
    {
        std::cout << "error: '" << input << "' is too short" << std::endl;
        return ;
    }
    // 去除首尾空格
    size_t start = input.find_first_not_of(" \t");
    if (start == std::string::npos) {
        return ; //空输入
    }
    
    size_t end = input.find_last_not_of(" \t");
    input = input.substr(start, end - start + 1);

    //std::string varName = input.substr(6);
    auto it = VarMap.find(input);
    if (it != VarMap.end()) {
        printVariableVariant(input,it->second);
    } else {
        std::cout << "error: '" << input << "' no found" << std::endl;
    }
}


bool g_flag = true;

int config_async_sub(std::string json_file) {
    

    AdApter_FuncTgtVisnID AdApter_FuncTgtVisnID_;
    AdApter_FuncTgtVisnID_.json_file = json_file;
    int flag=true;

    int domain_id=0;
    std::string topic ="IPCC/FuncTgtVisnID";
    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libFuncTgtVisnID", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    IDT_FuncTgtVisnID IDT_FuncTgtVisnID_;
    IDT_FuncTgtVisnID IDT_FuncTgtVisnID_old;

        std::map<std::string, VariableVariant > variableMap = {
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID},
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID},
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID},
        {"IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID},
        {"IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID},


    };


    std::thread inputThread(asyncInputThread);
    auto sub = MOS::communication::Subscriber::New(
        domain_id,
        topic, 
        proto_info, 
        [&data_in](MOS::message::spMsg tmp) {

        auto data_vec = tmp->GetDataRef()->GetDataVec();
        auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
        auto size = data_size_vec.size();
    



        for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        

        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
        if (g_flag)
        {
            std::cout << "data_in.data() hex: g_flag="  <<g_flag<< "size="<<data_in.size()<< std::endl;
            print_memory(data_in.data(),data_in.size());  
            g_flag=false;
        }
        }
    }
);
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


/* Print struct IDT_FuncTgtVisnID changed value */
void print_IDT_FuncTgtVisnID(IDT_FuncTgtVisnID& IDT_FuncTgtVisnID_,IDT_FuncTgtVisnID& IDT_FuncTgtVisnID_old){
// std::cout << "IDT_FuncTgtVisnID all variable:" << std::endl;
    if(IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID != IDT_FuncTgtVisnID_old.LongTgtVisnID.ACCTgtVisnID){
        std::cout << "IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID) << std::dec  << std::endl;
        }
    if(IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID != IDT_FuncTgtVisnID_old.LongTgtVisnID.CutInTgtVisnID){
        std::cout << "IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID) << std::dec  << std::endl;
        }
    if(IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID != IDT_FuncTgtVisnID_old.LongTgtVisnID.FDWTgtVisnID){
        std::cout << "IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID) << std::dec  << std::endl;
        }
    if(IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID != IDT_FuncTgtVisnID_old.CATgtVisnID.AEBTgtVisnID){
        std::cout << "IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID) << std::dec  << std::endl;
        }
    if(IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID != IDT_FuncTgtVisnID_old.CATgtVisnID.FCWTgtVisnID){
        std::cout << "IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID) << std::dec  << std::endl;
        }
}









