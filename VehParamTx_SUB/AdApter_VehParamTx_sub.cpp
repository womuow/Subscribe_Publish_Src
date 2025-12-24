#include "AdApter_VehParamTx_sub.h"


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
    AdApter_VehParam_Tx AdApter_VehParam_Tx_;
    AdApter_VehParam_Tx_.json_file = json_file;
    int flag=true;

    int domain_id=0;
    std::string topic ="IPCC/VehParam_Tx";
    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libVehParam_Tx", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    VehParam_Tx VehParam_Tx_;
    VehParam_Tx VehParam_Tx_old;

    std::map<std::string, VariableVariant > variableMap = {
    {"VehParam_Tx_.AxleDstReToVehFrnt(float32)" , &VehParam_Tx_.AxleDstReToVehFrnt},
    {"VehParam_Tx_.SingleTrackAxleDistFrnt(float32)" , &VehParam_Tx_.SingleTrackAxleDistFrnt},
    {"VehParam_Tx_.SteerWhlPosn(uint8)" , &VehParam_Tx_.SteerWhlPosn},
    {"VehParam_Tx_.Len(float32)" , &VehParam_Tx_.Len},
    {"VehParam_Tx_.Weight(float32)" , &VehParam_Tx_.Weight},
    {"VehParam_Tx_.WhlBas(float32)" , &VehParam_Tx_.WhlBas},
    {"VehParam_Tx_.Width(float32)" , &VehParam_Tx_.Width},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[0](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[0]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[1](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[1]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[2](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[2]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[3](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[3]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[4](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[4]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[5](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[5]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[6](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[6]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[7](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[7]},
    {"VehParam_Tx_.SingleTrackCornrgStfnFrnt(float32)" , &VehParam_Tx_.SingleTrackCornrgStfnFrnt},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[0](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[0]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[1](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[1]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[2](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[2]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[3](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[3]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[4](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[4]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[5](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[5]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[6](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[6]},
    {"VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[7](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[7]},
    {"VehParam_Tx_.SingleTrackCornrgStfnRe(float32)" , &VehParam_Tx_.SingleTrackCornrgStfnRe},
    {"VehParam_Tx_.SteerWhlAgRat(float32)" , &VehParam_Tx_.SteerWhlAgRat},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[0](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[0]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[1](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[1]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[2](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[2]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[3](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[3]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[4](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[4]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[5](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[5]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[6](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[6]},
    {"VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[7](float32)" , &VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[7]},
    {"VehParam_Tx_.BltFrntExist(uint8)" , &VehParam_Tx_.BltFrntExist},
    {"VehParam_Tx_.OncomingBrk(uint8)" , &VehParam_Tx_.OncomingBrk},
    {"VehParam_Tx_.SelfStrGrdt(float32)" , &VehParam_Tx_.SelfStrGrdt},
    {"VehParam_Tx_.TrafficAssist(uint8)" , &VehParam_Tx_.TrafficAssist},
    {"VehParam_Tx_.LongCtrlBrkLim(uint8)" , &VehParam_Tx_.LongCtrlBrkLim},
    {"VehParam_Tx_.LongCtrEco(uint8)" , &VehParam_Tx_.LongCtrEco},
    {"VehParam_Tx_.LongCtrSpdLoLim(float32)" , &VehParam_Tx_.LongCtrSpdLoLim},
    {"VehParam_Tx_.LongCtrStopNGo(uint8)" , &VehParam_Tx_.LongCtrStopNGo},
    {"VehParam_Tx_.SingleTrackMomentOfInertia(float32)" , &VehParam_Tx_.SingleTrackMomentOfInertia},
    {"VehParam_Tx_.VehTyp(uint8)" , &VehParam_Tx_.VehTyp},
    {"VehParam_Tx_.WhlRadius(float32)" , &VehParam_Tx_.WhlRadius},



    };

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
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::memcpy(&VehParam_Tx_, data_in.data(), sizeof(VehParam_Tx));

        // std::cout<< "Print VehParam_Tx changed value"<< std::endl;
        print_VehParam_Tx(VehParam_Tx_,VehParam_Tx_old);
        // std::cout<< "----------------------------------End"<< std::endl;




        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::memcpy(&VehParam_Tx_old, data_in.data(), sizeof(VehParam_Tx));
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
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_VehParam_Tx objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
return 0;
}


/* Print struct VehParam_Tx changed value */
void print_VehParam_Tx(VehParam_Tx& VehParam_Tx_,VehParam_Tx& VehParam_Tx_old){
// std::cout << "VehParam_Tx all variable:" << std::endl;
    if(VehParam_Tx_.AxleDstReToVehFrnt != VehParam_Tx_old.AxleDstReToVehFrnt){
        std::cout << "VehParam_Tx_.AxleDstReToVehFrnt(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.AxleDstReToVehFrnt << std::endl;
        }
    if(VehParam_Tx_.SingleTrackAxleDistFrnt != VehParam_Tx_old.SingleTrackAxleDistFrnt){
        std::cout << "VehParam_Tx_.SingleTrackAxleDistFrnt(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackAxleDistFrnt << std::endl;
        }
    if(VehParam_Tx_.SteerWhlPosn != VehParam_Tx_old.SteerWhlPosn){
        std::cout << "VehParam_Tx_.SteerWhlPosn(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.SteerWhlPosn) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.Len != VehParam_Tx_old.Len){
        std::cout << "VehParam_Tx_.Len(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.Len << std::endl;
        }
    if(VehParam_Tx_.Weight != VehParam_Tx_old.Weight){
        std::cout << "VehParam_Tx_.Weight(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.Weight << std::endl;
        }
    if(VehParam_Tx_.WhlBas != VehParam_Tx_old.WhlBas){
        std::cout << "VehParam_Tx_.WhlBas(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.WhlBas << std::endl;
        }
    if(VehParam_Tx_.Width != VehParam_Tx_old.Width){
        std::cout << "VehParam_Tx_.Width(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.Width << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[0] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[0]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[0](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[0] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[1] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[1]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[1](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[1] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[2] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[2]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[2](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[2] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[3] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[3]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[3](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[3] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[4] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[4]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[4](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[4] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[5] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[5]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[5](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[5] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[6] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[6]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[6](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[6] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[7] != VehParam_Tx_old.SingleTrackCornrgStfnFrntByVehSpd[7]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[7](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrntByVehSpd[7] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnFrnt != VehParam_Tx_old.SingleTrackCornrgStfnFrnt){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnFrnt(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnFrnt << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[0] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[0]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[0](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[0] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[1] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[1]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[1](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[1] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[2] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[2]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[2](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[2] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[3] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[3]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[3](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[3] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[4] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[4]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[4](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[4] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[5] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[5]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[5](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[5] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[6] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[6]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[6](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[6] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[7] != VehParam_Tx_old.SingleTrackCornrgStfnReByVehSpd[7]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[7](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnReByVehSpd[7] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnRe != VehParam_Tx_old.SingleTrackCornrgStfnRe){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnRe(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnRe << std::endl;
        }
    if(VehParam_Tx_.SteerWhlAgRat != VehParam_Tx_old.SteerWhlAgRat){
        std::cout << "VehParam_Tx_.SteerWhlAgRat(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SteerWhlAgRat << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[0] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[0]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[0](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[0] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[1] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[1]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[1](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[1] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[2] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[2]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[2](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[2] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[3] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[3]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[3](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[3] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[4] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[4]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[4](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[4] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[5] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[5]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[5](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[5] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[6] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[6]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[6](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[6] << std::endl;
        }
    if(VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[7] != VehParam_Tx_old.SingleTrackCornrgStfnTable_Spd[7]){
        std::cout << "VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[7](float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackCornrgStfnTable_Spd[7] << std::endl;
        }
    if(VehParam_Tx_.BltFrntExist != VehParam_Tx_old.BltFrntExist){
        std::cout << "VehParam_Tx_.BltFrntExist(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.BltFrntExist) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.OncomingBrk != VehParam_Tx_old.OncomingBrk){
        std::cout << "VehParam_Tx_.OncomingBrk(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.OncomingBrk) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.SelfStrGrdt != VehParam_Tx_old.SelfStrGrdt){
        std::cout << "VehParam_Tx_.SelfStrGrdt(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SelfStrGrdt << std::endl;
        }
    if(VehParam_Tx_.TrafficAssist != VehParam_Tx_old.TrafficAssist){
        std::cout << "VehParam_Tx_.TrafficAssist(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.TrafficAssist) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.LongCtrlBrkLim != VehParam_Tx_old.LongCtrlBrkLim){
        std::cout << "VehParam_Tx_.LongCtrlBrkLim(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.LongCtrlBrkLim) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.LongCtrEco != VehParam_Tx_old.LongCtrEco){
        std::cout << "VehParam_Tx_.LongCtrEco(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.LongCtrEco) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.LongCtrSpdLoLim != VehParam_Tx_old.LongCtrSpdLoLim){
        std::cout << "VehParam_Tx_.LongCtrSpdLoLim(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.LongCtrSpdLoLim << std::endl;
        }
    if(VehParam_Tx_.LongCtrStopNGo != VehParam_Tx_old.LongCtrStopNGo){
        std::cout << "VehParam_Tx_.LongCtrStopNGo(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.LongCtrStopNGo) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.SingleTrackMomentOfInertia != VehParam_Tx_old.SingleTrackMomentOfInertia){
        std::cout << "VehParam_Tx_.SingleTrackMomentOfInertia(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.SingleTrackMomentOfInertia << std::endl;
        }
    if(VehParam_Tx_.VehTyp != VehParam_Tx_old.VehTyp){
        std::cout << "VehParam_Tx_.VehTyp(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehParam_Tx_.VehTyp) << std::dec  << std::endl;
        }
    if(VehParam_Tx_.WhlRadius != VehParam_Tx_old.WhlRadius){
        std::cout << "VehParam_Tx_.WhlRadius(float32): " << std::fixed << std::setprecision(2) << VehParam_Tx_.WhlRadius << std::endl;
        }
}





