#include "AdApter_VehBusIn_sub.h"

using namespace std::chrono_literals;



void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

bool g_flag = true;

int config_async_sub(std::string json_file) {
    AdApter_VehBusIn AdApter_VehBusIn_;
    AdApter_VehBusIn_.json_file = json_file;
    int flag=true;

    int domain_id=0;
    std::string topic ="IPCC/VehBusIn";
    std::string data_in="";
    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libVehBusIn", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;

    VehBusIn VehBusIn_;
    VehBusIn VehBusIn_old;    



    auto sub = MOS::communication::Subscriber::New(
        domain_id,
        topic, 
        proto_info, 
        [&data_in](MOS::message::spMsg tmp) {

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


/* Print struct VehBusIn changed value */
void print_VehBusIn(VehBusIn& VehBusIn_,VehBusIn& VehBusIn_old){
// std::cout << "VehBusIn all variable:" << std::endl;
    if(VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB != VehBusIn_old.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB != VehBusIn_old.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_CB_APP.Message_QF != VehBusIn_old.SysSigGrp_HPCGW_CB_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_CB_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCGW_CB_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected != VehBusIn_old.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD != VehBusIn_old.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts != VehBusIn_old.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.Message_QF != VehBusIn_old.SysSigGrp_HPCGW_3B0_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_HPCGW_3B0_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCGW_3B0_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.Message_QF != VehBusIn_old.SysSigGrp_HPCVCU_B6_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B6_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B6_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB(sint16): " << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.Message_QF != VehBusIn_old.SysSigGrp_HPCVCU_B7_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_HPCVCU_B7_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCVCU_B7_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BackDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_BackDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BackDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BackDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FLDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FRDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FRDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FRDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FRDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3 != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RLDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RRDoorSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RRDoorSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RRDoorSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RRDoorSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts != VehBusIn_old.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_290.MessageQf != VehBusIn_old.SysSigGrp_HPCBCM_290.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_290.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_290.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_24D.ICU_AccelerationMode != VehBusIn_old.SysSigGrp_ICU_24D.ICU_AccelerationMode){
        std::cout << "VehBusIn_.SysSigGrp_ICU_24D.ICU_AccelerationMode(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_24D.ICU_AccelerationMode) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_24D.ICU_TotalOdometerkm != VehBusIn_old.SysSigGrp_ICU_24D.ICU_TotalOdometerkm){
        std::cout << "VehBusIn_.SysSigGrp_ICU_24D.ICU_TotalOdometerkm(uint32): 0x" << std::hex << std::setw(8) << std::setfill('0') << VehBusIn_.SysSigGrp_ICU_24D.ICU_TotalOdometerkm << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_24D.MessageQf != VehBusIn_old.SysSigGrp_ICU_24D.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_ICU_24D.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_24D.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset(sint8): " << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_SLWF_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_SLIF_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_ISA_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ELK_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_ELK_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ELK_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ELK_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LKA_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LKA_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LKA_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LKA_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LDW_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LCC_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_LCC_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LCC_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LCC_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IHBC_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_IHBC_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IHBC_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IHBC_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_FCW_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_AEB_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_AEB_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_AEB_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_AEB_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTB_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_RCTB_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTB_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTB_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCW_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_RCW_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCW_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCW_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTA_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_RCTA_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTA_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTA_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_DOW_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_DOW_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_DOW_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_DOW_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_BSD_SW != VehBusIn_old.SysSigGrp_ICU_332_APP.ICU_BSD_SW){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.ICU_BSD_SW(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.ICU_BSD_SW) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ICU_332_APP.Message_QF != VehBusIn_old.SysSigGrp_ICU_332_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_ICU_332_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ICU_332_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Data != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Data){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Data(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Data) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Hour != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Hour){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Hour(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Hour) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Minute != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Minute){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Minute(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Minute) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Month != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Month){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Month(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Month) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Second != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Second){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Second(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Second) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_5CE.IVI_Year != VehBusIn_old.SysSigGrp_IVI_5CE.IVI_Year){
        std::cout << "VehBusIn_.SysSigGrp_IVI_5CE.IVI_Year(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IVI_5CE.IVI_Year << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_CrashOutputSts != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_CrashOutputSts){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_CrashOutputSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_CrashOutputSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt != VehBusIn_old.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_2A3.MessageQf != VehBusIn_old.SysSigGrp_ACU_2A3.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_ACU_2A3.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_2A3.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcce != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_LateralAcce){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcce(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcce << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcce != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_LongitAcce){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcce(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcce << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRate != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_YawRate){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRate(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRate << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRateValid != VehBusIn_old.SysSigGrp_ACU_233_APP.ACU_YawRateValid){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRateValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRateValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_ACU_233_APP.Message_QF != VehBusIn_old.SysSigGrp_ACU_233_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_ACU_233_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_ACU_233_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_MotorTorque != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_MotorTorque){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_MotorTorque(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_MotorTorque << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralActive != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_LateralActive){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralActive(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralActive) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_ != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_ << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_FailureSts != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_FailureSts){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_FailureSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_FailureSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_Fault != VehBusIn_old.SysSigGrp_EPS_18D_APP.EPS_Fault){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_Fault(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_Fault) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_EPS_18D_APP.Message_QF != VehBusIn_old.SysSigGrp_EPS_18D_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_EPS_18D_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_EPS_18D_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCFailed != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_VDCFailed){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCFailed(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCFailed) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCActive != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_VDCActive){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCActive(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCActive) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_EBDFailed != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_EBDFailed){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_EBDFailed(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_EBDFailed) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSFailed != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_TCSFailed){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSFailed(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSFailed) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSActive != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_TCSActive){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSActive(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSActive) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSFailed != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_ABSFailed){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSFailed(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSFailed) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSActive != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_ABSActive){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSActive(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSActive) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed != VehBusIn_old.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_182_APP.Message_QF != VehBusIn_old.SysSigGrp_IBC_182_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_IBC_182_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_182_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied != VehBusIn_old.SysSigGrp_IBC_183.IBC_BrakePedalApplied){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q != VehBusIn_old.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct != VehBusIn_old.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer != VehBusIn_old.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q != VehBusIn_old.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_183.MessageQf != VehBusIn_old.SysSigGrp_IBC_183.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_IBC_183.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_183.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC(uint16): 0x" << std::hex << std::setw(4) << std::setfill('0') << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH != VehBusIn_old.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_184_APP.Message_QF != VehBusIn_old.SysSigGrp_IBC_184_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_IBC_184_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_184_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_HDCStatus != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_HDCStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_HDCStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_HDCStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCACC != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_QDCACC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCACC(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCACC) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCAEB != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_QDCAEB){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCAEB(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCAEB) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHAvailable != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AVHAvailable){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHAvailable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHAvailable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHStatus != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AVHStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEB_active != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AEB_active){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEB_active(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEB_active) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LdmBLC != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_LdmBLC){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LdmBLC(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LdmBLC) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABP_active != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ABP_active){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABP_active(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABP_active) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABPAviliable != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ABPAviliable){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABPAviliable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABPAviliable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWB_active != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AWB_active){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWB_active(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWB_active) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABA_active != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ABA_active){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABA_active(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABA_active) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_PlungerPressure){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure(float32): " << std::fixed << std::setprecision(2) << VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus != VehBusIn_old.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_185_APP.Message_QF != VehBusIn_old.SysSigGrp_IBC_185_APP.Message_QF){
        std::cout << "VehBusIn_.SysSigGrp_IBC_185_APP.Message_QF(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_185_APP.Message_QF) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available != VehBusIn_old.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available){
        std::cout << "VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_FailStatus != VehBusIn_old.SysSigGrp_IBC_227.IBC_EPB_FailStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_FailStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_FailStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_Status != VehBusIn_old.SysSigGrp_IBC_227.IBC_EPB_Status){
        std::cout << "VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_Status(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_Status) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_227.MessageQf != VehBusIn_old.SysSigGrp_IBC_227.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_IBC_227.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_227.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_3CE.iTPMS_SystemStatus != VehBusIn_old.SysSigGrp_IBC_3CE.iTPMS_SystemStatus){
        std::cout << "VehBusIn_.SysSigGrp_IBC_3CE.iTPMS_SystemStatus(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_3CE.iTPMS_SystemStatus) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_3CE.MessageQf != VehBusIn_old.SysSigGrp_IBC_3CE.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_IBC_3CE.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_3CE.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq != VehBusIn_old.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq){
        std::cout << "VehBusIn_.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A != VehBusIn_old.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B != VehBusIn_old.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C != VehBusIn_old.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D != VehBusIn_old.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A != VehBusIn_old.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B != VehBusIn_old.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C != VehBusIn_old.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D != VehBusIn_old.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D){
        std::cout << "VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW1 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW1){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW1(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW1) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW10 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW10){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW10(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW10) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW2 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW2){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW2(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW2) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW3 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW3){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW3(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW3) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW4 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW4){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW4(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW4) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW5 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW5){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW5(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW5) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW6 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW6){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW6(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW6) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW7 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW7){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW7(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW7) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW8 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW8){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW8(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW8) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW9 != VehBusIn_old.SysSigGrp_IBC_6F3.IBC_UDS_SW9){
        std::cout << "VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW9(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW9) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_25A.LMC_LSFCActiveSt != VehBusIn_old.SysSigGrp_LMC_25A.LMC_LSFCActiveSt){
        std::cout << "VehBusIn_.SysSigGrp_LMC_25A.LMC_LSFCActiveSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_25A.LMC_LSFCActiveSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_25A.MessageQf != VehBusIn_old.SysSigGrp_LMC_25A.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_LMC_25A.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_25A.MessageQf) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt != VehBusIn_old.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt){
        std::cout << "VehBusIn_.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt != VehBusIn_old.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt){
        std::cout << "VehBusIn_.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt != VehBusIn_old.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt){
        std::cout << "VehBusIn_.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt) << std::dec  << std::endl;
        }
    if(VehBusIn_.SysSigGrp_LMC_FD.MessageQf != VehBusIn_old.SysSigGrp_LMC_FD.MessageQf){
        std::cout << "VehBusIn_.SysSigGrp_LMC_FD.MessageQf(uint8): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(VehBusIn_.SysSigGrp_LMC_FD.MessageQf) << std::dec  << std::endl;
        }
}
















