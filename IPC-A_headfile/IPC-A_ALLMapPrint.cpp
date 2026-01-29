#include "IPC-A_ALLMapPrint.h"


std::queue<std::string> inputQueue;
int flag=true;
std::string data_in={0x0};
std::string data_in2={0x0};
std::atomic_bool stop{ false };
std::atomic_bool stop2{ false };
IPC_MSG_DATA_SIZE_MAX ipc_msg_;







void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}


void asyncInputThreadTTY() {




    int serial_fd = open("/dev/ttyS1",  O_NOCTTY | O_RDWR );
    if (serial_fd == -1) {
        std::cerr << "can not open serial port /dev/ttyS1" << std::endl;
        return ;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));


    
    // 2. 配置串口参数
    struct termios options;
    struct termios options_old;
    tcgetattr(serial_fd, &options);
    options_old = options;
    //print_flags(&options_old);
    // 设置波特率
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);
    
    // 重要：禁用本地回显，不读取我们自己的输出
    // options.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);    

    // 设置行结束符为换行符'\n'
    options.c_cc[VEOL] = '\n';    // 行结束符
    //options.c_cc[VEOL2] = '\0';   // 第二个行结束符（可选）
    
    // 设置超时
    options.c_cc[VMIN] = 0;     // 最小读取字符数
    options.c_cc[VTIME] = 5;    // 0.5秒超时
    // 应用设置
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
        std::cerr << "serial port configure Fail" << std::endl;
        close(serial_fd);
        return ;
    }
    
    std::cout << "Serial  have opened..." << std::endl;
    
    // 清空输入缓冲区，确保只读取新数据
    tcflush(serial_fd, TCIFLUSH);
    
    // 3. 从串口读取数据
    char buffer[1024];
    std::string lineBuffer;  // 用于累积行数据
    
    while(true)
    {
        memset(buffer, 0, sizeof(buffer));

        // 读取数据
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);

        // std::cout<<"bytes_read="<<bytes_read<<std::endl;
        if(bytes_read>0)
        {
        // 将读取的数据追加到行缓冲区
        lineBuffer.append(buffer, bytes_read);
        // 检查行缓冲区中是否有完整的行
        size_t newline_pos;
        newline_pos = lineBuffer.find('\n');
        {
            // 提取一行（包含换行符）
            std::string line = lineBuffer.substr(0, newline_pos + 1);
            
            // 从行缓冲区中移除已处理的这一行
            lineBuffer.erase(0, newline_pos + 1);
            
        // std::cout<<"line="<<line<<std::endl<<"line="<<line<<std::endl;
            // 去除可能的回车符\r
            if (!line.empty() && line.back() == '\n') 
            {
                if ( line.compare("quit\n")==0 || line.compare("Quit\n")==0||line.compare("QUIT\n")==0)
                {
                    if(tcsetattr(serial_fd,TCSANOW,&options_old)==0)
                             std::cout << "Serial port settings have been restored" << std::endl;
                    else
                        std::cerr << "Failed to restore serial port settings, error: " << strerror(errno) << std::endl;
                    close(serial_fd);

                    return ;
                }
                if (line.data()[0]=='\n' || line.data()[0]=='\0' ||line.data()[0]=='.')
                {
                    continue ;
                }
                if (line.size() > 1 && line[line.size() - 1] == '\n') {
                    // 如果是\r\n结尾，去除\r
                    line.pop_back();  // 先去掉\n
                    // line.push_back('\n');  // 重新添加\n
                }
                if (line.size() > 1 && line[line.size() - 1] == '\r') {
                    // 如果是\r\n结尾，去除\r
                    line.pop_back();  // 再去掉\r
                    // line.push_back('\n');  // 重新添加\n
                }
                
            }
            
            // 将完整的行推入队列            
            if (inputQueue.empty())
            {
                inputQueue.push(line);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                tcflush(serial_fd, TCIOFLUSH);
            // close(serial_fd);
            // break;
            continue;
            
            }
        }
        }
        else 
        {
            continue;
        }
        
    }
}


// #ifndef IPC_A_H

void printVariableVariant(const std::string& name, VariableVariant var) {
    std::visit([&name](auto&& ptr) {
        using T = std::decay_t<decltype(*ptr)>;
        
        // 根据类型决定打印格式
        if constexpr (std::is_same_v<T, bool> ) {
            // 8位类型：宽度2
            std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr (std::is_same_v<T, uint8_t> ) {
            // 8位类型：宽度2
            std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr (std::is_same_v<T, uint16_t> ) {
            // 16位类型：宽度4
            std::cout << name << ": 0x" << std::hex << std::setw(4) << std::setfill('0') 
                      << *ptr << std::dec  << std::endl;
        }
        else if constexpr (std::is_same_v<T, uint32_t> ) {
            // 32位类型：宽度8
            std::cout << name << ": 0x" << std::hex << std::setw(8) << std::setfill('0') 
                      << *ptr << std::dec  << std::endl;
        }
        else if constexpr (std::is_same_v<T, uint64_t> ) {
            // 32位类型：宽度8
            std::cout << name << ": 0x" << std::hex << std::setw(16) << std::setfill('0') 
                      << *ptr << std::dec  << std::endl;
        }
        else if constexpr ( std::is_same_v<T, short>) {
            // 16位类型：宽度4
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec  << std::endl;
        }
        else if constexpr ( std::is_same_v<T, int>) {
            // 32位类型：宽度8
            std::cout << name << ": " << std::dec<< std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec  << std::endl;
        }

        else if constexpr (std::is_same_v<T, float>) {
            // 浮点类型：十进制
            std::cout << name << ": " << std::fixed << std::setprecision(2) << *ptr  << std::endl;
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
    size_t start = input.find_first_not_of("\n");
    if (start == std::string::npos) {
        return ; //空输入
    }
    
    size_t end = input.find_last_not_of(" \t");
    input = input.substr(start, end - start + 1);

    //std::string varName = input.substr(6);
    auto it = VarMap.find(input);
    if (it != VarMap.end()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        printVariableVariant(input,it->second);
    } else {
        std::cout << "error: '" << input << "' no found" << std::endl;
        return ;
    }
}

// #endif


#ifdef IPC_MSG_DATA_SIZE_MAX_H

std::map<std::string, VariableVariant > IPC_MSG_DATA_SIZE_MAX_Map = {
//[IPC_MSG_DATA_SIZE_MAX]
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},

{"IPC_MSG_DATA_SIZE_MAX_.header.id(uint16_t)" , &IPC_MSG_DATA_SIZE_MAX_.header.id},
{"IPC_MSG_DATA_SIZE_MAX_.header.version(uint16_t)" , &IPC_MSG_DATA_SIZE_MAX_.header.version},
{"IPC_MSG_DATA_SIZE_MAX_.header.data_size(uint16_t)" , &IPC_MSG_DATA_SIZE_MAX_.header.data_size},
{"IPC_MSG_DATA_SIZE_MAX_.header.target(uint16_t)" , &IPC_MSG_DATA_SIZE_MAX_.header.target},
{"IPC_MSG_DATA_SIZE_MAX_.header.timestamp(uint64_t)" , &IPC_MSG_DATA_SIZE_MAX_.header.timestamp},
{"IPC_MSG_DATA_SIZE_MAX_.data[0](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[0]},
{"IPC_MSG_DATA_SIZE_MAX_.data[1](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[1]},
{"IPC_MSG_DATA_SIZE_MAX_.data[2](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[2]},
{"IPC_MSG_DATA_SIZE_MAX_.data[3](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[3]},
{"IPC_MSG_DATA_SIZE_MAX_.data[4](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[4]},
{"IPC_MSG_DATA_SIZE_MAX_.data[5](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[5]},
{"IPC_MSG_DATA_SIZE_MAX_.data[6](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[6]},
{"IPC_MSG_DATA_SIZE_MAX_.data[7](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[7]},
{"IPC_MSG_DATA_SIZE_MAX_.data[8](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[8]},
{"IPC_MSG_DATA_SIZE_MAX_.data[9](uint8_t)" , &IPC_MSG_DATA_SIZE_MAX_.data[9]},
};
#endif




#ifdef IPC_MSG_DATA_SIZE_MAX_H
/* Print struct IPC_MSG_DATA_SIZE_MAX changed value */
void print_IPC_MSG_DATA_SIZE_MAX(IPC_MSG_DATA_SIZE_MAX& IPC_MSG_DATA_SIZE_MAX_,IPC_MSG_DATA_SIZE_MAX& IPC_MSG_DATA_SIZE_MAX_old){
// std::cout << "IPC_MSG_DATA_SIZE_MAX all variable:" << std::endl;
    if(IPC_MSG_DATA_SIZE_MAX_.header.id != IPC_MSG_DATA_SIZE_MAX_old.header.id){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.header.id(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << IPC_MSG_DATA_SIZE_MAX_.header.id << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.header.version != IPC_MSG_DATA_SIZE_MAX_old.header.version){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.header.version(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << IPC_MSG_DATA_SIZE_MAX_.header.version << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.header.data_size != IPC_MSG_DATA_SIZE_MAX_old.header.data_size){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.header.data_size(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << IPC_MSG_DATA_SIZE_MAX_.header.data_size << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.header.target != IPC_MSG_DATA_SIZE_MAX_old.header.target){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.header.target(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << IPC_MSG_DATA_SIZE_MAX_.header.target << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.header.timestamp != IPC_MSG_DATA_SIZE_MAX_old.header.timestamp){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.header.timestamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << IPC_MSG_DATA_SIZE_MAX_.header.timestamp << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[0] != IPC_MSG_DATA_SIZE_MAX_old.data[0]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[0]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[1] != IPC_MSG_DATA_SIZE_MAX_old.data[1]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[1]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[2] != IPC_MSG_DATA_SIZE_MAX_old.data[2]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[2]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[3] != IPC_MSG_DATA_SIZE_MAX_old.data[3]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[3]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[4] != IPC_MSG_DATA_SIZE_MAX_old.data[4]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[4]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[5] != IPC_MSG_DATA_SIZE_MAX_old.data[5]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[5]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[6] != IPC_MSG_DATA_SIZE_MAX_old.data[6]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[6]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[7] != IPC_MSG_DATA_SIZE_MAX_old.data[7]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[7]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[8] != IPC_MSG_DATA_SIZE_MAX_old.data[8]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[8]) << std::dec  << std::endl;
        }
    if(IPC_MSG_DATA_SIZE_MAX_.data[9] != IPC_MSG_DATA_SIZE_MAX_old.data[9]){
        std::cout << "IPC_MSG_DATA_SIZE_MAX_.data[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(IPC_MSG_DATA_SIZE_MAX_.data[9]) << std::dec  << std::endl;
        }
}
#endif


#if defined(INTRINSIC_CALIBRATION_PARAMETERS_H) || defined(PERCEPTION_VEHICLE_PARAMETERS_H)
std::map<std::string, VariableVariant > Parameter_Map = {

{"ipc_msg_crc16_.header.id",&ipc_msg_crc16_.header.id},
{"ipc_msg_crc16_.header.version",&ipc_msg_crc16_.header.version},
{"ipc_msg_crc16_.header.data_size",&ipc_msg_crc16_.header.data_size},
{"ipc_msg_crc16_.header.target",&ipc_msg_crc16_.header.target},
{"ipc_msg_crc16_.header.timestamp",&ipc_msg_crc16_.header.timestamp},
{"ipc_msg_crc16_.header.data[0]",&ipc_msg_crc16_.header.data[0]},
{"ipc_msg_crc16_.CRC16",&ipc_msg_crc16_.CRC16},

//[Perception_Vehicle_parameters]
{"Perception_Vehicle_parameters_.timestamps(uint64_t)" , &Perception_Vehicle_parameters_.timestamps},
{"Perception_Vehicle_parameters_.veh_config_type(uint8_t)" , &Perception_Vehicle_parameters_.veh_config_type},
{"Perception_Vehicle_parameters_.Platform_Type(uint8_t)" , &Perception_Vehicle_parameters_.Platform_Type},
{"Perception_Vehicle_parameters_.k_BSW_DistFrontToRearAxle(uint16_t)" , &Perception_Vehicle_parameters_.k_BSW_DistFrontToRearAxle},
{"Perception_Vehicle_parameters_.k_WheelBase(uint16_t)" , &Perception_Vehicle_parameters_.k_WheelBase},
{"Perception_Vehicle_parameters_.k_BSW_HostVehLength(uint16_t)" , &Perception_Vehicle_parameters_.k_BSW_HostVehLength},
{"Perception_Vehicle_parameters_.k_front_cornering_compliance(float)" , &Perception_Vehicle_parameters_.k_front_cornering_compliance},
{"Perception_Vehicle_parameters_.k_YawInertiaAdjustFac(float)" , &Perception_Vehicle_parameters_.k_YawInertiaAdjustFac},
{"Perception_Vehicle_parameters_.k_RearAxleLoadRatio(float)" , &Perception_Vehicle_parameters_.k_RearAxleLoadRatio},
{"Perception_Vehicle_parameters_.k_rear_compliance(float)" , &Perception_Vehicle_parameters_.k_rear_compliance},
{"Perception_Vehicle_parameters_.k_VehWidth_Min(uint16_t)" , &Perception_Vehicle_parameters_.k_VehWidth_Min},
{"Perception_Vehicle_parameters_.k_VehWidth_Max(uint16_t)" , &Perception_Vehicle_parameters_.k_VehWidth_Max},
{"Perception_Vehicle_parameters_.k_WheelRadius(uint16_t)" , &Perception_Vehicle_parameters_.k_WheelRadius},
{"Perception_Vehicle_parameters_.k_WheelWidthAve(uint16_t)" , &Perception_Vehicle_parameters_.k_WheelWidthAve},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_x(sint16)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_x},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_y(sint16)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_y},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_z(uint16_t)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_z},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta_Online(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta_Online},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta_Online(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta_Online},
{"Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta_Online(float)" , &Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta_Online},
};
#endif



#ifdef EDR_INFO_H
std::map<std::string, VariableVariant > EDR_Info_Map = {
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},

//[EDR_Info]
{"EDR_Info_.HPCADAS_AEBDecCtrl(uint8_t)" , &EDR_Info_.HPCADAS_AEBDecCtrl},
{"EDR_Info_.FSC_LCC_Mode(uint8_t)" , &EDR_Info_.FSC_LCC_Mode},
{"EDR_Info_.FSC_LCC_EscapeLevel(uint8_t)" , &EDR_Info_.FSC_LCC_EscapeLevel},
{"EDR_Info_.FSC_emergencyLightReq(uint8_t)" , &EDR_Info_.FSC_emergencyLightReq},
{"EDR_Info_.HPCADAS_LKA_SteeringWheelAngle(uint16_t)" , &EDR_Info_.HPCADAS_LKA_SteeringWheelAngle},
{"EDR_Info_.ADDC_ACCTargetTrq(uint16_t)" , &EDR_Info_.ADDC_ACCTargetTrq},
{"EDR_Info_.ADDC_ACCTargetTrqBrk(uint16_t)" , &EDR_Info_.ADDC_ACCTargetTrqBrk},
{"EDR_Info_.HPCADAS_ACCMode(uint8_t)" , &EDR_Info_.HPCADAS_ACCMode},
};
#endif



#ifdef LOGISTICS_DATA_H
std::map<std::string, VariableVariant > Logistics_data_Map = {
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},

//[Logistics_data]
{"Logistics_data_.HW_Version[0](uint8_t)" , &Logistics_data_.HW_Version[0]},
{"Logistics_data_.HW_Version[1](uint8_t)" , &Logistics_data_.HW_Version[1]},
{"Logistics_data_.HW_Version[2](uint8_t)" , &Logistics_data_.HW_Version[2]},
{"Logistics_data_.HW_Version[3](uint8_t)" , &Logistics_data_.HW_Version[3]},
{"Logistics_data_.HW_Version[4](uint8_t)" , &Logistics_data_.HW_Version[4]},
{"Logistics_data_.HW_Version[5](uint8_t)" , &Logistics_data_.HW_Version[5]},
{"Logistics_data_.HW_Version[6](uint8_t)" , &Logistics_data_.HW_Version[6]},
{"Logistics_data_.HW_Version[7](uint8_t)" , &Logistics_data_.HW_Version[7]},
{"Logistics_data_.HW_Version[8](uint8_t)" , &Logistics_data_.HW_Version[8]},
{"Logistics_data_.HW_Version[9](uint8_t)" , &Logistics_data_.HW_Version[9]},
{"Logistics_data_.SW_Version[0](uint8_t)" , &Logistics_data_.SW_Version[0]},
{"Logistics_data_.SW_Version[1](uint8_t)" , &Logistics_data_.SW_Version[1]},
{"Logistics_data_.SW_Version[2](uint8_t)" , &Logistics_data_.SW_Version[2]},
{"Logistics_data_.SW_Version[3](uint8_t)" , &Logistics_data_.SW_Version[3]},
{"Logistics_data_.SW_Version[4](uint8_t)" , &Logistics_data_.SW_Version[4]},
{"Logistics_data_.SW_Version[5](uint8_t)" , &Logistics_data_.SW_Version[5]},
{"Logistics_data_.SW_Version[6](uint8_t)" , &Logistics_data_.SW_Version[6]},
{"Logistics_data_.SW_Version[7](uint8_t)" , &Logistics_data_.SW_Version[7]},
{"Logistics_data_.SW_Version[8](uint8_t)" , &Logistics_data_.SW_Version[8]},
{"Logistics_data_.SW_Version[9](uint8_t)" , &Logistics_data_.SW_Version[9]},
{"Logistics_data_.ECU_SerialNumber[0](uint8_t)" , &Logistics_data_.ECU_SerialNumber[0]},
{"Logistics_data_.ECU_SerialNumber[1](uint8_t)" , &Logistics_data_.ECU_SerialNumber[1]},
{"Logistics_data_.ECU_SerialNumber[2](uint8_t)" , &Logistics_data_.ECU_SerialNumber[2]},
{"Logistics_data_.ECU_SerialNumber[3](uint8_t)" , &Logistics_data_.ECU_SerialNumber[3]},
{"Logistics_data_.ECU_SerialNumber[4](uint8_t)" , &Logistics_data_.ECU_SerialNumber[4]},
{"Logistics_data_.ECU_SerialNumber[5](uint8_t)" , &Logistics_data_.ECU_SerialNumber[5]},
{"Logistics_data_.ECU_SerialNumber[6](uint8_t)" , &Logistics_data_.ECU_SerialNumber[6]},
{"Logistics_data_.ECU_SerialNumber[7](uint8_t)" , &Logistics_data_.ECU_SerialNumber[7]},
{"Logistics_data_.ECU_SerialNumber[8](uint8_t)" , &Logistics_data_.ECU_SerialNumber[8]},
{"Logistics_data_.ECU_SerialNumber[9](uint8_t)" , &Logistics_data_.ECU_SerialNumber[9]},
{"Logistics_data_.ECU_SerialNumber[10](uint8_t)" , &Logistics_data_.ECU_SerialNumber[10]},
{"Logistics_data_.ECU_SerialNumber[11](uint8_t)" , &Logistics_data_.ECU_SerialNumber[11]},
{"Logistics_data_.ECU_SerialNumber[12](uint8_t)" , &Logistics_data_.ECU_SerialNumber[12]},
{"Logistics_data_.ECU_SerialNumber[13](uint8_t)" , &Logistics_data_.ECU_SerialNumber[13]},
{"Logistics_data_.ECU_SerialNumber[14](uint8_t)" , &Logistics_data_.ECU_SerialNumber[14]},
{"Logistics_data_.ECU_SerialNumber[15](uint8_t)" , &Logistics_data_.ECU_SerialNumber[15]},
{"Logistics_data_.ECU_SerialNumber[16](uint8_t)" , &Logistics_data_.ECU_SerialNumber[16]},
{"Logistics_data_.ECU_SerialNumber[17](uint8_t)" , &Logistics_data_.ECU_SerialNumber[17]},
{"Logistics_data_.ECU_SerialNumber[18](uint8_t)" , &Logistics_data_.ECU_SerialNumber[18]},
{"Logistics_data_.ECU_SerialNumber[19](uint8_t)" , &Logistics_data_.ECU_SerialNumber[19]},
{"Logistics_data_.ECU_SerialNumber[20](uint8_t)" , &Logistics_data_.ECU_SerialNumber[20]},
{"Logistics_data_.ECU_SerialNumber[21](uint8_t)" , &Logistics_data_.ECU_SerialNumber[21]},
{"Logistics_data_.ECU_SerialNumber[22](uint8_t)" , &Logistics_data_.ECU_SerialNumber[22]},
{"Logistics_data_.ECU_SerialNumber[23](uint8_t)" , &Logistics_data_.ECU_SerialNumber[23]},
{"Logistics_data_.ECU_SerialNumber[24](uint8_t)" , &Logistics_data_.ECU_SerialNumber[24]},
{"Logistics_data_.ECU_SerialNumber[25](uint8_t)" , &Logistics_data_.ECU_SerialNumber[25]},
{"Logistics_data_.ECU_SerialNumber[26](uint8_t)" , &Logistics_data_.ECU_SerialNumber[26]},
{"Logistics_data_.ECU_SerialNumber[27](uint8_t)" , &Logistics_data_.ECU_SerialNumber[27]},
{"Logistics_data_.ECU_SerialNumber[28](uint8_t)" , &Logistics_data_.ECU_SerialNumber[28]},
{"Logistics_data_.ECU_SerialNumber[29](uint8_t)" , &Logistics_data_.ECU_SerialNumber[29]},
{"Logistics_data_.VIN_Code[0](uint8_t)" , &Logistics_data_.VIN_Code[0]},
{"Logistics_data_.VIN_Code[1](uint8_t)" , &Logistics_data_.VIN_Code[1]},
{"Logistics_data_.VIN_Code[2](uint8_t)" , &Logistics_data_.VIN_Code[2]},
{"Logistics_data_.VIN_Code[3](uint8_t)" , &Logistics_data_.VIN_Code[3]},
{"Logistics_data_.VIN_Code[4](uint8_t)" , &Logistics_data_.VIN_Code[4]},
{"Logistics_data_.VIN_Code[5](uint8_t)" , &Logistics_data_.VIN_Code[5]},
{"Logistics_data_.VIN_Code[6](uint8_t)" , &Logistics_data_.VIN_Code[6]},
{"Logistics_data_.VIN_Code[7](uint8_t)" , &Logistics_data_.VIN_Code[7]},
{"Logistics_data_.VIN_Code[8](uint8_t)" , &Logistics_data_.VIN_Code[8]},
{"Logistics_data_.VIN_Code[9](uint8_t)" , &Logistics_data_.VIN_Code[9]},
{"Logistics_data_.VIN_Code[10](uint8_t)" , &Logistics_data_.VIN_Code[10]},
{"Logistics_data_.VIN_Code[11](uint8_t)" , &Logistics_data_.VIN_Code[11]},
{"Logistics_data_.VIN_Code[12](uint8_t)" , &Logistics_data_.VIN_Code[12]},
{"Logistics_data_.VIN_Code[13](uint8_t)" , &Logistics_data_.VIN_Code[13]},
{"Logistics_data_.VIN_Code[14](uint8_t)" , &Logistics_data_.VIN_Code[14]},
{"Logistics_data_.VIN_Code[15](uint8_t)" , &Logistics_data_.VIN_Code[15]},
{"Logistics_data_.VIN_Code[16](uint8_t)" , &Logistics_data_.VIN_Code[16]},
};
#endif



#ifdef RCORE_RESET_REQUEST_H
std::map<std::string, VariableVariant > Rcore_reset_request_Map = {
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},

//[Rcore_reset_request]
{"Rcore_reset_request_.R_reset_request(bool)" , &Rcore_reset_request_.R_reset_request},
};
#endif



#ifdef FeaState_H
std::map<std::string, VariableVariant > FeaState_Map = {
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},

//[FeaState]
{"FeaState_.Feature_errcode(uint8_t)" , &FeaState_.Feature_errcode},
};
#endif





#ifdef PERCEPTION_VEHICLE_PARAMETERS_H
/* Print struct Perception_Vehicle_parameters changed value */
void print_Perception_Vehicle_parameters(Perception_Vehicle_parameters& Perception_Vehicle_parameters_,Perception_Vehicle_parameters& Perception_Vehicle_parameters_old){
// std::cout << "Perception_Vehicle_parameters all variable:" << std::endl;
    if(Perception_Vehicle_parameters_.timestamps != Perception_Vehicle_parameters_old.timestamps){
        std::cout << "Perception_Vehicle_parameters_.timestamps(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << Perception_Vehicle_parameters_.timestamps << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.veh_config_type != Perception_Vehicle_parameters_old.veh_config_type){
        std::cout << "Perception_Vehicle_parameters_.veh_config_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Perception_Vehicle_parameters_.veh_config_type) << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.Platform_Type != Perception_Vehicle_parameters_old.Platform_Type){
        std::cout << "Perception_Vehicle_parameters_.Platform_Type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Perception_Vehicle_parameters_.Platform_Type) << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_BSW_DistFrontToRearAxle != Perception_Vehicle_parameters_old.k_BSW_DistFrontToRearAxle){
        std::cout << "Perception_Vehicle_parameters_.k_BSW_DistFrontToRearAxle(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_BSW_DistFrontToRearAxle << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_WheelBase != Perception_Vehicle_parameters_old.k_WheelBase){
        std::cout << "Perception_Vehicle_parameters_.k_WheelBase(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_WheelBase << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_BSW_HostVehLength != Perception_Vehicle_parameters_old.k_BSW_HostVehLength){
        std::cout << "Perception_Vehicle_parameters_.k_BSW_HostVehLength(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_BSW_HostVehLength << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_front_cornering_compliance != Perception_Vehicle_parameters_old.k_front_cornering_compliance){
        std::cout << "Perception_Vehicle_parameters_.k_front_cornering_compliance(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_front_cornering_compliance << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_YawInertiaAdjustFac != Perception_Vehicle_parameters_old.k_YawInertiaAdjustFac){
        std::cout << "Perception_Vehicle_parameters_.k_YawInertiaAdjustFac(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_YawInertiaAdjustFac << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_RearAxleLoadRatio != Perception_Vehicle_parameters_old.k_RearAxleLoadRatio){
        std::cout << "Perception_Vehicle_parameters_.k_RearAxleLoadRatio(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_RearAxleLoadRatio << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_rear_compliance != Perception_Vehicle_parameters_old.k_rear_compliance){
        std::cout << "Perception_Vehicle_parameters_.k_rear_compliance(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_rear_compliance << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_VehWidth_Min != Perception_Vehicle_parameters_old.k_VehWidth_Min){
        std::cout << "Perception_Vehicle_parameters_.k_VehWidth_Min(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_VehWidth_Min << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_VehWidth_Max != Perception_Vehicle_parameters_old.k_VehWidth_Max){
        std::cout << "Perception_Vehicle_parameters_.k_VehWidth_Max(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_VehWidth_Max << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_WheelRadius != Perception_Vehicle_parameters_old.k_WheelRadius){
        std::cout << "Perception_Vehicle_parameters_.k_WheelRadius(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_WheelRadius << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_WheelWidthAve != Perception_Vehicle_parameters_old.k_WheelWidthAve){
        std::cout << "Perception_Vehicle_parameters_.k_WheelWidthAve(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_WheelWidthAve << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_x != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_x){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_x(sint16): " << static_cast<int>(Perception_Vehicle_parameters_.k_FrontCAM_Wide_x) << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_y != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_y){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_y(sint16): " << static_cast<int>(Perception_Vehicle_parameters_.k_FrontCAM_Wide_y) << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_z != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_z){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << Perception_Vehicle_parameters_.k_FrontCAM_Wide_z << std::dec  << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Yaw){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Pitch){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Roll){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Pitch_Delta){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Yaw_Delta){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Roll_Delta){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta_Online != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Pitch_Delta_Online){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Pitch_Delta_Online << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta_Online != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Yaw_Delta_Online){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Yaw_Delta_Online << std::endl;
        }
    if(Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta_Online != Perception_Vehicle_parameters_old.k_FrontCAM_Wide_Roll_Delta_Online){
        std::cout << "Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << Perception_Vehicle_parameters_.k_FrontCAM_Wide_Roll_Delta_Online << std::endl;
        }
}
#endif


#ifdef EDR_INFO_H
/* Print struct EDR_Info changed value */
void print_EDR_Info(EDR_Info& EDR_Info_,EDR_Info& EDR_Info_old){
// std::cout << "EDR_Info all variable:" << std::endl;
    if(EDR_Info_.HPCADAS_AEBDecCtrl != EDR_Info_old.HPCADAS_AEBDecCtrl){
        std::cout << "EDR_Info_.HPCADAS_AEBDecCtrl(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EDR_Info_.HPCADAS_AEBDecCtrl) << std::dec  << std::endl;
        }
    if(EDR_Info_.FSC_LCC_Mode != EDR_Info_old.FSC_LCC_Mode){
        std::cout << "EDR_Info_.FSC_LCC_Mode(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EDR_Info_.FSC_LCC_Mode) << std::dec  << std::endl;
        }
    if(EDR_Info_.FSC_LCC_EscapeLevel != EDR_Info_old.FSC_LCC_EscapeLevel){
        std::cout << "EDR_Info_.FSC_LCC_EscapeLevel(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EDR_Info_.FSC_LCC_EscapeLevel) << std::dec  << std::endl;
        }
    if(EDR_Info_.FSC_emergencyLightReq != EDR_Info_old.FSC_emergencyLightReq){
        std::cout << "EDR_Info_.FSC_emergencyLightReq(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EDR_Info_.FSC_emergencyLightReq) << std::dec  << std::endl;
        }
    if(EDR_Info_.HPCADAS_LKA_SteeringWheelAngle != EDR_Info_old.HPCADAS_LKA_SteeringWheelAngle){
        std::cout << "EDR_Info_.HPCADAS_LKA_SteeringWheelAngle(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << EDR_Info_.HPCADAS_LKA_SteeringWheelAngle << std::dec  << std::endl;
        }
    if(EDR_Info_.ADDC_ACCTargetTrq != EDR_Info_old.ADDC_ACCTargetTrq){
        std::cout << "EDR_Info_.ADDC_ACCTargetTrq(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << EDR_Info_.ADDC_ACCTargetTrq << std::dec  << std::endl;
        }
    if(EDR_Info_.ADDC_ACCTargetTrqBrk != EDR_Info_old.ADDC_ACCTargetTrqBrk){
        std::cout << "EDR_Info_.ADDC_ACCTargetTrqBrk(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << EDR_Info_.ADDC_ACCTargetTrqBrk << std::dec  << std::endl;
        }
    if(EDR_Info_.HPCADAS_ACCMode != EDR_Info_old.HPCADAS_ACCMode){
        std::cout << "EDR_Info_.HPCADAS_ACCMode(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EDR_Info_.HPCADAS_ACCMode) << std::dec  << std::endl;
        }
}
#endif


#ifdef LOGISTICS_DATA_H
/* Print struct Logistics_data changed value */
void print_Logistics_data(Logistics_data& Logistics_data_,Logistics_data& Logistics_data_old){
// std::cout << "Logistics_data all variable:" << std::endl;
    if(Logistics_data_.HW_Version[0] != Logistics_data_old.HW_Version[0]){
        std::cout << "Logistics_data_.HW_Version[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[0]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[1] != Logistics_data_old.HW_Version[1]){
        std::cout << "Logistics_data_.HW_Version[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[1]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[2] != Logistics_data_old.HW_Version[2]){
        std::cout << "Logistics_data_.HW_Version[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[2]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[3] != Logistics_data_old.HW_Version[3]){
        std::cout << "Logistics_data_.HW_Version[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[3]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[4] != Logistics_data_old.HW_Version[4]){
        std::cout << "Logistics_data_.HW_Version[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[4]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[5] != Logistics_data_old.HW_Version[5]){
        std::cout << "Logistics_data_.HW_Version[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[5]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[6] != Logistics_data_old.HW_Version[6]){
        std::cout << "Logistics_data_.HW_Version[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[6]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[7] != Logistics_data_old.HW_Version[7]){
        std::cout << "Logistics_data_.HW_Version[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[7]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[8] != Logistics_data_old.HW_Version[8]){
        std::cout << "Logistics_data_.HW_Version[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[8]) << std::dec  << std::endl;
        }
    if(Logistics_data_.HW_Version[9] != Logistics_data_old.HW_Version[9]){
        std::cout << "Logistics_data_.HW_Version[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.HW_Version[9]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[0] != Logistics_data_old.SW_Version[0]){
        std::cout << "Logistics_data_.SW_Version[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[0]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[1] != Logistics_data_old.SW_Version[1]){
        std::cout << "Logistics_data_.SW_Version[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[1]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[2] != Logistics_data_old.SW_Version[2]){
        std::cout << "Logistics_data_.SW_Version[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[2]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[3] != Logistics_data_old.SW_Version[3]){
        std::cout << "Logistics_data_.SW_Version[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[3]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[4] != Logistics_data_old.SW_Version[4]){
        std::cout << "Logistics_data_.SW_Version[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[4]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[5] != Logistics_data_old.SW_Version[5]){
        std::cout << "Logistics_data_.SW_Version[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[5]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[6] != Logistics_data_old.SW_Version[6]){
        std::cout << "Logistics_data_.SW_Version[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[6]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[7] != Logistics_data_old.SW_Version[7]){
        std::cout << "Logistics_data_.SW_Version[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[7]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[8] != Logistics_data_old.SW_Version[8]){
        std::cout << "Logistics_data_.SW_Version[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[8]) << std::dec  << std::endl;
        }
    if(Logistics_data_.SW_Version[9] != Logistics_data_old.SW_Version[9]){
        std::cout << "Logistics_data_.SW_Version[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.SW_Version[9]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[0] != Logistics_data_old.ECU_SerialNumber[0]){
        std::cout << "Logistics_data_.ECU_SerialNumber[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[0]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[1] != Logistics_data_old.ECU_SerialNumber[1]){
        std::cout << "Logistics_data_.ECU_SerialNumber[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[1]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[2] != Logistics_data_old.ECU_SerialNumber[2]){
        std::cout << "Logistics_data_.ECU_SerialNumber[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[2]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[3] != Logistics_data_old.ECU_SerialNumber[3]){
        std::cout << "Logistics_data_.ECU_SerialNumber[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[3]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[4] != Logistics_data_old.ECU_SerialNumber[4]){
        std::cout << "Logistics_data_.ECU_SerialNumber[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[4]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[5] != Logistics_data_old.ECU_SerialNumber[5]){
        std::cout << "Logistics_data_.ECU_SerialNumber[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[5]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[6] != Logistics_data_old.ECU_SerialNumber[6]){
        std::cout << "Logistics_data_.ECU_SerialNumber[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[6]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[7] != Logistics_data_old.ECU_SerialNumber[7]){
        std::cout << "Logistics_data_.ECU_SerialNumber[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[7]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[8] != Logistics_data_old.ECU_SerialNumber[8]){
        std::cout << "Logistics_data_.ECU_SerialNumber[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[8]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[9] != Logistics_data_old.ECU_SerialNumber[9]){
        std::cout << "Logistics_data_.ECU_SerialNumber[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[9]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[10] != Logistics_data_old.ECU_SerialNumber[10]){
        std::cout << "Logistics_data_.ECU_SerialNumber[10](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[10]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[11] != Logistics_data_old.ECU_SerialNumber[11]){
        std::cout << "Logistics_data_.ECU_SerialNumber[11](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[11]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[12] != Logistics_data_old.ECU_SerialNumber[12]){
        std::cout << "Logistics_data_.ECU_SerialNumber[12](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[12]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[13] != Logistics_data_old.ECU_SerialNumber[13]){
        std::cout << "Logistics_data_.ECU_SerialNumber[13](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[13]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[14] != Logistics_data_old.ECU_SerialNumber[14]){
        std::cout << "Logistics_data_.ECU_SerialNumber[14](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[14]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[15] != Logistics_data_old.ECU_SerialNumber[15]){
        std::cout << "Logistics_data_.ECU_SerialNumber[15](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[15]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[16] != Logistics_data_old.ECU_SerialNumber[16]){
        std::cout << "Logistics_data_.ECU_SerialNumber[16](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[16]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[17] != Logistics_data_old.ECU_SerialNumber[17]){
        std::cout << "Logistics_data_.ECU_SerialNumber[17](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[17]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[18] != Logistics_data_old.ECU_SerialNumber[18]){
        std::cout << "Logistics_data_.ECU_SerialNumber[18](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[18]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[19] != Logistics_data_old.ECU_SerialNumber[19]){
        std::cout << "Logistics_data_.ECU_SerialNumber[19](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[19]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[20] != Logistics_data_old.ECU_SerialNumber[20]){
        std::cout << "Logistics_data_.ECU_SerialNumber[20](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[20]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[21] != Logistics_data_old.ECU_SerialNumber[21]){
        std::cout << "Logistics_data_.ECU_SerialNumber[21](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[21]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[22] != Logistics_data_old.ECU_SerialNumber[22]){
        std::cout << "Logistics_data_.ECU_SerialNumber[22](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[22]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[23] != Logistics_data_old.ECU_SerialNumber[23]){
        std::cout << "Logistics_data_.ECU_SerialNumber[23](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[23]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[24] != Logistics_data_old.ECU_SerialNumber[24]){
        std::cout << "Logistics_data_.ECU_SerialNumber[24](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[24]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[25] != Logistics_data_old.ECU_SerialNumber[25]){
        std::cout << "Logistics_data_.ECU_SerialNumber[25](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[25]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[26] != Logistics_data_old.ECU_SerialNumber[26]){
        std::cout << "Logistics_data_.ECU_SerialNumber[26](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[26]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[27] != Logistics_data_old.ECU_SerialNumber[27]){
        std::cout << "Logistics_data_.ECU_SerialNumber[27](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[27]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[28] != Logistics_data_old.ECU_SerialNumber[28]){
        std::cout << "Logistics_data_.ECU_SerialNumber[28](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[28]) << std::dec  << std::endl;
        }
    if(Logistics_data_.ECU_SerialNumber[29] != Logistics_data_old.ECU_SerialNumber[29]){
        std::cout << "Logistics_data_.ECU_SerialNumber[29](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.ECU_SerialNumber[29]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[0] != Logistics_data_old.VIN_Code[0]){
        std::cout << "Logistics_data_.VIN_Code[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[0]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[1] != Logistics_data_old.VIN_Code[1]){
        std::cout << "Logistics_data_.VIN_Code[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[1]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[2] != Logistics_data_old.VIN_Code[2]){
        std::cout << "Logistics_data_.VIN_Code[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[2]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[3] != Logistics_data_old.VIN_Code[3]){
        std::cout << "Logistics_data_.VIN_Code[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[3]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[4] != Logistics_data_old.VIN_Code[4]){
        std::cout << "Logistics_data_.VIN_Code[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[4]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[5] != Logistics_data_old.VIN_Code[5]){
        std::cout << "Logistics_data_.VIN_Code[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[5]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[6] != Logistics_data_old.VIN_Code[6]){
        std::cout << "Logistics_data_.VIN_Code[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[6]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[7] != Logistics_data_old.VIN_Code[7]){
        std::cout << "Logistics_data_.VIN_Code[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[7]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[8] != Logistics_data_old.VIN_Code[8]){
        std::cout << "Logistics_data_.VIN_Code[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[8]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[9] != Logistics_data_old.VIN_Code[9]){
        std::cout << "Logistics_data_.VIN_Code[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[9]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[10] != Logistics_data_old.VIN_Code[10]){
        std::cout << "Logistics_data_.VIN_Code[10](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[10]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[11] != Logistics_data_old.VIN_Code[11]){
        std::cout << "Logistics_data_.VIN_Code[11](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[11]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[12] != Logistics_data_old.VIN_Code[12]){
        std::cout << "Logistics_data_.VIN_Code[12](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[12]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[13] != Logistics_data_old.VIN_Code[13]){
        std::cout << "Logistics_data_.VIN_Code[13](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[13]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[14] != Logistics_data_old.VIN_Code[14]){
        std::cout << "Logistics_data_.VIN_Code[14](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[14]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[15] != Logistics_data_old.VIN_Code[15]){
        std::cout << "Logistics_data_.VIN_Code[15](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[15]) << std::dec  << std::endl;
        }
    if(Logistics_data_.VIN_Code[16] != Logistics_data_old.VIN_Code[16]){
        std::cout << "Logistics_data_.VIN_Code[16](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Logistics_data_.VIN_Code[16]) << std::dec  << std::endl;
        }
}
#endif


#ifdef RCORE_RESET_REQUEST_H
/* Print struct Rcore_reset_request changed value */
void print_Rcore_reset_request(Rcore_reset_request& Rcore_reset_request_,Rcore_reset_request& Rcore_reset_request_old){
// std::cout << "Rcore_reset_request all variable:" << std::endl;
    if(Rcore_reset_request_.R_reset_request != Rcore_reset_request_old.R_reset_request){
        std::cout << "Rcore_reset_request_.R_reset_request(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Rcore_reset_request_.R_reset_request) << std::dec  << std::endl;
        }
}
#endif


#ifdef FeaState_H
/* Print struct FeaState changed value */
void print_FeaState(FeaState& FeaState_,FeaState& FeaState_old){
// std::cout << "FeaState all variable:" << std::endl;
    if(FeaState_.Feature_errcode != FeaState_old.Feature_errcode){
        std::cout << "FeaState_.Feature_errcode(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FeaState_.Feature_errcode) << std::dec  << std::endl;
        }
}
#endif





#ifdef UDS_IPC_RESPONSE_H
std::map<std::string, VariableVariant > UDS_ipc_response_Map = {
//[UDS_ipc_response]
{"UDS_ipc_response_.addressing_format(uint8_t)" , &UDS_ipc_response_.addressing_format},
{"UDS_ipc_response_.Doip_message_length(uint16_t)" , &UDS_ipc_response_.Doip_message_length},
{"UDS_ipc_response_.Doip_message[0](uint8_t)" , &UDS_ipc_response_.Doip_message[0]},
{"UDS_ipc_response_.Doip_message[1](uint8_t)" , &UDS_ipc_response_.Doip_message[1]},
{"UDS_ipc_response_.Doip_message[2](uint8_t)" , &UDS_ipc_response_.Doip_message[2]},
{"UDS_ipc_response_.Doip_message[3](uint8_t)" , &UDS_ipc_response_.Doip_message[3]},
{"UDS_ipc_response_.Doip_message[4](uint8_t)" , &UDS_ipc_response_.Doip_message[4]},
{"UDS_ipc_response_.Doip_message[5](uint8_t)" , &UDS_ipc_response_.Doip_message[5]},
{"UDS_ipc_response_.Doip_message[6](uint8_t)" , &UDS_ipc_response_.Doip_message[6]},
{"UDS_ipc_response_.Doip_message[7](uint8_t)" , &UDS_ipc_response_.Doip_message[7]},
{"UDS_ipc_response_.Doip_message[8](uint8_t)" , &UDS_ipc_response_.Doip_message[8]},
{"UDS_ipc_response_.Doip_message[9](uint8_t)" , &UDS_ipc_response_.Doip_message[9]},
};
#endif



#ifdef UDS_IPC_REQ_H
std::map<std::string, VariableVariant > UDS_ipc_req_Map = {
//[UDS_ipc_req]
{"UDS_ipc_req_.addressing_format(uint8_t)" , &UDS_ipc_req_.addressing_format},
{"UDS_ipc_req_.Doip_message_length(uint16_t)" , &UDS_ipc_req_.Doip_message_length},
{"UDS_ipc_req_.Doip_message[0](uint8_t)" , &UDS_ipc_req_.Doip_message[0]},
{"UDS_ipc_req_.Doip_message[1](uint8_t)" , &UDS_ipc_req_.Doip_message[1]},
{"UDS_ipc_req_.Doip_message[2](uint8_t)" , &UDS_ipc_req_.Doip_message[2]},
{"UDS_ipc_req_.Doip_message[3](uint8_t)" , &UDS_ipc_req_.Doip_message[3]},
{"UDS_ipc_req_.Doip_message[4](uint8_t)" , &UDS_ipc_req_.Doip_message[4]},
{"UDS_ipc_req_.Doip_message[5](uint8_t)" , &UDS_ipc_req_.Doip_message[5]},
{"UDS_ipc_req_.Doip_message[6](uint8_t)" , &UDS_ipc_req_.Doip_message[6]},
{"UDS_ipc_req_.Doip_message[7](uint8_t)" , &UDS_ipc_req_.Doip_message[7]},
{"UDS_ipc_req_.Doip_message[8](uint8_t)" , &UDS_ipc_req_.Doip_message[8]},
{"UDS_ipc_req_.Doip_message[9](uint8_t)" , &UDS_ipc_req_.Doip_message[9]},
{"UDS_ipc_req_.Doip_message[10](uint8_t)" , &UDS_ipc_req_.Doip_message[10]},
};
#endif


#ifdef UDS_IPC_RESPONSE_H
/* Print struct UDS_ipc_response changed value */
void print_UDS_ipc_response(UDS_ipc_response& UDS_ipc_response_,UDS_ipc_response& UDS_ipc_response_old){
// std::cout << "UDS_ipc_response all variable:" << std::endl;
    if(UDS_ipc_response_.addressing_format != UDS_ipc_response_old.addressing_format){
        std::cout << "UDS_ipc_response_.addressing_format(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.addressing_format) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message_length != UDS_ipc_response_old.Doip_message_length){
        std::cout << "UDS_ipc_response_.Doip_message_length(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << UDS_ipc_response_.Doip_message_length << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[0] != UDS_ipc_response_old.Doip_message[0]){
        std::cout << "UDS_ipc_response_.Doip_message[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[0]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[1] != UDS_ipc_response_old.Doip_message[1]){
        std::cout << "UDS_ipc_response_.Doip_message[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[1]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[2] != UDS_ipc_response_old.Doip_message[2]){
        std::cout << "UDS_ipc_response_.Doip_message[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[2]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[3] != UDS_ipc_response_old.Doip_message[3]){
        std::cout << "UDS_ipc_response_.Doip_message[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[3]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[4] != UDS_ipc_response_old.Doip_message[4]){
        std::cout << "UDS_ipc_response_.Doip_message[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[4]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[5] != UDS_ipc_response_old.Doip_message[5]){
        std::cout << "UDS_ipc_response_.Doip_message[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[5]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[6] != UDS_ipc_response_old.Doip_message[6]){
        std::cout << "UDS_ipc_response_.Doip_message[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[6]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[7] != UDS_ipc_response_old.Doip_message[7]){
        std::cout << "UDS_ipc_response_.Doip_message[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[7]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[8] != UDS_ipc_response_old.Doip_message[8]){
        std::cout << "UDS_ipc_response_.Doip_message[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[8]) << std::dec  << std::endl;
        }
    if(UDS_ipc_response_.Doip_message[9] != UDS_ipc_response_old.Doip_message[9]){
        std::cout << "UDS_ipc_response_.Doip_message[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_response_.Doip_message[9]) << std::dec  << std::endl;
        }
}
#endif


#ifdef UDS_IPC_REQ_H
/* Print struct UDS_ipc_req changed value */
void print_UDS_ipc_req(UDS_ipc_req& UDS_ipc_req_,UDS_ipc_req& UDS_ipc_req_old){
// std::cout << "UDS_ipc_req all variable:" << std::endl;
    if(UDS_ipc_req_.addressing_format != UDS_ipc_req_old.addressing_format){
        std::cout << "UDS_ipc_req_.addressing_format(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.addressing_format) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message_length != UDS_ipc_req_old.Doip_message_length){
        std::cout << "UDS_ipc_req_.Doip_message_length(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << UDS_ipc_req_.Doip_message_length << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[0] != UDS_ipc_req_old.Doip_message[0]){
        std::cout << "UDS_ipc_req_.Doip_message[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[0]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[1] != UDS_ipc_req_old.Doip_message[1]){
        std::cout << "UDS_ipc_req_.Doip_message[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[1]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[2] != UDS_ipc_req_old.Doip_message[2]){
        std::cout << "UDS_ipc_req_.Doip_message[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[2]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[3] != UDS_ipc_req_old.Doip_message[3]){
        std::cout << "UDS_ipc_req_.Doip_message[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[3]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[4] != UDS_ipc_req_old.Doip_message[4]){
        std::cout << "UDS_ipc_req_.Doip_message[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[4]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[5] != UDS_ipc_req_old.Doip_message[5]){
        std::cout << "UDS_ipc_req_.Doip_message[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[5]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[6] != UDS_ipc_req_old.Doip_message[6]){
        std::cout << "UDS_ipc_req_.Doip_message[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[6]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[7] != UDS_ipc_req_old.Doip_message[7]){
        std::cout << "UDS_ipc_req_.Doip_message[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[7]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[8] != UDS_ipc_req_old.Doip_message[8]){
        std::cout << "UDS_ipc_req_.Doip_message[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[8]) << std::dec  << std::endl;
        }
    if(UDS_ipc_req_.Doip_message[9] != UDS_ipc_req_old.Doip_message[9]){
        std::cout << "UDS_ipc_req_.Doip_message[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(UDS_ipc_req_.Doip_message[9]) << std::dec  << std::endl;
        }
}
#endif











#if defined(MAGNAPARAMCSRES_H) || defined(MAGNACAMEXTPARAM_H) || defined(MAGNACAMINTPARAM_H) || defined(MAGNATACCALIBINPUT_H)
std::map<std::string, VariableVariant > CSTriggerA1_Map = {
{"ipc_msg_.header.id",&ipc_msg_.header.id},
{"ipc_msg_.header.version",&ipc_msg_.header.version},
{"ipc_msg_.header.data_size",&ipc_msg_.header.data_size},
{"ipc_msg_.header.target",&ipc_msg_.header.target},
{"ipc_msg_.header.timestamp",&ipc_msg_.header.timestamp},
{"ipc_msg_.header.data[0]",&ipc_msg_.header.data[0]},    
//[MagnaParamCSRes]
{"MagnaParamCSRes_.is_success(uint8_t)" , &MagnaParamCSRes_.is_success},
{"MagnaParamCSRes_.status_code(uint8_t)" , &MagnaParamCSRes_.status_code},
{"MagnaParamCSRes_.meta_res_data[0](uint8_t)" , &MagnaParamCSRes_.meta_res_data[0]},
{"MagnaParamCSRes_.meta_res_data[1](uint8_t)" , &MagnaParamCSRes_.meta_res_data[1]},
{"MagnaParamCSRes_.meta_res_data[2](uint8_t)" , &MagnaParamCSRes_.meta_res_data[2]},
{"MagnaParamCSRes_.meta_res_data[3](uint8_t)" , &MagnaParamCSRes_.meta_res_data[3]},
{"MagnaParamCSRes_.meta_res_data[4](uint8_t)" , &MagnaParamCSRes_.meta_res_data[4]},
{"MagnaParamCSRes_.meta_res_data[5](uint8_t)" , &MagnaParamCSRes_.meta_res_data[5]},
{"MagnaParamCSRes_.meta_res_data[6](uint8_t)" , &MagnaParamCSRes_.meta_res_data[6]},
{"MagnaParamCSRes_.meta_res_data[7](uint8_t)" , &MagnaParamCSRes_.meta_res_data[7]},
{"MagnaParamCSRes_.meta_res_data[8](uint8_t)" , &MagnaParamCSRes_.meta_res_data[8]},
{"MagnaParamCSRes_.meta_res_data[9](uint8_t)" , &MagnaParamCSRes_.meta_res_data[9]},

//[MagnaCamExtParam]
{"MagnaCamExtParam_.yaw(float)" , &MagnaCamExtParam_.yaw},
{"MagnaCamExtParam_.roll(float)" , &MagnaCamExtParam_.roll},
{"MagnaCamExtParam_.pitch(float)" , &MagnaCamExtParam_.pitch},

//[MagnaCamIntParam]
{"MagnaCamIntParam_.pixelPitch_X(float)" , &MagnaCamIntParam_.pixelPitch_X},
{"MagnaCamIntParam_.pixelPitch_Y(float)" , &MagnaCamIntParam_.pixelPitch_Y},
{"MagnaCamIntParam_.focalLength(float)" , &MagnaCamIntParam_.focalLength},
{"MagnaCamIntParam_.focalLength_X(float)" , &MagnaCamIntParam_.focalLength_X},
{"MagnaCamIntParam_.focalLength_Y(float)" , &MagnaCamIntParam_.focalLength_Y},
{"MagnaCamIntParam_.undistort_focalLength(float)" , &MagnaCamIntParam_.undistort_focalLength},
{"MagnaCamIntParam_.undistort_focalLength_X(float)" , &MagnaCamIntParam_.undistort_focalLength_X},
{"MagnaCamIntParam_.undistort_focalLength_Y(float)" , &MagnaCamIntParam_.undistort_focalLength_Y},
{"MagnaCamIntParam_.distort_imgWidth(short)" , &MagnaCamIntParam_.distort_imgWidth},
{"MagnaCamIntParam_.distort_imgHeight(short)" , &MagnaCamIntParam_.distort_imgHeight},
{"MagnaCamIntParam_.distort_principalPoint_X(float)" , &MagnaCamIntParam_.distort_principalPoint_X},
{"MagnaCamIntParam_.distort_principalPoint_Y(float)" , &MagnaCamIntParam_.distort_principalPoint_Y},
{"MagnaCamIntParam_.undistort_imgWidth(short)" , &MagnaCamIntParam_.undistort_imgWidth},
{"MagnaCamIntParam_.undistort_imgHeight(short)" , &MagnaCamIntParam_.undistort_imgHeight},
{"MagnaCamIntParam_.undistort_principalPoint_X(float)" , &MagnaCamIntParam_.undistort_principalPoint_X},
{"MagnaCamIntParam_.undistort_principalPoint_Y(float)" , &MagnaCamIntParam_.undistort_principalPoint_Y},
{"MagnaCamIntParam_.k1(float)" , &MagnaCamIntParam_.k1},
{"MagnaCamIntParam_.k2(float)" , &MagnaCamIntParam_.k2},
{"MagnaCamIntParam_.k3(float)" , &MagnaCamIntParam_.k3},
{"MagnaCamIntParam_.k4(float)" , &MagnaCamIntParam_.k4},
{"MagnaCamIntParam_.k5(float)" , &MagnaCamIntParam_.k5},
{"MagnaCamIntParam_.k6(float)" , &MagnaCamIntParam_.k6},
{"MagnaCamIntParam_.p1(float)" , &MagnaCamIntParam_.p1},
{"MagnaCamIntParam_.p2(float)" , &MagnaCamIntParam_.p2},

//[MagnaTACCalibInput]
{"MagnaTACCalibInput_.cam_pos_x(float)" , &MagnaTACCalibInput_.cam_pos_x},
{"MagnaTACCalibInput_.cam_pos_y(float)" , &MagnaTACCalibInput_.cam_pos_y},
{"MagnaTACCalibInput_.cam_pos_z(float)" , &MagnaTACCalibInput_.cam_pos_z},
{"MagnaTACCalibInput_.april_tag_info[0].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[0].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[0].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[0].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[0].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[0].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[0].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[0].plate_width},
{"MagnaTACCalibInput_.april_tag_info[0].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[0].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[1].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[1].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[1].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[1].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[1].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[1].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[1].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[1].plate_width},
{"MagnaTACCalibInput_.april_tag_info[1].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[1].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[2].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[2].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[2].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[2].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[2].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[2].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[2].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[2].plate_width},
{"MagnaTACCalibInput_.april_tag_info[2].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[2].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[3].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[3].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[3].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[3].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[3].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[3].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[3].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[3].plate_width},
{"MagnaTACCalibInput_.april_tag_info[3].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[3].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[4].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[4].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[4].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[4].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[4].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[4].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[4].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[4].plate_width},
{"MagnaTACCalibInput_.april_tag_info[4].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[4].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[5].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[5].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[5].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[5].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[5].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[5].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[5].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[5].plate_width},
{"MagnaTACCalibInput_.april_tag_info[5].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[5].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[6].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[6].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[6].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[6].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[6].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[6].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[6].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[6].plate_width},
{"MagnaTACCalibInput_.april_tag_info[6].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[6].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[7].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[7].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[7].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[7].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[7].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[7].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[7].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[7].plate_width},
{"MagnaTACCalibInput_.april_tag_info[7].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[7].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[8].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[8].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[8].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[8].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[8].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[8].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[8].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[8].plate_width},
{"MagnaTACCalibInput_.april_tag_info[8].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[8].april_tag_id},
{"MagnaTACCalibInput_.april_tag_info[9].plate_pos_x(float)" , &MagnaTACCalibInput_.april_tag_info[9].plate_pos_x},
{"MagnaTACCalibInput_.april_tag_info[9].plate_pos_y(float)" , &MagnaTACCalibInput_.april_tag_info[9].plate_pos_y},
{"MagnaTACCalibInput_.april_tag_info[9].plate_pos_z(float)" , &MagnaTACCalibInput_.april_tag_info[9].plate_pos_z},
{"MagnaTACCalibInput_.april_tag_info[9].plate_width(float)" , &MagnaTACCalibInput_.april_tag_info[9].plate_width},
{"MagnaTACCalibInput_.april_tag_info[9].april_tag_id(int16_t)" , &MagnaTACCalibInput_.april_tag_info[9].april_tag_id},
{"MagnaTACCalibInput_.april_tag_num(int16_t)" , &MagnaTACCalibInput_.april_tag_num},
};
#endif

#ifdef MAGNAPARAMCSRES_H
/* Print struct MagnaParamCSRes changed value */
void print_MagnaParamCSRes(MagnaParamCSRes& MagnaParamCSRes_,MagnaParamCSRes& MagnaParamCSRes_old){
// std::cout << "MagnaParamCSRes all variable:" << std::endl;
    if(MagnaParamCSRes_.is_success != MagnaParamCSRes_old.is_success){
        std::cout << "MagnaParamCSRes_.is_success(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.is_success) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.status_code != MagnaParamCSRes_old.status_code){
        std::cout << "MagnaParamCSRes_.status_code(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.status_code) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[0] != MagnaParamCSRes_old.meta_res_data[0]){
        std::cout << "MagnaParamCSRes_.meta_res_data[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[0]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[1] != MagnaParamCSRes_old.meta_res_data[1]){
        std::cout << "MagnaParamCSRes_.meta_res_data[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[1]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[2] != MagnaParamCSRes_old.meta_res_data[2]){
        std::cout << "MagnaParamCSRes_.meta_res_data[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[2]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[3] != MagnaParamCSRes_old.meta_res_data[3]){
        std::cout << "MagnaParamCSRes_.meta_res_data[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[3]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[4] != MagnaParamCSRes_old.meta_res_data[4]){
        std::cout << "MagnaParamCSRes_.meta_res_data[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[4]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[5] != MagnaParamCSRes_old.meta_res_data[5]){
        std::cout << "MagnaParamCSRes_.meta_res_data[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[5]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[6] != MagnaParamCSRes_old.meta_res_data[6]){
        std::cout << "MagnaParamCSRes_.meta_res_data[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[6]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[7] != MagnaParamCSRes_old.meta_res_data[7]){
        std::cout << "MagnaParamCSRes_.meta_res_data[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[7]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[8] != MagnaParamCSRes_old.meta_res_data[8]){
        std::cout << "MagnaParamCSRes_.meta_res_data[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[8]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[9] != MagnaParamCSRes_old.meta_res_data[9]){
        std::cout << "MagnaParamCSRes_.meta_res_data[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[9]) << std::dec  << std::endl;
        }
}
#endif


#ifdef MAGNACAMEXTPARAM_H
/* Print struct MagnaCamExtParam changed value */
void print_MagnaCamExtParam(MagnaCamExtParam& MagnaCamExtParam_,MagnaCamExtParam& MagnaCamExtParam_old){
// std::cout << "MagnaCamExtParam all variable:" << std::endl;
    if(MagnaCamExtParam_.yaw != MagnaCamExtParam_old.yaw){
        std::cout << "MagnaCamExtParam_.yaw(float): " << std::fixed << std::setprecision(2) << MagnaCamExtParam_.yaw << std::endl;
        }
    if(MagnaCamExtParam_.roll != MagnaCamExtParam_old.roll){
        std::cout << "MagnaCamExtParam_.roll(float): " << std::fixed << std::setprecision(2) << MagnaCamExtParam_.roll << std::endl;
        }
    if(MagnaCamExtParam_.pitch != MagnaCamExtParam_old.pitch){
        std::cout << "MagnaCamExtParam_.pitch(float): " << std::fixed << std::setprecision(2) << MagnaCamExtParam_.pitch << std::endl;
        }
}
#endif


#ifdef MAGNACAMINTPARAM_H
/* Print struct MagnaCamIntParam changed value */
void print_MagnaCamIntParam(MagnaCamIntParam& MagnaCamIntParam_,MagnaCamIntParam& MagnaCamIntParam_old){
// std::cout << "MagnaCamIntParam all variable:" << std::endl;
    if(MagnaCamIntParam_.pixelPitch_X != MagnaCamIntParam_old.pixelPitch_X){
        std::cout << "MagnaCamIntParam_.pixelPitch_X(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.pixelPitch_X << std::endl;
        }
    if(MagnaCamIntParam_.pixelPitch_Y != MagnaCamIntParam_old.pixelPitch_Y){
        std::cout << "MagnaCamIntParam_.pixelPitch_Y(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.pixelPitch_Y << std::endl;
        }
    if(MagnaCamIntParam_.focalLength != MagnaCamIntParam_old.focalLength){
        std::cout << "MagnaCamIntParam_.focalLength(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.focalLength << std::endl;
        }
    if(MagnaCamIntParam_.focalLength_X != MagnaCamIntParam_old.focalLength_X){
        std::cout << "MagnaCamIntParam_.focalLength_X(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.focalLength_X << std::endl;
        }
    if(MagnaCamIntParam_.focalLength_Y != MagnaCamIntParam_old.focalLength_Y){
        std::cout << "MagnaCamIntParam_.focalLength_Y(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.focalLength_Y << std::endl;
        }
    if(MagnaCamIntParam_.undistort_focalLength != MagnaCamIntParam_old.undistort_focalLength){
        std::cout << "MagnaCamIntParam_.undistort_focalLength(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.undistort_focalLength << std::endl;
        }
    if(MagnaCamIntParam_.undistort_focalLength_X != MagnaCamIntParam_old.undistort_focalLength_X){
        std::cout << "MagnaCamIntParam_.undistort_focalLength_X(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.undistort_focalLength_X << std::endl;
        }
    if(MagnaCamIntParam_.undistort_focalLength_Y != MagnaCamIntParam_old.undistort_focalLength_Y){
        std::cout << "MagnaCamIntParam_.undistort_focalLength_Y(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.undistort_focalLength_Y << std::endl;
        }
    if(MagnaCamIntParam_.distort_imgWidth != MagnaCamIntParam_old.distort_imgWidth){
        std::cout << "MagnaCamIntParam_.distort_imgWidth(short): " << static_cast<int>(MagnaCamIntParam_.distort_imgWidth) << std::dec  << std::endl;
        }
    if(MagnaCamIntParam_.distort_imgHeight != MagnaCamIntParam_old.distort_imgHeight){
        std::cout << "MagnaCamIntParam_.distort_imgHeight(short): " << static_cast<int>(MagnaCamIntParam_.distort_imgHeight) << std::dec  << std::endl;
        }
    if(MagnaCamIntParam_.distort_principalPoint_X != MagnaCamIntParam_old.distort_principalPoint_X){
        std::cout << "MagnaCamIntParam_.distort_principalPoint_X(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.distort_principalPoint_X << std::endl;
        }
    if(MagnaCamIntParam_.distort_principalPoint_Y != MagnaCamIntParam_old.distort_principalPoint_Y){
        std::cout << "MagnaCamIntParam_.distort_principalPoint_Y(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.distort_principalPoint_Y << std::endl;
        }
    if(MagnaCamIntParam_.undistort_imgWidth != MagnaCamIntParam_old.undistort_imgWidth){
        std::cout << "MagnaCamIntParam_.undistort_imgWidth(short): " << static_cast<int>(MagnaCamIntParam_.undistort_imgWidth) << std::dec  << std::endl;
        }
    if(MagnaCamIntParam_.undistort_imgHeight != MagnaCamIntParam_old.undistort_imgHeight){
        std::cout << "MagnaCamIntParam_.undistort_imgHeight(short): " << static_cast<int>(MagnaCamIntParam_.undistort_imgHeight) << std::dec  << std::endl;
        }
    if(MagnaCamIntParam_.undistort_principalPoint_X != MagnaCamIntParam_old.undistort_principalPoint_X){
        std::cout << "MagnaCamIntParam_.undistort_principalPoint_X(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.undistort_principalPoint_X << std::endl;
        }
    if(MagnaCamIntParam_.undistort_principalPoint_Y != MagnaCamIntParam_old.undistort_principalPoint_Y){
        std::cout << "MagnaCamIntParam_.undistort_principalPoint_Y(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.undistort_principalPoint_Y << std::endl;
        }
    if(MagnaCamIntParam_.k1 != MagnaCamIntParam_old.k1){
        std::cout << "MagnaCamIntParam_.k1(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k1 << std::endl;
        }
    if(MagnaCamIntParam_.k2 != MagnaCamIntParam_old.k2){
        std::cout << "MagnaCamIntParam_.k2(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k2 << std::endl;
        }
    if(MagnaCamIntParam_.k3 != MagnaCamIntParam_old.k3){
        std::cout << "MagnaCamIntParam_.k3(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k3 << std::endl;
        }
    if(MagnaCamIntParam_.k4 != MagnaCamIntParam_old.k4){
        std::cout << "MagnaCamIntParam_.k4(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k4 << std::endl;
        }
    if(MagnaCamIntParam_.k5 != MagnaCamIntParam_old.k5){
        std::cout << "MagnaCamIntParam_.k5(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k5 << std::endl;
        }
    if(MagnaCamIntParam_.k6 != MagnaCamIntParam_old.k6){
        std::cout << "MagnaCamIntParam_.k6(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.k6 << std::endl;
        }
    if(MagnaCamIntParam_.p1 != MagnaCamIntParam_old.p1){
        std::cout << "MagnaCamIntParam_.p1(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.p1 << std::endl;
        }
    if(MagnaCamIntParam_.p2 != MagnaCamIntParam_old.p2){
        std::cout << "MagnaCamIntParam_.p2(float): " << std::fixed << std::setprecision(2) << MagnaCamIntParam_.p2 << std::endl;
        }
}
#endif


#ifdef MAGNATACCALIBINPUT_H
/* Print struct MagnaTACCalibInput changed value */
void print_MagnaTACCalibInput(MagnaTACCalibInput& MagnaTACCalibInput_,MagnaTACCalibInput& MagnaTACCalibInput_old){
// std::cout << "MagnaTACCalibInput all variable:" << std::endl;
    if(MagnaTACCalibInput_.cam_pos_x != MagnaTACCalibInput_old.cam_pos_x){
        std::cout << "MagnaTACCalibInput_.cam_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.cam_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.cam_pos_y != MagnaTACCalibInput_old.cam_pos_y){
        std::cout << "MagnaTACCalibInput_.cam_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.cam_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.cam_pos_z != MagnaTACCalibInput_old.cam_pos_z){
        std::cout << "MagnaTACCalibInput_.cam_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.cam_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[0].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[0].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[0].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[0].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[0].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[0].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[0].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[0].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[0].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[0].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[0].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[0].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[0].plate_width != MagnaTACCalibInput_old.april_tag_info[0].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[0].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[0].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[0].april_tag_id != MagnaTACCalibInput_old.april_tag_info[0].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[0].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[0].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[1].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[1].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[1].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[1].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[1].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[1].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[1].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[1].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[1].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[1].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[1].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[1].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[1].plate_width != MagnaTACCalibInput_old.april_tag_info[1].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[1].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[1].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[1].april_tag_id != MagnaTACCalibInput_old.april_tag_info[1].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[1].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[1].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[2].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[2].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[2].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[2].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[2].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[2].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[2].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[2].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[2].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[2].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[2].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[2].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[2].plate_width != MagnaTACCalibInput_old.april_tag_info[2].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[2].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[2].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[2].april_tag_id != MagnaTACCalibInput_old.april_tag_info[2].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[2].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[2].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[3].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[3].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[3].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[3].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[3].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[3].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[3].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[3].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[3].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[3].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[3].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[3].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[3].plate_width != MagnaTACCalibInput_old.april_tag_info[3].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[3].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[3].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[3].april_tag_id != MagnaTACCalibInput_old.april_tag_info[3].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[3].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[3].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[4].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[4].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[4].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[4].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[4].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[4].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[4].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[4].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[4].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[4].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[4].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[4].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[4].plate_width != MagnaTACCalibInput_old.april_tag_info[4].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[4].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[4].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[4].april_tag_id != MagnaTACCalibInput_old.april_tag_info[4].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[4].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[4].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[5].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[5].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[5].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[5].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[5].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[5].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[5].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[5].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[5].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[5].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[5].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[5].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[5].plate_width != MagnaTACCalibInput_old.april_tag_info[5].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[5].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[5].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[5].april_tag_id != MagnaTACCalibInput_old.april_tag_info[5].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[5].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[5].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[6].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[6].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[6].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[6].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[6].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[6].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[6].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[6].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[6].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[6].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[6].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[6].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[6].plate_width != MagnaTACCalibInput_old.april_tag_info[6].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[6].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[6].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[6].april_tag_id != MagnaTACCalibInput_old.april_tag_info[6].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[6].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[6].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[7].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[7].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[7].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[7].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[7].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[7].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[7].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[7].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[7].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[7].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[7].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[7].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[7].plate_width != MagnaTACCalibInput_old.april_tag_info[7].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[7].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[7].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[7].april_tag_id != MagnaTACCalibInput_old.april_tag_info[7].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[7].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[7].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[8].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[8].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[8].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[8].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[8].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[8].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[8].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[8].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[8].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[8].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[8].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[8].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[8].plate_width != MagnaTACCalibInput_old.april_tag_info[8].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[8].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[8].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[8].april_tag_id != MagnaTACCalibInput_old.april_tag_info[8].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[8].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[8].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[9].plate_pos_x != MagnaTACCalibInput_old.april_tag_info[9].plate_pos_x){
        std::cout << "MagnaTACCalibInput_.april_tag_info[9].plate_pos_x(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[9].plate_pos_x << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[9].plate_pos_y != MagnaTACCalibInput_old.april_tag_info[9].plate_pos_y){
        std::cout << "MagnaTACCalibInput_.april_tag_info[9].plate_pos_y(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[9].plate_pos_y << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[9].plate_pos_z != MagnaTACCalibInput_old.april_tag_info[9].plate_pos_z){
        std::cout << "MagnaTACCalibInput_.april_tag_info[9].plate_pos_z(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[9].plate_pos_z << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[9].plate_width != MagnaTACCalibInput_old.april_tag_info[9].plate_width){
        std::cout << "MagnaTACCalibInput_.april_tag_info[9].plate_width(float): " << std::fixed << std::setprecision(2) << MagnaTACCalibInput_.april_tag_info[9].plate_width << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_info[9].april_tag_id != MagnaTACCalibInput_old.april_tag_info[9].april_tag_id){
        std::cout << "MagnaTACCalibInput_.april_tag_info[9].april_tag_id(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_info[9].april_tag_id) << std::dec  << std::endl;
        }
    if(MagnaTACCalibInput_.april_tag_num != MagnaTACCalibInput_old.april_tag_num){
        std::cout << "MagnaTACCalibInput_.april_tag_num(int16_t): " << static_cast<int>(MagnaTACCalibInput_.april_tag_num) << std::dec  << std::endl;
        }
}
#endif













