#include "Feature_ALLSetPrint.h"




// std::string getCurrentTime() {
// auto now = std::chrono::high_resolution_clock::now();
    
//     // 转换为time_t用于日历时间
//     auto now_time_t = std::chrono::system_clock::to_time_t(
//         std::chrono::system_clock::now()
//     );
    
//     // 获取本地时间
//     std::tm* local_time = std::localtime(&now_time_t);
    
//     // 获取纳秒部分
//     auto duration = now.time_since_epoch();
//     auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
//         duration
//     ).count() % 1000000000;  // 取模1秒内的纳秒数
    
//     std::ostringstream oss;
//     oss << "[" << std::put_time(local_time, "%Y-%m-%d %H:%M:%S")
//         << "." << std::setfill('0') << std::setw(9) << nanoseconds << "] ";
    
//     return oss.str();
// }



void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}


#if defined(VEHBUSIN_H)  || defined(VEHPARAM_TX_H) || defined(IDT_FUNCTGTVISNID_H)

std::queue<std::string> inputQueue;
int flag=true;
std::string data_in={0x0};
std::atomic_bool stop{ false };


uint32_t floatToUint32Memcpy(float f) {
    // 注意：使用reinterpret_cast将float指针转换为uint32_t指针
    //return *reinterpret_cast<uint32_t*>(&f);
    uint32_t bits=0;
    memcpy(&bits, &f, sizeof(f));
    return bits;
}
// 1. sint8 转 uint32 (保持位模式)
uint32_t sint8ToUint32Memcpy(int8_t value) {
    uint32_t result = 0;
    std::memcpy(&result, &value, sizeof(int8_t));
    return result;
}

// 2. sint16 转 uint32 (保持位模式)
uint32_t sint16ToUint32Memcpy(int16_t value) {
    uint32_t result = 0;
    std::memcpy(&result, &value, sizeof(int16_t));
    return result;
}

// 3. sint32 转 uint32 (保持位模式)
uint32_t sint32ToUint32Memcpy(int32_t value) {
    uint32_t result = 0;
    std::memcpy(&result, &value, sizeof(int32_t));
    return result;
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
            std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') << sint8ToUint32Memcpy(*ptr)<< ";" << std::dec << std::setfill('0') << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr ( std::is_same_v<T, sint16>) {
            // 16位类型：宽度4
            std::cout << name << ": 0x" << std::hex << std::setw(4) << std::setfill('0') <<sint16ToUint32Memcpy(*ptr)<< ";" << std::dec << std::setfill('0') << static_cast<int>(*ptr) << std::dec << std::endl;
        }
        else if constexpr ( std::is_same_v<T, sint32>) {
            // 32位类型：宽度8
            std::cout << name << ": 0x" << std::hex << std::setw(8) << std::setfill('0') <<sint32ToUint32Memcpy(*ptr) << ";" << std::dec<< std::setfill('0') << static_cast<int>(*ptr) << std::dec << std::endl;
        }

        else if constexpr (std::is_same_v<T, float32>) {
            // 浮点类型：十进制
            std::cout << name <<  ": 0x" << std::hex << std::setw(8) << std::setfill('0')  <<floatToUint32Memcpy(*ptr)<<";"  << std::fixed <<std::setprecision(2) << *ptr << std::endl;
        }
    }, var);
}

// void printVariableVariant(const std::string& name, VariableVariant var) {
//     std::visit([&name](auto&& ptr) {
//         using T = std::decay_t<decltype(*ptr)>;
        
//         // 根据类型决定打印格式
//         if constexpr (std::is_same_v<T, uint8_t> ) {
//             // 8位类型：宽度2
//             std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
//                       << static_cast<int>(*ptr) << std::dec << std::endl;
//         } 
//         // if constexpr (std::is_same_v<T, unsigned char> ) {
//         //     // 8位类型：宽度2
//         //     std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
//         //               << static_cast<int>(*ptr) << std::dec << std::endl;
//         // } 
//         else if constexpr (std::is_same_v<T, uint16_t> ) {
//             // 16位类型：宽度4
//             std::cout << name << ": 0x" << std::hex << std::setw(4) << std::setfill('0') 
//                       << *ptr << std::dec  << std::endl;
//         }
//         else if constexpr (std::is_same_v<T, uint32_t> ) {
//             // 32位类型：宽度8
//             std::cout << name << ": 0x" << std::hex << std::setw(8) << std::setfill('0') 
//                       << *ptr << std::dec  << std::endl;
//         }
//         else if constexpr (std::is_same_v<T, uint64_t> ) {
//             // 32位类型：宽度8
//             std::cout << name << ": 0x" << std::hex << std::setw(16) << std::setfill('0') 
//                       << *ptr << std::dec  << std::endl;
//         }
        

//         else if constexpr ( std::is_same_v<T, short>) {
//             // 16位类型：宽度4
//             std::cout << name << ": " << std::dec << std::setfill('0') 
//                       << static_cast<int>(*ptr) << std::dec  << std::endl;
//         }
//         else if constexpr ( std::is_same_v<T, int>) {
//             // 32位类型：宽度8
//             std::cout << name << ": " << std::dec<< std::setfill('0') 
//                       << static_cast<int>(*ptr) << std::dec  << std::endl;
//         }

//         else if constexpr (std::is_same_v<T, float>) {
//             // 浮点类型：十进制
//             std::cout << name << ": " << std::fixed << std::setprecision(2) << *ptr  << std::endl;
//         }
//     }, var);
// }


void asyncInputThreadTTY() {


    // while (true) {

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
    
    // 3. 从串口读取数据
    char buffer[1024];
    std::string lineBuffer;  // 用于累积行数据
    
    while(true){
        memset(buffer, 0, sizeof(buffer));

        // 读取数据
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);


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
            
            // 去除可能的回车符\r
            if (!line.empty() && line.back() == '\n') 
            {
                if ( line.compare("quit\n")==0 || line.compare("Quit\n")==0||line.compare("QUIT\n")==0)
                {
                    if(tcsetattr(serial_fd,TCSANOW,&options_old)==0)
                             std::cout << "串口设置已恢复" << std::endl;
                    else
                        std::cerr << "恢复串口设置失败: " << strerror(errno) << std::endl;
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
                
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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
#endif














#ifdef EGOVEHICLESTATE_H
/* Set and Print struct EgoVehicleState initial value */
void setIntialValue_EgoVehicleState(EgoVehicleState& EgoVehicleState_){
    std::cout << "Set struct EgoVehicleState variable and Publish:" << std::endl;
    EgoVehicleState_.VLgt = 1.10;
    std::cout << "EgoVehicleState_.VLgt(float32): " << EgoVehicleState_.VLgt << std::endl;
    EgoVehicleState_.ALgt = 2.20;
    std::cout << "EgoVehicleState_.ALgt(float32): " << EgoVehicleState_.ALgt << std::endl;
    EgoVehicleState_.ALgtRaw = 3.30;
    std::cout << "EgoVehicleState_.ALgtRaw(float32): " << EgoVehicleState_.ALgtRaw << std::endl;
    EgoVehicleState_.ALatRaw = 4.40;
    std::cout << "EgoVehicleState_.ALatRaw(float32): " << EgoVehicleState_.ALatRaw << std::endl;
    EgoVehicleState_.YawRate = 5.50;
    std::cout << "EgoVehicleState_.YawRate(float32): " << EgoVehicleState_.YawRate << std::endl;
    EgoVehicleState_.YawRateRaw = 6.60;
    std::cout << "EgoVehicleState_.YawRateRaw(float32): " << EgoVehicleState_.YawRateRaw << std::endl;
    EgoVehicleState_.SequenceID = 1;
    std::cout << "EgoVehicleState_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(EgoVehicleState_.SequenceID) << std::dec  << std::endl;
    EgoVehicleState_.valid = 1;
    std::cout << "EgoVehicleState_.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(EgoVehicleState_.valid) << std::dec  << std::endl;
};
#endif



#ifdef FUSEDFRONTOBJECT_H
/* Set and Print struct FusedFrontObject initial value */
void setIntialValue_FusedFrontObject(FusedFrontObject& FusedFrontObject_){
    std::cout << "Set struct FusedFrontObject variable and Publish:" << std::endl;
    FusedFrontObject_.Properties[0].accelerationStdDev = 1.10;
    std::cout << "FusedFrontObject_.Properties[0].accelerationStdDev(float32): " << FusedFrontObject_.Properties[0].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[0].brakeLight = 1;
    std::cout << "FusedFrontObject_.Properties[0].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].classificationConfidence = 2.20;
    std::cout << "FusedFrontObject_.Properties[0].classificationConfidence(float32): " << FusedFrontObject_.Properties[0].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[0].cmsConfidence = 2;
    std::cout << "FusedFrontObject_.Properties[0].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].cmbbSecConfidence = 3;
    std::cout << "FusedFrontObject_.Properties[0].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].distanceToLeftNearLaneMarking = 3.30;
    std::cout << "FusedFrontObject_.Properties[0].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[0].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[0].distanceToRightNearLaneMarking = 4.40;
    std::cout << "FusedFrontObject_.Properties[0].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[0].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[0].elkaQly = 4;
    std::cout << "FusedFrontObject_.Properties[0].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].existenceConfidence = 5.50;
    std::cout << "FusedFrontObject_.Properties[0].existenceConfidence(float32): " << FusedFrontObject_.Properties[0].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[0].fcwQly = 5;
    std::cout << "FusedFrontObject_.Properties[0].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].fusionSource = 6;
    std::cout << "FusedFrontObject_.Properties[0].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].hazardLightStatus = 7;
    std::cout << "FusedFrontObject_.Properties[0].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].headingStdDev = 6.60;
    std::cout << "FusedFrontObject_.Properties[0].headingStdDev(float32): " << FusedFrontObject_.Properties[0].headingStdDev << std::endl;
    FusedFrontObject_.Properties[0].id = 8;
    std::cout << "FusedFrontObject_.Properties[0].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].innovationFactor = 7.70;
    std::cout << "FusedFrontObject_.Properties[0].innovationFactor(float32): " << FusedFrontObject_.Properties[0].innovationFactor << std::endl;
    FusedFrontObject_.Properties[0].latPositionStdDev = 8.80;
    std::cout << "FusedFrontObject_.Properties[0].latPositionStdDev(float32): " << FusedFrontObject_.Properties[0].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[0].leftNearLaneMarkingConfidence = 9;
    std::cout << "FusedFrontObject_.Properties[0].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].longPositionStdDev = 9.90;
    std::cout << "FusedFrontObject_.Properties[0].longPositionStdDev(float32): " << FusedFrontObject_.Properties[0].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[0].motionHistory = 10;
    std::cout << "FusedFrontObject_.Properties[0].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].motionModel = 11;
    std::cout << "FusedFrontObject_.Properties[0].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].motionPattern = 12;
    std::cout << "FusedFrontObject_.Properties[0].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].radarId = 13;
    std::cout << "FusedFrontObject_.Properties[0].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].referencePoint = 14;
    std::cout << "FusedFrontObject_.Properties[0].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].reserved = 11.00;
    std::cout << "FusedFrontObject_.Properties[0].reserved(float32): " << FusedFrontObject_.Properties[0].reserved << std::endl;
    FusedFrontObject_.Properties[0].rightNearLaneMarkingConfidence = 15;
    std::cout << "FusedFrontObject_.Properties[0].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].speedStdDev = 12.10;
    std::cout << "FusedFrontObject_.Properties[0].speedStdDev(float32): " << FusedFrontObject_.Properties[0].speedStdDev << std::endl;
    FusedFrontObject_.Properties[0].trackStatus = 16;
    std::cout << "FusedFrontObject_.Properties[0].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].trafficScenario = 17;
    std::cout << "FusedFrontObject_.Properties[0].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].turnIndicator = 18;
    std::cout << "FusedFrontObject_.Properties[0].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].type = 19;
    std::cout << "FusedFrontObject_.Properties[0].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].visionId = 20;
    std::cout << "FusedFrontObject_.Properties[0].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[0].width = 13.20;
    std::cout << "FusedFrontObject_.Properties[0].width(float32): " << FusedFrontObject_.Properties[0].width << std::endl;
    FusedFrontObject_.Properties[0].length = 14.30;
    std::cout << "FusedFrontObject_.Properties[0].length(float32): " << FusedFrontObject_.Properties[0].length << std::endl;
    FusedFrontObject_.Properties[0].SensorUpdateStatus = 21;
    std::cout << "FusedFrontObject_.Properties[0].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[0].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].accelerationStdDev = 15.40;
    std::cout << "FusedFrontObject_.Properties[1].accelerationStdDev(float32): " << FusedFrontObject_.Properties[1].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[1].brakeLight = 22;
    std::cout << "FusedFrontObject_.Properties[1].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].classificationConfidence = 16.50;
    std::cout << "FusedFrontObject_.Properties[1].classificationConfidence(float32): " << FusedFrontObject_.Properties[1].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[1].cmsConfidence = 23;
    std::cout << "FusedFrontObject_.Properties[1].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].cmbbSecConfidence = 24;
    std::cout << "FusedFrontObject_.Properties[1].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].distanceToLeftNearLaneMarking = 17.60;
    std::cout << "FusedFrontObject_.Properties[1].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[1].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[1].distanceToRightNearLaneMarking = 18.70;
    std::cout << "FusedFrontObject_.Properties[1].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[1].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[1].elkaQly = 25;
    std::cout << "FusedFrontObject_.Properties[1].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].existenceConfidence = 19.80;
    std::cout << "FusedFrontObject_.Properties[1].existenceConfidence(float32): " << FusedFrontObject_.Properties[1].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[1].fcwQly = 26;
    std::cout << "FusedFrontObject_.Properties[1].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].fusionSource = 27;
    std::cout << "FusedFrontObject_.Properties[1].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].hazardLightStatus = 28;
    std::cout << "FusedFrontObject_.Properties[1].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].headingStdDev = 20.90;
    std::cout << "FusedFrontObject_.Properties[1].headingStdDev(float32): " << FusedFrontObject_.Properties[1].headingStdDev << std::endl;
    FusedFrontObject_.Properties[1].id = 29;
    std::cout << "FusedFrontObject_.Properties[1].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].innovationFactor = 22.00;
    std::cout << "FusedFrontObject_.Properties[1].innovationFactor(float32): " << FusedFrontObject_.Properties[1].innovationFactor << std::endl;
    FusedFrontObject_.Properties[1].latPositionStdDev = 23.10;
    std::cout << "FusedFrontObject_.Properties[1].latPositionStdDev(float32): " << FusedFrontObject_.Properties[1].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[1].leftNearLaneMarkingConfidence = 30;
    std::cout << "FusedFrontObject_.Properties[1].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].longPositionStdDev = 24.20;
    std::cout << "FusedFrontObject_.Properties[1].longPositionStdDev(float32): " << FusedFrontObject_.Properties[1].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[1].motionHistory = 31;
    std::cout << "FusedFrontObject_.Properties[1].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].motionModel = 32;
    std::cout << "FusedFrontObject_.Properties[1].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].motionPattern = 33;
    std::cout << "FusedFrontObject_.Properties[1].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].radarId = 34;
    std::cout << "FusedFrontObject_.Properties[1].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].referencePoint = 35;
    std::cout << "FusedFrontObject_.Properties[1].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].reserved = 25.30;
    std::cout << "FusedFrontObject_.Properties[1].reserved(float32): " << FusedFrontObject_.Properties[1].reserved << std::endl;
    FusedFrontObject_.Properties[1].rightNearLaneMarkingConfidence = 36;
    std::cout << "FusedFrontObject_.Properties[1].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].speedStdDev = 26.40;
    std::cout << "FusedFrontObject_.Properties[1].speedStdDev(float32): " << FusedFrontObject_.Properties[1].speedStdDev << std::endl;
    FusedFrontObject_.Properties[1].trackStatus = 37;
    std::cout << "FusedFrontObject_.Properties[1].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].trafficScenario = 38;
    std::cout << "FusedFrontObject_.Properties[1].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].turnIndicator = 39;
    std::cout << "FusedFrontObject_.Properties[1].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].type = 40;
    std::cout << "FusedFrontObject_.Properties[1].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].visionId = 41;
    std::cout << "FusedFrontObject_.Properties[1].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[1].width = 27.50;
    std::cout << "FusedFrontObject_.Properties[1].width(float32): " << FusedFrontObject_.Properties[1].width << std::endl;
    FusedFrontObject_.Properties[1].length = 28.60;
    std::cout << "FusedFrontObject_.Properties[1].length(float32): " << FusedFrontObject_.Properties[1].length << std::endl;
    FusedFrontObject_.Properties[1].SensorUpdateStatus = 42;
    std::cout << "FusedFrontObject_.Properties[1].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[1].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].accelerationStdDev = 29.70;
    std::cout << "FusedFrontObject_.Properties[2].accelerationStdDev(float32): " << FusedFrontObject_.Properties[2].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[2].brakeLight = 43;
    std::cout << "FusedFrontObject_.Properties[2].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].classificationConfidence = 30.80;
    std::cout << "FusedFrontObject_.Properties[2].classificationConfidence(float32): " << FusedFrontObject_.Properties[2].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[2].cmsConfidence = 44;
    std::cout << "FusedFrontObject_.Properties[2].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].cmbbSecConfidence = 45;
    std::cout << "FusedFrontObject_.Properties[2].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].distanceToLeftNearLaneMarking = 31.90;
    std::cout << "FusedFrontObject_.Properties[2].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[2].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[2].distanceToRightNearLaneMarking = 33.00;
    std::cout << "FusedFrontObject_.Properties[2].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[2].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[2].elkaQly = 46;
    std::cout << "FusedFrontObject_.Properties[2].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].existenceConfidence = 34.10;
    std::cout << "FusedFrontObject_.Properties[2].existenceConfidence(float32): " << FusedFrontObject_.Properties[2].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[2].fcwQly = 47;
    std::cout << "FusedFrontObject_.Properties[2].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].fusionSource = 48;
    std::cout << "FusedFrontObject_.Properties[2].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].hazardLightStatus = 49;
    std::cout << "FusedFrontObject_.Properties[2].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].headingStdDev = 35.20;
    std::cout << "FusedFrontObject_.Properties[2].headingStdDev(float32): " << FusedFrontObject_.Properties[2].headingStdDev << std::endl;
    FusedFrontObject_.Properties[2].id = 50;
    std::cout << "FusedFrontObject_.Properties[2].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].innovationFactor = 36.30;
    std::cout << "FusedFrontObject_.Properties[2].innovationFactor(float32): " << FusedFrontObject_.Properties[2].innovationFactor << std::endl;
    FusedFrontObject_.Properties[2].latPositionStdDev = 37.40;
    std::cout << "FusedFrontObject_.Properties[2].latPositionStdDev(float32): " << FusedFrontObject_.Properties[2].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[2].leftNearLaneMarkingConfidence = 51;
    std::cout << "FusedFrontObject_.Properties[2].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].longPositionStdDev = 38.50;
    std::cout << "FusedFrontObject_.Properties[2].longPositionStdDev(float32): " << FusedFrontObject_.Properties[2].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[2].motionHistory = 52;
    std::cout << "FusedFrontObject_.Properties[2].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].motionModel = 53;
    std::cout << "FusedFrontObject_.Properties[2].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].motionPattern = 54;
    std::cout << "FusedFrontObject_.Properties[2].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].radarId = 55;
    std::cout << "FusedFrontObject_.Properties[2].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].referencePoint = 56;
    std::cout << "FusedFrontObject_.Properties[2].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].reserved = 39.60;
    std::cout << "FusedFrontObject_.Properties[2].reserved(float32): " << FusedFrontObject_.Properties[2].reserved << std::endl;
    FusedFrontObject_.Properties[2].rightNearLaneMarkingConfidence = 57;
    std::cout << "FusedFrontObject_.Properties[2].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].speedStdDev = 40.70;
    std::cout << "FusedFrontObject_.Properties[2].speedStdDev(float32): " << FusedFrontObject_.Properties[2].speedStdDev << std::endl;
    FusedFrontObject_.Properties[2].trackStatus = 58;
    std::cout << "FusedFrontObject_.Properties[2].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].trafficScenario = 59;
    std::cout << "FusedFrontObject_.Properties[2].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].turnIndicator = 60;
    std::cout << "FusedFrontObject_.Properties[2].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].type = 61;
    std::cout << "FusedFrontObject_.Properties[2].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].visionId = 62;
    std::cout << "FusedFrontObject_.Properties[2].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[2].width = 41.80;
    std::cout << "FusedFrontObject_.Properties[2].width(float32): " << FusedFrontObject_.Properties[2].width << std::endl;
    FusedFrontObject_.Properties[2].length = 42.90;
    std::cout << "FusedFrontObject_.Properties[2].length(float32): " << FusedFrontObject_.Properties[2].length << std::endl;
    FusedFrontObject_.Properties[2].SensorUpdateStatus = 63;
    std::cout << "FusedFrontObject_.Properties[2].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[2].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].accelerationStdDev = 44.00;
    std::cout << "FusedFrontObject_.Properties[3].accelerationStdDev(float32): " << FusedFrontObject_.Properties[3].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[3].brakeLight = 64;
    std::cout << "FusedFrontObject_.Properties[3].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].classificationConfidence = 45.10;
    std::cout << "FusedFrontObject_.Properties[3].classificationConfidence(float32): " << FusedFrontObject_.Properties[3].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[3].cmsConfidence = 65;
    std::cout << "FusedFrontObject_.Properties[3].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].cmbbSecConfidence = 66;
    std::cout << "FusedFrontObject_.Properties[3].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].distanceToLeftNearLaneMarking = 46.20;
    std::cout << "FusedFrontObject_.Properties[3].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[3].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[3].distanceToRightNearLaneMarking = 47.30;
    std::cout << "FusedFrontObject_.Properties[3].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[3].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[3].elkaQly = 67;
    std::cout << "FusedFrontObject_.Properties[3].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].existenceConfidence = 48.40;
    std::cout << "FusedFrontObject_.Properties[3].existenceConfidence(float32): " << FusedFrontObject_.Properties[3].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[3].fcwQly = 68;
    std::cout << "FusedFrontObject_.Properties[3].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].fusionSource = 69;
    std::cout << "FusedFrontObject_.Properties[3].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].hazardLightStatus = 70;
    std::cout << "FusedFrontObject_.Properties[3].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].headingStdDev = 49.50;
    std::cout << "FusedFrontObject_.Properties[3].headingStdDev(float32): " << FusedFrontObject_.Properties[3].headingStdDev << std::endl;
    FusedFrontObject_.Properties[3].id = 71;
    std::cout << "FusedFrontObject_.Properties[3].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].innovationFactor = 50.60;
    std::cout << "FusedFrontObject_.Properties[3].innovationFactor(float32): " << FusedFrontObject_.Properties[3].innovationFactor << std::endl;
    FusedFrontObject_.Properties[3].latPositionStdDev = 51.70;
    std::cout << "FusedFrontObject_.Properties[3].latPositionStdDev(float32): " << FusedFrontObject_.Properties[3].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[3].leftNearLaneMarkingConfidence = 72;
    std::cout << "FusedFrontObject_.Properties[3].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].longPositionStdDev = 52.80;
    std::cout << "FusedFrontObject_.Properties[3].longPositionStdDev(float32): " << FusedFrontObject_.Properties[3].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[3].motionHistory = 73;
    std::cout << "FusedFrontObject_.Properties[3].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].motionModel = 74;
    std::cout << "FusedFrontObject_.Properties[3].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].motionPattern = 75;
    std::cout << "FusedFrontObject_.Properties[3].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].radarId = 76;
    std::cout << "FusedFrontObject_.Properties[3].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].referencePoint = 77;
    std::cout << "FusedFrontObject_.Properties[3].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].reserved = 53.90;
    std::cout << "FusedFrontObject_.Properties[3].reserved(float32): " << FusedFrontObject_.Properties[3].reserved << std::endl;
    FusedFrontObject_.Properties[3].rightNearLaneMarkingConfidence = 78;
    std::cout << "FusedFrontObject_.Properties[3].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].speedStdDev = 55.00;
    std::cout << "FusedFrontObject_.Properties[3].speedStdDev(float32): " << FusedFrontObject_.Properties[3].speedStdDev << std::endl;
    FusedFrontObject_.Properties[3].trackStatus = 79;
    std::cout << "FusedFrontObject_.Properties[3].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].trafficScenario = 80;
    std::cout << "FusedFrontObject_.Properties[3].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].turnIndicator = 81;
    std::cout << "FusedFrontObject_.Properties[3].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].type = 82;
    std::cout << "FusedFrontObject_.Properties[3].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].visionId = 83;
    std::cout << "FusedFrontObject_.Properties[3].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[3].width = 56.10;
    std::cout << "FusedFrontObject_.Properties[3].width(float32): " << FusedFrontObject_.Properties[3].width << std::endl;
    FusedFrontObject_.Properties[3].length = 57.20;
    std::cout << "FusedFrontObject_.Properties[3].length(float32): " << FusedFrontObject_.Properties[3].length << std::endl;
    FusedFrontObject_.Properties[3].SensorUpdateStatus = 84;
    std::cout << "FusedFrontObject_.Properties[3].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[3].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].accelerationStdDev = 58.30;
    std::cout << "FusedFrontObject_.Properties[4].accelerationStdDev(float32): " << FusedFrontObject_.Properties[4].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[4].brakeLight = 85;
    std::cout << "FusedFrontObject_.Properties[4].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].classificationConfidence = 59.40;
    std::cout << "FusedFrontObject_.Properties[4].classificationConfidence(float32): " << FusedFrontObject_.Properties[4].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[4].cmsConfidence = 86;
    std::cout << "FusedFrontObject_.Properties[4].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].cmbbSecConfidence = 87;
    std::cout << "FusedFrontObject_.Properties[4].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].distanceToLeftNearLaneMarking = 60.50;
    std::cout << "FusedFrontObject_.Properties[4].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[4].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[4].distanceToRightNearLaneMarking = 61.60;
    std::cout << "FusedFrontObject_.Properties[4].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[4].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[4].elkaQly = 88;
    std::cout << "FusedFrontObject_.Properties[4].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].existenceConfidence = 62.70;
    std::cout << "FusedFrontObject_.Properties[4].existenceConfidence(float32): " << FusedFrontObject_.Properties[4].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[4].fcwQly = 89;
    std::cout << "FusedFrontObject_.Properties[4].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].fusionSource = 90;
    std::cout << "FusedFrontObject_.Properties[4].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].hazardLightStatus = 91;
    std::cout << "FusedFrontObject_.Properties[4].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].headingStdDev = 63.80;
    std::cout << "FusedFrontObject_.Properties[4].headingStdDev(float32): " << FusedFrontObject_.Properties[4].headingStdDev << std::endl;
    FusedFrontObject_.Properties[4].id = 92;
    std::cout << "FusedFrontObject_.Properties[4].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].innovationFactor = 64.90;
    std::cout << "FusedFrontObject_.Properties[4].innovationFactor(float32): " << FusedFrontObject_.Properties[4].innovationFactor << std::endl;
    FusedFrontObject_.Properties[4].latPositionStdDev = 66.00;
    std::cout << "FusedFrontObject_.Properties[4].latPositionStdDev(float32): " << FusedFrontObject_.Properties[4].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[4].leftNearLaneMarkingConfidence = 93;
    std::cout << "FusedFrontObject_.Properties[4].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].longPositionStdDev = 67.10;
    std::cout << "FusedFrontObject_.Properties[4].longPositionStdDev(float32): " << FusedFrontObject_.Properties[4].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[4].motionHistory = 94;
    std::cout << "FusedFrontObject_.Properties[4].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].motionModel = 95;
    std::cout << "FusedFrontObject_.Properties[4].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].motionPattern = 96;
    std::cout << "FusedFrontObject_.Properties[4].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].radarId = 97;
    std::cout << "FusedFrontObject_.Properties[4].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].referencePoint = 98;
    std::cout << "FusedFrontObject_.Properties[4].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].reserved = 68.20;
    std::cout << "FusedFrontObject_.Properties[4].reserved(float32): " << FusedFrontObject_.Properties[4].reserved << std::endl;
    FusedFrontObject_.Properties[4].rightNearLaneMarkingConfidence = 99;
    std::cout << "FusedFrontObject_.Properties[4].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].speedStdDev = 69.30;
    std::cout << "FusedFrontObject_.Properties[4].speedStdDev(float32): " << FusedFrontObject_.Properties[4].speedStdDev << std::endl;
    FusedFrontObject_.Properties[4].trackStatus = 100;
    std::cout << "FusedFrontObject_.Properties[4].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].trafficScenario = 101;
    std::cout << "FusedFrontObject_.Properties[4].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].turnIndicator = 102;
    std::cout << "FusedFrontObject_.Properties[4].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].type = 103;
    std::cout << "FusedFrontObject_.Properties[4].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].visionId = 104;
    std::cout << "FusedFrontObject_.Properties[4].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[4].width = 70.40;
    std::cout << "FusedFrontObject_.Properties[4].width(float32): " << FusedFrontObject_.Properties[4].width << std::endl;
    FusedFrontObject_.Properties[4].length = 71.50;
    std::cout << "FusedFrontObject_.Properties[4].length(float32): " << FusedFrontObject_.Properties[4].length << std::endl;
    FusedFrontObject_.Properties[4].SensorUpdateStatus = 105;
    std::cout << "FusedFrontObject_.Properties[4].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[4].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].accelerationStdDev = 72.60;
    std::cout << "FusedFrontObject_.Properties[5].accelerationStdDev(float32): " << FusedFrontObject_.Properties[5].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[5].brakeLight = 106;
    std::cout << "FusedFrontObject_.Properties[5].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].classificationConfidence = 73.70;
    std::cout << "FusedFrontObject_.Properties[5].classificationConfidence(float32): " << FusedFrontObject_.Properties[5].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[5].cmsConfidence = 107;
    std::cout << "FusedFrontObject_.Properties[5].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].cmbbSecConfidence = 108;
    std::cout << "FusedFrontObject_.Properties[5].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].distanceToLeftNearLaneMarking = 74.80;
    std::cout << "FusedFrontObject_.Properties[5].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[5].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[5].distanceToRightNearLaneMarking = 75.90;
    std::cout << "FusedFrontObject_.Properties[5].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[5].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[5].elkaQly = 109;
    std::cout << "FusedFrontObject_.Properties[5].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].existenceConfidence = 77.00;
    std::cout << "FusedFrontObject_.Properties[5].existenceConfidence(float32): " << FusedFrontObject_.Properties[5].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[5].fcwQly = 110;
    std::cout << "FusedFrontObject_.Properties[5].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].fusionSource = 111;
    std::cout << "FusedFrontObject_.Properties[5].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].hazardLightStatus = 112;
    std::cout << "FusedFrontObject_.Properties[5].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].headingStdDev = 78.10;
    std::cout << "FusedFrontObject_.Properties[5].headingStdDev(float32): " << FusedFrontObject_.Properties[5].headingStdDev << std::endl;
    FusedFrontObject_.Properties[5].id = 113;
    std::cout << "FusedFrontObject_.Properties[5].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].innovationFactor = 79.20;
    std::cout << "FusedFrontObject_.Properties[5].innovationFactor(float32): " << FusedFrontObject_.Properties[5].innovationFactor << std::endl;
    FusedFrontObject_.Properties[5].latPositionStdDev = 80.30;
    std::cout << "FusedFrontObject_.Properties[5].latPositionStdDev(float32): " << FusedFrontObject_.Properties[5].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[5].leftNearLaneMarkingConfidence = 114;
    std::cout << "FusedFrontObject_.Properties[5].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].longPositionStdDev = 81.40;
    std::cout << "FusedFrontObject_.Properties[5].longPositionStdDev(float32): " << FusedFrontObject_.Properties[5].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[5].motionHistory = 115;
    std::cout << "FusedFrontObject_.Properties[5].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].motionModel = 116;
    std::cout << "FusedFrontObject_.Properties[5].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].motionPattern = 117;
    std::cout << "FusedFrontObject_.Properties[5].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].radarId = 118;
    std::cout << "FusedFrontObject_.Properties[5].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].referencePoint = 119;
    std::cout << "FusedFrontObject_.Properties[5].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].reserved = 82.50;
    std::cout << "FusedFrontObject_.Properties[5].reserved(float32): " << FusedFrontObject_.Properties[5].reserved << std::endl;
    FusedFrontObject_.Properties[5].rightNearLaneMarkingConfidence = 120;
    std::cout << "FusedFrontObject_.Properties[5].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].speedStdDev = 83.60;
    std::cout << "FusedFrontObject_.Properties[5].speedStdDev(float32): " << FusedFrontObject_.Properties[5].speedStdDev << std::endl;
    FusedFrontObject_.Properties[5].trackStatus = 121;
    std::cout << "FusedFrontObject_.Properties[5].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].trafficScenario = 122;
    std::cout << "FusedFrontObject_.Properties[5].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].turnIndicator = 123;
    std::cout << "FusedFrontObject_.Properties[5].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].type = 124;
    std::cout << "FusedFrontObject_.Properties[5].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].visionId = 125;
    std::cout << "FusedFrontObject_.Properties[5].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[5].width = 84.70;
    std::cout << "FusedFrontObject_.Properties[5].width(float32): " << FusedFrontObject_.Properties[5].width << std::endl;
    FusedFrontObject_.Properties[5].length = 85.80;
    std::cout << "FusedFrontObject_.Properties[5].length(float32): " << FusedFrontObject_.Properties[5].length << std::endl;
    FusedFrontObject_.Properties[5].SensorUpdateStatus = 126;
    std::cout << "FusedFrontObject_.Properties[5].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[5].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].accelerationStdDev = 86.90;
    std::cout << "FusedFrontObject_.Properties[6].accelerationStdDev(float32): " << FusedFrontObject_.Properties[6].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[6].brakeLight = 127;
    std::cout << "FusedFrontObject_.Properties[6].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].classificationConfidence = 88.00;
    std::cout << "FusedFrontObject_.Properties[6].classificationConfidence(float32): " << FusedFrontObject_.Properties[6].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[6].cmsConfidence = 128;
    std::cout << "FusedFrontObject_.Properties[6].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].cmbbSecConfidence = 129;
    std::cout << "FusedFrontObject_.Properties[6].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].distanceToLeftNearLaneMarking = 89.10;
    std::cout << "FusedFrontObject_.Properties[6].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[6].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[6].distanceToRightNearLaneMarking = 90.20;
    std::cout << "FusedFrontObject_.Properties[6].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[6].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[6].elkaQly = 130;
    std::cout << "FusedFrontObject_.Properties[6].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].existenceConfidence = 91.30;
    std::cout << "FusedFrontObject_.Properties[6].existenceConfidence(float32): " << FusedFrontObject_.Properties[6].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[6].fcwQly = 131;
    std::cout << "FusedFrontObject_.Properties[6].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].fusionSource = 132;
    std::cout << "FusedFrontObject_.Properties[6].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].hazardLightStatus = 133;
    std::cout << "FusedFrontObject_.Properties[6].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].headingStdDev = 92.40;
    std::cout << "FusedFrontObject_.Properties[6].headingStdDev(float32): " << FusedFrontObject_.Properties[6].headingStdDev << std::endl;
    FusedFrontObject_.Properties[6].id = 134;
    std::cout << "FusedFrontObject_.Properties[6].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].innovationFactor = 93.50;
    std::cout << "FusedFrontObject_.Properties[6].innovationFactor(float32): " << FusedFrontObject_.Properties[6].innovationFactor << std::endl;
    FusedFrontObject_.Properties[6].latPositionStdDev = 94.60;
    std::cout << "FusedFrontObject_.Properties[6].latPositionStdDev(float32): " << FusedFrontObject_.Properties[6].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[6].leftNearLaneMarkingConfidence = 135;
    std::cout << "FusedFrontObject_.Properties[6].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].longPositionStdDev = 95.70;
    std::cout << "FusedFrontObject_.Properties[6].longPositionStdDev(float32): " << FusedFrontObject_.Properties[6].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[6].motionHistory = 136;
    std::cout << "FusedFrontObject_.Properties[6].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].motionModel = 137;
    std::cout << "FusedFrontObject_.Properties[6].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].motionPattern = 138;
    std::cout << "FusedFrontObject_.Properties[6].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].radarId = 139;
    std::cout << "FusedFrontObject_.Properties[6].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].referencePoint = 140;
    std::cout << "FusedFrontObject_.Properties[6].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].reserved = 96.80;
    std::cout << "FusedFrontObject_.Properties[6].reserved(float32): " << FusedFrontObject_.Properties[6].reserved << std::endl;
    FusedFrontObject_.Properties[6].rightNearLaneMarkingConfidence = 141;
    std::cout << "FusedFrontObject_.Properties[6].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].speedStdDev = 97.90;
    std::cout << "FusedFrontObject_.Properties[6].speedStdDev(float32): " << FusedFrontObject_.Properties[6].speedStdDev << std::endl;
    FusedFrontObject_.Properties[6].trackStatus = 142;
    std::cout << "FusedFrontObject_.Properties[6].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].trafficScenario = 143;
    std::cout << "FusedFrontObject_.Properties[6].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].turnIndicator = 144;
    std::cout << "FusedFrontObject_.Properties[6].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].type = 145;
    std::cout << "FusedFrontObject_.Properties[6].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].visionId = 146;
    std::cout << "FusedFrontObject_.Properties[6].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[6].width = 99.00;
    std::cout << "FusedFrontObject_.Properties[6].width(float32): " << FusedFrontObject_.Properties[6].width << std::endl;
    FusedFrontObject_.Properties[6].length = 100.10;
    std::cout << "FusedFrontObject_.Properties[6].length(float32): " << FusedFrontObject_.Properties[6].length << std::endl;
    FusedFrontObject_.Properties[6].SensorUpdateStatus = 147;
    std::cout << "FusedFrontObject_.Properties[6].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[6].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].accelerationStdDev = 101.20;
    std::cout << "FusedFrontObject_.Properties[7].accelerationStdDev(float32): " << FusedFrontObject_.Properties[7].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[7].brakeLight = 148;
    std::cout << "FusedFrontObject_.Properties[7].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].classificationConfidence = 102.30;
    std::cout << "FusedFrontObject_.Properties[7].classificationConfidence(float32): " << FusedFrontObject_.Properties[7].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[7].cmsConfidence = 149;
    std::cout << "FusedFrontObject_.Properties[7].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].cmbbSecConfidence = 150;
    std::cout << "FusedFrontObject_.Properties[7].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].distanceToLeftNearLaneMarking = 103.40;
    std::cout << "FusedFrontObject_.Properties[7].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[7].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[7].distanceToRightNearLaneMarking = 104.50;
    std::cout << "FusedFrontObject_.Properties[7].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[7].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[7].elkaQly = 151;
    std::cout << "FusedFrontObject_.Properties[7].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].existenceConfidence = 105.60;
    std::cout << "FusedFrontObject_.Properties[7].existenceConfidence(float32): " << FusedFrontObject_.Properties[7].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[7].fcwQly = 152;
    std::cout << "FusedFrontObject_.Properties[7].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].fusionSource = 153;
    std::cout << "FusedFrontObject_.Properties[7].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].hazardLightStatus = 154;
    std::cout << "FusedFrontObject_.Properties[7].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].headingStdDev = 106.70;
    std::cout << "FusedFrontObject_.Properties[7].headingStdDev(float32): " << FusedFrontObject_.Properties[7].headingStdDev << std::endl;
    FusedFrontObject_.Properties[7].id = 155;
    std::cout << "FusedFrontObject_.Properties[7].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].innovationFactor = 107.80;
    std::cout << "FusedFrontObject_.Properties[7].innovationFactor(float32): " << FusedFrontObject_.Properties[7].innovationFactor << std::endl;
    FusedFrontObject_.Properties[7].latPositionStdDev = 108.90;
    std::cout << "FusedFrontObject_.Properties[7].latPositionStdDev(float32): " << FusedFrontObject_.Properties[7].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[7].leftNearLaneMarkingConfidence = 156;
    std::cout << "FusedFrontObject_.Properties[7].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].longPositionStdDev = 110.00;
    std::cout << "FusedFrontObject_.Properties[7].longPositionStdDev(float32): " << FusedFrontObject_.Properties[7].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[7].motionHistory = 157;
    std::cout << "FusedFrontObject_.Properties[7].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].motionModel = 158;
    std::cout << "FusedFrontObject_.Properties[7].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].motionPattern = 159;
    std::cout << "FusedFrontObject_.Properties[7].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].radarId = 160;
    std::cout << "FusedFrontObject_.Properties[7].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].referencePoint = 161;
    std::cout << "FusedFrontObject_.Properties[7].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].reserved = 111.10;
    std::cout << "FusedFrontObject_.Properties[7].reserved(float32): " << FusedFrontObject_.Properties[7].reserved << std::endl;
    FusedFrontObject_.Properties[7].rightNearLaneMarkingConfidence = 162;
    std::cout << "FusedFrontObject_.Properties[7].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].speedStdDev = 112.20;
    std::cout << "FusedFrontObject_.Properties[7].speedStdDev(float32): " << FusedFrontObject_.Properties[7].speedStdDev << std::endl;
    FusedFrontObject_.Properties[7].trackStatus = 163;
    std::cout << "FusedFrontObject_.Properties[7].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].trafficScenario = 164;
    std::cout << "FusedFrontObject_.Properties[7].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].turnIndicator = 165;
    std::cout << "FusedFrontObject_.Properties[7].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].type = 166;
    std::cout << "FusedFrontObject_.Properties[7].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].visionId = 167;
    std::cout << "FusedFrontObject_.Properties[7].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[7].width = 113.30;
    std::cout << "FusedFrontObject_.Properties[7].width(float32): " << FusedFrontObject_.Properties[7].width << std::endl;
    FusedFrontObject_.Properties[7].length = 114.40;
    std::cout << "FusedFrontObject_.Properties[7].length(float32): " << FusedFrontObject_.Properties[7].length << std::endl;
    FusedFrontObject_.Properties[7].SensorUpdateStatus = 168;
    std::cout << "FusedFrontObject_.Properties[7].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[7].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].accelerationStdDev = 115.50;
    std::cout << "FusedFrontObject_.Properties[8].accelerationStdDev(float32): " << FusedFrontObject_.Properties[8].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[8].brakeLight = 169;
    std::cout << "FusedFrontObject_.Properties[8].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].classificationConfidence = 116.60;
    std::cout << "FusedFrontObject_.Properties[8].classificationConfidence(float32): " << FusedFrontObject_.Properties[8].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[8].cmsConfidence = 170;
    std::cout << "FusedFrontObject_.Properties[8].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].cmbbSecConfidence = 171;
    std::cout << "FusedFrontObject_.Properties[8].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].distanceToLeftNearLaneMarking = 117.70;
    std::cout << "FusedFrontObject_.Properties[8].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[8].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[8].distanceToRightNearLaneMarking = 118.80;
    std::cout << "FusedFrontObject_.Properties[8].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[8].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[8].elkaQly = 172;
    std::cout << "FusedFrontObject_.Properties[8].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].existenceConfidence = 119.90;
    std::cout << "FusedFrontObject_.Properties[8].existenceConfidence(float32): " << FusedFrontObject_.Properties[8].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[8].fcwQly = 173;
    std::cout << "FusedFrontObject_.Properties[8].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].fusionSource = 174;
    std::cout << "FusedFrontObject_.Properties[8].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].hazardLightStatus = 175;
    std::cout << "FusedFrontObject_.Properties[8].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].headingStdDev = 121.00;
    std::cout << "FusedFrontObject_.Properties[8].headingStdDev(float32): " << FusedFrontObject_.Properties[8].headingStdDev << std::endl;
    FusedFrontObject_.Properties[8].id = 176;
    std::cout << "FusedFrontObject_.Properties[8].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].innovationFactor = 122.10;
    std::cout << "FusedFrontObject_.Properties[8].innovationFactor(float32): " << FusedFrontObject_.Properties[8].innovationFactor << std::endl;
    FusedFrontObject_.Properties[8].latPositionStdDev = 123.20;
    std::cout << "FusedFrontObject_.Properties[8].latPositionStdDev(float32): " << FusedFrontObject_.Properties[8].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[8].leftNearLaneMarkingConfidence = 177;
    std::cout << "FusedFrontObject_.Properties[8].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].longPositionStdDev = 124.30;
    std::cout << "FusedFrontObject_.Properties[8].longPositionStdDev(float32): " << FusedFrontObject_.Properties[8].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[8].motionHistory = 178;
    std::cout << "FusedFrontObject_.Properties[8].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].motionModel = 179;
    std::cout << "FusedFrontObject_.Properties[8].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].motionPattern = 180;
    std::cout << "FusedFrontObject_.Properties[8].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].radarId = 181;
    std::cout << "FusedFrontObject_.Properties[8].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].referencePoint = 182;
    std::cout << "FusedFrontObject_.Properties[8].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].reserved = 125.40;
    std::cout << "FusedFrontObject_.Properties[8].reserved(float32): " << FusedFrontObject_.Properties[8].reserved << std::endl;
    FusedFrontObject_.Properties[8].rightNearLaneMarkingConfidence = 183;
    std::cout << "FusedFrontObject_.Properties[8].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].speedStdDev = 126.50;
    std::cout << "FusedFrontObject_.Properties[8].speedStdDev(float32): " << FusedFrontObject_.Properties[8].speedStdDev << std::endl;
    FusedFrontObject_.Properties[8].trackStatus = 184;
    std::cout << "FusedFrontObject_.Properties[8].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].trafficScenario = 185;
    std::cout << "FusedFrontObject_.Properties[8].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].turnIndicator = 186;
    std::cout << "FusedFrontObject_.Properties[8].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].type = 187;
    std::cout << "FusedFrontObject_.Properties[8].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].visionId = 188;
    std::cout << "FusedFrontObject_.Properties[8].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[8].width = 127.60;
    std::cout << "FusedFrontObject_.Properties[8].width(float32): " << FusedFrontObject_.Properties[8].width << std::endl;
    FusedFrontObject_.Properties[8].length = 128.70;
    std::cout << "FusedFrontObject_.Properties[8].length(float32): " << FusedFrontObject_.Properties[8].length << std::endl;
    FusedFrontObject_.Properties[8].SensorUpdateStatus = 189;
    std::cout << "FusedFrontObject_.Properties[8].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[8].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].accelerationStdDev = 129.80;
    std::cout << "FusedFrontObject_.Properties[9].accelerationStdDev(float32): " << FusedFrontObject_.Properties[9].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[9].brakeLight = 190;
    std::cout << "FusedFrontObject_.Properties[9].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].classificationConfidence = 130.90;
    std::cout << "FusedFrontObject_.Properties[9].classificationConfidence(float32): " << FusedFrontObject_.Properties[9].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[9].cmsConfidence = 191;
    std::cout << "FusedFrontObject_.Properties[9].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].cmbbSecConfidence = 192;
    std::cout << "FusedFrontObject_.Properties[9].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].distanceToLeftNearLaneMarking = 132.00;
    std::cout << "FusedFrontObject_.Properties[9].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[9].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[9].distanceToRightNearLaneMarking = 133.10;
    std::cout << "FusedFrontObject_.Properties[9].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[9].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[9].elkaQly = 193;
    std::cout << "FusedFrontObject_.Properties[9].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].existenceConfidence = 134.20;
    std::cout << "FusedFrontObject_.Properties[9].existenceConfidence(float32): " << FusedFrontObject_.Properties[9].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[9].fcwQly = 194;
    std::cout << "FusedFrontObject_.Properties[9].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].fusionSource = 195;
    std::cout << "FusedFrontObject_.Properties[9].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].hazardLightStatus = 196;
    std::cout << "FusedFrontObject_.Properties[9].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].headingStdDev = 135.30;
    std::cout << "FusedFrontObject_.Properties[9].headingStdDev(float32): " << FusedFrontObject_.Properties[9].headingStdDev << std::endl;
    FusedFrontObject_.Properties[9].id = 197;
    std::cout << "FusedFrontObject_.Properties[9].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].innovationFactor = 136.40;
    std::cout << "FusedFrontObject_.Properties[9].innovationFactor(float32): " << FusedFrontObject_.Properties[9].innovationFactor << std::endl;
    FusedFrontObject_.Properties[9].latPositionStdDev = 137.50;
    std::cout << "FusedFrontObject_.Properties[9].latPositionStdDev(float32): " << FusedFrontObject_.Properties[9].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[9].leftNearLaneMarkingConfidence = 198;
    std::cout << "FusedFrontObject_.Properties[9].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].longPositionStdDev = 138.60;
    std::cout << "FusedFrontObject_.Properties[9].longPositionStdDev(float32): " << FusedFrontObject_.Properties[9].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[9].motionHistory = 199;
    std::cout << "FusedFrontObject_.Properties[9].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].motionModel = 200;
    std::cout << "FusedFrontObject_.Properties[9].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].motionPattern = 201;
    std::cout << "FusedFrontObject_.Properties[9].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].radarId = 202;
    std::cout << "FusedFrontObject_.Properties[9].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].referencePoint = 203;
    std::cout << "FusedFrontObject_.Properties[9].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].reserved = 139.70;
    std::cout << "FusedFrontObject_.Properties[9].reserved(float32): " << FusedFrontObject_.Properties[9].reserved << std::endl;
    FusedFrontObject_.Properties[9].rightNearLaneMarkingConfidence = 204;
    std::cout << "FusedFrontObject_.Properties[9].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].speedStdDev = 140.80;
    std::cout << "FusedFrontObject_.Properties[9].speedStdDev(float32): " << FusedFrontObject_.Properties[9].speedStdDev << std::endl;
    FusedFrontObject_.Properties[9].trackStatus = 205;
    std::cout << "FusedFrontObject_.Properties[9].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].trafficScenario = 206;
    std::cout << "FusedFrontObject_.Properties[9].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].turnIndicator = 207;
    std::cout << "FusedFrontObject_.Properties[9].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].type = 208;
    std::cout << "FusedFrontObject_.Properties[9].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].visionId = 209;
    std::cout << "FusedFrontObject_.Properties[9].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[9].width = 141.90;
    std::cout << "FusedFrontObject_.Properties[9].width(float32): " << FusedFrontObject_.Properties[9].width << std::endl;
    FusedFrontObject_.Properties[9].length = 143.00;
    std::cout << "FusedFrontObject_.Properties[9].length(float32): " << FusedFrontObject_.Properties[9].length << std::endl;
    FusedFrontObject_.Properties[9].SensorUpdateStatus = 210;
    std::cout << "FusedFrontObject_.Properties[9].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[9].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].accelerationStdDev = 144.10;
    std::cout << "FusedFrontObject_.Properties[10].accelerationStdDev(float32): " << FusedFrontObject_.Properties[10].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[10].brakeLight = 211;
    std::cout << "FusedFrontObject_.Properties[10].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].classificationConfidence = 145.20;
    std::cout << "FusedFrontObject_.Properties[10].classificationConfidence(float32): " << FusedFrontObject_.Properties[10].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[10].cmsConfidence = 212;
    std::cout << "FusedFrontObject_.Properties[10].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].cmbbSecConfidence = 213;
    std::cout << "FusedFrontObject_.Properties[10].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].distanceToLeftNearLaneMarking = 146.30;
    std::cout << "FusedFrontObject_.Properties[10].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[10].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[10].distanceToRightNearLaneMarking = 147.40;
    std::cout << "FusedFrontObject_.Properties[10].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[10].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[10].elkaQly = 214;
    std::cout << "FusedFrontObject_.Properties[10].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].existenceConfidence = 148.50;
    std::cout << "FusedFrontObject_.Properties[10].existenceConfidence(float32): " << FusedFrontObject_.Properties[10].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[10].fcwQly = 215;
    std::cout << "FusedFrontObject_.Properties[10].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].fusionSource = 216;
    std::cout << "FusedFrontObject_.Properties[10].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].hazardLightStatus = 217;
    std::cout << "FusedFrontObject_.Properties[10].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].headingStdDev = 149.60;
    std::cout << "FusedFrontObject_.Properties[10].headingStdDev(float32): " << FusedFrontObject_.Properties[10].headingStdDev << std::endl;
    FusedFrontObject_.Properties[10].id = 218;
    std::cout << "FusedFrontObject_.Properties[10].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].innovationFactor = 150.70;
    std::cout << "FusedFrontObject_.Properties[10].innovationFactor(float32): " << FusedFrontObject_.Properties[10].innovationFactor << std::endl;
    FusedFrontObject_.Properties[10].latPositionStdDev = 151.80;
    std::cout << "FusedFrontObject_.Properties[10].latPositionStdDev(float32): " << FusedFrontObject_.Properties[10].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[10].leftNearLaneMarkingConfidence = 219;
    std::cout << "FusedFrontObject_.Properties[10].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].longPositionStdDev = 152.90;
    std::cout << "FusedFrontObject_.Properties[10].longPositionStdDev(float32): " << FusedFrontObject_.Properties[10].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[10].motionHistory = 220;
    std::cout << "FusedFrontObject_.Properties[10].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].motionModel = 221;
    std::cout << "FusedFrontObject_.Properties[10].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].motionPattern = 222;
    std::cout << "FusedFrontObject_.Properties[10].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].radarId = 223;
    std::cout << "FusedFrontObject_.Properties[10].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].referencePoint = 224;
    std::cout << "FusedFrontObject_.Properties[10].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].reserved = 154.00;
    std::cout << "FusedFrontObject_.Properties[10].reserved(float32): " << FusedFrontObject_.Properties[10].reserved << std::endl;
    FusedFrontObject_.Properties[10].rightNearLaneMarkingConfidence = 225;
    std::cout << "FusedFrontObject_.Properties[10].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].speedStdDev = 155.10;
    std::cout << "FusedFrontObject_.Properties[10].speedStdDev(float32): " << FusedFrontObject_.Properties[10].speedStdDev << std::endl;
    FusedFrontObject_.Properties[10].trackStatus = 226;
    std::cout << "FusedFrontObject_.Properties[10].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].trafficScenario = 227;
    std::cout << "FusedFrontObject_.Properties[10].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].turnIndicator = 228;
    std::cout << "FusedFrontObject_.Properties[10].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].type = 229;
    std::cout << "FusedFrontObject_.Properties[10].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].visionId = 230;
    std::cout << "FusedFrontObject_.Properties[10].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[10].width = 156.20;
    std::cout << "FusedFrontObject_.Properties[10].width(float32): " << FusedFrontObject_.Properties[10].width << std::endl;
    FusedFrontObject_.Properties[10].length = 157.30;
    std::cout << "FusedFrontObject_.Properties[10].length(float32): " << FusedFrontObject_.Properties[10].length << std::endl;
    FusedFrontObject_.Properties[10].SensorUpdateStatus = 231;
    std::cout << "FusedFrontObject_.Properties[10].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[10].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].accelerationStdDev = 158.40;
    std::cout << "FusedFrontObject_.Properties[11].accelerationStdDev(float32): " << FusedFrontObject_.Properties[11].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[11].brakeLight = 232;
    std::cout << "FusedFrontObject_.Properties[11].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].classificationConfidence = 159.50;
    std::cout << "FusedFrontObject_.Properties[11].classificationConfidence(float32): " << FusedFrontObject_.Properties[11].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[11].cmsConfidence = 233;
    std::cout << "FusedFrontObject_.Properties[11].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].cmbbSecConfidence = 234;
    std::cout << "FusedFrontObject_.Properties[11].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].distanceToLeftNearLaneMarking = 160.60;
    std::cout << "FusedFrontObject_.Properties[11].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[11].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[11].distanceToRightNearLaneMarking = 161.70;
    std::cout << "FusedFrontObject_.Properties[11].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[11].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[11].elkaQly = 235;
    std::cout << "FusedFrontObject_.Properties[11].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].existenceConfidence = 162.80;
    std::cout << "FusedFrontObject_.Properties[11].existenceConfidence(float32): " << FusedFrontObject_.Properties[11].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[11].fcwQly = 236;
    std::cout << "FusedFrontObject_.Properties[11].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].fusionSource = 237;
    std::cout << "FusedFrontObject_.Properties[11].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].hazardLightStatus = 238;
    std::cout << "FusedFrontObject_.Properties[11].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].headingStdDev = 163.90;
    std::cout << "FusedFrontObject_.Properties[11].headingStdDev(float32): " << FusedFrontObject_.Properties[11].headingStdDev << std::endl;
    FusedFrontObject_.Properties[11].id = 239;
    std::cout << "FusedFrontObject_.Properties[11].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].innovationFactor = 165.00;
    std::cout << "FusedFrontObject_.Properties[11].innovationFactor(float32): " << FusedFrontObject_.Properties[11].innovationFactor << std::endl;
    FusedFrontObject_.Properties[11].latPositionStdDev = 166.10;
    std::cout << "FusedFrontObject_.Properties[11].latPositionStdDev(float32): " << FusedFrontObject_.Properties[11].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[11].leftNearLaneMarkingConfidence = 240;
    std::cout << "FusedFrontObject_.Properties[11].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].longPositionStdDev = 167.20;
    std::cout << "FusedFrontObject_.Properties[11].longPositionStdDev(float32): " << FusedFrontObject_.Properties[11].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[11].motionHistory = 241;
    std::cout << "FusedFrontObject_.Properties[11].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].motionModel = 242;
    std::cout << "FusedFrontObject_.Properties[11].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].motionPattern = 243;
    std::cout << "FusedFrontObject_.Properties[11].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].radarId = 244;
    std::cout << "FusedFrontObject_.Properties[11].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].referencePoint = 245;
    std::cout << "FusedFrontObject_.Properties[11].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].reserved = 168.30;
    std::cout << "FusedFrontObject_.Properties[11].reserved(float32): " << FusedFrontObject_.Properties[11].reserved << std::endl;
    FusedFrontObject_.Properties[11].rightNearLaneMarkingConfidence = 246;
    std::cout << "FusedFrontObject_.Properties[11].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].speedStdDev = 169.40;
    std::cout << "FusedFrontObject_.Properties[11].speedStdDev(float32): " << FusedFrontObject_.Properties[11].speedStdDev << std::endl;
    FusedFrontObject_.Properties[11].trackStatus = 247;
    std::cout << "FusedFrontObject_.Properties[11].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].trafficScenario = 248;
    std::cout << "FusedFrontObject_.Properties[11].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].turnIndicator = 249;
    std::cout << "FusedFrontObject_.Properties[11].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].type = 250;
    std::cout << "FusedFrontObject_.Properties[11].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].visionId = 251;
    std::cout << "FusedFrontObject_.Properties[11].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[11].width = 170.50;
    std::cout << "FusedFrontObject_.Properties[11].width(float32): " << FusedFrontObject_.Properties[11].width << std::endl;
    FusedFrontObject_.Properties[11].length = 171.60;
    std::cout << "FusedFrontObject_.Properties[11].length(float32): " << FusedFrontObject_.Properties[11].length << std::endl;
    FusedFrontObject_.Properties[11].SensorUpdateStatus = 252;
    std::cout << "FusedFrontObject_.Properties[11].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[11].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].accelerationStdDev = 172.70;
    std::cout << "FusedFrontObject_.Properties[12].accelerationStdDev(float32): " << FusedFrontObject_.Properties[12].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[12].brakeLight = 253;
    std::cout << "FusedFrontObject_.Properties[12].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].classificationConfidence = 173.80;
    std::cout << "FusedFrontObject_.Properties[12].classificationConfidence(float32): " << FusedFrontObject_.Properties[12].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[12].cmsConfidence = 254;
    std::cout << "FusedFrontObject_.Properties[12].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].cmbbSecConfidence = 255;
    std::cout << "FusedFrontObject_.Properties[12].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].distanceToLeftNearLaneMarking = 174.90;
    std::cout << "FusedFrontObject_.Properties[12].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[12].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[12].distanceToRightNearLaneMarking = 176.00;
    std::cout << "FusedFrontObject_.Properties[12].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[12].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[12].elkaQly = 0;
    std::cout << "FusedFrontObject_.Properties[12].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].existenceConfidence = 177.10;
    std::cout << "FusedFrontObject_.Properties[12].existenceConfidence(float32): " << FusedFrontObject_.Properties[12].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[12].fcwQly = 1;
    std::cout << "FusedFrontObject_.Properties[12].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].fusionSource = 2;
    std::cout << "FusedFrontObject_.Properties[12].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].hazardLightStatus = 3;
    std::cout << "FusedFrontObject_.Properties[12].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].headingStdDev = 178.20;
    std::cout << "FusedFrontObject_.Properties[12].headingStdDev(float32): " << FusedFrontObject_.Properties[12].headingStdDev << std::endl;
    FusedFrontObject_.Properties[12].id = 4;
    std::cout << "FusedFrontObject_.Properties[12].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].innovationFactor = 179.30;
    std::cout << "FusedFrontObject_.Properties[12].innovationFactor(float32): " << FusedFrontObject_.Properties[12].innovationFactor << std::endl;
    FusedFrontObject_.Properties[12].latPositionStdDev = 180.40;
    std::cout << "FusedFrontObject_.Properties[12].latPositionStdDev(float32): " << FusedFrontObject_.Properties[12].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[12].leftNearLaneMarkingConfidence = 5;
    std::cout << "FusedFrontObject_.Properties[12].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].longPositionStdDev = 181.50;
    std::cout << "FusedFrontObject_.Properties[12].longPositionStdDev(float32): " << FusedFrontObject_.Properties[12].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[12].motionHistory = 6;
    std::cout << "FusedFrontObject_.Properties[12].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].motionModel = 7;
    std::cout << "FusedFrontObject_.Properties[12].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].motionPattern = 8;
    std::cout << "FusedFrontObject_.Properties[12].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].radarId = 9;
    std::cout << "FusedFrontObject_.Properties[12].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].referencePoint = 10;
    std::cout << "FusedFrontObject_.Properties[12].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].reserved = 182.60;
    std::cout << "FusedFrontObject_.Properties[12].reserved(float32): " << FusedFrontObject_.Properties[12].reserved << std::endl;
    FusedFrontObject_.Properties[12].rightNearLaneMarkingConfidence = 11;
    std::cout << "FusedFrontObject_.Properties[12].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].speedStdDev = 183.70;
    std::cout << "FusedFrontObject_.Properties[12].speedStdDev(float32): " << FusedFrontObject_.Properties[12].speedStdDev << std::endl;
    FusedFrontObject_.Properties[12].trackStatus = 12;
    std::cout << "FusedFrontObject_.Properties[12].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].trafficScenario = 13;
    std::cout << "FusedFrontObject_.Properties[12].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].turnIndicator = 14;
    std::cout << "FusedFrontObject_.Properties[12].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].type = 15;
    std::cout << "FusedFrontObject_.Properties[12].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].visionId = 16;
    std::cout << "FusedFrontObject_.Properties[12].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[12].width = 184.80;
    std::cout << "FusedFrontObject_.Properties[12].width(float32): " << FusedFrontObject_.Properties[12].width << std::endl;
    FusedFrontObject_.Properties[12].length = 185.90;
    std::cout << "FusedFrontObject_.Properties[12].length(float32): " << FusedFrontObject_.Properties[12].length << std::endl;
    FusedFrontObject_.Properties[12].SensorUpdateStatus = 17;
    std::cout << "FusedFrontObject_.Properties[12].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[12].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].accelerationStdDev = 187.00;
    std::cout << "FusedFrontObject_.Properties[13].accelerationStdDev(float32): " << FusedFrontObject_.Properties[13].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[13].brakeLight = 18;
    std::cout << "FusedFrontObject_.Properties[13].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].classificationConfidence = 188.10;
    std::cout << "FusedFrontObject_.Properties[13].classificationConfidence(float32): " << FusedFrontObject_.Properties[13].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[13].cmsConfidence = 19;
    std::cout << "FusedFrontObject_.Properties[13].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].cmbbSecConfidence = 20;
    std::cout << "FusedFrontObject_.Properties[13].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].distanceToLeftNearLaneMarking = 189.20;
    std::cout << "FusedFrontObject_.Properties[13].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[13].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[13].distanceToRightNearLaneMarking = 190.30;
    std::cout << "FusedFrontObject_.Properties[13].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[13].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[13].elkaQly = 21;
    std::cout << "FusedFrontObject_.Properties[13].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].existenceConfidence = 191.40;
    std::cout << "FusedFrontObject_.Properties[13].existenceConfidence(float32): " << FusedFrontObject_.Properties[13].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[13].fcwQly = 22;
    std::cout << "FusedFrontObject_.Properties[13].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].fusionSource = 23;
    std::cout << "FusedFrontObject_.Properties[13].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].hazardLightStatus = 24;
    std::cout << "FusedFrontObject_.Properties[13].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].headingStdDev = 192.50;
    std::cout << "FusedFrontObject_.Properties[13].headingStdDev(float32): " << FusedFrontObject_.Properties[13].headingStdDev << std::endl;
    FusedFrontObject_.Properties[13].id = 25;
    std::cout << "FusedFrontObject_.Properties[13].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].innovationFactor = 193.60;
    std::cout << "FusedFrontObject_.Properties[13].innovationFactor(float32): " << FusedFrontObject_.Properties[13].innovationFactor << std::endl;
    FusedFrontObject_.Properties[13].latPositionStdDev = 194.70;
    std::cout << "FusedFrontObject_.Properties[13].latPositionStdDev(float32): " << FusedFrontObject_.Properties[13].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[13].leftNearLaneMarkingConfidence = 26;
    std::cout << "FusedFrontObject_.Properties[13].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].longPositionStdDev = 195.80;
    std::cout << "FusedFrontObject_.Properties[13].longPositionStdDev(float32): " << FusedFrontObject_.Properties[13].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[13].motionHistory = 27;
    std::cout << "FusedFrontObject_.Properties[13].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].motionModel = 28;
    std::cout << "FusedFrontObject_.Properties[13].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].motionPattern = 29;
    std::cout << "FusedFrontObject_.Properties[13].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].radarId = 30;
    std::cout << "FusedFrontObject_.Properties[13].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].referencePoint = 31;
    std::cout << "FusedFrontObject_.Properties[13].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].reserved = 196.90;
    std::cout << "FusedFrontObject_.Properties[13].reserved(float32): " << FusedFrontObject_.Properties[13].reserved << std::endl;
    FusedFrontObject_.Properties[13].rightNearLaneMarkingConfidence = 32;
    std::cout << "FusedFrontObject_.Properties[13].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].speedStdDev = 198.00;
    std::cout << "FusedFrontObject_.Properties[13].speedStdDev(float32): " << FusedFrontObject_.Properties[13].speedStdDev << std::endl;
    FusedFrontObject_.Properties[13].trackStatus = 33;
    std::cout << "FusedFrontObject_.Properties[13].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].trafficScenario = 34;
    std::cout << "FusedFrontObject_.Properties[13].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].turnIndicator = 35;
    std::cout << "FusedFrontObject_.Properties[13].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].type = 36;
    std::cout << "FusedFrontObject_.Properties[13].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].visionId = 37;
    std::cout << "FusedFrontObject_.Properties[13].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[13].width = 199.10;
    std::cout << "FusedFrontObject_.Properties[13].width(float32): " << FusedFrontObject_.Properties[13].width << std::endl;
    FusedFrontObject_.Properties[13].length = 200.20;
    std::cout << "FusedFrontObject_.Properties[13].length(float32): " << FusedFrontObject_.Properties[13].length << std::endl;
    FusedFrontObject_.Properties[13].SensorUpdateStatus = 38;
    std::cout << "FusedFrontObject_.Properties[13].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[13].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].accelerationStdDev = 201.30;
    std::cout << "FusedFrontObject_.Properties[14].accelerationStdDev(float32): " << FusedFrontObject_.Properties[14].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[14].brakeLight = 39;
    std::cout << "FusedFrontObject_.Properties[14].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].classificationConfidence = 202.40;
    std::cout << "FusedFrontObject_.Properties[14].classificationConfidence(float32): " << FusedFrontObject_.Properties[14].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[14].cmsConfidence = 40;
    std::cout << "FusedFrontObject_.Properties[14].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].cmbbSecConfidence = 41;
    std::cout << "FusedFrontObject_.Properties[14].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].distanceToLeftNearLaneMarking = 203.50;
    std::cout << "FusedFrontObject_.Properties[14].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[14].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[14].distanceToRightNearLaneMarking = 204.60;
    std::cout << "FusedFrontObject_.Properties[14].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[14].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[14].elkaQly = 42;
    std::cout << "FusedFrontObject_.Properties[14].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].existenceConfidence = 205.70;
    std::cout << "FusedFrontObject_.Properties[14].existenceConfidence(float32): " << FusedFrontObject_.Properties[14].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[14].fcwQly = 43;
    std::cout << "FusedFrontObject_.Properties[14].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].fusionSource = 44;
    std::cout << "FusedFrontObject_.Properties[14].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].hazardLightStatus = 45;
    std::cout << "FusedFrontObject_.Properties[14].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].headingStdDev = 206.80;
    std::cout << "FusedFrontObject_.Properties[14].headingStdDev(float32): " << FusedFrontObject_.Properties[14].headingStdDev << std::endl;
    FusedFrontObject_.Properties[14].id = 46;
    std::cout << "FusedFrontObject_.Properties[14].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].innovationFactor = 207.90;
    std::cout << "FusedFrontObject_.Properties[14].innovationFactor(float32): " << FusedFrontObject_.Properties[14].innovationFactor << std::endl;
    FusedFrontObject_.Properties[14].latPositionStdDev = 209.00;
    std::cout << "FusedFrontObject_.Properties[14].latPositionStdDev(float32): " << FusedFrontObject_.Properties[14].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[14].leftNearLaneMarkingConfidence = 47;
    std::cout << "FusedFrontObject_.Properties[14].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].longPositionStdDev = 210.10;
    std::cout << "FusedFrontObject_.Properties[14].longPositionStdDev(float32): " << FusedFrontObject_.Properties[14].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[14].motionHistory = 48;
    std::cout << "FusedFrontObject_.Properties[14].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].motionModel = 49;
    std::cout << "FusedFrontObject_.Properties[14].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].motionPattern = 50;
    std::cout << "FusedFrontObject_.Properties[14].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].radarId = 51;
    std::cout << "FusedFrontObject_.Properties[14].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].referencePoint = 52;
    std::cout << "FusedFrontObject_.Properties[14].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].reserved = 211.20;
    std::cout << "FusedFrontObject_.Properties[14].reserved(float32): " << FusedFrontObject_.Properties[14].reserved << std::endl;
    FusedFrontObject_.Properties[14].rightNearLaneMarkingConfidence = 53;
    std::cout << "FusedFrontObject_.Properties[14].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].speedStdDev = 212.30;
    std::cout << "FusedFrontObject_.Properties[14].speedStdDev(float32): " << FusedFrontObject_.Properties[14].speedStdDev << std::endl;
    FusedFrontObject_.Properties[14].trackStatus = 54;
    std::cout << "FusedFrontObject_.Properties[14].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].trafficScenario = 55;
    std::cout << "FusedFrontObject_.Properties[14].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].turnIndicator = 56;
    std::cout << "FusedFrontObject_.Properties[14].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].type = 57;
    std::cout << "FusedFrontObject_.Properties[14].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].visionId = 58;
    std::cout << "FusedFrontObject_.Properties[14].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[14].width = 213.40;
    std::cout << "FusedFrontObject_.Properties[14].width(float32): " << FusedFrontObject_.Properties[14].width << std::endl;
    FusedFrontObject_.Properties[14].length = 214.50;
    std::cout << "FusedFrontObject_.Properties[14].length(float32): " << FusedFrontObject_.Properties[14].length << std::endl;
    FusedFrontObject_.Properties[14].SensorUpdateStatus = 59;
    std::cout << "FusedFrontObject_.Properties[14].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[14].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].accelerationStdDev = 215.60;
    std::cout << "FusedFrontObject_.Properties[15].accelerationStdDev(float32): " << FusedFrontObject_.Properties[15].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[15].brakeLight = 60;
    std::cout << "FusedFrontObject_.Properties[15].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].classificationConfidence = 216.70;
    std::cout << "FusedFrontObject_.Properties[15].classificationConfidence(float32): " << FusedFrontObject_.Properties[15].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[15].cmsConfidence = 61;
    std::cout << "FusedFrontObject_.Properties[15].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].cmbbSecConfidence = 62;
    std::cout << "FusedFrontObject_.Properties[15].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].distanceToLeftNearLaneMarking = 217.80;
    std::cout << "FusedFrontObject_.Properties[15].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[15].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[15].distanceToRightNearLaneMarking = 218.90;
    std::cout << "FusedFrontObject_.Properties[15].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[15].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[15].elkaQly = 63;
    std::cout << "FusedFrontObject_.Properties[15].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].existenceConfidence = 220.00;
    std::cout << "FusedFrontObject_.Properties[15].existenceConfidence(float32): " << FusedFrontObject_.Properties[15].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[15].fcwQly = 64;
    std::cout << "FusedFrontObject_.Properties[15].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].fusionSource = 65;
    std::cout << "FusedFrontObject_.Properties[15].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].hazardLightStatus = 66;
    std::cout << "FusedFrontObject_.Properties[15].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].headingStdDev = 221.10;
    std::cout << "FusedFrontObject_.Properties[15].headingStdDev(float32): " << FusedFrontObject_.Properties[15].headingStdDev << std::endl;
    FusedFrontObject_.Properties[15].id = 67;
    std::cout << "FusedFrontObject_.Properties[15].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].innovationFactor = 222.20;
    std::cout << "FusedFrontObject_.Properties[15].innovationFactor(float32): " << FusedFrontObject_.Properties[15].innovationFactor << std::endl;
    FusedFrontObject_.Properties[15].latPositionStdDev = 223.30;
    std::cout << "FusedFrontObject_.Properties[15].latPositionStdDev(float32): " << FusedFrontObject_.Properties[15].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[15].leftNearLaneMarkingConfidence = 68;
    std::cout << "FusedFrontObject_.Properties[15].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].longPositionStdDev = 224.40;
    std::cout << "FusedFrontObject_.Properties[15].longPositionStdDev(float32): " << FusedFrontObject_.Properties[15].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[15].motionHistory = 69;
    std::cout << "FusedFrontObject_.Properties[15].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].motionModel = 70;
    std::cout << "FusedFrontObject_.Properties[15].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].motionPattern = 71;
    std::cout << "FusedFrontObject_.Properties[15].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].radarId = 72;
    std::cout << "FusedFrontObject_.Properties[15].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].referencePoint = 73;
    std::cout << "FusedFrontObject_.Properties[15].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].reserved = 225.50;
    std::cout << "FusedFrontObject_.Properties[15].reserved(float32): " << FusedFrontObject_.Properties[15].reserved << std::endl;
    FusedFrontObject_.Properties[15].rightNearLaneMarkingConfidence = 74;
    std::cout << "FusedFrontObject_.Properties[15].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].speedStdDev = 226.60;
    std::cout << "FusedFrontObject_.Properties[15].speedStdDev(float32): " << FusedFrontObject_.Properties[15].speedStdDev << std::endl;
    FusedFrontObject_.Properties[15].trackStatus = 75;
    std::cout << "FusedFrontObject_.Properties[15].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].trafficScenario = 76;
    std::cout << "FusedFrontObject_.Properties[15].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].turnIndicator = 77;
    std::cout << "FusedFrontObject_.Properties[15].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].type = 78;
    std::cout << "FusedFrontObject_.Properties[15].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].visionId = 79;
    std::cout << "FusedFrontObject_.Properties[15].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[15].width = 227.70;
    std::cout << "FusedFrontObject_.Properties[15].width(float32): " << FusedFrontObject_.Properties[15].width << std::endl;
    FusedFrontObject_.Properties[15].length = 228.80;
    std::cout << "FusedFrontObject_.Properties[15].length(float32): " << FusedFrontObject_.Properties[15].length << std::endl;
    FusedFrontObject_.Properties[15].SensorUpdateStatus = 80;
    std::cout << "FusedFrontObject_.Properties[15].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[15].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].accelerationStdDev = 229.90;
    std::cout << "FusedFrontObject_.Properties[16].accelerationStdDev(float32): " << FusedFrontObject_.Properties[16].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[16].brakeLight = 81;
    std::cout << "FusedFrontObject_.Properties[16].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].classificationConfidence = 231.00;
    std::cout << "FusedFrontObject_.Properties[16].classificationConfidence(float32): " << FusedFrontObject_.Properties[16].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[16].cmsConfidence = 82;
    std::cout << "FusedFrontObject_.Properties[16].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].cmbbSecConfidence = 83;
    std::cout << "FusedFrontObject_.Properties[16].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].distanceToLeftNearLaneMarking = 232.10;
    std::cout << "FusedFrontObject_.Properties[16].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[16].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[16].distanceToRightNearLaneMarking = 233.20;
    std::cout << "FusedFrontObject_.Properties[16].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[16].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[16].elkaQly = 84;
    std::cout << "FusedFrontObject_.Properties[16].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].existenceConfidence = 234.30;
    std::cout << "FusedFrontObject_.Properties[16].existenceConfidence(float32): " << FusedFrontObject_.Properties[16].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[16].fcwQly = 85;
    std::cout << "FusedFrontObject_.Properties[16].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].fusionSource = 86;
    std::cout << "FusedFrontObject_.Properties[16].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].hazardLightStatus = 87;
    std::cout << "FusedFrontObject_.Properties[16].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].headingStdDev = 235.40;
    std::cout << "FusedFrontObject_.Properties[16].headingStdDev(float32): " << FusedFrontObject_.Properties[16].headingStdDev << std::endl;
    FusedFrontObject_.Properties[16].id = 88;
    std::cout << "FusedFrontObject_.Properties[16].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].innovationFactor = 236.50;
    std::cout << "FusedFrontObject_.Properties[16].innovationFactor(float32): " << FusedFrontObject_.Properties[16].innovationFactor << std::endl;
    FusedFrontObject_.Properties[16].latPositionStdDev = 237.60;
    std::cout << "FusedFrontObject_.Properties[16].latPositionStdDev(float32): " << FusedFrontObject_.Properties[16].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[16].leftNearLaneMarkingConfidence = 89;
    std::cout << "FusedFrontObject_.Properties[16].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].longPositionStdDev = 238.70;
    std::cout << "FusedFrontObject_.Properties[16].longPositionStdDev(float32): " << FusedFrontObject_.Properties[16].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[16].motionHistory = 90;
    std::cout << "FusedFrontObject_.Properties[16].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].motionModel = 91;
    std::cout << "FusedFrontObject_.Properties[16].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].motionPattern = 92;
    std::cout << "FusedFrontObject_.Properties[16].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].radarId = 93;
    std::cout << "FusedFrontObject_.Properties[16].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].referencePoint = 94;
    std::cout << "FusedFrontObject_.Properties[16].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].reserved = 239.80;
    std::cout << "FusedFrontObject_.Properties[16].reserved(float32): " << FusedFrontObject_.Properties[16].reserved << std::endl;
    FusedFrontObject_.Properties[16].rightNearLaneMarkingConfidence = 95;
    std::cout << "FusedFrontObject_.Properties[16].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].speedStdDev = 240.90;
    std::cout << "FusedFrontObject_.Properties[16].speedStdDev(float32): " << FusedFrontObject_.Properties[16].speedStdDev << std::endl;
    FusedFrontObject_.Properties[16].trackStatus = 96;
    std::cout << "FusedFrontObject_.Properties[16].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].trafficScenario = 97;
    std::cout << "FusedFrontObject_.Properties[16].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].turnIndicator = 98;
    std::cout << "FusedFrontObject_.Properties[16].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].type = 99;
    std::cout << "FusedFrontObject_.Properties[16].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].visionId = 100;
    std::cout << "FusedFrontObject_.Properties[16].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[16].width = 242.00;
    std::cout << "FusedFrontObject_.Properties[16].width(float32): " << FusedFrontObject_.Properties[16].width << std::endl;
    FusedFrontObject_.Properties[16].length = 243.10;
    std::cout << "FusedFrontObject_.Properties[16].length(float32): " << FusedFrontObject_.Properties[16].length << std::endl;
    FusedFrontObject_.Properties[16].SensorUpdateStatus = 101;
    std::cout << "FusedFrontObject_.Properties[16].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[16].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].accelerationStdDev = 244.20;
    std::cout << "FusedFrontObject_.Properties[17].accelerationStdDev(float32): " << FusedFrontObject_.Properties[17].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[17].brakeLight = 102;
    std::cout << "FusedFrontObject_.Properties[17].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].classificationConfidence = 245.30;
    std::cout << "FusedFrontObject_.Properties[17].classificationConfidence(float32): " << FusedFrontObject_.Properties[17].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[17].cmsConfidence = 103;
    std::cout << "FusedFrontObject_.Properties[17].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].cmbbSecConfidence = 104;
    std::cout << "FusedFrontObject_.Properties[17].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].distanceToLeftNearLaneMarking = 246.40;
    std::cout << "FusedFrontObject_.Properties[17].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[17].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[17].distanceToRightNearLaneMarking = 247.50;
    std::cout << "FusedFrontObject_.Properties[17].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[17].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[17].elkaQly = 105;
    std::cout << "FusedFrontObject_.Properties[17].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].existenceConfidence = 248.60;
    std::cout << "FusedFrontObject_.Properties[17].existenceConfidence(float32): " << FusedFrontObject_.Properties[17].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[17].fcwQly = 106;
    std::cout << "FusedFrontObject_.Properties[17].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].fusionSource = 107;
    std::cout << "FusedFrontObject_.Properties[17].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].hazardLightStatus = 108;
    std::cout << "FusedFrontObject_.Properties[17].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].headingStdDev = 249.70;
    std::cout << "FusedFrontObject_.Properties[17].headingStdDev(float32): " << FusedFrontObject_.Properties[17].headingStdDev << std::endl;
    FusedFrontObject_.Properties[17].id = 109;
    std::cout << "FusedFrontObject_.Properties[17].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].innovationFactor = 250.80;
    std::cout << "FusedFrontObject_.Properties[17].innovationFactor(float32): " << FusedFrontObject_.Properties[17].innovationFactor << std::endl;
    FusedFrontObject_.Properties[17].latPositionStdDev = 251.90;
    std::cout << "FusedFrontObject_.Properties[17].latPositionStdDev(float32): " << FusedFrontObject_.Properties[17].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[17].leftNearLaneMarkingConfidence = 110;
    std::cout << "FusedFrontObject_.Properties[17].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].longPositionStdDev = 253.00;
    std::cout << "FusedFrontObject_.Properties[17].longPositionStdDev(float32): " << FusedFrontObject_.Properties[17].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[17].motionHistory = 111;
    std::cout << "FusedFrontObject_.Properties[17].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].motionModel = 112;
    std::cout << "FusedFrontObject_.Properties[17].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].motionPattern = 113;
    std::cout << "FusedFrontObject_.Properties[17].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].radarId = 114;
    std::cout << "FusedFrontObject_.Properties[17].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].referencePoint = 115;
    std::cout << "FusedFrontObject_.Properties[17].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].reserved = 254.10;
    std::cout << "FusedFrontObject_.Properties[17].reserved(float32): " << FusedFrontObject_.Properties[17].reserved << std::endl;
    FusedFrontObject_.Properties[17].rightNearLaneMarkingConfidence = 116;
    std::cout << "FusedFrontObject_.Properties[17].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].speedStdDev = 255.20;
    std::cout << "FusedFrontObject_.Properties[17].speedStdDev(float32): " << FusedFrontObject_.Properties[17].speedStdDev << std::endl;
    FusedFrontObject_.Properties[17].trackStatus = 117;
    std::cout << "FusedFrontObject_.Properties[17].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].trafficScenario = 118;
    std::cout << "FusedFrontObject_.Properties[17].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].turnIndicator = 119;
    std::cout << "FusedFrontObject_.Properties[17].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].type = 120;
    std::cout << "FusedFrontObject_.Properties[17].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].visionId = 121;
    std::cout << "FusedFrontObject_.Properties[17].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[17].width = 256.30;
    std::cout << "FusedFrontObject_.Properties[17].width(float32): " << FusedFrontObject_.Properties[17].width << std::endl;
    FusedFrontObject_.Properties[17].length = 257.40;
    std::cout << "FusedFrontObject_.Properties[17].length(float32): " << FusedFrontObject_.Properties[17].length << std::endl;
    FusedFrontObject_.Properties[17].SensorUpdateStatus = 122;
    std::cout << "FusedFrontObject_.Properties[17].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[17].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].accelerationStdDev = 258.50;
    std::cout << "FusedFrontObject_.Properties[18].accelerationStdDev(float32): " << FusedFrontObject_.Properties[18].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[18].brakeLight = 123;
    std::cout << "FusedFrontObject_.Properties[18].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].classificationConfidence = 259.60;
    std::cout << "FusedFrontObject_.Properties[18].classificationConfidence(float32): " << FusedFrontObject_.Properties[18].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[18].cmsConfidence = 124;
    std::cout << "FusedFrontObject_.Properties[18].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].cmbbSecConfidence = 125;
    std::cout << "FusedFrontObject_.Properties[18].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].distanceToLeftNearLaneMarking = 260.70;
    std::cout << "FusedFrontObject_.Properties[18].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[18].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[18].distanceToRightNearLaneMarking = 261.80;
    std::cout << "FusedFrontObject_.Properties[18].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[18].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[18].elkaQly = 126;
    std::cout << "FusedFrontObject_.Properties[18].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].existenceConfidence = 262.90;
    std::cout << "FusedFrontObject_.Properties[18].existenceConfidence(float32): " << FusedFrontObject_.Properties[18].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[18].fcwQly = 127;
    std::cout << "FusedFrontObject_.Properties[18].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].fusionSource = 128;
    std::cout << "FusedFrontObject_.Properties[18].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].hazardLightStatus = 129;
    std::cout << "FusedFrontObject_.Properties[18].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].headingStdDev = 264.00;
    std::cout << "FusedFrontObject_.Properties[18].headingStdDev(float32): " << FusedFrontObject_.Properties[18].headingStdDev << std::endl;
    FusedFrontObject_.Properties[18].id = 130;
    std::cout << "FusedFrontObject_.Properties[18].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].innovationFactor = 265.10;
    std::cout << "FusedFrontObject_.Properties[18].innovationFactor(float32): " << FusedFrontObject_.Properties[18].innovationFactor << std::endl;
    FusedFrontObject_.Properties[18].latPositionStdDev = 266.20;
    std::cout << "FusedFrontObject_.Properties[18].latPositionStdDev(float32): " << FusedFrontObject_.Properties[18].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[18].leftNearLaneMarkingConfidence = 131;
    std::cout << "FusedFrontObject_.Properties[18].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].longPositionStdDev = 267.30;
    std::cout << "FusedFrontObject_.Properties[18].longPositionStdDev(float32): " << FusedFrontObject_.Properties[18].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[18].motionHistory = 132;
    std::cout << "FusedFrontObject_.Properties[18].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].motionModel = 133;
    std::cout << "FusedFrontObject_.Properties[18].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].motionPattern = 134;
    std::cout << "FusedFrontObject_.Properties[18].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].radarId = 135;
    std::cout << "FusedFrontObject_.Properties[18].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].referencePoint = 136;
    std::cout << "FusedFrontObject_.Properties[18].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].reserved = 268.40;
    std::cout << "FusedFrontObject_.Properties[18].reserved(float32): " << FusedFrontObject_.Properties[18].reserved << std::endl;
    FusedFrontObject_.Properties[18].rightNearLaneMarkingConfidence = 137;
    std::cout << "FusedFrontObject_.Properties[18].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].speedStdDev = 269.50;
    std::cout << "FusedFrontObject_.Properties[18].speedStdDev(float32): " << FusedFrontObject_.Properties[18].speedStdDev << std::endl;
    FusedFrontObject_.Properties[18].trackStatus = 138;
    std::cout << "FusedFrontObject_.Properties[18].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].trafficScenario = 139;
    std::cout << "FusedFrontObject_.Properties[18].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].turnIndicator = 140;
    std::cout << "FusedFrontObject_.Properties[18].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].type = 141;
    std::cout << "FusedFrontObject_.Properties[18].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].visionId = 142;
    std::cout << "FusedFrontObject_.Properties[18].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[18].width = 270.60;
    std::cout << "FusedFrontObject_.Properties[18].width(float32): " << FusedFrontObject_.Properties[18].width << std::endl;
    FusedFrontObject_.Properties[18].length = 271.70;
    std::cout << "FusedFrontObject_.Properties[18].length(float32): " << FusedFrontObject_.Properties[18].length << std::endl;
    FusedFrontObject_.Properties[18].SensorUpdateStatus = 143;
    std::cout << "FusedFrontObject_.Properties[18].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[18].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].accelerationStdDev = 272.80;
    std::cout << "FusedFrontObject_.Properties[19].accelerationStdDev(float32): " << FusedFrontObject_.Properties[19].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[19].brakeLight = 144;
    std::cout << "FusedFrontObject_.Properties[19].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].classificationConfidence = 273.90;
    std::cout << "FusedFrontObject_.Properties[19].classificationConfidence(float32): " << FusedFrontObject_.Properties[19].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[19].cmsConfidence = 145;
    std::cout << "FusedFrontObject_.Properties[19].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].cmbbSecConfidence = 146;
    std::cout << "FusedFrontObject_.Properties[19].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].distanceToLeftNearLaneMarking = 275.00;
    std::cout << "FusedFrontObject_.Properties[19].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[19].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[19].distanceToRightNearLaneMarking = 276.10;
    std::cout << "FusedFrontObject_.Properties[19].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[19].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[19].elkaQly = 147;
    std::cout << "FusedFrontObject_.Properties[19].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].existenceConfidence = 277.20;
    std::cout << "FusedFrontObject_.Properties[19].existenceConfidence(float32): " << FusedFrontObject_.Properties[19].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[19].fcwQly = 148;
    std::cout << "FusedFrontObject_.Properties[19].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].fusionSource = 149;
    std::cout << "FusedFrontObject_.Properties[19].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].hazardLightStatus = 150;
    std::cout << "FusedFrontObject_.Properties[19].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].headingStdDev = 278.30;
    std::cout << "FusedFrontObject_.Properties[19].headingStdDev(float32): " << FusedFrontObject_.Properties[19].headingStdDev << std::endl;
    FusedFrontObject_.Properties[19].id = 151;
    std::cout << "FusedFrontObject_.Properties[19].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].innovationFactor = 279.40;
    std::cout << "FusedFrontObject_.Properties[19].innovationFactor(float32): " << FusedFrontObject_.Properties[19].innovationFactor << std::endl;
    FusedFrontObject_.Properties[19].latPositionStdDev = 280.50;
    std::cout << "FusedFrontObject_.Properties[19].latPositionStdDev(float32): " << FusedFrontObject_.Properties[19].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[19].leftNearLaneMarkingConfidence = 152;
    std::cout << "FusedFrontObject_.Properties[19].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].longPositionStdDev = 281.60;
    std::cout << "FusedFrontObject_.Properties[19].longPositionStdDev(float32): " << FusedFrontObject_.Properties[19].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[19].motionHistory = 153;
    std::cout << "FusedFrontObject_.Properties[19].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].motionModel = 154;
    std::cout << "FusedFrontObject_.Properties[19].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].motionPattern = 155;
    std::cout << "FusedFrontObject_.Properties[19].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].radarId = 156;
    std::cout << "FusedFrontObject_.Properties[19].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].referencePoint = 157;
    std::cout << "FusedFrontObject_.Properties[19].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].reserved = 282.70;
    std::cout << "FusedFrontObject_.Properties[19].reserved(float32): " << FusedFrontObject_.Properties[19].reserved << std::endl;
    FusedFrontObject_.Properties[19].rightNearLaneMarkingConfidence = 158;
    std::cout << "FusedFrontObject_.Properties[19].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].speedStdDev = 283.80;
    std::cout << "FusedFrontObject_.Properties[19].speedStdDev(float32): " << FusedFrontObject_.Properties[19].speedStdDev << std::endl;
    FusedFrontObject_.Properties[19].trackStatus = 159;
    std::cout << "FusedFrontObject_.Properties[19].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].trafficScenario = 160;
    std::cout << "FusedFrontObject_.Properties[19].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].turnIndicator = 161;
    std::cout << "FusedFrontObject_.Properties[19].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].type = 162;
    std::cout << "FusedFrontObject_.Properties[19].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].visionId = 163;
    std::cout << "FusedFrontObject_.Properties[19].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[19].width = 284.90;
    std::cout << "FusedFrontObject_.Properties[19].width(float32): " << FusedFrontObject_.Properties[19].width << std::endl;
    FusedFrontObject_.Properties[19].length = 286.00;
    std::cout << "FusedFrontObject_.Properties[19].length(float32): " << FusedFrontObject_.Properties[19].length << std::endl;
    FusedFrontObject_.Properties[19].SensorUpdateStatus = 164;
    std::cout << "FusedFrontObject_.Properties[19].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[19].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].accelerationStdDev = 287.10;
    std::cout << "FusedFrontObject_.Properties[20].accelerationStdDev(float32): " << FusedFrontObject_.Properties[20].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[20].brakeLight = 165;
    std::cout << "FusedFrontObject_.Properties[20].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].classificationConfidence = 288.20;
    std::cout << "FusedFrontObject_.Properties[20].classificationConfidence(float32): " << FusedFrontObject_.Properties[20].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[20].cmsConfidence = 166;
    std::cout << "FusedFrontObject_.Properties[20].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].cmbbSecConfidence = 167;
    std::cout << "FusedFrontObject_.Properties[20].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].distanceToLeftNearLaneMarking = 289.30;
    std::cout << "FusedFrontObject_.Properties[20].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[20].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[20].distanceToRightNearLaneMarking = 290.40;
    std::cout << "FusedFrontObject_.Properties[20].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[20].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[20].elkaQly = 168;
    std::cout << "FusedFrontObject_.Properties[20].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].existenceConfidence = 291.50;
    std::cout << "FusedFrontObject_.Properties[20].existenceConfidence(float32): " << FusedFrontObject_.Properties[20].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[20].fcwQly = 169;
    std::cout << "FusedFrontObject_.Properties[20].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].fusionSource = 170;
    std::cout << "FusedFrontObject_.Properties[20].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].hazardLightStatus = 171;
    std::cout << "FusedFrontObject_.Properties[20].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].headingStdDev = 292.60;
    std::cout << "FusedFrontObject_.Properties[20].headingStdDev(float32): " << FusedFrontObject_.Properties[20].headingStdDev << std::endl;
    FusedFrontObject_.Properties[20].id = 172;
    std::cout << "FusedFrontObject_.Properties[20].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].innovationFactor = 293.70;
    std::cout << "FusedFrontObject_.Properties[20].innovationFactor(float32): " << FusedFrontObject_.Properties[20].innovationFactor << std::endl;
    FusedFrontObject_.Properties[20].latPositionStdDev = 294.80;
    std::cout << "FusedFrontObject_.Properties[20].latPositionStdDev(float32): " << FusedFrontObject_.Properties[20].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[20].leftNearLaneMarkingConfidence = 173;
    std::cout << "FusedFrontObject_.Properties[20].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].longPositionStdDev = 295.90;
    std::cout << "FusedFrontObject_.Properties[20].longPositionStdDev(float32): " << FusedFrontObject_.Properties[20].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[20].motionHistory = 174;
    std::cout << "FusedFrontObject_.Properties[20].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].motionModel = 175;
    std::cout << "FusedFrontObject_.Properties[20].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].motionPattern = 176;
    std::cout << "FusedFrontObject_.Properties[20].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].radarId = 177;
    std::cout << "FusedFrontObject_.Properties[20].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].referencePoint = 178;
    std::cout << "FusedFrontObject_.Properties[20].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].reserved = 297.00;
    std::cout << "FusedFrontObject_.Properties[20].reserved(float32): " << FusedFrontObject_.Properties[20].reserved << std::endl;
    FusedFrontObject_.Properties[20].rightNearLaneMarkingConfidence = 179;
    std::cout << "FusedFrontObject_.Properties[20].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].speedStdDev = 298.10;
    std::cout << "FusedFrontObject_.Properties[20].speedStdDev(float32): " << FusedFrontObject_.Properties[20].speedStdDev << std::endl;
    FusedFrontObject_.Properties[20].trackStatus = 180;
    std::cout << "FusedFrontObject_.Properties[20].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].trafficScenario = 181;
    std::cout << "FusedFrontObject_.Properties[20].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].turnIndicator = 182;
    std::cout << "FusedFrontObject_.Properties[20].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].type = 183;
    std::cout << "FusedFrontObject_.Properties[20].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].visionId = 184;
    std::cout << "FusedFrontObject_.Properties[20].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[20].width = 299.20;
    std::cout << "FusedFrontObject_.Properties[20].width(float32): " << FusedFrontObject_.Properties[20].width << std::endl;
    FusedFrontObject_.Properties[20].length = 300.30;
    std::cout << "FusedFrontObject_.Properties[20].length(float32): " << FusedFrontObject_.Properties[20].length << std::endl;
    FusedFrontObject_.Properties[20].SensorUpdateStatus = 185;
    std::cout << "FusedFrontObject_.Properties[20].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[20].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].accelerationStdDev = 301.40;
    std::cout << "FusedFrontObject_.Properties[21].accelerationStdDev(float32): " << FusedFrontObject_.Properties[21].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[21].brakeLight = 186;
    std::cout << "FusedFrontObject_.Properties[21].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].classificationConfidence = 302.50;
    std::cout << "FusedFrontObject_.Properties[21].classificationConfidence(float32): " << FusedFrontObject_.Properties[21].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[21].cmsConfidence = 187;
    std::cout << "FusedFrontObject_.Properties[21].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].cmbbSecConfidence = 188;
    std::cout << "FusedFrontObject_.Properties[21].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].distanceToLeftNearLaneMarking = 303.60;
    std::cout << "FusedFrontObject_.Properties[21].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[21].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[21].distanceToRightNearLaneMarking = 304.70;
    std::cout << "FusedFrontObject_.Properties[21].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[21].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[21].elkaQly = 189;
    std::cout << "FusedFrontObject_.Properties[21].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].existenceConfidence = 305.80;
    std::cout << "FusedFrontObject_.Properties[21].existenceConfidence(float32): " << FusedFrontObject_.Properties[21].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[21].fcwQly = 190;
    std::cout << "FusedFrontObject_.Properties[21].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].fusionSource = 191;
    std::cout << "FusedFrontObject_.Properties[21].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].hazardLightStatus = 192;
    std::cout << "FusedFrontObject_.Properties[21].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].headingStdDev = 306.90;
    std::cout << "FusedFrontObject_.Properties[21].headingStdDev(float32): " << FusedFrontObject_.Properties[21].headingStdDev << std::endl;
    FusedFrontObject_.Properties[21].id = 193;
    std::cout << "FusedFrontObject_.Properties[21].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].innovationFactor = 308.00;
    std::cout << "FusedFrontObject_.Properties[21].innovationFactor(float32): " << FusedFrontObject_.Properties[21].innovationFactor << std::endl;
    FusedFrontObject_.Properties[21].latPositionStdDev = 309.10;
    std::cout << "FusedFrontObject_.Properties[21].latPositionStdDev(float32): " << FusedFrontObject_.Properties[21].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[21].leftNearLaneMarkingConfidence = 194;
    std::cout << "FusedFrontObject_.Properties[21].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].longPositionStdDev = 310.20;
    std::cout << "FusedFrontObject_.Properties[21].longPositionStdDev(float32): " << FusedFrontObject_.Properties[21].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[21].motionHistory = 195;
    std::cout << "FusedFrontObject_.Properties[21].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].motionModel = 196;
    std::cout << "FusedFrontObject_.Properties[21].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].motionPattern = 197;
    std::cout << "FusedFrontObject_.Properties[21].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].radarId = 198;
    std::cout << "FusedFrontObject_.Properties[21].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].referencePoint = 199;
    std::cout << "FusedFrontObject_.Properties[21].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].reserved = 311.30;
    std::cout << "FusedFrontObject_.Properties[21].reserved(float32): " << FusedFrontObject_.Properties[21].reserved << std::endl;
    FusedFrontObject_.Properties[21].rightNearLaneMarkingConfidence = 200;
    std::cout << "FusedFrontObject_.Properties[21].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].speedStdDev = 312.40;
    std::cout << "FusedFrontObject_.Properties[21].speedStdDev(float32): " << FusedFrontObject_.Properties[21].speedStdDev << std::endl;
    FusedFrontObject_.Properties[21].trackStatus = 201;
    std::cout << "FusedFrontObject_.Properties[21].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].trafficScenario = 202;
    std::cout << "FusedFrontObject_.Properties[21].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].turnIndicator = 203;
    std::cout << "FusedFrontObject_.Properties[21].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].type = 204;
    std::cout << "FusedFrontObject_.Properties[21].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].visionId = 205;
    std::cout << "FusedFrontObject_.Properties[21].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[21].width = 313.50;
    std::cout << "FusedFrontObject_.Properties[21].width(float32): " << FusedFrontObject_.Properties[21].width << std::endl;
    FusedFrontObject_.Properties[21].length = 314.60;
    std::cout << "FusedFrontObject_.Properties[21].length(float32): " << FusedFrontObject_.Properties[21].length << std::endl;
    FusedFrontObject_.Properties[21].SensorUpdateStatus = 206;
    std::cout << "FusedFrontObject_.Properties[21].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[21].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].accelerationStdDev = 315.70;
    std::cout << "FusedFrontObject_.Properties[22].accelerationStdDev(float32): " << FusedFrontObject_.Properties[22].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[22].brakeLight = 207;
    std::cout << "FusedFrontObject_.Properties[22].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].classificationConfidence = 316.80;
    std::cout << "FusedFrontObject_.Properties[22].classificationConfidence(float32): " << FusedFrontObject_.Properties[22].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[22].cmsConfidence = 208;
    std::cout << "FusedFrontObject_.Properties[22].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].cmbbSecConfidence = 209;
    std::cout << "FusedFrontObject_.Properties[22].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].distanceToLeftNearLaneMarking = 317.90;
    std::cout << "FusedFrontObject_.Properties[22].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[22].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[22].distanceToRightNearLaneMarking = 319.00;
    std::cout << "FusedFrontObject_.Properties[22].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[22].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[22].elkaQly = 210;
    std::cout << "FusedFrontObject_.Properties[22].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].existenceConfidence = 320.10;
    std::cout << "FusedFrontObject_.Properties[22].existenceConfidence(float32): " << FusedFrontObject_.Properties[22].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[22].fcwQly = 211;
    std::cout << "FusedFrontObject_.Properties[22].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].fusionSource = 212;
    std::cout << "FusedFrontObject_.Properties[22].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].hazardLightStatus = 213;
    std::cout << "FusedFrontObject_.Properties[22].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].headingStdDev = 321.20;
    std::cout << "FusedFrontObject_.Properties[22].headingStdDev(float32): " << FusedFrontObject_.Properties[22].headingStdDev << std::endl;
    FusedFrontObject_.Properties[22].id = 214;
    std::cout << "FusedFrontObject_.Properties[22].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].innovationFactor = 322.30;
    std::cout << "FusedFrontObject_.Properties[22].innovationFactor(float32): " << FusedFrontObject_.Properties[22].innovationFactor << std::endl;
    FusedFrontObject_.Properties[22].latPositionStdDev = 323.40;
    std::cout << "FusedFrontObject_.Properties[22].latPositionStdDev(float32): " << FusedFrontObject_.Properties[22].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[22].leftNearLaneMarkingConfidence = 215;
    std::cout << "FusedFrontObject_.Properties[22].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].longPositionStdDev = 324.50;
    std::cout << "FusedFrontObject_.Properties[22].longPositionStdDev(float32): " << FusedFrontObject_.Properties[22].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[22].motionHistory = 216;
    std::cout << "FusedFrontObject_.Properties[22].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].motionModel = 217;
    std::cout << "FusedFrontObject_.Properties[22].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].motionPattern = 218;
    std::cout << "FusedFrontObject_.Properties[22].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].radarId = 219;
    std::cout << "FusedFrontObject_.Properties[22].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].referencePoint = 220;
    std::cout << "FusedFrontObject_.Properties[22].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].reserved = 325.60;
    std::cout << "FusedFrontObject_.Properties[22].reserved(float32): " << FusedFrontObject_.Properties[22].reserved << std::endl;
    FusedFrontObject_.Properties[22].rightNearLaneMarkingConfidence = 221;
    std::cout << "FusedFrontObject_.Properties[22].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].speedStdDev = 326.70;
    std::cout << "FusedFrontObject_.Properties[22].speedStdDev(float32): " << FusedFrontObject_.Properties[22].speedStdDev << std::endl;
    FusedFrontObject_.Properties[22].trackStatus = 222;
    std::cout << "FusedFrontObject_.Properties[22].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].trafficScenario = 223;
    std::cout << "FusedFrontObject_.Properties[22].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].turnIndicator = 224;
    std::cout << "FusedFrontObject_.Properties[22].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].type = 225;
    std::cout << "FusedFrontObject_.Properties[22].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].visionId = 226;
    std::cout << "FusedFrontObject_.Properties[22].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[22].width = 327.80;
    std::cout << "FusedFrontObject_.Properties[22].width(float32): " << FusedFrontObject_.Properties[22].width << std::endl;
    FusedFrontObject_.Properties[22].length = 328.90;
    std::cout << "FusedFrontObject_.Properties[22].length(float32): " << FusedFrontObject_.Properties[22].length << std::endl;
    FusedFrontObject_.Properties[22].SensorUpdateStatus = 227;
    std::cout << "FusedFrontObject_.Properties[22].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[22].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].accelerationStdDev = 330.00;
    std::cout << "FusedFrontObject_.Properties[23].accelerationStdDev(float32): " << FusedFrontObject_.Properties[23].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[23].brakeLight = 228;
    std::cout << "FusedFrontObject_.Properties[23].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].classificationConfidence = 331.10;
    std::cout << "FusedFrontObject_.Properties[23].classificationConfidence(float32): " << FusedFrontObject_.Properties[23].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[23].cmsConfidence = 229;
    std::cout << "FusedFrontObject_.Properties[23].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].cmbbSecConfidence = 230;
    std::cout << "FusedFrontObject_.Properties[23].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].distanceToLeftNearLaneMarking = 332.20;
    std::cout << "FusedFrontObject_.Properties[23].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[23].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[23].distanceToRightNearLaneMarking = 333.30;
    std::cout << "FusedFrontObject_.Properties[23].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[23].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[23].elkaQly = 231;
    std::cout << "FusedFrontObject_.Properties[23].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].existenceConfidence = 334.40;
    std::cout << "FusedFrontObject_.Properties[23].existenceConfidence(float32): " << FusedFrontObject_.Properties[23].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[23].fcwQly = 232;
    std::cout << "FusedFrontObject_.Properties[23].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].fusionSource = 233;
    std::cout << "FusedFrontObject_.Properties[23].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].hazardLightStatus = 234;
    std::cout << "FusedFrontObject_.Properties[23].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].headingStdDev = 335.50;
    std::cout << "FusedFrontObject_.Properties[23].headingStdDev(float32): " << FusedFrontObject_.Properties[23].headingStdDev << std::endl;
    FusedFrontObject_.Properties[23].id = 235;
    std::cout << "FusedFrontObject_.Properties[23].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].innovationFactor = 336.60;
    std::cout << "FusedFrontObject_.Properties[23].innovationFactor(float32): " << FusedFrontObject_.Properties[23].innovationFactor << std::endl;
    FusedFrontObject_.Properties[23].latPositionStdDev = 337.70;
    std::cout << "FusedFrontObject_.Properties[23].latPositionStdDev(float32): " << FusedFrontObject_.Properties[23].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[23].leftNearLaneMarkingConfidence = 236;
    std::cout << "FusedFrontObject_.Properties[23].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].longPositionStdDev = 338.80;
    std::cout << "FusedFrontObject_.Properties[23].longPositionStdDev(float32): " << FusedFrontObject_.Properties[23].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[23].motionHistory = 237;
    std::cout << "FusedFrontObject_.Properties[23].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].motionModel = 238;
    std::cout << "FusedFrontObject_.Properties[23].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].motionPattern = 239;
    std::cout << "FusedFrontObject_.Properties[23].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].radarId = 240;
    std::cout << "FusedFrontObject_.Properties[23].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].referencePoint = 241;
    std::cout << "FusedFrontObject_.Properties[23].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].reserved = 339.90;
    std::cout << "FusedFrontObject_.Properties[23].reserved(float32): " << FusedFrontObject_.Properties[23].reserved << std::endl;
    FusedFrontObject_.Properties[23].rightNearLaneMarkingConfidence = 242;
    std::cout << "FusedFrontObject_.Properties[23].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].speedStdDev = 341.00;
    std::cout << "FusedFrontObject_.Properties[23].speedStdDev(float32): " << FusedFrontObject_.Properties[23].speedStdDev << std::endl;
    FusedFrontObject_.Properties[23].trackStatus = 243;
    std::cout << "FusedFrontObject_.Properties[23].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].trafficScenario = 244;
    std::cout << "FusedFrontObject_.Properties[23].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].turnIndicator = 245;
    std::cout << "FusedFrontObject_.Properties[23].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].type = 246;
    std::cout << "FusedFrontObject_.Properties[23].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].visionId = 247;
    std::cout << "FusedFrontObject_.Properties[23].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[23].width = 342.10;
    std::cout << "FusedFrontObject_.Properties[23].width(float32): " << FusedFrontObject_.Properties[23].width << std::endl;
    FusedFrontObject_.Properties[23].length = 343.20;
    std::cout << "FusedFrontObject_.Properties[23].length(float32): " << FusedFrontObject_.Properties[23].length << std::endl;
    FusedFrontObject_.Properties[23].SensorUpdateStatus = 248;
    std::cout << "FusedFrontObject_.Properties[23].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[23].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].accelerationStdDev = 344.30;
    std::cout << "FusedFrontObject_.Properties[24].accelerationStdDev(float32): " << FusedFrontObject_.Properties[24].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[24].brakeLight = 249;
    std::cout << "FusedFrontObject_.Properties[24].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].classificationConfidence = 345.40;
    std::cout << "FusedFrontObject_.Properties[24].classificationConfidence(float32): " << FusedFrontObject_.Properties[24].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[24].cmsConfidence = 250;
    std::cout << "FusedFrontObject_.Properties[24].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].cmbbSecConfidence = 251;
    std::cout << "FusedFrontObject_.Properties[24].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].distanceToLeftNearLaneMarking = 346.50;
    std::cout << "FusedFrontObject_.Properties[24].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[24].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[24].distanceToRightNearLaneMarking = 347.60;
    std::cout << "FusedFrontObject_.Properties[24].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[24].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[24].elkaQly = 252;
    std::cout << "FusedFrontObject_.Properties[24].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].existenceConfidence = 348.70;
    std::cout << "FusedFrontObject_.Properties[24].existenceConfidence(float32): " << FusedFrontObject_.Properties[24].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[24].fcwQly = 253;
    std::cout << "FusedFrontObject_.Properties[24].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].fusionSource = 254;
    std::cout << "FusedFrontObject_.Properties[24].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].hazardLightStatus = 255;
    std::cout << "FusedFrontObject_.Properties[24].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].headingStdDev = 349.80;
    std::cout << "FusedFrontObject_.Properties[24].headingStdDev(float32): " << FusedFrontObject_.Properties[24].headingStdDev << std::endl;
    FusedFrontObject_.Properties[24].id = 0;
    std::cout << "FusedFrontObject_.Properties[24].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].innovationFactor = 350.90;
    std::cout << "FusedFrontObject_.Properties[24].innovationFactor(float32): " << FusedFrontObject_.Properties[24].innovationFactor << std::endl;
    FusedFrontObject_.Properties[24].latPositionStdDev = 352.00;
    std::cout << "FusedFrontObject_.Properties[24].latPositionStdDev(float32): " << FusedFrontObject_.Properties[24].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[24].leftNearLaneMarkingConfidence = 1;
    std::cout << "FusedFrontObject_.Properties[24].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].longPositionStdDev = 353.10;
    std::cout << "FusedFrontObject_.Properties[24].longPositionStdDev(float32): " << FusedFrontObject_.Properties[24].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[24].motionHistory = 2;
    std::cout << "FusedFrontObject_.Properties[24].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].motionModel = 3;
    std::cout << "FusedFrontObject_.Properties[24].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].motionPattern = 4;
    std::cout << "FusedFrontObject_.Properties[24].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].radarId = 5;
    std::cout << "FusedFrontObject_.Properties[24].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].referencePoint = 6;
    std::cout << "FusedFrontObject_.Properties[24].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].reserved = 354.20;
    std::cout << "FusedFrontObject_.Properties[24].reserved(float32): " << FusedFrontObject_.Properties[24].reserved << std::endl;
    FusedFrontObject_.Properties[24].rightNearLaneMarkingConfidence = 7;
    std::cout << "FusedFrontObject_.Properties[24].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].speedStdDev = 355.30;
    std::cout << "FusedFrontObject_.Properties[24].speedStdDev(float32): " << FusedFrontObject_.Properties[24].speedStdDev << std::endl;
    FusedFrontObject_.Properties[24].trackStatus = 8;
    std::cout << "FusedFrontObject_.Properties[24].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].trafficScenario = 9;
    std::cout << "FusedFrontObject_.Properties[24].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].turnIndicator = 10;
    std::cout << "FusedFrontObject_.Properties[24].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].type = 11;
    std::cout << "FusedFrontObject_.Properties[24].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].visionId = 12;
    std::cout << "FusedFrontObject_.Properties[24].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[24].width = 356.40;
    std::cout << "FusedFrontObject_.Properties[24].width(float32): " << FusedFrontObject_.Properties[24].width << std::endl;
    FusedFrontObject_.Properties[24].length = 357.50;
    std::cout << "FusedFrontObject_.Properties[24].length(float32): " << FusedFrontObject_.Properties[24].length << std::endl;
    FusedFrontObject_.Properties[24].SensorUpdateStatus = 13;
    std::cout << "FusedFrontObject_.Properties[24].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[24].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].accelerationStdDev = 358.60;
    std::cout << "FusedFrontObject_.Properties[25].accelerationStdDev(float32): " << FusedFrontObject_.Properties[25].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[25].brakeLight = 14;
    std::cout << "FusedFrontObject_.Properties[25].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].classificationConfidence = 359.70;
    std::cout << "FusedFrontObject_.Properties[25].classificationConfidence(float32): " << FusedFrontObject_.Properties[25].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[25].cmsConfidence = 15;
    std::cout << "FusedFrontObject_.Properties[25].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].cmbbSecConfidence = 16;
    std::cout << "FusedFrontObject_.Properties[25].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].distanceToLeftNearLaneMarking = 360.80;
    std::cout << "FusedFrontObject_.Properties[25].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[25].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[25].distanceToRightNearLaneMarking = 361.90;
    std::cout << "FusedFrontObject_.Properties[25].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[25].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[25].elkaQly = 17;
    std::cout << "FusedFrontObject_.Properties[25].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].existenceConfidence = 363.00;
    std::cout << "FusedFrontObject_.Properties[25].existenceConfidence(float32): " << FusedFrontObject_.Properties[25].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[25].fcwQly = 18;
    std::cout << "FusedFrontObject_.Properties[25].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].fusionSource = 19;
    std::cout << "FusedFrontObject_.Properties[25].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].hazardLightStatus = 20;
    std::cout << "FusedFrontObject_.Properties[25].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].headingStdDev = 364.10;
    std::cout << "FusedFrontObject_.Properties[25].headingStdDev(float32): " << FusedFrontObject_.Properties[25].headingStdDev << std::endl;
    FusedFrontObject_.Properties[25].id = 21;
    std::cout << "FusedFrontObject_.Properties[25].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].innovationFactor = 365.20;
    std::cout << "FusedFrontObject_.Properties[25].innovationFactor(float32): " << FusedFrontObject_.Properties[25].innovationFactor << std::endl;
    FusedFrontObject_.Properties[25].latPositionStdDev = 366.30;
    std::cout << "FusedFrontObject_.Properties[25].latPositionStdDev(float32): " << FusedFrontObject_.Properties[25].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[25].leftNearLaneMarkingConfidence = 22;
    std::cout << "FusedFrontObject_.Properties[25].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].longPositionStdDev = 367.40;
    std::cout << "FusedFrontObject_.Properties[25].longPositionStdDev(float32): " << FusedFrontObject_.Properties[25].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[25].motionHistory = 23;
    std::cout << "FusedFrontObject_.Properties[25].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].motionModel = 24;
    std::cout << "FusedFrontObject_.Properties[25].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].motionPattern = 25;
    std::cout << "FusedFrontObject_.Properties[25].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].radarId = 26;
    std::cout << "FusedFrontObject_.Properties[25].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].referencePoint = 27;
    std::cout << "FusedFrontObject_.Properties[25].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].reserved = 368.50;
    std::cout << "FusedFrontObject_.Properties[25].reserved(float32): " << FusedFrontObject_.Properties[25].reserved << std::endl;
    FusedFrontObject_.Properties[25].rightNearLaneMarkingConfidence = 28;
    std::cout << "FusedFrontObject_.Properties[25].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].speedStdDev = 369.60;
    std::cout << "FusedFrontObject_.Properties[25].speedStdDev(float32): " << FusedFrontObject_.Properties[25].speedStdDev << std::endl;
    FusedFrontObject_.Properties[25].trackStatus = 29;
    std::cout << "FusedFrontObject_.Properties[25].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].trafficScenario = 30;
    std::cout << "FusedFrontObject_.Properties[25].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].turnIndicator = 31;
    std::cout << "FusedFrontObject_.Properties[25].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].type = 32;
    std::cout << "FusedFrontObject_.Properties[25].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].visionId = 33;
    std::cout << "FusedFrontObject_.Properties[25].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[25].width = 370.70;
    std::cout << "FusedFrontObject_.Properties[25].width(float32): " << FusedFrontObject_.Properties[25].width << std::endl;
    FusedFrontObject_.Properties[25].length = 371.80;
    std::cout << "FusedFrontObject_.Properties[25].length(float32): " << FusedFrontObject_.Properties[25].length << std::endl;
    FusedFrontObject_.Properties[25].SensorUpdateStatus = 34;
    std::cout << "FusedFrontObject_.Properties[25].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[25].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].accelerationStdDev = 372.90;
    std::cout << "FusedFrontObject_.Properties[26].accelerationStdDev(float32): " << FusedFrontObject_.Properties[26].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[26].brakeLight = 35;
    std::cout << "FusedFrontObject_.Properties[26].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].classificationConfidence = 374.00;
    std::cout << "FusedFrontObject_.Properties[26].classificationConfidence(float32): " << FusedFrontObject_.Properties[26].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[26].cmsConfidence = 36;
    std::cout << "FusedFrontObject_.Properties[26].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].cmbbSecConfidence = 37;
    std::cout << "FusedFrontObject_.Properties[26].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].distanceToLeftNearLaneMarking = 375.10;
    std::cout << "FusedFrontObject_.Properties[26].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[26].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[26].distanceToRightNearLaneMarking = 376.20;
    std::cout << "FusedFrontObject_.Properties[26].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[26].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[26].elkaQly = 38;
    std::cout << "FusedFrontObject_.Properties[26].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].existenceConfidence = 377.30;
    std::cout << "FusedFrontObject_.Properties[26].existenceConfidence(float32): " << FusedFrontObject_.Properties[26].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[26].fcwQly = 39;
    std::cout << "FusedFrontObject_.Properties[26].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].fusionSource = 40;
    std::cout << "FusedFrontObject_.Properties[26].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].hazardLightStatus = 41;
    std::cout << "FusedFrontObject_.Properties[26].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].headingStdDev = 378.40;
    std::cout << "FusedFrontObject_.Properties[26].headingStdDev(float32): " << FusedFrontObject_.Properties[26].headingStdDev << std::endl;
    FusedFrontObject_.Properties[26].id = 42;
    std::cout << "FusedFrontObject_.Properties[26].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].innovationFactor = 379.50;
    std::cout << "FusedFrontObject_.Properties[26].innovationFactor(float32): " << FusedFrontObject_.Properties[26].innovationFactor << std::endl;
    FusedFrontObject_.Properties[26].latPositionStdDev = 380.60;
    std::cout << "FusedFrontObject_.Properties[26].latPositionStdDev(float32): " << FusedFrontObject_.Properties[26].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[26].leftNearLaneMarkingConfidence = 43;
    std::cout << "FusedFrontObject_.Properties[26].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].longPositionStdDev = 381.70;
    std::cout << "FusedFrontObject_.Properties[26].longPositionStdDev(float32): " << FusedFrontObject_.Properties[26].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[26].motionHistory = 44;
    std::cout << "FusedFrontObject_.Properties[26].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].motionModel = 45;
    std::cout << "FusedFrontObject_.Properties[26].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].motionPattern = 46;
    std::cout << "FusedFrontObject_.Properties[26].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].radarId = 47;
    std::cout << "FusedFrontObject_.Properties[26].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].referencePoint = 48;
    std::cout << "FusedFrontObject_.Properties[26].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].reserved = 382.80;
    std::cout << "FusedFrontObject_.Properties[26].reserved(float32): " << FusedFrontObject_.Properties[26].reserved << std::endl;
    FusedFrontObject_.Properties[26].rightNearLaneMarkingConfidence = 49;
    std::cout << "FusedFrontObject_.Properties[26].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].speedStdDev = 383.90;
    std::cout << "FusedFrontObject_.Properties[26].speedStdDev(float32): " << FusedFrontObject_.Properties[26].speedStdDev << std::endl;
    FusedFrontObject_.Properties[26].trackStatus = 50;
    std::cout << "FusedFrontObject_.Properties[26].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].trafficScenario = 51;
    std::cout << "FusedFrontObject_.Properties[26].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].turnIndicator = 52;
    std::cout << "FusedFrontObject_.Properties[26].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].type = 53;
    std::cout << "FusedFrontObject_.Properties[26].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].visionId = 54;
    std::cout << "FusedFrontObject_.Properties[26].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[26].width = 385.00;
    std::cout << "FusedFrontObject_.Properties[26].width(float32): " << FusedFrontObject_.Properties[26].width << std::endl;
    FusedFrontObject_.Properties[26].length = 386.10;
    std::cout << "FusedFrontObject_.Properties[26].length(float32): " << FusedFrontObject_.Properties[26].length << std::endl;
    FusedFrontObject_.Properties[26].SensorUpdateStatus = 55;
    std::cout << "FusedFrontObject_.Properties[26].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[26].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].accelerationStdDev = 387.20;
    std::cout << "FusedFrontObject_.Properties[27].accelerationStdDev(float32): " << FusedFrontObject_.Properties[27].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[27].brakeLight = 56;
    std::cout << "FusedFrontObject_.Properties[27].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].classificationConfidence = 388.30;
    std::cout << "FusedFrontObject_.Properties[27].classificationConfidence(float32): " << FusedFrontObject_.Properties[27].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[27].cmsConfidence = 57;
    std::cout << "FusedFrontObject_.Properties[27].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].cmbbSecConfidence = 58;
    std::cout << "FusedFrontObject_.Properties[27].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].distanceToLeftNearLaneMarking = 389.40;
    std::cout << "FusedFrontObject_.Properties[27].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[27].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[27].distanceToRightNearLaneMarking = 390.50;
    std::cout << "FusedFrontObject_.Properties[27].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[27].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[27].elkaQly = 59;
    std::cout << "FusedFrontObject_.Properties[27].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].existenceConfidence = 391.60;
    std::cout << "FusedFrontObject_.Properties[27].existenceConfidence(float32): " << FusedFrontObject_.Properties[27].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[27].fcwQly = 60;
    std::cout << "FusedFrontObject_.Properties[27].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].fusionSource = 61;
    std::cout << "FusedFrontObject_.Properties[27].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].hazardLightStatus = 62;
    std::cout << "FusedFrontObject_.Properties[27].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].headingStdDev = 392.70;
    std::cout << "FusedFrontObject_.Properties[27].headingStdDev(float32): " << FusedFrontObject_.Properties[27].headingStdDev << std::endl;
    FusedFrontObject_.Properties[27].id = 63;
    std::cout << "FusedFrontObject_.Properties[27].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].innovationFactor = 393.80;
    std::cout << "FusedFrontObject_.Properties[27].innovationFactor(float32): " << FusedFrontObject_.Properties[27].innovationFactor << std::endl;
    FusedFrontObject_.Properties[27].latPositionStdDev = 394.90;
    std::cout << "FusedFrontObject_.Properties[27].latPositionStdDev(float32): " << FusedFrontObject_.Properties[27].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[27].leftNearLaneMarkingConfidence = 64;
    std::cout << "FusedFrontObject_.Properties[27].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].longPositionStdDev = 396.00;
    std::cout << "FusedFrontObject_.Properties[27].longPositionStdDev(float32): " << FusedFrontObject_.Properties[27].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[27].motionHistory = 65;
    std::cout << "FusedFrontObject_.Properties[27].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].motionModel = 66;
    std::cout << "FusedFrontObject_.Properties[27].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].motionPattern = 67;
    std::cout << "FusedFrontObject_.Properties[27].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].radarId = 68;
    std::cout << "FusedFrontObject_.Properties[27].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].referencePoint = 69;
    std::cout << "FusedFrontObject_.Properties[27].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].reserved = 397.10;
    std::cout << "FusedFrontObject_.Properties[27].reserved(float32): " << FusedFrontObject_.Properties[27].reserved << std::endl;
    FusedFrontObject_.Properties[27].rightNearLaneMarkingConfidence = 70;
    std::cout << "FusedFrontObject_.Properties[27].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].speedStdDev = 398.20;
    std::cout << "FusedFrontObject_.Properties[27].speedStdDev(float32): " << FusedFrontObject_.Properties[27].speedStdDev << std::endl;
    FusedFrontObject_.Properties[27].trackStatus = 71;
    std::cout << "FusedFrontObject_.Properties[27].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].trafficScenario = 72;
    std::cout << "FusedFrontObject_.Properties[27].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].turnIndicator = 73;
    std::cout << "FusedFrontObject_.Properties[27].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].type = 74;
    std::cout << "FusedFrontObject_.Properties[27].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].visionId = 75;
    std::cout << "FusedFrontObject_.Properties[27].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[27].width = 399.30;
    std::cout << "FusedFrontObject_.Properties[27].width(float32): " << FusedFrontObject_.Properties[27].width << std::endl;
    FusedFrontObject_.Properties[27].length = 400.40;
    std::cout << "FusedFrontObject_.Properties[27].length(float32): " << FusedFrontObject_.Properties[27].length << std::endl;
    FusedFrontObject_.Properties[27].SensorUpdateStatus = 76;
    std::cout << "FusedFrontObject_.Properties[27].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[27].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].accelerationStdDev = 401.50;
    std::cout << "FusedFrontObject_.Properties[28].accelerationStdDev(float32): " << FusedFrontObject_.Properties[28].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[28].brakeLight = 77;
    std::cout << "FusedFrontObject_.Properties[28].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].classificationConfidence = 402.60;
    std::cout << "FusedFrontObject_.Properties[28].classificationConfidence(float32): " << FusedFrontObject_.Properties[28].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[28].cmsConfidence = 78;
    std::cout << "FusedFrontObject_.Properties[28].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].cmbbSecConfidence = 79;
    std::cout << "FusedFrontObject_.Properties[28].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].distanceToLeftNearLaneMarking = 403.70;
    std::cout << "FusedFrontObject_.Properties[28].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[28].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[28].distanceToRightNearLaneMarking = 404.80;
    std::cout << "FusedFrontObject_.Properties[28].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[28].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[28].elkaQly = 80;
    std::cout << "FusedFrontObject_.Properties[28].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].existenceConfidence = 405.90;
    std::cout << "FusedFrontObject_.Properties[28].existenceConfidence(float32): " << FusedFrontObject_.Properties[28].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[28].fcwQly = 81;
    std::cout << "FusedFrontObject_.Properties[28].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].fusionSource = 82;
    std::cout << "FusedFrontObject_.Properties[28].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].hazardLightStatus = 83;
    std::cout << "FusedFrontObject_.Properties[28].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].headingStdDev = 407.00;
    std::cout << "FusedFrontObject_.Properties[28].headingStdDev(float32): " << FusedFrontObject_.Properties[28].headingStdDev << std::endl;
    FusedFrontObject_.Properties[28].id = 84;
    std::cout << "FusedFrontObject_.Properties[28].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].innovationFactor = 408.10;
    std::cout << "FusedFrontObject_.Properties[28].innovationFactor(float32): " << FusedFrontObject_.Properties[28].innovationFactor << std::endl;
    FusedFrontObject_.Properties[28].latPositionStdDev = 409.20;
    std::cout << "FusedFrontObject_.Properties[28].latPositionStdDev(float32): " << FusedFrontObject_.Properties[28].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[28].leftNearLaneMarkingConfidence = 85;
    std::cout << "FusedFrontObject_.Properties[28].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].longPositionStdDev = 410.30;
    std::cout << "FusedFrontObject_.Properties[28].longPositionStdDev(float32): " << FusedFrontObject_.Properties[28].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[28].motionHistory = 86;
    std::cout << "FusedFrontObject_.Properties[28].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].motionModel = 87;
    std::cout << "FusedFrontObject_.Properties[28].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].motionPattern = 88;
    std::cout << "FusedFrontObject_.Properties[28].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].radarId = 89;
    std::cout << "FusedFrontObject_.Properties[28].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].referencePoint = 90;
    std::cout << "FusedFrontObject_.Properties[28].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].reserved = 411.40;
    std::cout << "FusedFrontObject_.Properties[28].reserved(float32): " << FusedFrontObject_.Properties[28].reserved << std::endl;
    FusedFrontObject_.Properties[28].rightNearLaneMarkingConfidence = 91;
    std::cout << "FusedFrontObject_.Properties[28].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].speedStdDev = 412.50;
    std::cout << "FusedFrontObject_.Properties[28].speedStdDev(float32): " << FusedFrontObject_.Properties[28].speedStdDev << std::endl;
    FusedFrontObject_.Properties[28].trackStatus = 92;
    std::cout << "FusedFrontObject_.Properties[28].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].trafficScenario = 93;
    std::cout << "FusedFrontObject_.Properties[28].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].turnIndicator = 94;
    std::cout << "FusedFrontObject_.Properties[28].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].type = 95;
    std::cout << "FusedFrontObject_.Properties[28].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].visionId = 96;
    std::cout << "FusedFrontObject_.Properties[28].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[28].width = 413.60;
    std::cout << "FusedFrontObject_.Properties[28].width(float32): " << FusedFrontObject_.Properties[28].width << std::endl;
    FusedFrontObject_.Properties[28].length = 414.70;
    std::cout << "FusedFrontObject_.Properties[28].length(float32): " << FusedFrontObject_.Properties[28].length << std::endl;
    FusedFrontObject_.Properties[28].SensorUpdateStatus = 97;
    std::cout << "FusedFrontObject_.Properties[28].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[28].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].accelerationStdDev = 415.80;
    std::cout << "FusedFrontObject_.Properties[29].accelerationStdDev(float32): " << FusedFrontObject_.Properties[29].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[29].brakeLight = 98;
    std::cout << "FusedFrontObject_.Properties[29].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].classificationConfidence = 416.90;
    std::cout << "FusedFrontObject_.Properties[29].classificationConfidence(float32): " << FusedFrontObject_.Properties[29].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[29].cmsConfidence = 99;
    std::cout << "FusedFrontObject_.Properties[29].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].cmbbSecConfidence = 100;
    std::cout << "FusedFrontObject_.Properties[29].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].distanceToLeftNearLaneMarking = 418.00;
    std::cout << "FusedFrontObject_.Properties[29].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[29].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[29].distanceToRightNearLaneMarking = 419.10;
    std::cout << "FusedFrontObject_.Properties[29].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[29].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[29].elkaQly = 101;
    std::cout << "FusedFrontObject_.Properties[29].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].existenceConfidence = 420.20;
    std::cout << "FusedFrontObject_.Properties[29].existenceConfidence(float32): " << FusedFrontObject_.Properties[29].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[29].fcwQly = 102;
    std::cout << "FusedFrontObject_.Properties[29].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].fusionSource = 103;
    std::cout << "FusedFrontObject_.Properties[29].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].hazardLightStatus = 104;
    std::cout << "FusedFrontObject_.Properties[29].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].headingStdDev = 421.30;
    std::cout << "FusedFrontObject_.Properties[29].headingStdDev(float32): " << FusedFrontObject_.Properties[29].headingStdDev << std::endl;
    FusedFrontObject_.Properties[29].id = 105;
    std::cout << "FusedFrontObject_.Properties[29].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].innovationFactor = 422.40;
    std::cout << "FusedFrontObject_.Properties[29].innovationFactor(float32): " << FusedFrontObject_.Properties[29].innovationFactor << std::endl;
    FusedFrontObject_.Properties[29].latPositionStdDev = 423.50;
    std::cout << "FusedFrontObject_.Properties[29].latPositionStdDev(float32): " << FusedFrontObject_.Properties[29].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[29].leftNearLaneMarkingConfidence = 106;
    std::cout << "FusedFrontObject_.Properties[29].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].longPositionStdDev = 424.60;
    std::cout << "FusedFrontObject_.Properties[29].longPositionStdDev(float32): " << FusedFrontObject_.Properties[29].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[29].motionHistory = 107;
    std::cout << "FusedFrontObject_.Properties[29].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].motionModel = 108;
    std::cout << "FusedFrontObject_.Properties[29].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].motionPattern = 109;
    std::cout << "FusedFrontObject_.Properties[29].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].radarId = 110;
    std::cout << "FusedFrontObject_.Properties[29].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].referencePoint = 111;
    std::cout << "FusedFrontObject_.Properties[29].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].reserved = 425.70;
    std::cout << "FusedFrontObject_.Properties[29].reserved(float32): " << FusedFrontObject_.Properties[29].reserved << std::endl;
    FusedFrontObject_.Properties[29].rightNearLaneMarkingConfidence = 112;
    std::cout << "FusedFrontObject_.Properties[29].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].speedStdDev = 426.80;
    std::cout << "FusedFrontObject_.Properties[29].speedStdDev(float32): " << FusedFrontObject_.Properties[29].speedStdDev << std::endl;
    FusedFrontObject_.Properties[29].trackStatus = 113;
    std::cout << "FusedFrontObject_.Properties[29].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].trafficScenario = 114;
    std::cout << "FusedFrontObject_.Properties[29].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].turnIndicator = 115;
    std::cout << "FusedFrontObject_.Properties[29].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].type = 116;
    std::cout << "FusedFrontObject_.Properties[29].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].visionId = 117;
    std::cout << "FusedFrontObject_.Properties[29].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[29].width = 427.90;
    std::cout << "FusedFrontObject_.Properties[29].width(float32): " << FusedFrontObject_.Properties[29].width << std::endl;
    FusedFrontObject_.Properties[29].length = 429.00;
    std::cout << "FusedFrontObject_.Properties[29].length(float32): " << FusedFrontObject_.Properties[29].length << std::endl;
    FusedFrontObject_.Properties[29].SensorUpdateStatus = 118;
    std::cout << "FusedFrontObject_.Properties[29].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[29].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].accelerationStdDev = 430.10;
    std::cout << "FusedFrontObject_.Properties[30].accelerationStdDev(float32): " << FusedFrontObject_.Properties[30].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[30].brakeLight = 119;
    std::cout << "FusedFrontObject_.Properties[30].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].classificationConfidence = 431.20;
    std::cout << "FusedFrontObject_.Properties[30].classificationConfidence(float32): " << FusedFrontObject_.Properties[30].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[30].cmsConfidence = 120;
    std::cout << "FusedFrontObject_.Properties[30].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].cmbbSecConfidence = 121;
    std::cout << "FusedFrontObject_.Properties[30].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].distanceToLeftNearLaneMarking = 432.30;
    std::cout << "FusedFrontObject_.Properties[30].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[30].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[30].distanceToRightNearLaneMarking = 433.40;
    std::cout << "FusedFrontObject_.Properties[30].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[30].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[30].elkaQly = 122;
    std::cout << "FusedFrontObject_.Properties[30].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].existenceConfidence = 434.50;
    std::cout << "FusedFrontObject_.Properties[30].existenceConfidence(float32): " << FusedFrontObject_.Properties[30].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[30].fcwQly = 123;
    std::cout << "FusedFrontObject_.Properties[30].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].fusionSource = 124;
    std::cout << "FusedFrontObject_.Properties[30].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].hazardLightStatus = 125;
    std::cout << "FusedFrontObject_.Properties[30].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].headingStdDev = 435.60;
    std::cout << "FusedFrontObject_.Properties[30].headingStdDev(float32): " << FusedFrontObject_.Properties[30].headingStdDev << std::endl;
    FusedFrontObject_.Properties[30].id = 126;
    std::cout << "FusedFrontObject_.Properties[30].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].innovationFactor = 436.70;
    std::cout << "FusedFrontObject_.Properties[30].innovationFactor(float32): " << FusedFrontObject_.Properties[30].innovationFactor << std::endl;
    FusedFrontObject_.Properties[30].latPositionStdDev = 437.80;
    std::cout << "FusedFrontObject_.Properties[30].latPositionStdDev(float32): " << FusedFrontObject_.Properties[30].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[30].leftNearLaneMarkingConfidence = 127;
    std::cout << "FusedFrontObject_.Properties[30].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].longPositionStdDev = 438.90;
    std::cout << "FusedFrontObject_.Properties[30].longPositionStdDev(float32): " << FusedFrontObject_.Properties[30].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[30].motionHistory = 128;
    std::cout << "FusedFrontObject_.Properties[30].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].motionModel = 129;
    std::cout << "FusedFrontObject_.Properties[30].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].motionPattern = 130;
    std::cout << "FusedFrontObject_.Properties[30].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].radarId = 131;
    std::cout << "FusedFrontObject_.Properties[30].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].referencePoint = 132;
    std::cout << "FusedFrontObject_.Properties[30].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].reserved = 440.00;
    std::cout << "FusedFrontObject_.Properties[30].reserved(float32): " << FusedFrontObject_.Properties[30].reserved << std::endl;
    FusedFrontObject_.Properties[30].rightNearLaneMarkingConfidence = 133;
    std::cout << "FusedFrontObject_.Properties[30].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].speedStdDev = 441.10;
    std::cout << "FusedFrontObject_.Properties[30].speedStdDev(float32): " << FusedFrontObject_.Properties[30].speedStdDev << std::endl;
    FusedFrontObject_.Properties[30].trackStatus = 134;
    std::cout << "FusedFrontObject_.Properties[30].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].trafficScenario = 135;
    std::cout << "FusedFrontObject_.Properties[30].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].turnIndicator = 136;
    std::cout << "FusedFrontObject_.Properties[30].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].type = 137;
    std::cout << "FusedFrontObject_.Properties[30].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].visionId = 138;
    std::cout << "FusedFrontObject_.Properties[30].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[30].width = 442.20;
    std::cout << "FusedFrontObject_.Properties[30].width(float32): " << FusedFrontObject_.Properties[30].width << std::endl;
    FusedFrontObject_.Properties[30].length = 443.30;
    std::cout << "FusedFrontObject_.Properties[30].length(float32): " << FusedFrontObject_.Properties[30].length << std::endl;
    FusedFrontObject_.Properties[30].SensorUpdateStatus = 139;
    std::cout << "FusedFrontObject_.Properties[30].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[30].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].accelerationStdDev = 444.40;
    std::cout << "FusedFrontObject_.Properties[31].accelerationStdDev(float32): " << FusedFrontObject_.Properties[31].accelerationStdDev << std::endl;
    FusedFrontObject_.Properties[31].brakeLight = 140;
    std::cout << "FusedFrontObject_.Properties[31].brakeLight(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].brakeLight) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].classificationConfidence = 445.50;
    std::cout << "FusedFrontObject_.Properties[31].classificationConfidence(float32): " << FusedFrontObject_.Properties[31].classificationConfidence << std::endl;
    FusedFrontObject_.Properties[31].cmsConfidence = 141;
    std::cout << "FusedFrontObject_.Properties[31].cmsConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].cmsConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].cmbbSecConfidence = 142;
    std::cout << "FusedFrontObject_.Properties[31].cmbbSecConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].cmbbSecConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].distanceToLeftNearLaneMarking = 446.60;
    std::cout << "FusedFrontObject_.Properties[31].distanceToLeftNearLaneMarking(float32): " << FusedFrontObject_.Properties[31].distanceToLeftNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[31].distanceToRightNearLaneMarking = 447.70;
    std::cout << "FusedFrontObject_.Properties[31].distanceToRightNearLaneMarking(float32): " << FusedFrontObject_.Properties[31].distanceToRightNearLaneMarking << std::endl;
    FusedFrontObject_.Properties[31].elkaQly = 143;
    std::cout << "FusedFrontObject_.Properties[31].elkaQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].elkaQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].existenceConfidence = 448.80;
    std::cout << "FusedFrontObject_.Properties[31].existenceConfidence(float32): " << FusedFrontObject_.Properties[31].existenceConfidence << std::endl;
    FusedFrontObject_.Properties[31].fcwQly = 144;
    std::cout << "FusedFrontObject_.Properties[31].fcwQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].fcwQly) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].fusionSource = 145;
    std::cout << "FusedFrontObject_.Properties[31].fusionSource(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].fusionSource) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].hazardLightStatus = 146;
    std::cout << "FusedFrontObject_.Properties[31].hazardLightStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].hazardLightStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].headingStdDev = 449.90;
    std::cout << "FusedFrontObject_.Properties[31].headingStdDev(float32): " << FusedFrontObject_.Properties[31].headingStdDev << std::endl;
    FusedFrontObject_.Properties[31].id = 147;
    std::cout << "FusedFrontObject_.Properties[31].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].id) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].innovationFactor = 451.00;
    std::cout << "FusedFrontObject_.Properties[31].innovationFactor(float32): " << FusedFrontObject_.Properties[31].innovationFactor << std::endl;
    FusedFrontObject_.Properties[31].latPositionStdDev = 452.10;
    std::cout << "FusedFrontObject_.Properties[31].latPositionStdDev(float32): " << FusedFrontObject_.Properties[31].latPositionStdDev << std::endl;
    FusedFrontObject_.Properties[31].leftNearLaneMarkingConfidence = 148;
    std::cout << "FusedFrontObject_.Properties[31].leftNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].leftNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].longPositionStdDev = 453.20;
    std::cout << "FusedFrontObject_.Properties[31].longPositionStdDev(float32): " << FusedFrontObject_.Properties[31].longPositionStdDev << std::endl;
    FusedFrontObject_.Properties[31].motionHistory = 149;
    std::cout << "FusedFrontObject_.Properties[31].motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].motionHistory) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].motionModel = 150;
    std::cout << "FusedFrontObject_.Properties[31].motionModel(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].motionModel) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].motionPattern = 151;
    std::cout << "FusedFrontObject_.Properties[31].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].motionPattern) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].radarId = 152;
    std::cout << "FusedFrontObject_.Properties[31].radarId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].radarId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].referencePoint = 153;
    std::cout << "FusedFrontObject_.Properties[31].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].referencePoint) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].reserved = 454.30;
    std::cout << "FusedFrontObject_.Properties[31].reserved(float32): " << FusedFrontObject_.Properties[31].reserved << std::endl;
    FusedFrontObject_.Properties[31].rightNearLaneMarkingConfidence = 154;
    std::cout << "FusedFrontObject_.Properties[31].rightNearLaneMarkingConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].rightNearLaneMarkingConfidence) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].speedStdDev = 455.40;
    std::cout << "FusedFrontObject_.Properties[31].speedStdDev(float32): " << FusedFrontObject_.Properties[31].speedStdDev << std::endl;
    FusedFrontObject_.Properties[31].trackStatus = 155;
    std::cout << "FusedFrontObject_.Properties[31].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].trackStatus) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].trafficScenario = 156;
    std::cout << "FusedFrontObject_.Properties[31].trafficScenario(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].trafficScenario) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].turnIndicator = 157;
    std::cout << "FusedFrontObject_.Properties[31].turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].turnIndicator) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].type = 158;
    std::cout << "FusedFrontObject_.Properties[31].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].type) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].visionId = 159;
    std::cout << "FusedFrontObject_.Properties[31].visionId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].visionId) << std::dec  << std::endl;
    FusedFrontObject_.Properties[31].width = 456.50;
    std::cout << "FusedFrontObject_.Properties[31].width(float32): " << FusedFrontObject_.Properties[31].width << std::endl;
    FusedFrontObject_.Properties[31].length = 457.60;
    std::cout << "FusedFrontObject_.Properties[31].length(float32): " << FusedFrontObject_.Properties[31].length << std::endl;
    FusedFrontObject_.Properties[31].SensorUpdateStatus = 160;
    std::cout << "FusedFrontObject_.Properties[31].SensorUpdateStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Properties[31].SensorUpdateStatus) << std::dec  << std::endl;
    FusedFrontObject_.SequenceID = 1;
    std::cout << "FusedFrontObject_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(FusedFrontObject_.SequenceID) << std::dec  << std::endl;
    FusedFrontObject_.States[0].acceleration = 458.70;
    std::cout << "FusedFrontObject_.States[0].acceleration(float32): " << FusedFrontObject_.States[0].acceleration << std::endl;
    FusedFrontObject_.States[0].curvature = 459.80;
    std::cout << "FusedFrontObject_.States[0].curvature(float32): " << FusedFrontObject_.States[0].curvature << std::endl;
    FusedFrontObject_.States[0].heading = 460.90;
    std::cout << "FusedFrontObject_.States[0].heading(float32): " << FusedFrontObject_.States[0].heading << std::endl;
    FusedFrontObject_.States[0].latAcceleration = 462.00;
    std::cout << "FusedFrontObject_.States[0].latAcceleration(float32): " << FusedFrontObject_.States[0].latAcceleration << std::endl;
    FusedFrontObject_.States[0].latPosition = 463.10;
    std::cout << "FusedFrontObject_.States[0].latPosition(float32): " << FusedFrontObject_.States[0].latPosition << std::endl;
    FusedFrontObject_.States[0].latVelocity = 464.20;
    std::cout << "FusedFrontObject_.States[0].latVelocity(float32): " << FusedFrontObject_.States[0].latVelocity << std::endl;
    FusedFrontObject_.States[0].longAcceleration = 465.30;
    std::cout << "FusedFrontObject_.States[0].longAcceleration(float32): " << FusedFrontObject_.States[0].longAcceleration << std::endl;
    FusedFrontObject_.States[0].longPosition = 466.40;
    std::cout << "FusedFrontObject_.States[0].longPosition(float32): " << FusedFrontObject_.States[0].longPosition << std::endl;
    FusedFrontObject_.States[0].longVelocity = 467.50;
    std::cout << "FusedFrontObject_.States[0].longVelocity(float32): " << FusedFrontObject_.States[0].longVelocity << std::endl;
    FusedFrontObject_.States[0].speed = 468.60;
    std::cout << "FusedFrontObject_.States[0].speed(float32): " << FusedFrontObject_.States[0].speed << std::endl;
    FusedFrontObject_.States[1].acceleration = 469.70;
    std::cout << "FusedFrontObject_.States[1].acceleration(float32): " << FusedFrontObject_.States[1].acceleration << std::endl;
    FusedFrontObject_.States[1].curvature = 470.80;
    std::cout << "FusedFrontObject_.States[1].curvature(float32): " << FusedFrontObject_.States[1].curvature << std::endl;
    FusedFrontObject_.States[1].heading = 471.90;
    std::cout << "FusedFrontObject_.States[1].heading(float32): " << FusedFrontObject_.States[1].heading << std::endl;
    FusedFrontObject_.States[1].latAcceleration = 473.00;
    std::cout << "FusedFrontObject_.States[1].latAcceleration(float32): " << FusedFrontObject_.States[1].latAcceleration << std::endl;
    FusedFrontObject_.States[1].latPosition = 474.10;
    std::cout << "FusedFrontObject_.States[1].latPosition(float32): " << FusedFrontObject_.States[1].latPosition << std::endl;
    FusedFrontObject_.States[1].latVelocity = 475.20;
    std::cout << "FusedFrontObject_.States[1].latVelocity(float32): " << FusedFrontObject_.States[1].latVelocity << std::endl;
    FusedFrontObject_.States[1].longAcceleration = 476.30;
    std::cout << "FusedFrontObject_.States[1].longAcceleration(float32): " << FusedFrontObject_.States[1].longAcceleration << std::endl;
    FusedFrontObject_.States[1].longPosition = 477.40;
    std::cout << "FusedFrontObject_.States[1].longPosition(float32): " << FusedFrontObject_.States[1].longPosition << std::endl;
    FusedFrontObject_.States[1].longVelocity = 478.50;
    std::cout << "FusedFrontObject_.States[1].longVelocity(float32): " << FusedFrontObject_.States[1].longVelocity << std::endl;
    FusedFrontObject_.States[1].speed = 479.60;
    std::cout << "FusedFrontObject_.States[1].speed(float32): " << FusedFrontObject_.States[1].speed << std::endl;
    FusedFrontObject_.States[2].acceleration = 480.70;
    std::cout << "FusedFrontObject_.States[2].acceleration(float32): " << FusedFrontObject_.States[2].acceleration << std::endl;
    FusedFrontObject_.States[2].curvature = 481.80;
    std::cout << "FusedFrontObject_.States[2].curvature(float32): " << FusedFrontObject_.States[2].curvature << std::endl;
    FusedFrontObject_.States[2].heading = 482.90;
    std::cout << "FusedFrontObject_.States[2].heading(float32): " << FusedFrontObject_.States[2].heading << std::endl;
    FusedFrontObject_.States[2].latAcceleration = 484.00;
    std::cout << "FusedFrontObject_.States[2].latAcceleration(float32): " << FusedFrontObject_.States[2].latAcceleration << std::endl;
    FusedFrontObject_.States[2].latPosition = 485.10;
    std::cout << "FusedFrontObject_.States[2].latPosition(float32): " << FusedFrontObject_.States[2].latPosition << std::endl;
    FusedFrontObject_.States[2].latVelocity = 486.20;
    std::cout << "FusedFrontObject_.States[2].latVelocity(float32): " << FusedFrontObject_.States[2].latVelocity << std::endl;
    FusedFrontObject_.States[2].longAcceleration = 487.30;
    std::cout << "FusedFrontObject_.States[2].longAcceleration(float32): " << FusedFrontObject_.States[2].longAcceleration << std::endl;
    FusedFrontObject_.States[2].longPosition = 488.40;
    std::cout << "FusedFrontObject_.States[2].longPosition(float32): " << FusedFrontObject_.States[2].longPosition << std::endl;
    FusedFrontObject_.States[2].longVelocity = 489.50;
    std::cout << "FusedFrontObject_.States[2].longVelocity(float32): " << FusedFrontObject_.States[2].longVelocity << std::endl;
    FusedFrontObject_.States[2].speed = 490.60;
    std::cout << "FusedFrontObject_.States[2].speed(float32): " << FusedFrontObject_.States[2].speed << std::endl;
    FusedFrontObject_.States[3].acceleration = 491.70;
    std::cout << "FusedFrontObject_.States[3].acceleration(float32): " << FusedFrontObject_.States[3].acceleration << std::endl;
    FusedFrontObject_.States[3].curvature = 492.80;
    std::cout << "FusedFrontObject_.States[3].curvature(float32): " << FusedFrontObject_.States[3].curvature << std::endl;
    FusedFrontObject_.States[3].heading = 493.90;
    std::cout << "FusedFrontObject_.States[3].heading(float32): " << FusedFrontObject_.States[3].heading << std::endl;
    FusedFrontObject_.States[3].latAcceleration = 495.00;
    std::cout << "FusedFrontObject_.States[3].latAcceleration(float32): " << FusedFrontObject_.States[3].latAcceleration << std::endl;
    FusedFrontObject_.States[3].latPosition = 496.10;
    std::cout << "FusedFrontObject_.States[3].latPosition(float32): " << FusedFrontObject_.States[3].latPosition << std::endl;
    FusedFrontObject_.States[3].latVelocity = 497.20;
    std::cout << "FusedFrontObject_.States[3].latVelocity(float32): " << FusedFrontObject_.States[3].latVelocity << std::endl;
    FusedFrontObject_.States[3].longAcceleration = 498.30;
    std::cout << "FusedFrontObject_.States[3].longAcceleration(float32): " << FusedFrontObject_.States[3].longAcceleration << std::endl;
    FusedFrontObject_.States[3].longPosition = 499.40;
    std::cout << "FusedFrontObject_.States[3].longPosition(float32): " << FusedFrontObject_.States[3].longPosition << std::endl;
    FusedFrontObject_.States[3].longVelocity = 500.50;
    std::cout << "FusedFrontObject_.States[3].longVelocity(float32): " << FusedFrontObject_.States[3].longVelocity << std::endl;
    FusedFrontObject_.States[3].speed = 501.60;
    std::cout << "FusedFrontObject_.States[3].speed(float32): " << FusedFrontObject_.States[3].speed << std::endl;
    FusedFrontObject_.States[4].acceleration = 502.70;
    std::cout << "FusedFrontObject_.States[4].acceleration(float32): " << FusedFrontObject_.States[4].acceleration << std::endl;
    FusedFrontObject_.States[4].curvature = 503.80;
    std::cout << "FusedFrontObject_.States[4].curvature(float32): " << FusedFrontObject_.States[4].curvature << std::endl;
    FusedFrontObject_.States[4].heading = 504.90;
    std::cout << "FusedFrontObject_.States[4].heading(float32): " << FusedFrontObject_.States[4].heading << std::endl;
    FusedFrontObject_.States[4].latAcceleration = 506.00;
    std::cout << "FusedFrontObject_.States[4].latAcceleration(float32): " << FusedFrontObject_.States[4].latAcceleration << std::endl;
    FusedFrontObject_.States[4].latPosition = 507.10;
    std::cout << "FusedFrontObject_.States[4].latPosition(float32): " << FusedFrontObject_.States[4].latPosition << std::endl;
    FusedFrontObject_.States[4].latVelocity = 508.20;
    std::cout << "FusedFrontObject_.States[4].latVelocity(float32): " << FusedFrontObject_.States[4].latVelocity << std::endl;
    FusedFrontObject_.States[4].longAcceleration = 509.30;
    std::cout << "FusedFrontObject_.States[4].longAcceleration(float32): " << FusedFrontObject_.States[4].longAcceleration << std::endl;
    FusedFrontObject_.States[4].longPosition = 510.40;
    std::cout << "FusedFrontObject_.States[4].longPosition(float32): " << FusedFrontObject_.States[4].longPosition << std::endl;
    FusedFrontObject_.States[4].longVelocity = 511.50;
    std::cout << "FusedFrontObject_.States[4].longVelocity(float32): " << FusedFrontObject_.States[4].longVelocity << std::endl;
    FusedFrontObject_.States[4].speed = 512.60;
    std::cout << "FusedFrontObject_.States[4].speed(float32): " << FusedFrontObject_.States[4].speed << std::endl;
    FusedFrontObject_.States[5].acceleration = 513.70;
    std::cout << "FusedFrontObject_.States[5].acceleration(float32): " << FusedFrontObject_.States[5].acceleration << std::endl;
    FusedFrontObject_.States[5].curvature = 514.80;
    std::cout << "FusedFrontObject_.States[5].curvature(float32): " << FusedFrontObject_.States[5].curvature << std::endl;
    FusedFrontObject_.States[5].heading = 515.90;
    std::cout << "FusedFrontObject_.States[5].heading(float32): " << FusedFrontObject_.States[5].heading << std::endl;
    FusedFrontObject_.States[5].latAcceleration = 517.00;
    std::cout << "FusedFrontObject_.States[5].latAcceleration(float32): " << FusedFrontObject_.States[5].latAcceleration << std::endl;
    FusedFrontObject_.States[5].latPosition = 518.10;
    std::cout << "FusedFrontObject_.States[5].latPosition(float32): " << FusedFrontObject_.States[5].latPosition << std::endl;
    FusedFrontObject_.States[5].latVelocity = 519.20;
    std::cout << "FusedFrontObject_.States[5].latVelocity(float32): " << FusedFrontObject_.States[5].latVelocity << std::endl;
    FusedFrontObject_.States[5].longAcceleration = 520.30;
    std::cout << "FusedFrontObject_.States[5].longAcceleration(float32): " << FusedFrontObject_.States[5].longAcceleration << std::endl;
    FusedFrontObject_.States[5].longPosition = 521.40;
    std::cout << "FusedFrontObject_.States[5].longPosition(float32): " << FusedFrontObject_.States[5].longPosition << std::endl;
    FusedFrontObject_.States[5].longVelocity = 522.50;
    std::cout << "FusedFrontObject_.States[5].longVelocity(float32): " << FusedFrontObject_.States[5].longVelocity << std::endl;
    FusedFrontObject_.States[5].speed = 523.60;
    std::cout << "FusedFrontObject_.States[5].speed(float32): " << FusedFrontObject_.States[5].speed << std::endl;
    FusedFrontObject_.States[6].acceleration = 524.70;
    std::cout << "FusedFrontObject_.States[6].acceleration(float32): " << FusedFrontObject_.States[6].acceleration << std::endl;
    FusedFrontObject_.States[6].curvature = 525.80;
    std::cout << "FusedFrontObject_.States[6].curvature(float32): " << FusedFrontObject_.States[6].curvature << std::endl;
    FusedFrontObject_.States[6].heading = 526.90;
    std::cout << "FusedFrontObject_.States[6].heading(float32): " << FusedFrontObject_.States[6].heading << std::endl;
    FusedFrontObject_.States[6].latAcceleration = 528.00;
    std::cout << "FusedFrontObject_.States[6].latAcceleration(float32): " << FusedFrontObject_.States[6].latAcceleration << std::endl;
    FusedFrontObject_.States[6].latPosition = 529.10;
    std::cout << "FusedFrontObject_.States[6].latPosition(float32): " << FusedFrontObject_.States[6].latPosition << std::endl;
    FusedFrontObject_.States[6].latVelocity = 530.20;
    std::cout << "FusedFrontObject_.States[6].latVelocity(float32): " << FusedFrontObject_.States[6].latVelocity << std::endl;
    FusedFrontObject_.States[6].longAcceleration = 531.30;
    std::cout << "FusedFrontObject_.States[6].longAcceleration(float32): " << FusedFrontObject_.States[6].longAcceleration << std::endl;
    FusedFrontObject_.States[6].longPosition = 532.40;
    std::cout << "FusedFrontObject_.States[6].longPosition(float32): " << FusedFrontObject_.States[6].longPosition << std::endl;
    FusedFrontObject_.States[6].longVelocity = 533.50;
    std::cout << "FusedFrontObject_.States[6].longVelocity(float32): " << FusedFrontObject_.States[6].longVelocity << std::endl;
    FusedFrontObject_.States[6].speed = 534.60;
    std::cout << "FusedFrontObject_.States[6].speed(float32): " << FusedFrontObject_.States[6].speed << std::endl;
    FusedFrontObject_.States[7].acceleration = 535.70;
    std::cout << "FusedFrontObject_.States[7].acceleration(float32): " << FusedFrontObject_.States[7].acceleration << std::endl;
    FusedFrontObject_.States[7].curvature = 536.80;
    std::cout << "FusedFrontObject_.States[7].curvature(float32): " << FusedFrontObject_.States[7].curvature << std::endl;
    FusedFrontObject_.States[7].heading = 537.90;
    std::cout << "FusedFrontObject_.States[7].heading(float32): " << FusedFrontObject_.States[7].heading << std::endl;
    FusedFrontObject_.States[7].latAcceleration = 539.00;
    std::cout << "FusedFrontObject_.States[7].latAcceleration(float32): " << FusedFrontObject_.States[7].latAcceleration << std::endl;
    FusedFrontObject_.States[7].latPosition = 540.10;
    std::cout << "FusedFrontObject_.States[7].latPosition(float32): " << FusedFrontObject_.States[7].latPosition << std::endl;
    FusedFrontObject_.States[7].latVelocity = 541.20;
    std::cout << "FusedFrontObject_.States[7].latVelocity(float32): " << FusedFrontObject_.States[7].latVelocity << std::endl;
    FusedFrontObject_.States[7].longAcceleration = 542.30;
    std::cout << "FusedFrontObject_.States[7].longAcceleration(float32): " << FusedFrontObject_.States[7].longAcceleration << std::endl;
    FusedFrontObject_.States[7].longPosition = 543.40;
    std::cout << "FusedFrontObject_.States[7].longPosition(float32): " << FusedFrontObject_.States[7].longPosition << std::endl;
    FusedFrontObject_.States[7].longVelocity = 544.50;
    std::cout << "FusedFrontObject_.States[7].longVelocity(float32): " << FusedFrontObject_.States[7].longVelocity << std::endl;
    FusedFrontObject_.States[7].speed = 545.60;
    std::cout << "FusedFrontObject_.States[7].speed(float32): " << FusedFrontObject_.States[7].speed << std::endl;
    FusedFrontObject_.States[8].acceleration = 546.70;
    std::cout << "FusedFrontObject_.States[8].acceleration(float32): " << FusedFrontObject_.States[8].acceleration << std::endl;
    FusedFrontObject_.States[8].curvature = 547.80;
    std::cout << "FusedFrontObject_.States[8].curvature(float32): " << FusedFrontObject_.States[8].curvature << std::endl;
    FusedFrontObject_.States[8].heading = 548.90;
    std::cout << "FusedFrontObject_.States[8].heading(float32): " << FusedFrontObject_.States[8].heading << std::endl;
    FusedFrontObject_.States[8].latAcceleration = 550.00;
    std::cout << "FusedFrontObject_.States[8].latAcceleration(float32): " << FusedFrontObject_.States[8].latAcceleration << std::endl;
    FusedFrontObject_.States[8].latPosition = 551.10;
    std::cout << "FusedFrontObject_.States[8].latPosition(float32): " << FusedFrontObject_.States[8].latPosition << std::endl;
    FusedFrontObject_.States[8].latVelocity = 552.20;
    std::cout << "FusedFrontObject_.States[8].latVelocity(float32): " << FusedFrontObject_.States[8].latVelocity << std::endl;
    FusedFrontObject_.States[8].longAcceleration = 553.30;
    std::cout << "FusedFrontObject_.States[8].longAcceleration(float32): " << FusedFrontObject_.States[8].longAcceleration << std::endl;
    FusedFrontObject_.States[8].longPosition = 554.40;
    std::cout << "FusedFrontObject_.States[8].longPosition(float32): " << FusedFrontObject_.States[8].longPosition << std::endl;
    FusedFrontObject_.States[8].longVelocity = 555.50;
    std::cout << "FusedFrontObject_.States[8].longVelocity(float32): " << FusedFrontObject_.States[8].longVelocity << std::endl;
    FusedFrontObject_.States[8].speed = 556.60;
    std::cout << "FusedFrontObject_.States[8].speed(float32): " << FusedFrontObject_.States[8].speed << std::endl;
    FusedFrontObject_.States[9].acceleration = 557.70;
    std::cout << "FusedFrontObject_.States[9].acceleration(float32): " << FusedFrontObject_.States[9].acceleration << std::endl;
    FusedFrontObject_.States[9].curvature = 558.80;
    std::cout << "FusedFrontObject_.States[9].curvature(float32): " << FusedFrontObject_.States[9].curvature << std::endl;
    FusedFrontObject_.States[9].heading = 559.90;
    std::cout << "FusedFrontObject_.States[9].heading(float32): " << FusedFrontObject_.States[9].heading << std::endl;
    FusedFrontObject_.States[9].latAcceleration = 561.00;
    std::cout << "FusedFrontObject_.States[9].latAcceleration(float32): " << FusedFrontObject_.States[9].latAcceleration << std::endl;
    FusedFrontObject_.States[9].latPosition = 562.10;
    std::cout << "FusedFrontObject_.States[9].latPosition(float32): " << FusedFrontObject_.States[9].latPosition << std::endl;
    FusedFrontObject_.States[9].latVelocity = 563.20;
    std::cout << "FusedFrontObject_.States[9].latVelocity(float32): " << FusedFrontObject_.States[9].latVelocity << std::endl;
    FusedFrontObject_.States[9].longAcceleration = 564.30;
    std::cout << "FusedFrontObject_.States[9].longAcceleration(float32): " << FusedFrontObject_.States[9].longAcceleration << std::endl;
    FusedFrontObject_.States[9].longPosition = 565.40;
    std::cout << "FusedFrontObject_.States[9].longPosition(float32): " << FusedFrontObject_.States[9].longPosition << std::endl;
    FusedFrontObject_.States[9].longVelocity = 566.50;
    std::cout << "FusedFrontObject_.States[9].longVelocity(float32): " << FusedFrontObject_.States[9].longVelocity << std::endl;
    FusedFrontObject_.States[9].speed = 567.60;
    std::cout << "FusedFrontObject_.States[9].speed(float32): " << FusedFrontObject_.States[9].speed << std::endl;
    FusedFrontObject_.States[10].acceleration = 568.70;
    std::cout << "FusedFrontObject_.States[10].acceleration(float32): " << FusedFrontObject_.States[10].acceleration << std::endl;
    FusedFrontObject_.States[10].curvature = 569.80;
    std::cout << "FusedFrontObject_.States[10].curvature(float32): " << FusedFrontObject_.States[10].curvature << std::endl;
    FusedFrontObject_.States[10].heading = 570.90;
    std::cout << "FusedFrontObject_.States[10].heading(float32): " << FusedFrontObject_.States[10].heading << std::endl;
    FusedFrontObject_.States[10].latAcceleration = 572.00;
    std::cout << "FusedFrontObject_.States[10].latAcceleration(float32): " << FusedFrontObject_.States[10].latAcceleration << std::endl;
    FusedFrontObject_.States[10].latPosition = 573.10;
    std::cout << "FusedFrontObject_.States[10].latPosition(float32): " << FusedFrontObject_.States[10].latPosition << std::endl;
    FusedFrontObject_.States[10].latVelocity = 574.20;
    std::cout << "FusedFrontObject_.States[10].latVelocity(float32): " << FusedFrontObject_.States[10].latVelocity << std::endl;
    FusedFrontObject_.States[10].longAcceleration = 575.30;
    std::cout << "FusedFrontObject_.States[10].longAcceleration(float32): " << FusedFrontObject_.States[10].longAcceleration << std::endl;
    FusedFrontObject_.States[10].longPosition = 576.40;
    std::cout << "FusedFrontObject_.States[10].longPosition(float32): " << FusedFrontObject_.States[10].longPosition << std::endl;
    FusedFrontObject_.States[10].longVelocity = 577.50;
    std::cout << "FusedFrontObject_.States[10].longVelocity(float32): " << FusedFrontObject_.States[10].longVelocity << std::endl;
    FusedFrontObject_.States[10].speed = 578.60;
    std::cout << "FusedFrontObject_.States[10].speed(float32): " << FusedFrontObject_.States[10].speed << std::endl;
    FusedFrontObject_.States[11].acceleration = 579.70;
    std::cout << "FusedFrontObject_.States[11].acceleration(float32): " << FusedFrontObject_.States[11].acceleration << std::endl;
    FusedFrontObject_.States[11].curvature = 580.80;
    std::cout << "FusedFrontObject_.States[11].curvature(float32): " << FusedFrontObject_.States[11].curvature << std::endl;
    FusedFrontObject_.States[11].heading = 581.90;
    std::cout << "FusedFrontObject_.States[11].heading(float32): " << FusedFrontObject_.States[11].heading << std::endl;
    FusedFrontObject_.States[11].latAcceleration = 583.00;
    std::cout << "FusedFrontObject_.States[11].latAcceleration(float32): " << FusedFrontObject_.States[11].latAcceleration << std::endl;
    FusedFrontObject_.States[11].latPosition = 584.10;
    std::cout << "FusedFrontObject_.States[11].latPosition(float32): " << FusedFrontObject_.States[11].latPosition << std::endl;
    FusedFrontObject_.States[11].latVelocity = 585.20;
    std::cout << "FusedFrontObject_.States[11].latVelocity(float32): " << FusedFrontObject_.States[11].latVelocity << std::endl;
    FusedFrontObject_.States[11].longAcceleration = 586.30;
    std::cout << "FusedFrontObject_.States[11].longAcceleration(float32): " << FusedFrontObject_.States[11].longAcceleration << std::endl;
    FusedFrontObject_.States[11].longPosition = 587.40;
    std::cout << "FusedFrontObject_.States[11].longPosition(float32): " << FusedFrontObject_.States[11].longPosition << std::endl;
    FusedFrontObject_.States[11].longVelocity = 588.50;
    std::cout << "FusedFrontObject_.States[11].longVelocity(float32): " << FusedFrontObject_.States[11].longVelocity << std::endl;
    FusedFrontObject_.States[11].speed = 589.60;
    std::cout << "FusedFrontObject_.States[11].speed(float32): " << FusedFrontObject_.States[11].speed << std::endl;
    FusedFrontObject_.States[12].acceleration = 590.70;
    std::cout << "FusedFrontObject_.States[12].acceleration(float32): " << FusedFrontObject_.States[12].acceleration << std::endl;
    FusedFrontObject_.States[12].curvature = 591.80;
    std::cout << "FusedFrontObject_.States[12].curvature(float32): " << FusedFrontObject_.States[12].curvature << std::endl;
    FusedFrontObject_.States[12].heading = 592.90;
    std::cout << "FusedFrontObject_.States[12].heading(float32): " << FusedFrontObject_.States[12].heading << std::endl;
    FusedFrontObject_.States[12].latAcceleration = 594.00;
    std::cout << "FusedFrontObject_.States[12].latAcceleration(float32): " << FusedFrontObject_.States[12].latAcceleration << std::endl;
    FusedFrontObject_.States[12].latPosition = 595.10;
    std::cout << "FusedFrontObject_.States[12].latPosition(float32): " << FusedFrontObject_.States[12].latPosition << std::endl;
    FusedFrontObject_.States[12].latVelocity = 596.20;
    std::cout << "FusedFrontObject_.States[12].latVelocity(float32): " << FusedFrontObject_.States[12].latVelocity << std::endl;
    FusedFrontObject_.States[12].longAcceleration = 597.30;
    std::cout << "FusedFrontObject_.States[12].longAcceleration(float32): " << FusedFrontObject_.States[12].longAcceleration << std::endl;
    FusedFrontObject_.States[12].longPosition = 598.40;
    std::cout << "FusedFrontObject_.States[12].longPosition(float32): " << FusedFrontObject_.States[12].longPosition << std::endl;
    FusedFrontObject_.States[12].longVelocity = 599.50;
    std::cout << "FusedFrontObject_.States[12].longVelocity(float32): " << FusedFrontObject_.States[12].longVelocity << std::endl;
    FusedFrontObject_.States[12].speed = 600.60;
    std::cout << "FusedFrontObject_.States[12].speed(float32): " << FusedFrontObject_.States[12].speed << std::endl;
    FusedFrontObject_.States[13].acceleration = 601.70;
    std::cout << "FusedFrontObject_.States[13].acceleration(float32): " << FusedFrontObject_.States[13].acceleration << std::endl;
    FusedFrontObject_.States[13].curvature = 602.80;
    std::cout << "FusedFrontObject_.States[13].curvature(float32): " << FusedFrontObject_.States[13].curvature << std::endl;
    FusedFrontObject_.States[13].heading = 603.90;
    std::cout << "FusedFrontObject_.States[13].heading(float32): " << FusedFrontObject_.States[13].heading << std::endl;
    FusedFrontObject_.States[13].latAcceleration = 605.00;
    std::cout << "FusedFrontObject_.States[13].latAcceleration(float32): " << FusedFrontObject_.States[13].latAcceleration << std::endl;
    FusedFrontObject_.States[13].latPosition = 606.10;
    std::cout << "FusedFrontObject_.States[13].latPosition(float32): " << FusedFrontObject_.States[13].latPosition << std::endl;
    FusedFrontObject_.States[13].latVelocity = 607.20;
    std::cout << "FusedFrontObject_.States[13].latVelocity(float32): " << FusedFrontObject_.States[13].latVelocity << std::endl;
    FusedFrontObject_.States[13].longAcceleration = 608.30;
    std::cout << "FusedFrontObject_.States[13].longAcceleration(float32): " << FusedFrontObject_.States[13].longAcceleration << std::endl;
    FusedFrontObject_.States[13].longPosition = 609.40;
    std::cout << "FusedFrontObject_.States[13].longPosition(float32): " << FusedFrontObject_.States[13].longPosition << std::endl;
    FusedFrontObject_.States[13].longVelocity = 610.50;
    std::cout << "FusedFrontObject_.States[13].longVelocity(float32): " << FusedFrontObject_.States[13].longVelocity << std::endl;
    FusedFrontObject_.States[13].speed = 611.60;
    std::cout << "FusedFrontObject_.States[13].speed(float32): " << FusedFrontObject_.States[13].speed << std::endl;
    FusedFrontObject_.States[14].acceleration = 612.70;
    std::cout << "FusedFrontObject_.States[14].acceleration(float32): " << FusedFrontObject_.States[14].acceleration << std::endl;
    FusedFrontObject_.States[14].curvature = 613.80;
    std::cout << "FusedFrontObject_.States[14].curvature(float32): " << FusedFrontObject_.States[14].curvature << std::endl;
    FusedFrontObject_.States[14].heading = 614.90;
    std::cout << "FusedFrontObject_.States[14].heading(float32): " << FusedFrontObject_.States[14].heading << std::endl;
    FusedFrontObject_.States[14].latAcceleration = 616.00;
    std::cout << "FusedFrontObject_.States[14].latAcceleration(float32): " << FusedFrontObject_.States[14].latAcceleration << std::endl;
    FusedFrontObject_.States[14].latPosition = 617.10;
    std::cout << "FusedFrontObject_.States[14].latPosition(float32): " << FusedFrontObject_.States[14].latPosition << std::endl;
    FusedFrontObject_.States[14].latVelocity = 618.20;
    std::cout << "FusedFrontObject_.States[14].latVelocity(float32): " << FusedFrontObject_.States[14].latVelocity << std::endl;
    FusedFrontObject_.States[14].longAcceleration = 619.30;
    std::cout << "FusedFrontObject_.States[14].longAcceleration(float32): " << FusedFrontObject_.States[14].longAcceleration << std::endl;
    FusedFrontObject_.States[14].longPosition = 620.40;
    std::cout << "FusedFrontObject_.States[14].longPosition(float32): " << FusedFrontObject_.States[14].longPosition << std::endl;
    FusedFrontObject_.States[14].longVelocity = 621.50;
    std::cout << "FusedFrontObject_.States[14].longVelocity(float32): " << FusedFrontObject_.States[14].longVelocity << std::endl;
    FusedFrontObject_.States[14].speed = 622.60;
    std::cout << "FusedFrontObject_.States[14].speed(float32): " << FusedFrontObject_.States[14].speed << std::endl;
    FusedFrontObject_.States[15].acceleration = 623.70;
    std::cout << "FusedFrontObject_.States[15].acceleration(float32): " << FusedFrontObject_.States[15].acceleration << std::endl;
    FusedFrontObject_.States[15].curvature = 624.80;
    std::cout << "FusedFrontObject_.States[15].curvature(float32): " << FusedFrontObject_.States[15].curvature << std::endl;
    FusedFrontObject_.States[15].heading = 625.90;
    std::cout << "FusedFrontObject_.States[15].heading(float32): " << FusedFrontObject_.States[15].heading << std::endl;
    FusedFrontObject_.States[15].latAcceleration = 627.00;
    std::cout << "FusedFrontObject_.States[15].latAcceleration(float32): " << FusedFrontObject_.States[15].latAcceleration << std::endl;
    FusedFrontObject_.States[15].latPosition = 628.10;
    std::cout << "FusedFrontObject_.States[15].latPosition(float32): " << FusedFrontObject_.States[15].latPosition << std::endl;
    FusedFrontObject_.States[15].latVelocity = 629.20;
    std::cout << "FusedFrontObject_.States[15].latVelocity(float32): " << FusedFrontObject_.States[15].latVelocity << std::endl;
    FusedFrontObject_.States[15].longAcceleration = 630.30;
    std::cout << "FusedFrontObject_.States[15].longAcceleration(float32): " << FusedFrontObject_.States[15].longAcceleration << std::endl;
    FusedFrontObject_.States[15].longPosition = 631.40;
    std::cout << "FusedFrontObject_.States[15].longPosition(float32): " << FusedFrontObject_.States[15].longPosition << std::endl;
    FusedFrontObject_.States[15].longVelocity = 632.50;
    std::cout << "FusedFrontObject_.States[15].longVelocity(float32): " << FusedFrontObject_.States[15].longVelocity << std::endl;
    FusedFrontObject_.States[15].speed = 633.60;
    std::cout << "FusedFrontObject_.States[15].speed(float32): " << FusedFrontObject_.States[15].speed << std::endl;
    FusedFrontObject_.States[16].acceleration = 634.70;
    std::cout << "FusedFrontObject_.States[16].acceleration(float32): " << FusedFrontObject_.States[16].acceleration << std::endl;
    FusedFrontObject_.States[16].curvature = 635.80;
    std::cout << "FusedFrontObject_.States[16].curvature(float32): " << FusedFrontObject_.States[16].curvature << std::endl;
    FusedFrontObject_.States[16].heading = 636.90;
    std::cout << "FusedFrontObject_.States[16].heading(float32): " << FusedFrontObject_.States[16].heading << std::endl;
    FusedFrontObject_.States[16].latAcceleration = 638.00;
    std::cout << "FusedFrontObject_.States[16].latAcceleration(float32): " << FusedFrontObject_.States[16].latAcceleration << std::endl;
    FusedFrontObject_.States[16].latPosition = 639.10;
    std::cout << "FusedFrontObject_.States[16].latPosition(float32): " << FusedFrontObject_.States[16].latPosition << std::endl;
    FusedFrontObject_.States[16].latVelocity = 640.20;
    std::cout << "FusedFrontObject_.States[16].latVelocity(float32): " << FusedFrontObject_.States[16].latVelocity << std::endl;
    FusedFrontObject_.States[16].longAcceleration = 641.30;
    std::cout << "FusedFrontObject_.States[16].longAcceleration(float32): " << FusedFrontObject_.States[16].longAcceleration << std::endl;
    FusedFrontObject_.States[16].longPosition = 642.40;
    std::cout << "FusedFrontObject_.States[16].longPosition(float32): " << FusedFrontObject_.States[16].longPosition << std::endl;
    FusedFrontObject_.States[16].longVelocity = 643.50;
    std::cout << "FusedFrontObject_.States[16].longVelocity(float32): " << FusedFrontObject_.States[16].longVelocity << std::endl;
    FusedFrontObject_.States[16].speed = 644.60;
    std::cout << "FusedFrontObject_.States[16].speed(float32): " << FusedFrontObject_.States[16].speed << std::endl;
    FusedFrontObject_.States[17].acceleration = 645.70;
    std::cout << "FusedFrontObject_.States[17].acceleration(float32): " << FusedFrontObject_.States[17].acceleration << std::endl;
    FusedFrontObject_.States[17].curvature = 646.80;
    std::cout << "FusedFrontObject_.States[17].curvature(float32): " << FusedFrontObject_.States[17].curvature << std::endl;
    FusedFrontObject_.States[17].heading = 647.90;
    std::cout << "FusedFrontObject_.States[17].heading(float32): " << FusedFrontObject_.States[17].heading << std::endl;
    FusedFrontObject_.States[17].latAcceleration = 649.00;
    std::cout << "FusedFrontObject_.States[17].latAcceleration(float32): " << FusedFrontObject_.States[17].latAcceleration << std::endl;
    FusedFrontObject_.States[17].latPosition = 650.10;
    std::cout << "FusedFrontObject_.States[17].latPosition(float32): " << FusedFrontObject_.States[17].latPosition << std::endl;
    FusedFrontObject_.States[17].latVelocity = 651.20;
    std::cout << "FusedFrontObject_.States[17].latVelocity(float32): " << FusedFrontObject_.States[17].latVelocity << std::endl;
    FusedFrontObject_.States[17].longAcceleration = 652.30;
    std::cout << "FusedFrontObject_.States[17].longAcceleration(float32): " << FusedFrontObject_.States[17].longAcceleration << std::endl;
    FusedFrontObject_.States[17].longPosition = 653.40;
    std::cout << "FusedFrontObject_.States[17].longPosition(float32): " << FusedFrontObject_.States[17].longPosition << std::endl;
    FusedFrontObject_.States[17].longVelocity = 654.50;
    std::cout << "FusedFrontObject_.States[17].longVelocity(float32): " << FusedFrontObject_.States[17].longVelocity << std::endl;
    FusedFrontObject_.States[17].speed = 655.60;
    std::cout << "FusedFrontObject_.States[17].speed(float32): " << FusedFrontObject_.States[17].speed << std::endl;
    FusedFrontObject_.States[18].acceleration = 656.70;
    std::cout << "FusedFrontObject_.States[18].acceleration(float32): " << FusedFrontObject_.States[18].acceleration << std::endl;
    FusedFrontObject_.States[18].curvature = 657.80;
    std::cout << "FusedFrontObject_.States[18].curvature(float32): " << FusedFrontObject_.States[18].curvature << std::endl;
    FusedFrontObject_.States[18].heading = 658.90;
    std::cout << "FusedFrontObject_.States[18].heading(float32): " << FusedFrontObject_.States[18].heading << std::endl;
    FusedFrontObject_.States[18].latAcceleration = 660.00;
    std::cout << "FusedFrontObject_.States[18].latAcceleration(float32): " << FusedFrontObject_.States[18].latAcceleration << std::endl;
    FusedFrontObject_.States[18].latPosition = 661.10;
    std::cout << "FusedFrontObject_.States[18].latPosition(float32): " << FusedFrontObject_.States[18].latPosition << std::endl;
    FusedFrontObject_.States[18].latVelocity = 662.20;
    std::cout << "FusedFrontObject_.States[18].latVelocity(float32): " << FusedFrontObject_.States[18].latVelocity << std::endl;
    FusedFrontObject_.States[18].longAcceleration = 663.30;
    std::cout << "FusedFrontObject_.States[18].longAcceleration(float32): " << FusedFrontObject_.States[18].longAcceleration << std::endl;
    FusedFrontObject_.States[18].longPosition = 664.40;
    std::cout << "FusedFrontObject_.States[18].longPosition(float32): " << FusedFrontObject_.States[18].longPosition << std::endl;
    FusedFrontObject_.States[18].longVelocity = 665.50;
    std::cout << "FusedFrontObject_.States[18].longVelocity(float32): " << FusedFrontObject_.States[18].longVelocity << std::endl;
    FusedFrontObject_.States[18].speed = 666.60;
    std::cout << "FusedFrontObject_.States[18].speed(float32): " << FusedFrontObject_.States[18].speed << std::endl;
    FusedFrontObject_.States[19].acceleration = 667.70;
    std::cout << "FusedFrontObject_.States[19].acceleration(float32): " << FusedFrontObject_.States[19].acceleration << std::endl;
    FusedFrontObject_.States[19].curvature = 668.80;
    std::cout << "FusedFrontObject_.States[19].curvature(float32): " << FusedFrontObject_.States[19].curvature << std::endl;
    FusedFrontObject_.States[19].heading = 669.90;
    std::cout << "FusedFrontObject_.States[19].heading(float32): " << FusedFrontObject_.States[19].heading << std::endl;
    FusedFrontObject_.States[19].latAcceleration = 671.00;
    std::cout << "FusedFrontObject_.States[19].latAcceleration(float32): " << FusedFrontObject_.States[19].latAcceleration << std::endl;
    FusedFrontObject_.States[19].latPosition = 672.10;
    std::cout << "FusedFrontObject_.States[19].latPosition(float32): " << FusedFrontObject_.States[19].latPosition << std::endl;
    FusedFrontObject_.States[19].latVelocity = 673.20;
    std::cout << "FusedFrontObject_.States[19].latVelocity(float32): " << FusedFrontObject_.States[19].latVelocity << std::endl;
    FusedFrontObject_.States[19].longAcceleration = 674.30;
    std::cout << "FusedFrontObject_.States[19].longAcceleration(float32): " << FusedFrontObject_.States[19].longAcceleration << std::endl;
    FusedFrontObject_.States[19].longPosition = 675.40;
    std::cout << "FusedFrontObject_.States[19].longPosition(float32): " << FusedFrontObject_.States[19].longPosition << std::endl;
    FusedFrontObject_.States[19].longVelocity = 676.50;
    std::cout << "FusedFrontObject_.States[19].longVelocity(float32): " << FusedFrontObject_.States[19].longVelocity << std::endl;
    FusedFrontObject_.States[19].speed = 677.60;
    std::cout << "FusedFrontObject_.States[19].speed(float32): " << FusedFrontObject_.States[19].speed << std::endl;
    FusedFrontObject_.States[20].acceleration = 678.70;
    std::cout << "FusedFrontObject_.States[20].acceleration(float32): " << FusedFrontObject_.States[20].acceleration << std::endl;
    FusedFrontObject_.States[20].curvature = 679.80;
    std::cout << "FusedFrontObject_.States[20].curvature(float32): " << FusedFrontObject_.States[20].curvature << std::endl;
    FusedFrontObject_.States[20].heading = 680.90;
    std::cout << "FusedFrontObject_.States[20].heading(float32): " << FusedFrontObject_.States[20].heading << std::endl;
    FusedFrontObject_.States[20].latAcceleration = 682.00;
    std::cout << "FusedFrontObject_.States[20].latAcceleration(float32): " << FusedFrontObject_.States[20].latAcceleration << std::endl;
    FusedFrontObject_.States[20].latPosition = 683.10;
    std::cout << "FusedFrontObject_.States[20].latPosition(float32): " << FusedFrontObject_.States[20].latPosition << std::endl;
    FusedFrontObject_.States[20].latVelocity = 684.20;
    std::cout << "FusedFrontObject_.States[20].latVelocity(float32): " << FusedFrontObject_.States[20].latVelocity << std::endl;
    FusedFrontObject_.States[20].longAcceleration = 685.30;
    std::cout << "FusedFrontObject_.States[20].longAcceleration(float32): " << FusedFrontObject_.States[20].longAcceleration << std::endl;
    FusedFrontObject_.States[20].longPosition = 686.40;
    std::cout << "FusedFrontObject_.States[20].longPosition(float32): " << FusedFrontObject_.States[20].longPosition << std::endl;
    FusedFrontObject_.States[20].longVelocity = 687.50;
    std::cout << "FusedFrontObject_.States[20].longVelocity(float32): " << FusedFrontObject_.States[20].longVelocity << std::endl;
    FusedFrontObject_.States[20].speed = 688.60;
    std::cout << "FusedFrontObject_.States[20].speed(float32): " << FusedFrontObject_.States[20].speed << std::endl;
    FusedFrontObject_.States[21].acceleration = 689.70;
    std::cout << "FusedFrontObject_.States[21].acceleration(float32): " << FusedFrontObject_.States[21].acceleration << std::endl;
    FusedFrontObject_.States[21].curvature = 690.80;
    std::cout << "FusedFrontObject_.States[21].curvature(float32): " << FusedFrontObject_.States[21].curvature << std::endl;
    FusedFrontObject_.States[21].heading = 691.90;
    std::cout << "FusedFrontObject_.States[21].heading(float32): " << FusedFrontObject_.States[21].heading << std::endl;
    FusedFrontObject_.States[21].latAcceleration = 693.00;
    std::cout << "FusedFrontObject_.States[21].latAcceleration(float32): " << FusedFrontObject_.States[21].latAcceleration << std::endl;
    FusedFrontObject_.States[21].latPosition = 694.10;
    std::cout << "FusedFrontObject_.States[21].latPosition(float32): " << FusedFrontObject_.States[21].latPosition << std::endl;
    FusedFrontObject_.States[21].latVelocity = 695.20;
    std::cout << "FusedFrontObject_.States[21].latVelocity(float32): " << FusedFrontObject_.States[21].latVelocity << std::endl;
    FusedFrontObject_.States[21].longAcceleration = 696.30;
    std::cout << "FusedFrontObject_.States[21].longAcceleration(float32): " << FusedFrontObject_.States[21].longAcceleration << std::endl;
    FusedFrontObject_.States[21].longPosition = 697.40;
    std::cout << "FusedFrontObject_.States[21].longPosition(float32): " << FusedFrontObject_.States[21].longPosition << std::endl;
    FusedFrontObject_.States[21].longVelocity = 698.50;
    std::cout << "FusedFrontObject_.States[21].longVelocity(float32): " << FusedFrontObject_.States[21].longVelocity << std::endl;
    FusedFrontObject_.States[21].speed = 699.60;
    std::cout << "FusedFrontObject_.States[21].speed(float32): " << FusedFrontObject_.States[21].speed << std::endl;
    FusedFrontObject_.States[22].acceleration = 700.70;
    std::cout << "FusedFrontObject_.States[22].acceleration(float32): " << FusedFrontObject_.States[22].acceleration << std::endl;
    FusedFrontObject_.States[22].curvature = 701.80;
    std::cout << "FusedFrontObject_.States[22].curvature(float32): " << FusedFrontObject_.States[22].curvature << std::endl;
    FusedFrontObject_.States[22].heading = 702.90;
    std::cout << "FusedFrontObject_.States[22].heading(float32): " << FusedFrontObject_.States[22].heading << std::endl;
    FusedFrontObject_.States[22].latAcceleration = 704.00;
    std::cout << "FusedFrontObject_.States[22].latAcceleration(float32): " << FusedFrontObject_.States[22].latAcceleration << std::endl;
    FusedFrontObject_.States[22].latPosition = 705.10;
    std::cout << "FusedFrontObject_.States[22].latPosition(float32): " << FusedFrontObject_.States[22].latPosition << std::endl;
    FusedFrontObject_.States[22].latVelocity = 706.20;
    std::cout << "FusedFrontObject_.States[22].latVelocity(float32): " << FusedFrontObject_.States[22].latVelocity << std::endl;
    FusedFrontObject_.States[22].longAcceleration = 707.30;
    std::cout << "FusedFrontObject_.States[22].longAcceleration(float32): " << FusedFrontObject_.States[22].longAcceleration << std::endl;
    FusedFrontObject_.States[22].longPosition = 708.40;
    std::cout << "FusedFrontObject_.States[22].longPosition(float32): " << FusedFrontObject_.States[22].longPosition << std::endl;
    FusedFrontObject_.States[22].longVelocity = 709.50;
    std::cout << "FusedFrontObject_.States[22].longVelocity(float32): " << FusedFrontObject_.States[22].longVelocity << std::endl;
    FusedFrontObject_.States[22].speed = 710.60;
    std::cout << "FusedFrontObject_.States[22].speed(float32): " << FusedFrontObject_.States[22].speed << std::endl;
    FusedFrontObject_.States[23].acceleration = 711.70;
    std::cout << "FusedFrontObject_.States[23].acceleration(float32): " << FusedFrontObject_.States[23].acceleration << std::endl;
    FusedFrontObject_.States[23].curvature = 712.80;
    std::cout << "FusedFrontObject_.States[23].curvature(float32): " << FusedFrontObject_.States[23].curvature << std::endl;
    FusedFrontObject_.States[23].heading = 713.90;
    std::cout << "FusedFrontObject_.States[23].heading(float32): " << FusedFrontObject_.States[23].heading << std::endl;
    FusedFrontObject_.States[23].latAcceleration = 715.00;
    std::cout << "FusedFrontObject_.States[23].latAcceleration(float32): " << FusedFrontObject_.States[23].latAcceleration << std::endl;
    FusedFrontObject_.States[23].latPosition = 716.10;
    std::cout << "FusedFrontObject_.States[23].latPosition(float32): " << FusedFrontObject_.States[23].latPosition << std::endl;
    FusedFrontObject_.States[23].latVelocity = 717.20;
    std::cout << "FusedFrontObject_.States[23].latVelocity(float32): " << FusedFrontObject_.States[23].latVelocity << std::endl;
    FusedFrontObject_.States[23].longAcceleration = 718.30;
    std::cout << "FusedFrontObject_.States[23].longAcceleration(float32): " << FusedFrontObject_.States[23].longAcceleration << std::endl;
    FusedFrontObject_.States[23].longPosition = 719.40;
    std::cout << "FusedFrontObject_.States[23].longPosition(float32): " << FusedFrontObject_.States[23].longPosition << std::endl;
    FusedFrontObject_.States[23].longVelocity = 720.50;
    std::cout << "FusedFrontObject_.States[23].longVelocity(float32): " << FusedFrontObject_.States[23].longVelocity << std::endl;
    FusedFrontObject_.States[23].speed = 721.60;
    std::cout << "FusedFrontObject_.States[23].speed(float32): " << FusedFrontObject_.States[23].speed << std::endl;
    FusedFrontObject_.States[24].acceleration = 722.70;
    std::cout << "FusedFrontObject_.States[24].acceleration(float32): " << FusedFrontObject_.States[24].acceleration << std::endl;
    FusedFrontObject_.States[24].curvature = 723.80;
    std::cout << "FusedFrontObject_.States[24].curvature(float32): " << FusedFrontObject_.States[24].curvature << std::endl;
    FusedFrontObject_.States[24].heading = 724.90;
    std::cout << "FusedFrontObject_.States[24].heading(float32): " << FusedFrontObject_.States[24].heading << std::endl;
    FusedFrontObject_.States[24].latAcceleration = 726.00;
    std::cout << "FusedFrontObject_.States[24].latAcceleration(float32): " << FusedFrontObject_.States[24].latAcceleration << std::endl;
    FusedFrontObject_.States[24].latPosition = 727.10;
    std::cout << "FusedFrontObject_.States[24].latPosition(float32): " << FusedFrontObject_.States[24].latPosition << std::endl;
    FusedFrontObject_.States[24].latVelocity = 728.20;
    std::cout << "FusedFrontObject_.States[24].latVelocity(float32): " << FusedFrontObject_.States[24].latVelocity << std::endl;
    FusedFrontObject_.States[24].longAcceleration = 729.30;
    std::cout << "FusedFrontObject_.States[24].longAcceleration(float32): " << FusedFrontObject_.States[24].longAcceleration << std::endl;
    FusedFrontObject_.States[24].longPosition = 730.40;
    std::cout << "FusedFrontObject_.States[24].longPosition(float32): " << FusedFrontObject_.States[24].longPosition << std::endl;
    FusedFrontObject_.States[24].longVelocity = 731.50;
    std::cout << "FusedFrontObject_.States[24].longVelocity(float32): " << FusedFrontObject_.States[24].longVelocity << std::endl;
    FusedFrontObject_.States[24].speed = 732.60;
    std::cout << "FusedFrontObject_.States[24].speed(float32): " << FusedFrontObject_.States[24].speed << std::endl;
    FusedFrontObject_.States[25].acceleration = 733.70;
    std::cout << "FusedFrontObject_.States[25].acceleration(float32): " << FusedFrontObject_.States[25].acceleration << std::endl;
    FusedFrontObject_.States[25].curvature = 734.80;
    std::cout << "FusedFrontObject_.States[25].curvature(float32): " << FusedFrontObject_.States[25].curvature << std::endl;
    FusedFrontObject_.States[25].heading = 735.90;
    std::cout << "FusedFrontObject_.States[25].heading(float32): " << FusedFrontObject_.States[25].heading << std::endl;
    FusedFrontObject_.States[25].latAcceleration = 737.00;
    std::cout << "FusedFrontObject_.States[25].latAcceleration(float32): " << FusedFrontObject_.States[25].latAcceleration << std::endl;
    FusedFrontObject_.States[25].latPosition = 738.10;
    std::cout << "FusedFrontObject_.States[25].latPosition(float32): " << FusedFrontObject_.States[25].latPosition << std::endl;
    FusedFrontObject_.States[25].latVelocity = 739.20;
    std::cout << "FusedFrontObject_.States[25].latVelocity(float32): " << FusedFrontObject_.States[25].latVelocity << std::endl;
    FusedFrontObject_.States[25].longAcceleration = 740.30;
    std::cout << "FusedFrontObject_.States[25].longAcceleration(float32): " << FusedFrontObject_.States[25].longAcceleration << std::endl;
    FusedFrontObject_.States[25].longPosition = 741.40;
    std::cout << "FusedFrontObject_.States[25].longPosition(float32): " << FusedFrontObject_.States[25].longPosition << std::endl;
    FusedFrontObject_.States[25].longVelocity = 742.50;
    std::cout << "FusedFrontObject_.States[25].longVelocity(float32): " << FusedFrontObject_.States[25].longVelocity << std::endl;
    FusedFrontObject_.States[25].speed = 743.60;
    std::cout << "FusedFrontObject_.States[25].speed(float32): " << FusedFrontObject_.States[25].speed << std::endl;
    FusedFrontObject_.States[26].acceleration = 744.70;
    std::cout << "FusedFrontObject_.States[26].acceleration(float32): " << FusedFrontObject_.States[26].acceleration << std::endl;
    FusedFrontObject_.States[26].curvature = 745.80;
    std::cout << "FusedFrontObject_.States[26].curvature(float32): " << FusedFrontObject_.States[26].curvature << std::endl;
    FusedFrontObject_.States[26].heading = 746.90;
    std::cout << "FusedFrontObject_.States[26].heading(float32): " << FusedFrontObject_.States[26].heading << std::endl;
    FusedFrontObject_.States[26].latAcceleration = 748.00;
    std::cout << "FusedFrontObject_.States[26].latAcceleration(float32): " << FusedFrontObject_.States[26].latAcceleration << std::endl;
    FusedFrontObject_.States[26].latPosition = 749.10;
    std::cout << "FusedFrontObject_.States[26].latPosition(float32): " << FusedFrontObject_.States[26].latPosition << std::endl;
    FusedFrontObject_.States[26].latVelocity = 750.20;
    std::cout << "FusedFrontObject_.States[26].latVelocity(float32): " << FusedFrontObject_.States[26].latVelocity << std::endl;
    FusedFrontObject_.States[26].longAcceleration = 751.30;
    std::cout << "FusedFrontObject_.States[26].longAcceleration(float32): " << FusedFrontObject_.States[26].longAcceleration << std::endl;
    FusedFrontObject_.States[26].longPosition = 752.40;
    std::cout << "FusedFrontObject_.States[26].longPosition(float32): " << FusedFrontObject_.States[26].longPosition << std::endl;
    FusedFrontObject_.States[26].longVelocity = 753.50;
    std::cout << "FusedFrontObject_.States[26].longVelocity(float32): " << FusedFrontObject_.States[26].longVelocity << std::endl;
    FusedFrontObject_.States[26].speed = 754.60;
    std::cout << "FusedFrontObject_.States[26].speed(float32): " << FusedFrontObject_.States[26].speed << std::endl;
    FusedFrontObject_.States[27].acceleration = 755.70;
    std::cout << "FusedFrontObject_.States[27].acceleration(float32): " << FusedFrontObject_.States[27].acceleration << std::endl;
    FusedFrontObject_.States[27].curvature = 756.80;
    std::cout << "FusedFrontObject_.States[27].curvature(float32): " << FusedFrontObject_.States[27].curvature << std::endl;
    FusedFrontObject_.States[27].heading = 757.90;
    std::cout << "FusedFrontObject_.States[27].heading(float32): " << FusedFrontObject_.States[27].heading << std::endl;
    FusedFrontObject_.States[27].latAcceleration = 759.00;
    std::cout << "FusedFrontObject_.States[27].latAcceleration(float32): " << FusedFrontObject_.States[27].latAcceleration << std::endl;
    FusedFrontObject_.States[27].latPosition = 760.10;
    std::cout << "FusedFrontObject_.States[27].latPosition(float32): " << FusedFrontObject_.States[27].latPosition << std::endl;
    FusedFrontObject_.States[27].latVelocity = 761.20;
    std::cout << "FusedFrontObject_.States[27].latVelocity(float32): " << FusedFrontObject_.States[27].latVelocity << std::endl;
    FusedFrontObject_.States[27].longAcceleration = 762.30;
    std::cout << "FusedFrontObject_.States[27].longAcceleration(float32): " << FusedFrontObject_.States[27].longAcceleration << std::endl;
    FusedFrontObject_.States[27].longPosition = 763.40;
    std::cout << "FusedFrontObject_.States[27].longPosition(float32): " << FusedFrontObject_.States[27].longPosition << std::endl;
    FusedFrontObject_.States[27].longVelocity = 764.50;
    std::cout << "FusedFrontObject_.States[27].longVelocity(float32): " << FusedFrontObject_.States[27].longVelocity << std::endl;
    FusedFrontObject_.States[27].speed = 765.60;
    std::cout << "FusedFrontObject_.States[27].speed(float32): " << FusedFrontObject_.States[27].speed << std::endl;
    FusedFrontObject_.States[28].acceleration = 766.70;
    std::cout << "FusedFrontObject_.States[28].acceleration(float32): " << FusedFrontObject_.States[28].acceleration << std::endl;
    FusedFrontObject_.States[28].curvature = 767.80;
    std::cout << "FusedFrontObject_.States[28].curvature(float32): " << FusedFrontObject_.States[28].curvature << std::endl;
    FusedFrontObject_.States[28].heading = 768.90;
    std::cout << "FusedFrontObject_.States[28].heading(float32): " << FusedFrontObject_.States[28].heading << std::endl;
    FusedFrontObject_.States[28].latAcceleration = 770.00;
    std::cout << "FusedFrontObject_.States[28].latAcceleration(float32): " << FusedFrontObject_.States[28].latAcceleration << std::endl;
    FusedFrontObject_.States[28].latPosition = 771.10;
    std::cout << "FusedFrontObject_.States[28].latPosition(float32): " << FusedFrontObject_.States[28].latPosition << std::endl;
    FusedFrontObject_.States[28].latVelocity = 772.20;
    std::cout << "FusedFrontObject_.States[28].latVelocity(float32): " << FusedFrontObject_.States[28].latVelocity << std::endl;
    FusedFrontObject_.States[28].longAcceleration = 773.30;
    std::cout << "FusedFrontObject_.States[28].longAcceleration(float32): " << FusedFrontObject_.States[28].longAcceleration << std::endl;
    FusedFrontObject_.States[28].longPosition = 774.40;
    std::cout << "FusedFrontObject_.States[28].longPosition(float32): " << FusedFrontObject_.States[28].longPosition << std::endl;
    FusedFrontObject_.States[28].longVelocity = 775.50;
    std::cout << "FusedFrontObject_.States[28].longVelocity(float32): " << FusedFrontObject_.States[28].longVelocity << std::endl;
    FusedFrontObject_.States[28].speed = 776.60;
    std::cout << "FusedFrontObject_.States[28].speed(float32): " << FusedFrontObject_.States[28].speed << std::endl;
    FusedFrontObject_.States[29].acceleration = 777.70;
    std::cout << "FusedFrontObject_.States[29].acceleration(float32): " << FusedFrontObject_.States[29].acceleration << std::endl;
    FusedFrontObject_.States[29].curvature = 778.80;
    std::cout << "FusedFrontObject_.States[29].curvature(float32): " << FusedFrontObject_.States[29].curvature << std::endl;
    FusedFrontObject_.States[29].heading = 779.90;
    std::cout << "FusedFrontObject_.States[29].heading(float32): " << FusedFrontObject_.States[29].heading << std::endl;
    FusedFrontObject_.States[29].latAcceleration = 781.00;
    std::cout << "FusedFrontObject_.States[29].latAcceleration(float32): " << FusedFrontObject_.States[29].latAcceleration << std::endl;
    FusedFrontObject_.States[29].latPosition = 782.10;
    std::cout << "FusedFrontObject_.States[29].latPosition(float32): " << FusedFrontObject_.States[29].latPosition << std::endl;
    FusedFrontObject_.States[29].latVelocity = 783.20;
    std::cout << "FusedFrontObject_.States[29].latVelocity(float32): " << FusedFrontObject_.States[29].latVelocity << std::endl;
    FusedFrontObject_.States[29].longAcceleration = 784.30;
    std::cout << "FusedFrontObject_.States[29].longAcceleration(float32): " << FusedFrontObject_.States[29].longAcceleration << std::endl;
    FusedFrontObject_.States[29].longPosition = 785.40;
    std::cout << "FusedFrontObject_.States[29].longPosition(float32): " << FusedFrontObject_.States[29].longPosition << std::endl;
    FusedFrontObject_.States[29].longVelocity = 786.50;
    std::cout << "FusedFrontObject_.States[29].longVelocity(float32): " << FusedFrontObject_.States[29].longVelocity << std::endl;
    FusedFrontObject_.States[29].speed = 787.60;
    std::cout << "FusedFrontObject_.States[29].speed(float32): " << FusedFrontObject_.States[29].speed << std::endl;
    FusedFrontObject_.States[30].acceleration = 788.70;
    std::cout << "FusedFrontObject_.States[30].acceleration(float32): " << FusedFrontObject_.States[30].acceleration << std::endl;
    FusedFrontObject_.States[30].curvature = 789.80;
    std::cout << "FusedFrontObject_.States[30].curvature(float32): " << FusedFrontObject_.States[30].curvature << std::endl;
    FusedFrontObject_.States[30].heading = 790.90;
    std::cout << "FusedFrontObject_.States[30].heading(float32): " << FusedFrontObject_.States[30].heading << std::endl;
    FusedFrontObject_.States[30].latAcceleration = 792.00;
    std::cout << "FusedFrontObject_.States[30].latAcceleration(float32): " << FusedFrontObject_.States[30].latAcceleration << std::endl;
    FusedFrontObject_.States[30].latPosition = 793.10;
    std::cout << "FusedFrontObject_.States[30].latPosition(float32): " << FusedFrontObject_.States[30].latPosition << std::endl;
    FusedFrontObject_.States[30].latVelocity = 794.20;
    std::cout << "FusedFrontObject_.States[30].latVelocity(float32): " << FusedFrontObject_.States[30].latVelocity << std::endl;
    FusedFrontObject_.States[30].longAcceleration = 795.30;
    std::cout << "FusedFrontObject_.States[30].longAcceleration(float32): " << FusedFrontObject_.States[30].longAcceleration << std::endl;
    FusedFrontObject_.States[30].longPosition = 796.40;
    std::cout << "FusedFrontObject_.States[30].longPosition(float32): " << FusedFrontObject_.States[30].longPosition << std::endl;
    FusedFrontObject_.States[30].longVelocity = 797.50;
    std::cout << "FusedFrontObject_.States[30].longVelocity(float32): " << FusedFrontObject_.States[30].longVelocity << std::endl;
    FusedFrontObject_.States[30].speed = 798.60;
    std::cout << "FusedFrontObject_.States[30].speed(float32): " << FusedFrontObject_.States[30].speed << std::endl;
    FusedFrontObject_.States[31].acceleration = 799.70;
    std::cout << "FusedFrontObject_.States[31].acceleration(float32): " << FusedFrontObject_.States[31].acceleration << std::endl;
    FusedFrontObject_.States[31].curvature = 800.80;
    std::cout << "FusedFrontObject_.States[31].curvature(float32): " << FusedFrontObject_.States[31].curvature << std::endl;
    FusedFrontObject_.States[31].heading = 801.90;
    std::cout << "FusedFrontObject_.States[31].heading(float32): " << FusedFrontObject_.States[31].heading << std::endl;
    FusedFrontObject_.States[31].latAcceleration = 803.00;
    std::cout << "FusedFrontObject_.States[31].latAcceleration(float32): " << FusedFrontObject_.States[31].latAcceleration << std::endl;
    FusedFrontObject_.States[31].latPosition = 804.10;
    std::cout << "FusedFrontObject_.States[31].latPosition(float32): " << FusedFrontObject_.States[31].latPosition << std::endl;
    FusedFrontObject_.States[31].latVelocity = 805.20;
    std::cout << "FusedFrontObject_.States[31].latVelocity(float32): " << FusedFrontObject_.States[31].latVelocity << std::endl;
    FusedFrontObject_.States[31].longAcceleration = 806.30;
    std::cout << "FusedFrontObject_.States[31].longAcceleration(float32): " << FusedFrontObject_.States[31].longAcceleration << std::endl;
    FusedFrontObject_.States[31].longPosition = 807.40;
    std::cout << "FusedFrontObject_.States[31].longPosition(float32): " << FusedFrontObject_.States[31].longPosition << std::endl;
    FusedFrontObject_.States[31].longVelocity = 808.50;
    std::cout << "FusedFrontObject_.States[31].longVelocity(float32): " << FusedFrontObject_.States[31].longVelocity << std::endl;
    FusedFrontObject_.States[31].speed = 809.60;
    std::cout << "FusedFrontObject_.States[31].speed(float32): " << FusedFrontObject_.States[31].speed << std::endl;
    FusedFrontObject_.Timestamp = 1;
    std::cout << "FusedFrontObject_.Timestamp(uint64): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(FusedFrontObject_.Timestamp) << std::dec  << std::endl;
};
#endif



#ifdef LANEMARKER_H
/* Set and Print struct LaneMarker initial value */
void setIntialValue_LaneMarker(LaneMarker& LaneMarker_){
    std::cout << "Set struct LaneMarker variable and Publish:" << std::endl;
    LaneMarker_.EgoLane.Left.Lane.color = 1;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.color) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.a = 1.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.a(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.a << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.b = 2.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.b(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.b << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.c = 3.30;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.c(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.c << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1 = 4.40;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2 = 5.50;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd = 6.60;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart = 7.70;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive = 2;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist = 8.80;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance = 9.90;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance = 11.00;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance = 12.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance = 13.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance = 14.30;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset = 15.40;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading = 16.50;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt = 17.60;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate = 18.70;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.EgoLane.Left.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.EgoLane.Left.Lane.id = 3;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.id) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.markingType = 4;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.markingWidth = 19.80;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.markingWidth(float32): " << LaneMarker_.EgoLane.Left.Lane.markingWidth << std::endl;
    LaneMarker_.EgoLane.Left.Lane.measurementQuality = 20.90;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.measurementQuality(float32): " << LaneMarker_.EgoLane.Left.Lane.measurementQuality << std::endl;
    LaneMarker_.EgoLane.Left.Lane.modelError = 22.00;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.modelError(float32): " << LaneMarker_.EgoLane.Left.Lane.modelError << std::endl;
    LaneMarker_.EgoLane.Left.Lane.secondMarkingType = 5;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.selectionConfidence = 23.10;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.selectionConfidence(float32): " << LaneMarker_.EgoLane.Left.Lane.selectionConfidence << std::endl;
    LaneMarker_.EgoLane.Left.Lane.structure = 6;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth = 24.20;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth(float32): " << LaneMarker_.EgoLane.Left.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.EgoLane.Left.Lane.trackingStatus = 7;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.valid = 8;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Left.Lane.isVerified = 9;
    std::cout << "LaneMarker_.EgoLane.Left.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Left.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.color = 10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.color) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.a = 25.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.a(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.a << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.b = 26.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.b(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.b << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.c = 27.50;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.c(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.c << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1 = 28.60;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2 = 29.70;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd = 30.80;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart = 31.90;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive = 11;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist = 33.00;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance = 34.10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance = 35.20;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance = 36.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance = 37.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance = 38.50;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset = 39.60;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading = 40.70;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt = 41.80;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate = 42.90;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.EgoLane.Right.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.EgoLane.Right.Lane.id = 12;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.id) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.markingType = 13;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.markingWidth = 44.00;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.markingWidth(float32): " << LaneMarker_.EgoLane.Right.Lane.markingWidth << std::endl;
    LaneMarker_.EgoLane.Right.Lane.measurementQuality = 45.10;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.measurementQuality(float32): " << LaneMarker_.EgoLane.Right.Lane.measurementQuality << std::endl;
    LaneMarker_.EgoLane.Right.Lane.modelError = 46.20;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.modelError(float32): " << LaneMarker_.EgoLane.Right.Lane.modelError << std::endl;
    LaneMarker_.EgoLane.Right.Lane.secondMarkingType = 14;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.selectionConfidence = 47.30;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.selectionConfidence(float32): " << LaneMarker_.EgoLane.Right.Lane.selectionConfidence << std::endl;
    LaneMarker_.EgoLane.Right.Lane.structure = 15;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth = 48.40;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth(float32): " << LaneMarker_.EgoLane.Right.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.EgoLane.Right.Lane.trackingStatus = 16;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.valid = 17;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.EgoLane.Right.Lane.isVerified = 18;
    std::cout << "LaneMarker_.EgoLane.Right.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.Right.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.EgoLane.parallelDistance = 49.50;
    std::cout << "LaneMarker_.EgoLane.parallelDistance(float32): " << LaneMarker_.EgoLane.parallelDistance << std::endl;
    LaneMarker_.EgoLane.validProjectionDistance = 50.60;
    std::cout << "LaneMarker_.EgoLane.validProjectionDistance(float32): " << LaneMarker_.EgoLane.validProjectionDistance << std::endl;
    LaneMarker_.EgoLane.attentionMarkerDetected = 19;
    std::cout << "LaneMarker_.EgoLane.attentionMarkerDetected(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.attentionMarkerDetected) << std::dec  << std::endl;
    LaneMarker_.EgoLane.isMergeLane = 20;
    std::cout << "LaneMarker_.EgoLane.isMergeLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.EgoLane.isMergeLane) << std::dec  << std::endl;
    LaneMarker_.EgoLane.MergeRange = 51.70;
    std::cout << "LaneMarker_.EgoLane.MergeRange(float32): " << LaneMarker_.EgoLane.MergeRange << std::endl;
    LaneMarker_.DebugBus.debugFloat1 = 52.80;
    std::cout << "LaneMarker_.DebugBus.debugFloat1(float32): " << LaneMarker_.DebugBus.debugFloat1 << std::endl;
    LaneMarker_.DebugBus.debugFloat2 = 53.90;
    std::cout << "LaneMarker_.DebugBus.debugFloat2(float32): " << LaneMarker_.DebugBus.debugFloat2 << std::endl;
    LaneMarker_.DebugBus.debugFloat3 = 55.00;
    std::cout << "LaneMarker_.DebugBus.debugFloat3(float32): " << LaneMarker_.DebugBus.debugFloat3 << std::endl;
    LaneMarker_.DebugBus.debugFloat4 = 56.10;
    std::cout << "LaneMarker_.DebugBus.debugFloat4(float32): " << LaneMarker_.DebugBus.debugFloat4 << std::endl;
    LaneMarker_.DebugBus.debugFloat5 = 57.20;
    std::cout << "LaneMarker_.DebugBus.debugFloat5(float32): " << LaneMarker_.DebugBus.debugFloat5 << std::endl;
    LaneMarker_.DebugBus.debugFloat6 = 58.30;
    std::cout << "LaneMarker_.DebugBus.debugFloat6(float32): " << LaneMarker_.DebugBus.debugFloat6 << std::endl;
    LaneMarker_.DebugBus.debugFloat7 = 59.40;
    std::cout << "LaneMarker_.DebugBus.debugFloat7(float32): " << LaneMarker_.DebugBus.debugFloat7 << std::endl;
    LaneMarker_.DebugBus.debugFloat8 = 60.50;
    std::cout << "LaneMarker_.DebugBus.debugFloat8(float32): " << LaneMarker_.DebugBus.debugFloat8 << std::endl;
    LaneMarker_.DebugBus.debugInteger1 = 21;
    std::cout << "LaneMarker_.DebugBus.debugInteger1(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger1) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger2 = 22;
    std::cout << "LaneMarker_.DebugBus.debugInteger2(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger2) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger3 = 23;
    std::cout << "LaneMarker_.DebugBus.debugInteger3(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger3) << std::dec  << std::endl;
    LaneMarker_.DebugBus.debugInteger4 = 24;
    std::cout << "LaneMarker_.DebugBus.debugInteger4(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.DebugBus.debugInteger4) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.color = 25;
    std::cout << "LaneMarker_.SecClsLeft.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.color) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.a = 61.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.a(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.a << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.b = 62.70;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.b(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.b << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.c = 63.80;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.c(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.c << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d1 = 64.90;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d1(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d2 = 66.00;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d2(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd = 67.10;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart = 68.20;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive = 26;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist = 69.30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance = 70.40;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance = 71.50;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance = 72.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance = 73.70;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance = 74.80;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset = 75.90;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading = 77.00;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt = 78.10;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate = 79.20;
    std::cout << "LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.SecClsLeft.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.SecClsLeft.Lane.id = 27;
    std::cout << "LaneMarker_.SecClsLeft.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.id) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.markingType = 28;
    std::cout << "LaneMarker_.SecClsLeft.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.markingWidth = 80.30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.markingWidth(float32): " << LaneMarker_.SecClsLeft.Lane.markingWidth << std::endl;
    LaneMarker_.SecClsLeft.Lane.measurementQuality = 81.40;
    std::cout << "LaneMarker_.SecClsLeft.Lane.measurementQuality(float32): " << LaneMarker_.SecClsLeft.Lane.measurementQuality << std::endl;
    LaneMarker_.SecClsLeft.Lane.modelError = 82.50;
    std::cout << "LaneMarker_.SecClsLeft.Lane.modelError(float32): " << LaneMarker_.SecClsLeft.Lane.modelError << std::endl;
    LaneMarker_.SecClsLeft.Lane.secondMarkingType = 29;
    std::cout << "LaneMarker_.SecClsLeft.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.selectionConfidence = 83.60;
    std::cout << "LaneMarker_.SecClsLeft.Lane.selectionConfidence(float32): " << LaneMarker_.SecClsLeft.Lane.selectionConfidence << std::endl;
    LaneMarker_.SecClsLeft.Lane.structure = 30;
    std::cout << "LaneMarker_.SecClsLeft.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.totalMarkingWidth = 84.70;
    std::cout << "LaneMarker_.SecClsLeft.Lane.totalMarkingWidth(float32): " << LaneMarker_.SecClsLeft.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.SecClsLeft.Lane.trackingStatus = 31;
    std::cout << "LaneMarker_.SecClsLeft.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.valid = 32;
    std::cout << "LaneMarker_.SecClsLeft.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.SecClsLeft.Lane.isVerified = 33;
    std::cout << "LaneMarker_.SecClsLeft.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsLeft.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.color = 34;
    std::cout << "LaneMarker_.SecClsRight.Lane.color(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.color) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.a = 85.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.a(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.a << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.b = 86.90;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.b(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.b << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.c = 88.00;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.c(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.c << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d1 = 89.10;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d1(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d1 << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d2 = 90.20;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d2(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d2 << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd = 91.30;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToEnd << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart = 92.40;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.longDistToStart << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive = 35;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.DblClothoid.secClothoidActive) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist = 93.50;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.transitionDist << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance = 94.60;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.aVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance = 95.70;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.bVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance = 96.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.cVariance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance = 97.90;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d1Variance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance = 99.00;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.d2Variance << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset = 100.10;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtOffset << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading = 101.20;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtHeading << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt = 102.30;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvt << std::endl;
    LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate = 103.40;
    std::cout << "LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate(float32): " << LaneMarker_.SecClsRight.Lane.DblClothoid.vrtCrvtRate << std::endl;
    LaneMarker_.SecClsRight.Lane.id = 36;
    std::cout << "LaneMarker_.SecClsRight.Lane.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.id) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.markingType = 37;
    std::cout << "LaneMarker_.SecClsRight.Lane.markingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.markingType) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.markingWidth = 104.50;
    std::cout << "LaneMarker_.SecClsRight.Lane.markingWidth(float32): " << LaneMarker_.SecClsRight.Lane.markingWidth << std::endl;
    LaneMarker_.SecClsRight.Lane.measurementQuality = 105.60;
    std::cout << "LaneMarker_.SecClsRight.Lane.measurementQuality(float32): " << LaneMarker_.SecClsRight.Lane.measurementQuality << std::endl;
    LaneMarker_.SecClsRight.Lane.modelError = 106.70;
    std::cout << "LaneMarker_.SecClsRight.Lane.modelError(float32): " << LaneMarker_.SecClsRight.Lane.modelError << std::endl;
    LaneMarker_.SecClsRight.Lane.secondMarkingType = 38;
    std::cout << "LaneMarker_.SecClsRight.Lane.secondMarkingType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.secondMarkingType) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.selectionConfidence = 107.80;
    std::cout << "LaneMarker_.SecClsRight.Lane.selectionConfidence(float32): " << LaneMarker_.SecClsRight.Lane.selectionConfidence << std::endl;
    LaneMarker_.SecClsRight.Lane.structure = 39;
    std::cout << "LaneMarker_.SecClsRight.Lane.structure(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.structure) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.totalMarkingWidth = 108.90;
    std::cout << "LaneMarker_.SecClsRight.Lane.totalMarkingWidth(float32): " << LaneMarker_.SecClsRight.Lane.totalMarkingWidth << std::endl;
    LaneMarker_.SecClsRight.Lane.trackingStatus = 40;
    std::cout << "LaneMarker_.SecClsRight.Lane.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.trackingStatus) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.valid = 41;
    std::cout << "LaneMarker_.SecClsRight.Lane.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.valid) << std::dec  << std::endl;
    LaneMarker_.SecClsRight.Lane.isVerified = 42;
    std::cout << "LaneMarker_.SecClsRight.Lane.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.SecClsRight.Lane.isVerified) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].distance = 110.00;
    std::cout << "LaneMarker_.LaneEvent[0].distance(float32): " << LaneMarker_.LaneEvent[0].distance << std::endl;
    LaneMarker_.LaneEvent[0].eventType = 43;
    std::cout << "LaneMarker_.LaneEvent[0].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].id = 44;
    std::cout << "LaneMarker_.LaneEvent[0].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[0].laneTrack = 45;
    std::cout << "LaneMarker_.LaneEvent[0].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[0].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].distance = 111.10;
    std::cout << "LaneMarker_.LaneEvent[1].distance(float32): " << LaneMarker_.LaneEvent[1].distance << std::endl;
    LaneMarker_.LaneEvent[1].eventType = 46;
    std::cout << "LaneMarker_.LaneEvent[1].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].id = 47;
    std::cout << "LaneMarker_.LaneEvent[1].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[1].laneTrack = 48;
    std::cout << "LaneMarker_.LaneEvent[1].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[1].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].distance = 112.20;
    std::cout << "LaneMarker_.LaneEvent[2].distance(float32): " << LaneMarker_.LaneEvent[2].distance << std::endl;
    LaneMarker_.LaneEvent[2].eventType = 49;
    std::cout << "LaneMarker_.LaneEvent[2].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].id = 50;
    std::cout << "LaneMarker_.LaneEvent[2].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[2].laneTrack = 51;
    std::cout << "LaneMarker_.LaneEvent[2].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[2].laneTrack) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].distance = 113.30;
    std::cout << "LaneMarker_.LaneEvent[3].distance(float32): " << LaneMarker_.LaneEvent[3].distance << std::endl;
    LaneMarker_.LaneEvent[3].eventType = 52;
    std::cout << "LaneMarker_.LaneEvent[3].eventType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].eventType) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].id = 53;
    std::cout << "LaneMarker_.LaneEvent[3].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].id) << std::dec  << std::endl;
    LaneMarker_.LaneEvent[3].laneTrack = 54;
    std::cout << "LaneMarker_.LaneEvent[3].laneTrack(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.LaneEvent[3].laneTrack) << std::dec  << std::endl;
    LaneMarker_.TemporaryMarking.longDistanceToStart = 114.40;
    std::cout << "LaneMarker_.TemporaryMarking.longDistanceToStart(float32): " << LaneMarker_.TemporaryMarking.longDistanceToStart << std::endl;
    LaneMarker_.TemporaryMarking.type = 55;
    std::cout << "LaneMarker_.TemporaryMarking.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.TemporaryMarking.type) << std::dec  << std::endl;
    LaneMarker_.sideSuggestion = 56;
    std::cout << "LaneMarker_.sideSuggestion(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.sideSuggestion) << std::dec  << std::endl;
    LaneMarker_.laneChange = 57;
    std::cout << "LaneMarker_.laneChange(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LaneMarker_.laneChange) << std::dec  << std::endl;
    LaneMarker_.SequenceID = 1;
    std::cout << "LaneMarker_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(LaneMarker_.SequenceID) << std::dec  << std::endl;
    LaneMarker_.Timestamp = 1;
    std::cout << "LaneMarker_.Timestamp(uint64): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(LaneMarker_.Timestamp) << std::dec  << std::endl;
};
#endif



#ifdef REAROBJECT_H
/* Set and Print struct RearObject initial value */
void setIntialValue_RearObject(RearObject& RearObject_){
    std::cout << "Set struct RearObject variable and Publish:" << std::endl;
    RearObject_.EgoStates[0].heading = 1.10;
    std::cout << "RearObject_.EgoStates[0].heading(float32): " << RearObject_.EgoStates[0].heading << std::endl;
    RearObject_.EgoStates[0].latAcceleration = 2.20;
    std::cout << "RearObject_.EgoStates[0].latAcceleration(float32): " << RearObject_.EgoStates[0].latAcceleration << std::endl;
    RearObject_.EgoStates[0].latPosition = 3.30;
    std::cout << "RearObject_.EgoStates[0].latPosition(float32): " << RearObject_.EgoStates[0].latPosition << std::endl;
    RearObject_.EgoStates[0].latPositionPath = 4.40;
    std::cout << "RearObject_.EgoStates[0].latPositionPath(float32): " << RearObject_.EgoStates[0].latPositionPath << std::endl;
    RearObject_.EgoStates[0].latVelocity = 5.50;
    std::cout << "RearObject_.EgoStates[0].latVelocity(float32): " << RearObject_.EgoStates[0].latVelocity << std::endl;
    RearObject_.EgoStates[0].longAcceleration = 6.60;
    std::cout << "RearObject_.EgoStates[0].longAcceleration(float32): " << RearObject_.EgoStates[0].longAcceleration << std::endl;
    RearObject_.EgoStates[0].longPosition = 7.70;
    std::cout << "RearObject_.EgoStates[0].longPosition(float32): " << RearObject_.EgoStates[0].longPosition << std::endl;
    RearObject_.EgoStates[0].longPositionPath = 8.80;
    std::cout << "RearObject_.EgoStates[0].longPositionPath(float32): " << RearObject_.EgoStates[0].longPositionPath << std::endl;
    RearObject_.EgoStates[0].longVelocity = 9.90;
    std::cout << "RearObject_.EgoStates[0].longVelocity(float32): " << RearObject_.EgoStates[0].longVelocity << std::endl;
    RearObject_.EgoStates[1].heading = 11.00;
    std::cout << "RearObject_.EgoStates[1].heading(float32): " << RearObject_.EgoStates[1].heading << std::endl;
    RearObject_.EgoStates[1].latAcceleration = 12.10;
    std::cout << "RearObject_.EgoStates[1].latAcceleration(float32): " << RearObject_.EgoStates[1].latAcceleration << std::endl;
    RearObject_.EgoStates[1].latPosition = 13.20;
    std::cout << "RearObject_.EgoStates[1].latPosition(float32): " << RearObject_.EgoStates[1].latPosition << std::endl;
    RearObject_.EgoStates[1].latPositionPath = 14.30;
    std::cout << "RearObject_.EgoStates[1].latPositionPath(float32): " << RearObject_.EgoStates[1].latPositionPath << std::endl;
    RearObject_.EgoStates[1].latVelocity = 15.40;
    std::cout << "RearObject_.EgoStates[1].latVelocity(float32): " << RearObject_.EgoStates[1].latVelocity << std::endl;
    RearObject_.EgoStates[1].longAcceleration = 16.50;
    std::cout << "RearObject_.EgoStates[1].longAcceleration(float32): " << RearObject_.EgoStates[1].longAcceleration << std::endl;
    RearObject_.EgoStates[1].longPosition = 17.60;
    std::cout << "RearObject_.EgoStates[1].longPosition(float32): " << RearObject_.EgoStates[1].longPosition << std::endl;
    RearObject_.EgoStates[1].longPositionPath = 18.70;
    std::cout << "RearObject_.EgoStates[1].longPositionPath(float32): " << RearObject_.EgoStates[1].longPositionPath << std::endl;
    RearObject_.EgoStates[1].longVelocity = 19.80;
    std::cout << "RearObject_.EgoStates[1].longVelocity(float32): " << RearObject_.EgoStates[1].longVelocity << std::endl;
    RearObject_.EgoStates[2].heading = 20.90;
    std::cout << "RearObject_.EgoStates[2].heading(float32): " << RearObject_.EgoStates[2].heading << std::endl;
    RearObject_.EgoStates[2].latAcceleration = 22.00;
    std::cout << "RearObject_.EgoStates[2].latAcceleration(float32): " << RearObject_.EgoStates[2].latAcceleration << std::endl;
    RearObject_.EgoStates[2].latPosition = 23.10;
    std::cout << "RearObject_.EgoStates[2].latPosition(float32): " << RearObject_.EgoStates[2].latPosition << std::endl;
    RearObject_.EgoStates[2].latPositionPath = 24.20;
    std::cout << "RearObject_.EgoStates[2].latPositionPath(float32): " << RearObject_.EgoStates[2].latPositionPath << std::endl;
    RearObject_.EgoStates[2].latVelocity = 25.30;
    std::cout << "RearObject_.EgoStates[2].latVelocity(float32): " << RearObject_.EgoStates[2].latVelocity << std::endl;
    RearObject_.EgoStates[2].longAcceleration = 26.40;
    std::cout << "RearObject_.EgoStates[2].longAcceleration(float32): " << RearObject_.EgoStates[2].longAcceleration << std::endl;
    RearObject_.EgoStates[2].longPosition = 27.50;
    std::cout << "RearObject_.EgoStates[2].longPosition(float32): " << RearObject_.EgoStates[2].longPosition << std::endl;
    RearObject_.EgoStates[2].longPositionPath = 28.60;
    std::cout << "RearObject_.EgoStates[2].longPositionPath(float32): " << RearObject_.EgoStates[2].longPositionPath << std::endl;
    RearObject_.EgoStates[2].longVelocity = 29.70;
    std::cout << "RearObject_.EgoStates[2].longVelocity(float32): " << RearObject_.EgoStates[2].longVelocity << std::endl;
    RearObject_.EgoStates[3].heading = 30.80;
    std::cout << "RearObject_.EgoStates[3].heading(float32): " << RearObject_.EgoStates[3].heading << std::endl;
    RearObject_.EgoStates[3].latAcceleration = 31.90;
    std::cout << "RearObject_.EgoStates[3].latAcceleration(float32): " << RearObject_.EgoStates[3].latAcceleration << std::endl;
    RearObject_.EgoStates[3].latPosition = 33.00;
    std::cout << "RearObject_.EgoStates[3].latPosition(float32): " << RearObject_.EgoStates[3].latPosition << std::endl;
    RearObject_.EgoStates[3].latPositionPath = 34.10;
    std::cout << "RearObject_.EgoStates[3].latPositionPath(float32): " << RearObject_.EgoStates[3].latPositionPath << std::endl;
    RearObject_.EgoStates[3].latVelocity = 35.20;
    std::cout << "RearObject_.EgoStates[3].latVelocity(float32): " << RearObject_.EgoStates[3].latVelocity << std::endl;
    RearObject_.EgoStates[3].longAcceleration = 36.30;
    std::cout << "RearObject_.EgoStates[3].longAcceleration(float32): " << RearObject_.EgoStates[3].longAcceleration << std::endl;
    RearObject_.EgoStates[3].longPosition = 37.40;
    std::cout << "RearObject_.EgoStates[3].longPosition(float32): " << RearObject_.EgoStates[3].longPosition << std::endl;
    RearObject_.EgoStates[3].longPositionPath = 38.50;
    std::cout << "RearObject_.EgoStates[3].longPositionPath(float32): " << RearObject_.EgoStates[3].longPositionPath << std::endl;
    RearObject_.EgoStates[3].longVelocity = 39.60;
    std::cout << "RearObject_.EgoStates[3].longVelocity(float32): " << RearObject_.EgoStates[3].longVelocity << std::endl;
    RearObject_.EgoStates[4].heading = 40.70;
    std::cout << "RearObject_.EgoStates[4].heading(float32): " << RearObject_.EgoStates[4].heading << std::endl;
    RearObject_.EgoStates[4].latAcceleration = 41.80;
    std::cout << "RearObject_.EgoStates[4].latAcceleration(float32): " << RearObject_.EgoStates[4].latAcceleration << std::endl;
    RearObject_.EgoStates[4].latPosition = 42.90;
    std::cout << "RearObject_.EgoStates[4].latPosition(float32): " << RearObject_.EgoStates[4].latPosition << std::endl;
    RearObject_.EgoStates[4].latPositionPath = 44.00;
    std::cout << "RearObject_.EgoStates[4].latPositionPath(float32): " << RearObject_.EgoStates[4].latPositionPath << std::endl;
    RearObject_.EgoStates[4].latVelocity = 45.10;
    std::cout << "RearObject_.EgoStates[4].latVelocity(float32): " << RearObject_.EgoStates[4].latVelocity << std::endl;
    RearObject_.EgoStates[4].longAcceleration = 46.20;
    std::cout << "RearObject_.EgoStates[4].longAcceleration(float32): " << RearObject_.EgoStates[4].longAcceleration << std::endl;
    RearObject_.EgoStates[4].longPosition = 47.30;
    std::cout << "RearObject_.EgoStates[4].longPosition(float32): " << RearObject_.EgoStates[4].longPosition << std::endl;
    RearObject_.EgoStates[4].longPositionPath = 48.40;
    std::cout << "RearObject_.EgoStates[4].longPositionPath(float32): " << RearObject_.EgoStates[4].longPositionPath << std::endl;
    RearObject_.EgoStates[4].longVelocity = 49.50;
    std::cout << "RearObject_.EgoStates[4].longVelocity(float32): " << RearObject_.EgoStates[4].longVelocity << std::endl;
    RearObject_.EgoStates[5].heading = 50.60;
    std::cout << "RearObject_.EgoStates[5].heading(float32): " << RearObject_.EgoStates[5].heading << std::endl;
    RearObject_.EgoStates[5].latAcceleration = 51.70;
    std::cout << "RearObject_.EgoStates[5].latAcceleration(float32): " << RearObject_.EgoStates[5].latAcceleration << std::endl;
    RearObject_.EgoStates[5].latPosition = 52.80;
    std::cout << "RearObject_.EgoStates[5].latPosition(float32): " << RearObject_.EgoStates[5].latPosition << std::endl;
    RearObject_.EgoStates[5].latPositionPath = 53.90;
    std::cout << "RearObject_.EgoStates[5].latPositionPath(float32): " << RearObject_.EgoStates[5].latPositionPath << std::endl;
    RearObject_.EgoStates[5].latVelocity = 55.00;
    std::cout << "RearObject_.EgoStates[5].latVelocity(float32): " << RearObject_.EgoStates[5].latVelocity << std::endl;
    RearObject_.EgoStates[5].longAcceleration = 56.10;
    std::cout << "RearObject_.EgoStates[5].longAcceleration(float32): " << RearObject_.EgoStates[5].longAcceleration << std::endl;
    RearObject_.EgoStates[5].longPosition = 57.20;
    std::cout << "RearObject_.EgoStates[5].longPosition(float32): " << RearObject_.EgoStates[5].longPosition << std::endl;
    RearObject_.EgoStates[5].longPositionPath = 58.30;
    std::cout << "RearObject_.EgoStates[5].longPositionPath(float32): " << RearObject_.EgoStates[5].longPositionPath << std::endl;
    RearObject_.EgoStates[5].longVelocity = 59.40;
    std::cout << "RearObject_.EgoStates[5].longVelocity(float32): " << RearObject_.EgoStates[5].longVelocity << std::endl;
    RearObject_.EgoStates[6].heading = 60.50;
    std::cout << "RearObject_.EgoStates[6].heading(float32): " << RearObject_.EgoStates[6].heading << std::endl;
    RearObject_.EgoStates[6].latAcceleration = 61.60;
    std::cout << "RearObject_.EgoStates[6].latAcceleration(float32): " << RearObject_.EgoStates[6].latAcceleration << std::endl;
    RearObject_.EgoStates[6].latPosition = 62.70;
    std::cout << "RearObject_.EgoStates[6].latPosition(float32): " << RearObject_.EgoStates[6].latPosition << std::endl;
    RearObject_.EgoStates[6].latPositionPath = 63.80;
    std::cout << "RearObject_.EgoStates[6].latPositionPath(float32): " << RearObject_.EgoStates[6].latPositionPath << std::endl;
    RearObject_.EgoStates[6].latVelocity = 64.90;
    std::cout << "RearObject_.EgoStates[6].latVelocity(float32): " << RearObject_.EgoStates[6].latVelocity << std::endl;
    RearObject_.EgoStates[6].longAcceleration = 66.00;
    std::cout << "RearObject_.EgoStates[6].longAcceleration(float32): " << RearObject_.EgoStates[6].longAcceleration << std::endl;
    RearObject_.EgoStates[6].longPosition = 67.10;
    std::cout << "RearObject_.EgoStates[6].longPosition(float32): " << RearObject_.EgoStates[6].longPosition << std::endl;
    RearObject_.EgoStates[6].longPositionPath = 68.20;
    std::cout << "RearObject_.EgoStates[6].longPositionPath(float32): " << RearObject_.EgoStates[6].longPositionPath << std::endl;
    RearObject_.EgoStates[6].longVelocity = 69.30;
    std::cout << "RearObject_.EgoStates[6].longVelocity(float32): " << RearObject_.EgoStates[6].longVelocity << std::endl;
    RearObject_.EgoStates[7].heading = 70.40;
    std::cout << "RearObject_.EgoStates[7].heading(float32): " << RearObject_.EgoStates[7].heading << std::endl;
    RearObject_.EgoStates[7].latAcceleration = 71.50;
    std::cout << "RearObject_.EgoStates[7].latAcceleration(float32): " << RearObject_.EgoStates[7].latAcceleration << std::endl;
    RearObject_.EgoStates[7].latPosition = 72.60;
    std::cout << "RearObject_.EgoStates[7].latPosition(float32): " << RearObject_.EgoStates[7].latPosition << std::endl;
    RearObject_.EgoStates[7].latPositionPath = 73.70;
    std::cout << "RearObject_.EgoStates[7].latPositionPath(float32): " << RearObject_.EgoStates[7].latPositionPath << std::endl;
    RearObject_.EgoStates[7].latVelocity = 74.80;
    std::cout << "RearObject_.EgoStates[7].latVelocity(float32): " << RearObject_.EgoStates[7].latVelocity << std::endl;
    RearObject_.EgoStates[7].longAcceleration = 75.90;
    std::cout << "RearObject_.EgoStates[7].longAcceleration(float32): " << RearObject_.EgoStates[7].longAcceleration << std::endl;
    RearObject_.EgoStates[7].longPosition = 77.00;
    std::cout << "RearObject_.EgoStates[7].longPosition(float32): " << RearObject_.EgoStates[7].longPosition << std::endl;
    RearObject_.EgoStates[7].longPositionPath = 78.10;
    std::cout << "RearObject_.EgoStates[7].longPositionPath(float32): " << RearObject_.EgoStates[7].longPositionPath << std::endl;
    RearObject_.EgoStates[7].longVelocity = 79.20;
    std::cout << "RearObject_.EgoStates[7].longVelocity(float32): " << RearObject_.EgoStates[7].longVelocity << std::endl;
    RearObject_.EgoStates[8].heading = 80.30;
    std::cout << "RearObject_.EgoStates[8].heading(float32): " << RearObject_.EgoStates[8].heading << std::endl;
    RearObject_.EgoStates[8].latAcceleration = 81.40;
    std::cout << "RearObject_.EgoStates[8].latAcceleration(float32): " << RearObject_.EgoStates[8].latAcceleration << std::endl;
    RearObject_.EgoStates[8].latPosition = 82.50;
    std::cout << "RearObject_.EgoStates[8].latPosition(float32): " << RearObject_.EgoStates[8].latPosition << std::endl;
    RearObject_.EgoStates[8].latPositionPath = 83.60;
    std::cout << "RearObject_.EgoStates[8].latPositionPath(float32): " << RearObject_.EgoStates[8].latPositionPath << std::endl;
    RearObject_.EgoStates[8].latVelocity = 84.70;
    std::cout << "RearObject_.EgoStates[8].latVelocity(float32): " << RearObject_.EgoStates[8].latVelocity << std::endl;
    RearObject_.EgoStates[8].longAcceleration = 85.80;
    std::cout << "RearObject_.EgoStates[8].longAcceleration(float32): " << RearObject_.EgoStates[8].longAcceleration << std::endl;
    RearObject_.EgoStates[8].longPosition = 86.90;
    std::cout << "RearObject_.EgoStates[8].longPosition(float32): " << RearObject_.EgoStates[8].longPosition << std::endl;
    RearObject_.EgoStates[8].longPositionPath = 88.00;
    std::cout << "RearObject_.EgoStates[8].longPositionPath(float32): " << RearObject_.EgoStates[8].longPositionPath << std::endl;
    RearObject_.EgoStates[8].longVelocity = 89.10;
    std::cout << "RearObject_.EgoStates[8].longVelocity(float32): " << RearObject_.EgoStates[8].longVelocity << std::endl;
    RearObject_.EgoStates[9].heading = 90.20;
    std::cout << "RearObject_.EgoStates[9].heading(float32): " << RearObject_.EgoStates[9].heading << std::endl;
    RearObject_.EgoStates[9].latAcceleration = 91.30;
    std::cout << "RearObject_.EgoStates[9].latAcceleration(float32): " << RearObject_.EgoStates[9].latAcceleration << std::endl;
    RearObject_.EgoStates[9].latPosition = 92.40;
    std::cout << "RearObject_.EgoStates[9].latPosition(float32): " << RearObject_.EgoStates[9].latPosition << std::endl;
    RearObject_.EgoStates[9].latPositionPath = 93.50;
    std::cout << "RearObject_.EgoStates[9].latPositionPath(float32): " << RearObject_.EgoStates[9].latPositionPath << std::endl;
    RearObject_.EgoStates[9].latVelocity = 94.60;
    std::cout << "RearObject_.EgoStates[9].latVelocity(float32): " << RearObject_.EgoStates[9].latVelocity << std::endl;
    RearObject_.EgoStates[9].longAcceleration = 95.70;
    std::cout << "RearObject_.EgoStates[9].longAcceleration(float32): " << RearObject_.EgoStates[9].longAcceleration << std::endl;
    RearObject_.EgoStates[9].longPosition = 96.80;
    std::cout << "RearObject_.EgoStates[9].longPosition(float32): " << RearObject_.EgoStates[9].longPosition << std::endl;
    RearObject_.EgoStates[9].longPositionPath = 97.90;
    std::cout << "RearObject_.EgoStates[9].longPositionPath(float32): " << RearObject_.EgoStates[9].longPositionPath << std::endl;
    RearObject_.EgoStates[9].longVelocity = 99.00;
    std::cout << "RearObject_.EgoStates[9].longVelocity(float32): " << RearObject_.EgoStates[9].longVelocity << std::endl;
    RearObject_.RoadStates[0].headingRoad = 100.10;
    std::cout << "RearObject_.RoadStates[0].headingRoad(float32): " << RearObject_.RoadStates[0].headingRoad << std::endl;
    RearObject_.RoadStates[0].latAccelerationRoad = 101.20;
    std::cout << "RearObject_.RoadStates[0].latAccelerationRoad(float32): " << RearObject_.RoadStates[0].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[0].latPositionRoad = 102.30;
    std::cout << "RearObject_.RoadStates[0].latPositionRoad(float32): " << RearObject_.RoadStates[0].latPositionRoad << std::endl;
    RearObject_.RoadStates[0].latVelocityRoad = 103.40;
    std::cout << "RearObject_.RoadStates[0].latVelocityRoad(float32): " << RearObject_.RoadStates[0].latVelocityRoad << std::endl;
    RearObject_.RoadStates[0].longAccelerationRoad = 104.50;
    std::cout << "RearObject_.RoadStates[0].longAccelerationRoad(float32): " << RearObject_.RoadStates[0].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[0].longPositionRoad = 105.60;
    std::cout << "RearObject_.RoadStates[0].longPositionRoad(float32): " << RearObject_.RoadStates[0].longPositionRoad << std::endl;
    RearObject_.RoadStates[0].longVelocityRoad = 106.70;
    std::cout << "RearObject_.RoadStates[0].longVelocityRoad(float32): " << RearObject_.RoadStates[0].longVelocityRoad << std::endl;
    RearObject_.RoadStates[1].headingRoad = 107.80;
    std::cout << "RearObject_.RoadStates[1].headingRoad(float32): " << RearObject_.RoadStates[1].headingRoad << std::endl;
    RearObject_.RoadStates[1].latAccelerationRoad = 108.90;
    std::cout << "RearObject_.RoadStates[1].latAccelerationRoad(float32): " << RearObject_.RoadStates[1].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[1].latPositionRoad = 110.00;
    std::cout << "RearObject_.RoadStates[1].latPositionRoad(float32): " << RearObject_.RoadStates[1].latPositionRoad << std::endl;
    RearObject_.RoadStates[1].latVelocityRoad = 111.10;
    std::cout << "RearObject_.RoadStates[1].latVelocityRoad(float32): " << RearObject_.RoadStates[1].latVelocityRoad << std::endl;
    RearObject_.RoadStates[1].longAccelerationRoad = 112.20;
    std::cout << "RearObject_.RoadStates[1].longAccelerationRoad(float32): " << RearObject_.RoadStates[1].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[1].longPositionRoad = 113.30;
    std::cout << "RearObject_.RoadStates[1].longPositionRoad(float32): " << RearObject_.RoadStates[1].longPositionRoad << std::endl;
    RearObject_.RoadStates[1].longVelocityRoad = 114.40;
    std::cout << "RearObject_.RoadStates[1].longVelocityRoad(float32): " << RearObject_.RoadStates[1].longVelocityRoad << std::endl;
    RearObject_.RoadStates[2].headingRoad = 115.50;
    std::cout << "RearObject_.RoadStates[2].headingRoad(float32): " << RearObject_.RoadStates[2].headingRoad << std::endl;
    RearObject_.RoadStates[2].latAccelerationRoad = 116.60;
    std::cout << "RearObject_.RoadStates[2].latAccelerationRoad(float32): " << RearObject_.RoadStates[2].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[2].latPositionRoad = 117.70;
    std::cout << "RearObject_.RoadStates[2].latPositionRoad(float32): " << RearObject_.RoadStates[2].latPositionRoad << std::endl;
    RearObject_.RoadStates[2].latVelocityRoad = 118.80;
    std::cout << "RearObject_.RoadStates[2].latVelocityRoad(float32): " << RearObject_.RoadStates[2].latVelocityRoad << std::endl;
    RearObject_.RoadStates[2].longAccelerationRoad = 119.90;
    std::cout << "RearObject_.RoadStates[2].longAccelerationRoad(float32): " << RearObject_.RoadStates[2].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[2].longPositionRoad = 121.00;
    std::cout << "RearObject_.RoadStates[2].longPositionRoad(float32): " << RearObject_.RoadStates[2].longPositionRoad << std::endl;
    RearObject_.RoadStates[2].longVelocityRoad = 122.10;
    std::cout << "RearObject_.RoadStates[2].longVelocityRoad(float32): " << RearObject_.RoadStates[2].longVelocityRoad << std::endl;
    RearObject_.RoadStates[3].headingRoad = 123.20;
    std::cout << "RearObject_.RoadStates[3].headingRoad(float32): " << RearObject_.RoadStates[3].headingRoad << std::endl;
    RearObject_.RoadStates[3].latAccelerationRoad = 124.30;
    std::cout << "RearObject_.RoadStates[3].latAccelerationRoad(float32): " << RearObject_.RoadStates[3].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[3].latPositionRoad = 125.40;
    std::cout << "RearObject_.RoadStates[3].latPositionRoad(float32): " << RearObject_.RoadStates[3].latPositionRoad << std::endl;
    RearObject_.RoadStates[3].latVelocityRoad = 126.50;
    std::cout << "RearObject_.RoadStates[3].latVelocityRoad(float32): " << RearObject_.RoadStates[3].latVelocityRoad << std::endl;
    RearObject_.RoadStates[3].longAccelerationRoad = 127.60;
    std::cout << "RearObject_.RoadStates[3].longAccelerationRoad(float32): " << RearObject_.RoadStates[3].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[3].longPositionRoad = 128.70;
    std::cout << "RearObject_.RoadStates[3].longPositionRoad(float32): " << RearObject_.RoadStates[3].longPositionRoad << std::endl;
    RearObject_.RoadStates[3].longVelocityRoad = 129.80;
    std::cout << "RearObject_.RoadStates[3].longVelocityRoad(float32): " << RearObject_.RoadStates[3].longVelocityRoad << std::endl;
    RearObject_.RoadStates[4].headingRoad = 130.90;
    std::cout << "RearObject_.RoadStates[4].headingRoad(float32): " << RearObject_.RoadStates[4].headingRoad << std::endl;
    RearObject_.RoadStates[4].latAccelerationRoad = 132.00;
    std::cout << "RearObject_.RoadStates[4].latAccelerationRoad(float32): " << RearObject_.RoadStates[4].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[4].latPositionRoad = 133.10;
    std::cout << "RearObject_.RoadStates[4].latPositionRoad(float32): " << RearObject_.RoadStates[4].latPositionRoad << std::endl;
    RearObject_.RoadStates[4].latVelocityRoad = 134.20;
    std::cout << "RearObject_.RoadStates[4].latVelocityRoad(float32): " << RearObject_.RoadStates[4].latVelocityRoad << std::endl;
    RearObject_.RoadStates[4].longAccelerationRoad = 135.30;
    std::cout << "RearObject_.RoadStates[4].longAccelerationRoad(float32): " << RearObject_.RoadStates[4].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[4].longPositionRoad = 136.40;
    std::cout << "RearObject_.RoadStates[4].longPositionRoad(float32): " << RearObject_.RoadStates[4].longPositionRoad << std::endl;
    RearObject_.RoadStates[4].longVelocityRoad = 137.50;
    std::cout << "RearObject_.RoadStates[4].longVelocityRoad(float32): " << RearObject_.RoadStates[4].longVelocityRoad << std::endl;
    RearObject_.RoadStates[5].headingRoad = 138.60;
    std::cout << "RearObject_.RoadStates[5].headingRoad(float32): " << RearObject_.RoadStates[5].headingRoad << std::endl;
    RearObject_.RoadStates[5].latAccelerationRoad = 139.70;
    std::cout << "RearObject_.RoadStates[5].latAccelerationRoad(float32): " << RearObject_.RoadStates[5].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[5].latPositionRoad = 140.80;
    std::cout << "RearObject_.RoadStates[5].latPositionRoad(float32): " << RearObject_.RoadStates[5].latPositionRoad << std::endl;
    RearObject_.RoadStates[5].latVelocityRoad = 141.90;
    std::cout << "RearObject_.RoadStates[5].latVelocityRoad(float32): " << RearObject_.RoadStates[5].latVelocityRoad << std::endl;
    RearObject_.RoadStates[5].longAccelerationRoad = 143.00;
    std::cout << "RearObject_.RoadStates[5].longAccelerationRoad(float32): " << RearObject_.RoadStates[5].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[5].longPositionRoad = 144.10;
    std::cout << "RearObject_.RoadStates[5].longPositionRoad(float32): " << RearObject_.RoadStates[5].longPositionRoad << std::endl;
    RearObject_.RoadStates[5].longVelocityRoad = 145.20;
    std::cout << "RearObject_.RoadStates[5].longVelocityRoad(float32): " << RearObject_.RoadStates[5].longVelocityRoad << std::endl;
    RearObject_.RoadStates[6].headingRoad = 146.30;
    std::cout << "RearObject_.RoadStates[6].headingRoad(float32): " << RearObject_.RoadStates[6].headingRoad << std::endl;
    RearObject_.RoadStates[6].latAccelerationRoad = 147.40;
    std::cout << "RearObject_.RoadStates[6].latAccelerationRoad(float32): " << RearObject_.RoadStates[6].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[6].latPositionRoad = 148.50;
    std::cout << "RearObject_.RoadStates[6].latPositionRoad(float32): " << RearObject_.RoadStates[6].latPositionRoad << std::endl;
    RearObject_.RoadStates[6].latVelocityRoad = 149.60;
    std::cout << "RearObject_.RoadStates[6].latVelocityRoad(float32): " << RearObject_.RoadStates[6].latVelocityRoad << std::endl;
    RearObject_.RoadStates[6].longAccelerationRoad = 150.70;
    std::cout << "RearObject_.RoadStates[6].longAccelerationRoad(float32): " << RearObject_.RoadStates[6].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[6].longPositionRoad = 151.80;
    std::cout << "RearObject_.RoadStates[6].longPositionRoad(float32): " << RearObject_.RoadStates[6].longPositionRoad << std::endl;
    RearObject_.RoadStates[6].longVelocityRoad = 152.90;
    std::cout << "RearObject_.RoadStates[6].longVelocityRoad(float32): " << RearObject_.RoadStates[6].longVelocityRoad << std::endl;
    RearObject_.RoadStates[7].headingRoad = 154.00;
    std::cout << "RearObject_.RoadStates[7].headingRoad(float32): " << RearObject_.RoadStates[7].headingRoad << std::endl;
    RearObject_.RoadStates[7].latAccelerationRoad = 155.10;
    std::cout << "RearObject_.RoadStates[7].latAccelerationRoad(float32): " << RearObject_.RoadStates[7].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[7].latPositionRoad = 156.20;
    std::cout << "RearObject_.RoadStates[7].latPositionRoad(float32): " << RearObject_.RoadStates[7].latPositionRoad << std::endl;
    RearObject_.RoadStates[7].latVelocityRoad = 157.30;
    std::cout << "RearObject_.RoadStates[7].latVelocityRoad(float32): " << RearObject_.RoadStates[7].latVelocityRoad << std::endl;
    RearObject_.RoadStates[7].longAccelerationRoad = 158.40;
    std::cout << "RearObject_.RoadStates[7].longAccelerationRoad(float32): " << RearObject_.RoadStates[7].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[7].longPositionRoad = 159.50;
    std::cout << "RearObject_.RoadStates[7].longPositionRoad(float32): " << RearObject_.RoadStates[7].longPositionRoad << std::endl;
    RearObject_.RoadStates[7].longVelocityRoad = 160.60;
    std::cout << "RearObject_.RoadStates[7].longVelocityRoad(float32): " << RearObject_.RoadStates[7].longVelocityRoad << std::endl;
    RearObject_.RoadStates[8].headingRoad = 161.70;
    std::cout << "RearObject_.RoadStates[8].headingRoad(float32): " << RearObject_.RoadStates[8].headingRoad << std::endl;
    RearObject_.RoadStates[8].latAccelerationRoad = 162.80;
    std::cout << "RearObject_.RoadStates[8].latAccelerationRoad(float32): " << RearObject_.RoadStates[8].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[8].latPositionRoad = 163.90;
    std::cout << "RearObject_.RoadStates[8].latPositionRoad(float32): " << RearObject_.RoadStates[8].latPositionRoad << std::endl;
    RearObject_.RoadStates[8].latVelocityRoad = 165.00;
    std::cout << "RearObject_.RoadStates[8].latVelocityRoad(float32): " << RearObject_.RoadStates[8].latVelocityRoad << std::endl;
    RearObject_.RoadStates[8].longAccelerationRoad = 166.10;
    std::cout << "RearObject_.RoadStates[8].longAccelerationRoad(float32): " << RearObject_.RoadStates[8].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[8].longPositionRoad = 167.20;
    std::cout << "RearObject_.RoadStates[8].longPositionRoad(float32): " << RearObject_.RoadStates[8].longPositionRoad << std::endl;
    RearObject_.RoadStates[8].longVelocityRoad = 168.30;
    std::cout << "RearObject_.RoadStates[8].longVelocityRoad(float32): " << RearObject_.RoadStates[8].longVelocityRoad << std::endl;
    RearObject_.RoadStates[9].headingRoad = 169.40;
    std::cout << "RearObject_.RoadStates[9].headingRoad(float32): " << RearObject_.RoadStates[9].headingRoad << std::endl;
    RearObject_.RoadStates[9].latAccelerationRoad = 170.50;
    std::cout << "RearObject_.RoadStates[9].latAccelerationRoad(float32): " << RearObject_.RoadStates[9].latAccelerationRoad << std::endl;
    RearObject_.RoadStates[9].latPositionRoad = 171.60;
    std::cout << "RearObject_.RoadStates[9].latPositionRoad(float32): " << RearObject_.RoadStates[9].latPositionRoad << std::endl;
    RearObject_.RoadStates[9].latVelocityRoad = 172.70;
    std::cout << "RearObject_.RoadStates[9].latVelocityRoad(float32): " << RearObject_.RoadStates[9].latVelocityRoad << std::endl;
    RearObject_.RoadStates[9].longAccelerationRoad = 173.80;
    std::cout << "RearObject_.RoadStates[9].longAccelerationRoad(float32): " << RearObject_.RoadStates[9].longAccelerationRoad << std::endl;
    RearObject_.RoadStates[9].longPositionRoad = 174.90;
    std::cout << "RearObject_.RoadStates[9].longPositionRoad(float32): " << RearObject_.RoadStates[9].longPositionRoad << std::endl;
    RearObject_.RoadStates[9].longVelocityRoad = 176.00;
    std::cout << "RearObject_.RoadStates[9].longVelocityRoad(float32): " << RearObject_.RoadStates[9].longVelocityRoad << std::endl;
    RearObject_.Properties[0].latPositionConfidence = 1;
    std::cout << "RearObject_.Properties[0].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[0].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[0].length = 177.10;
    std::cout << "RearObject_.Properties[0].length(float32): " << RearObject_.Properties[0].length << std::endl;
    RearObject_.Properties[0].confidence = 2;
    std::cout << "RearObject_.Properties[0].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[0].confidence) << std::dec  << std::endl;
    RearObject_.Properties[0].type = 3;
    std::cout << "RearObject_.Properties[0].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[0].type) << std::dec  << std::endl;
    RearObject_.Properties[0].width = 178.20;
    std::cout << "RearObject_.Properties[0].width(float32): " << RearObject_.Properties[0].width << std::endl;
    RearObject_.Properties[0].id = 4;
    std::cout << "RearObject_.Properties[0].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[0].id) << std::dec  << std::endl;
    RearObject_.Properties[0].trackStatus = 5;
    std::cout << "RearObject_.Properties[0].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[0].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[1].latPositionConfidence = 6;
    std::cout << "RearObject_.Properties[1].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[1].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[1].length = 179.30;
    std::cout << "RearObject_.Properties[1].length(float32): " << RearObject_.Properties[1].length << std::endl;
    RearObject_.Properties[1].confidence = 7;
    std::cout << "RearObject_.Properties[1].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[1].confidence) << std::dec  << std::endl;
    RearObject_.Properties[1].type = 8;
    std::cout << "RearObject_.Properties[1].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[1].type) << std::dec  << std::endl;
    RearObject_.Properties[1].width = 180.40;
    std::cout << "RearObject_.Properties[1].width(float32): " << RearObject_.Properties[1].width << std::endl;
    RearObject_.Properties[1].id = 9;
    std::cout << "RearObject_.Properties[1].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[1].id) << std::dec  << std::endl;
    RearObject_.Properties[1].trackStatus = 10;
    std::cout << "RearObject_.Properties[1].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[1].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[2].latPositionConfidence = 11;
    std::cout << "RearObject_.Properties[2].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[2].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[2].length = 181.50;
    std::cout << "RearObject_.Properties[2].length(float32): " << RearObject_.Properties[2].length << std::endl;
    RearObject_.Properties[2].confidence = 12;
    std::cout << "RearObject_.Properties[2].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[2].confidence) << std::dec  << std::endl;
    RearObject_.Properties[2].type = 13;
    std::cout << "RearObject_.Properties[2].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[2].type) << std::dec  << std::endl;
    RearObject_.Properties[2].width = 182.60;
    std::cout << "RearObject_.Properties[2].width(float32): " << RearObject_.Properties[2].width << std::endl;
    RearObject_.Properties[2].id = 14;
    std::cout << "RearObject_.Properties[2].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[2].id) << std::dec  << std::endl;
    RearObject_.Properties[2].trackStatus = 15;
    std::cout << "RearObject_.Properties[2].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[2].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[3].latPositionConfidence = 16;
    std::cout << "RearObject_.Properties[3].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[3].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[3].length = 183.70;
    std::cout << "RearObject_.Properties[3].length(float32): " << RearObject_.Properties[3].length << std::endl;
    RearObject_.Properties[3].confidence = 17;
    std::cout << "RearObject_.Properties[3].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[3].confidence) << std::dec  << std::endl;
    RearObject_.Properties[3].type = 18;
    std::cout << "RearObject_.Properties[3].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[3].type) << std::dec  << std::endl;
    RearObject_.Properties[3].width = 184.80;
    std::cout << "RearObject_.Properties[3].width(float32): " << RearObject_.Properties[3].width << std::endl;
    RearObject_.Properties[3].id = 19;
    std::cout << "RearObject_.Properties[3].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[3].id) << std::dec  << std::endl;
    RearObject_.Properties[3].trackStatus = 20;
    std::cout << "RearObject_.Properties[3].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[3].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[4].latPositionConfidence = 21;
    std::cout << "RearObject_.Properties[4].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[4].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[4].length = 185.90;
    std::cout << "RearObject_.Properties[4].length(float32): " << RearObject_.Properties[4].length << std::endl;
    RearObject_.Properties[4].confidence = 22;
    std::cout << "RearObject_.Properties[4].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[4].confidence) << std::dec  << std::endl;
    RearObject_.Properties[4].type = 23;
    std::cout << "RearObject_.Properties[4].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[4].type) << std::dec  << std::endl;
    RearObject_.Properties[4].width = 187.00;
    std::cout << "RearObject_.Properties[4].width(float32): " << RearObject_.Properties[4].width << std::endl;
    RearObject_.Properties[4].id = 24;
    std::cout << "RearObject_.Properties[4].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[4].id) << std::dec  << std::endl;
    RearObject_.Properties[4].trackStatus = 25;
    std::cout << "RearObject_.Properties[4].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[4].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[5].latPositionConfidence = 26;
    std::cout << "RearObject_.Properties[5].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[5].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[5].length = 188.10;
    std::cout << "RearObject_.Properties[5].length(float32): " << RearObject_.Properties[5].length << std::endl;
    RearObject_.Properties[5].confidence = 27;
    std::cout << "RearObject_.Properties[5].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[5].confidence) << std::dec  << std::endl;
    RearObject_.Properties[5].type = 28;
    std::cout << "RearObject_.Properties[5].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[5].type) << std::dec  << std::endl;
    RearObject_.Properties[5].width = 189.20;
    std::cout << "RearObject_.Properties[5].width(float32): " << RearObject_.Properties[5].width << std::endl;
    RearObject_.Properties[5].id = 29;
    std::cout << "RearObject_.Properties[5].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[5].id) << std::dec  << std::endl;
    RearObject_.Properties[5].trackStatus = 30;
    std::cout << "RearObject_.Properties[5].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[5].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[6].latPositionConfidence = 31;
    std::cout << "RearObject_.Properties[6].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[6].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[6].length = 190.30;
    std::cout << "RearObject_.Properties[6].length(float32): " << RearObject_.Properties[6].length << std::endl;
    RearObject_.Properties[6].confidence = 32;
    std::cout << "RearObject_.Properties[6].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[6].confidence) << std::dec  << std::endl;
    RearObject_.Properties[6].type = 33;
    std::cout << "RearObject_.Properties[6].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[6].type) << std::dec  << std::endl;
    RearObject_.Properties[6].width = 191.40;
    std::cout << "RearObject_.Properties[6].width(float32): " << RearObject_.Properties[6].width << std::endl;
    RearObject_.Properties[6].id = 34;
    std::cout << "RearObject_.Properties[6].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[6].id) << std::dec  << std::endl;
    RearObject_.Properties[6].trackStatus = 35;
    std::cout << "RearObject_.Properties[6].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[6].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[7].latPositionConfidence = 36;
    std::cout << "RearObject_.Properties[7].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[7].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[7].length = 192.50;
    std::cout << "RearObject_.Properties[7].length(float32): " << RearObject_.Properties[7].length << std::endl;
    RearObject_.Properties[7].confidence = 37;
    std::cout << "RearObject_.Properties[7].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[7].confidence) << std::dec  << std::endl;
    RearObject_.Properties[7].type = 38;
    std::cout << "RearObject_.Properties[7].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[7].type) << std::dec  << std::endl;
    RearObject_.Properties[7].width = 193.60;
    std::cout << "RearObject_.Properties[7].width(float32): " << RearObject_.Properties[7].width << std::endl;
    RearObject_.Properties[7].id = 39;
    std::cout << "RearObject_.Properties[7].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[7].id) << std::dec  << std::endl;
    RearObject_.Properties[7].trackStatus = 40;
    std::cout << "RearObject_.Properties[7].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[7].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[8].latPositionConfidence = 41;
    std::cout << "RearObject_.Properties[8].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[8].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[8].length = 194.70;
    std::cout << "RearObject_.Properties[8].length(float32): " << RearObject_.Properties[8].length << std::endl;
    RearObject_.Properties[8].confidence = 42;
    std::cout << "RearObject_.Properties[8].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[8].confidence) << std::dec  << std::endl;
    RearObject_.Properties[8].type = 43;
    std::cout << "RearObject_.Properties[8].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[8].type) << std::dec  << std::endl;
    RearObject_.Properties[8].width = 195.80;
    std::cout << "RearObject_.Properties[8].width(float32): " << RearObject_.Properties[8].width << std::endl;
    RearObject_.Properties[8].id = 44;
    std::cout << "RearObject_.Properties[8].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[8].id) << std::dec  << std::endl;
    RearObject_.Properties[8].trackStatus = 45;
    std::cout << "RearObject_.Properties[8].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[8].trackStatus) << std::dec  << std::endl;
    RearObject_.Properties[9].latPositionConfidence = 46;
    std::cout << "RearObject_.Properties[9].latPositionConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[9].latPositionConfidence) << std::dec  << std::endl;
    RearObject_.Properties[9].length = 196.90;
    std::cout << "RearObject_.Properties[9].length(float32): " << RearObject_.Properties[9].length << std::endl;
    RearObject_.Properties[9].confidence = 47;
    std::cout << "RearObject_.Properties[9].confidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[9].confidence) << std::dec  << std::endl;
    RearObject_.Properties[9].type = 48;
    std::cout << "RearObject_.Properties[9].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[9].type) << std::dec  << std::endl;
    RearObject_.Properties[9].width = 198.00;
    std::cout << "RearObject_.Properties[9].width(float32): " << RearObject_.Properties[9].width << std::endl;
    RearObject_.Properties[9].id = 49;
    std::cout << "RearObject_.Properties[9].id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[9].id) << std::dec  << std::endl;
    RearObject_.Properties[9].trackStatus = 50;
    std::cout << "RearObject_.Properties[9].trackStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RearObject_.Properties[9].trackStatus) << std::dec  << std::endl;
    RearObject_.SequenceID = 1;
    std::cout << "RearObject_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(RearObject_.SequenceID) << std::dec  << std::endl;
};
#endif



#ifdef FLCROADCOVER_H
/* Set and Print struct FlcRoadCover initial value */
void setIntialValue_FlcRoadCover(FlcRoadCover& FlcRoadCover_){
    std::cout << "Set struct FlcRoadCover variable and Publish:" << std::endl;
    FlcRoadCover_.Snow.isDetected = 1;
    std::cout << "FlcRoadCover_.Snow.isDetected(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FlcRoadCover_.Snow.isDetected) << std::dec  << std::endl;
    FlcRoadCover_.Snow.confidence = 1.10;
    std::cout << "FlcRoadCover_.Snow.confidence(float32): " << FlcRoadCover_.Snow.confidence << std::endl;
    FlcRoadCover_.Gravel.isDetected = 2;
    std::cout << "FlcRoadCover_.Gravel.isDetected(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FlcRoadCover_.Gravel.isDetected) << std::dec  << std::endl;
    FlcRoadCover_.Gravel.confidence = 2.20;
    std::cout << "FlcRoadCover_.Gravel.confidence(float32): " << FlcRoadCover_.Gravel.confidence << std::endl;
    FlcRoadCover_.Wet.isDetected = 3;
    std::cout << "FlcRoadCover_.Wet.isDetected(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(FlcRoadCover_.Wet.isDetected) << std::dec  << std::endl;
    FlcRoadCover_.Wet.confidence = 3.30;
    std::cout << "FlcRoadCover_.Wet.confidence(float32): " << FlcRoadCover_.Wet.confidence << std::endl;
};
#endif



#ifdef ROADEDGE_H
/* Set and Print struct RoadEdge initial value */
void setIntialValue_RoadEdge(RoadEdge& RoadEdge_){
    std::cout << "Set struct RoadEdge variable and Publish:" << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.id = 1;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.measurementQuality = 1.10;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.LeftNonTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.modelError = 2.20;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.modelError(float32): " << RoadEdge_.LeftNonTrvsble.Edge.modelError << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a = 3.30;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance = 4.40;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b = 5.50;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance = 6.60;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c = 7.70;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance = 8.80;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d = 9.90;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance = 11.00;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance = 12.10;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd = 13.20;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart = 14.30;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset = 15.40;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading = 16.50;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt = 17.60;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate = 18.70;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.LeftNonTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.trackingStatus = 2;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.valid = 3;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.isVerified = 4;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftNonTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence = 19.80;
    std::cout << "RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.LeftNonTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.id = 5;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.measurementQuality = 20.90;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.RightNonTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.modelError = 22.00;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.modelError(float32): " << RoadEdge_.RightNonTrvsble.Edge.modelError << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a = 23.10;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance = 24.20;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b = 25.30;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance = 26.40;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c = 27.50;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance = 28.60;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d = 29.70;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance = 30.80;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance = 31.90;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd = 33.00;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart = 34.10;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset = 35.20;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading = 36.30;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt = 37.40;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate = 38.50;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.RightNonTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.trackingStatus = 6;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.valid = 7;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.isVerified = 8;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightNonTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.RightNonTrvsble.Edge.selectionConfidence = 39.60;
    std::cout << "RoadEdge_.RightNonTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.RightNonTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.LeftTrvsble.Edge.id = 9;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.measurementQuality = 40.70;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.LeftTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.LeftTrvsble.Edge.modelError = 41.80;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.modelError(float32): " << RoadEdge_.LeftTrvsble.Edge.modelError << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.a = 42.90;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance = 44.00;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.b = 45.10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance = 46.20;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.c = 47.30;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance = 48.40;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d = 49.50;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance = 50.60;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance = 51.70;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd = 52.80;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart = 53.90;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset = 55.00;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading = 56.10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt = 57.20;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate = 58.30;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.LeftTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.LeftTrvsble.Edge.trackingStatus = 10;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.valid = 11;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.isVerified = 12;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.LeftTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.LeftTrvsble.Edge.selectionConfidence = 59.40;
    std::cout << "RoadEdge_.LeftTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.LeftTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.RightTrvsble.Edge.id = 13;
    std::cout << "RoadEdge_.RightTrvsble.Edge.id(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.id) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.measurementQuality = 60.50;
    std::cout << "RoadEdge_.RightTrvsble.Edge.measurementQuality(float32): " << RoadEdge_.RightTrvsble.Edge.measurementQuality << std::endl;
    RoadEdge_.RightTrvsble.Edge.modelError = 61.60;
    std::cout << "RoadEdge_.RightTrvsble.Edge.modelError(float32): " << RoadEdge_.RightTrvsble.Edge.modelError << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.a = 62.70;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.a(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.a << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance = 63.80;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.aVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.b = 64.90;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.b(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.b << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance = 66.00;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.bVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.c = 67.10;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.c(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.c << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance = 68.20;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.cVariance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d = 69.30;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance = 70.40;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d1Variance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance = 71.50;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.d2Variance << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd = 72.60;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToEnd << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart = 73.70;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.longDistToStart << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset = 74.80;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtOffset << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading = 75.90;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtHeading << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt = 77.00;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvt << std::endl;
    RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate = 78.10;
    std::cout << "RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate(float32): " << RoadEdge_.RightTrvsble.Edge.SingClothoid.vrtCrvtRate << std::endl;
    RoadEdge_.RightTrvsble.Edge.trackingStatus = 14;
    std::cout << "RoadEdge_.RightTrvsble.Edge.trackingStatus(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.trackingStatus) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.valid = 15;
    std::cout << "RoadEdge_.RightTrvsble.Edge.valid(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.valid) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.isVerified = 16;
    std::cout << "RoadEdge_.RightTrvsble.Edge.isVerified(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadEdge_.RightTrvsble.Edge.isVerified) << std::dec  << std::endl;
    RoadEdge_.RightTrvsble.Edge.selectionConfidence = 79.20;
    std::cout << "RoadEdge_.RightTrvsble.Edge.selectionConfidence(float32): " << RoadEdge_.RightTrvsble.Edge.selectionConfidence << std::endl;
    RoadEdge_.SequenceID = 1;
    std::cout << "RoadEdge_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(RoadEdge_.SequenceID) << std::dec  << std::endl;
};
#endif



#ifdef CROSSINGOBJECT_H
/* Set and Print struct CrossingObject initial value */
void setIntialValue_CrossingObject(CrossingObject& CrossingObject_){
    std::cout << "Set struct CrossingObject variable and Publish:" << std::endl;
    CrossingObject_.EgoStates[0].latAcceleration = 1.10;
    std::cout << "CrossingObject_.EgoStates[0].latAcceleration(float32): " << CrossingObject_.EgoStates[0].latAcceleration << std::endl;
    CrossingObject_.EgoStates[0].longAcceleration = 2.20;
    std::cout << "CrossingObject_.EgoStates[0].longAcceleration(float32): " << CrossingObject_.EgoStates[0].longAcceleration << std::endl;
    CrossingObject_.EgoStates[0].latPosition = 3.30;
    std::cout << "CrossingObject_.EgoStates[0].latPosition(float32): " << CrossingObject_.EgoStates[0].latPosition << std::endl;
    CrossingObject_.EgoStates[0].longPosition = 4.40;
    std::cout << "CrossingObject_.EgoStates[0].longPosition(float32): " << CrossingObject_.EgoStates[0].longPosition << std::endl;
    CrossingObject_.EgoStates[0].latVelocity = 5.50;
    std::cout << "CrossingObject_.EgoStates[0].latVelocity(float32): " << CrossingObject_.EgoStates[0].latVelocity << std::endl;
    CrossingObject_.EgoStates[0].longVelocity = 6.60;
    std::cout << "CrossingObject_.EgoStates[0].longVelocity(float32): " << CrossingObject_.EgoStates[0].longVelocity << std::endl;
    CrossingObject_.EgoStates[1].latAcceleration = 7.70;
    std::cout << "CrossingObject_.EgoStates[1].latAcceleration(float32): " << CrossingObject_.EgoStates[1].latAcceleration << std::endl;
    CrossingObject_.EgoStates[1].longAcceleration = 8.80;
    std::cout << "CrossingObject_.EgoStates[1].longAcceleration(float32): " << CrossingObject_.EgoStates[1].longAcceleration << std::endl;
    CrossingObject_.EgoStates[1].latPosition = 9.90;
    std::cout << "CrossingObject_.EgoStates[1].latPosition(float32): " << CrossingObject_.EgoStates[1].latPosition << std::endl;
    CrossingObject_.EgoStates[1].longPosition = 11.00;
    std::cout << "CrossingObject_.EgoStates[1].longPosition(float32): " << CrossingObject_.EgoStates[1].longPosition << std::endl;
    CrossingObject_.EgoStates[1].latVelocity = 12.10;
    std::cout << "CrossingObject_.EgoStates[1].latVelocity(float32): " << CrossingObject_.EgoStates[1].latVelocity << std::endl;
    CrossingObject_.EgoStates[1].longVelocity = 13.20;
    std::cout << "CrossingObject_.EgoStates[1].longVelocity(float32): " << CrossingObject_.EgoStates[1].longVelocity << std::endl;
    CrossingObject_.EgoStates[2].latAcceleration = 14.30;
    std::cout << "CrossingObject_.EgoStates[2].latAcceleration(float32): " << CrossingObject_.EgoStates[2].latAcceleration << std::endl;
    CrossingObject_.EgoStates[2].longAcceleration = 15.40;
    std::cout << "CrossingObject_.EgoStates[2].longAcceleration(float32): " << CrossingObject_.EgoStates[2].longAcceleration << std::endl;
    CrossingObject_.EgoStates[2].latPosition = 16.50;
    std::cout << "CrossingObject_.EgoStates[2].latPosition(float32): " << CrossingObject_.EgoStates[2].latPosition << std::endl;
    CrossingObject_.EgoStates[2].longPosition = 17.60;
    std::cout << "CrossingObject_.EgoStates[2].longPosition(float32): " << CrossingObject_.EgoStates[2].longPosition << std::endl;
    CrossingObject_.EgoStates[2].latVelocity = 18.70;
    std::cout << "CrossingObject_.EgoStates[2].latVelocity(float32): " << CrossingObject_.EgoStates[2].latVelocity << std::endl;
    CrossingObject_.EgoStates[2].longVelocity = 19.80;
    std::cout << "CrossingObject_.EgoStates[2].longVelocity(float32): " << CrossingObject_.EgoStates[2].longVelocity << std::endl;
    CrossingObject_.EgoStates[3].latAcceleration = 20.90;
    std::cout << "CrossingObject_.EgoStates[3].latAcceleration(float32): " << CrossingObject_.EgoStates[3].latAcceleration << std::endl;
    CrossingObject_.EgoStates[3].longAcceleration = 22.00;
    std::cout << "CrossingObject_.EgoStates[3].longAcceleration(float32): " << CrossingObject_.EgoStates[3].longAcceleration << std::endl;
    CrossingObject_.EgoStates[3].latPosition = 23.10;
    std::cout << "CrossingObject_.EgoStates[3].latPosition(float32): " << CrossingObject_.EgoStates[3].latPosition << std::endl;
    CrossingObject_.EgoStates[3].longPosition = 24.20;
    std::cout << "CrossingObject_.EgoStates[3].longPosition(float32): " << CrossingObject_.EgoStates[3].longPosition << std::endl;
    CrossingObject_.EgoStates[3].latVelocity = 25.30;
    std::cout << "CrossingObject_.EgoStates[3].latVelocity(float32): " << CrossingObject_.EgoStates[3].latVelocity << std::endl;
    CrossingObject_.EgoStates[3].longVelocity = 26.40;
    std::cout << "CrossingObject_.EgoStates[3].longVelocity(float32): " << CrossingObject_.EgoStates[3].longVelocity << std::endl;
    CrossingObject_.EgoStates[4].latAcceleration = 27.50;
    std::cout << "CrossingObject_.EgoStates[4].latAcceleration(float32): " << CrossingObject_.EgoStates[4].latAcceleration << std::endl;
    CrossingObject_.EgoStates[4].longAcceleration = 28.60;
    std::cout << "CrossingObject_.EgoStates[4].longAcceleration(float32): " << CrossingObject_.EgoStates[4].longAcceleration << std::endl;
    CrossingObject_.EgoStates[4].latPosition = 29.70;
    std::cout << "CrossingObject_.EgoStates[4].latPosition(float32): " << CrossingObject_.EgoStates[4].latPosition << std::endl;
    CrossingObject_.EgoStates[4].longPosition = 30.80;
    std::cout << "CrossingObject_.EgoStates[4].longPosition(float32): " << CrossingObject_.EgoStates[4].longPosition << std::endl;
    CrossingObject_.EgoStates[4].latVelocity = 31.90;
    std::cout << "CrossingObject_.EgoStates[4].latVelocity(float32): " << CrossingObject_.EgoStates[4].latVelocity << std::endl;
    CrossingObject_.EgoStates[4].longVelocity = 33.00;
    std::cout << "CrossingObject_.EgoStates[4].longVelocity(float32): " << CrossingObject_.EgoStates[4].longVelocity << std::endl;
    CrossingObject_.EgoStates[5].latAcceleration = 34.10;
    std::cout << "CrossingObject_.EgoStates[5].latAcceleration(float32): " << CrossingObject_.EgoStates[5].latAcceleration << std::endl;
    CrossingObject_.EgoStates[5].longAcceleration = 35.20;
    std::cout << "CrossingObject_.EgoStates[5].longAcceleration(float32): " << CrossingObject_.EgoStates[5].longAcceleration << std::endl;
    CrossingObject_.EgoStates[5].latPosition = 36.30;
    std::cout << "CrossingObject_.EgoStates[5].latPosition(float32): " << CrossingObject_.EgoStates[5].latPosition << std::endl;
    CrossingObject_.EgoStates[5].longPosition = 37.40;
    std::cout << "CrossingObject_.EgoStates[5].longPosition(float32): " << CrossingObject_.EgoStates[5].longPosition << std::endl;
    CrossingObject_.EgoStates[5].latVelocity = 38.50;
    std::cout << "CrossingObject_.EgoStates[5].latVelocity(float32): " << CrossingObject_.EgoStates[5].latVelocity << std::endl;
    CrossingObject_.EgoStates[5].longVelocity = 39.60;
    std::cout << "CrossingObject_.EgoStates[5].longVelocity(float32): " << CrossingObject_.EgoStates[5].longVelocity << std::endl;
    CrossingObject_.EgoStates[6].latAcceleration = 40.70;
    std::cout << "CrossingObject_.EgoStates[6].latAcceleration(float32): " << CrossingObject_.EgoStates[6].latAcceleration << std::endl;
    CrossingObject_.EgoStates[6].longAcceleration = 41.80;
    std::cout << "CrossingObject_.EgoStates[6].longAcceleration(float32): " << CrossingObject_.EgoStates[6].longAcceleration << std::endl;
    CrossingObject_.EgoStates[6].latPosition = 42.90;
    std::cout << "CrossingObject_.EgoStates[6].latPosition(float32): " << CrossingObject_.EgoStates[6].latPosition << std::endl;
    CrossingObject_.EgoStates[6].longPosition = 44.00;
    std::cout << "CrossingObject_.EgoStates[6].longPosition(float32): " << CrossingObject_.EgoStates[6].longPosition << std::endl;
    CrossingObject_.EgoStates[6].latVelocity = 45.10;
    std::cout << "CrossingObject_.EgoStates[6].latVelocity(float32): " << CrossingObject_.EgoStates[6].latVelocity << std::endl;
    CrossingObject_.EgoStates[6].longVelocity = 46.20;
    std::cout << "CrossingObject_.EgoStates[6].longVelocity(float32): " << CrossingObject_.EgoStates[6].longVelocity << std::endl;
    CrossingObject_.EgoStates[7].latAcceleration = 47.30;
    std::cout << "CrossingObject_.EgoStates[7].latAcceleration(float32): " << CrossingObject_.EgoStates[7].latAcceleration << std::endl;
    CrossingObject_.EgoStates[7].longAcceleration = 48.40;
    std::cout << "CrossingObject_.EgoStates[7].longAcceleration(float32): " << CrossingObject_.EgoStates[7].longAcceleration << std::endl;
    CrossingObject_.EgoStates[7].latPosition = 49.50;
    std::cout << "CrossingObject_.EgoStates[7].latPosition(float32): " << CrossingObject_.EgoStates[7].latPosition << std::endl;
    CrossingObject_.EgoStates[7].longPosition = 50.60;
    std::cout << "CrossingObject_.EgoStates[7].longPosition(float32): " << CrossingObject_.EgoStates[7].longPosition << std::endl;
    CrossingObject_.EgoStates[7].latVelocity = 51.70;
    std::cout << "CrossingObject_.EgoStates[7].latVelocity(float32): " << CrossingObject_.EgoStates[7].latVelocity << std::endl;
    CrossingObject_.EgoStates[7].longVelocity = 52.80;
    std::cout << "CrossingObject_.EgoStates[7].longVelocity(float32): " << CrossingObject_.EgoStates[7].longVelocity << std::endl;
    CrossingObject_.EgoStates[8].latAcceleration = 53.90;
    std::cout << "CrossingObject_.EgoStates[8].latAcceleration(float32): " << CrossingObject_.EgoStates[8].latAcceleration << std::endl;
    CrossingObject_.EgoStates[8].longAcceleration = 55.00;
    std::cout << "CrossingObject_.EgoStates[8].longAcceleration(float32): " << CrossingObject_.EgoStates[8].longAcceleration << std::endl;
    CrossingObject_.EgoStates[8].latPosition = 56.10;
    std::cout << "CrossingObject_.EgoStates[8].latPosition(float32): " << CrossingObject_.EgoStates[8].latPosition << std::endl;
    CrossingObject_.EgoStates[8].longPosition = 57.20;
    std::cout << "CrossingObject_.EgoStates[8].longPosition(float32): " << CrossingObject_.EgoStates[8].longPosition << std::endl;
    CrossingObject_.EgoStates[8].latVelocity = 58.30;
    std::cout << "CrossingObject_.EgoStates[8].latVelocity(float32): " << CrossingObject_.EgoStates[8].latVelocity << std::endl;
    CrossingObject_.EgoStates[8].longVelocity = 59.40;
    std::cout << "CrossingObject_.EgoStates[8].longVelocity(float32): " << CrossingObject_.EgoStates[8].longVelocity << std::endl;
    CrossingObject_.EgoStates[9].latAcceleration = 60.50;
    std::cout << "CrossingObject_.EgoStates[9].latAcceleration(float32): " << CrossingObject_.EgoStates[9].latAcceleration << std::endl;
    CrossingObject_.EgoStates[9].longAcceleration = 61.60;
    std::cout << "CrossingObject_.EgoStates[9].longAcceleration(float32): " << CrossingObject_.EgoStates[9].longAcceleration << std::endl;
    CrossingObject_.EgoStates[9].latPosition = 62.70;
    std::cout << "CrossingObject_.EgoStates[9].latPosition(float32): " << CrossingObject_.EgoStates[9].latPosition << std::endl;
    CrossingObject_.EgoStates[9].longPosition = 63.80;
    std::cout << "CrossingObject_.EgoStates[9].longPosition(float32): " << CrossingObject_.EgoStates[9].longPosition << std::endl;
    CrossingObject_.EgoStates[9].latVelocity = 64.90;
    std::cout << "CrossingObject_.EgoStates[9].latVelocity(float32): " << CrossingObject_.EgoStates[9].latVelocity << std::endl;
    CrossingObject_.EgoStates[9].longVelocity = 66.00;
    std::cout << "CrossingObject_.EgoStates[9].longVelocity(float32): " << CrossingObject_.EgoStates[9].longVelocity << std::endl;
    CrossingObject_.EgoStates[10].latAcceleration = 67.10;
    std::cout << "CrossingObject_.EgoStates[10].latAcceleration(float32): " << CrossingObject_.EgoStates[10].latAcceleration << std::endl;
    CrossingObject_.EgoStates[10].longAcceleration = 68.20;
    std::cout << "CrossingObject_.EgoStates[10].longAcceleration(float32): " << CrossingObject_.EgoStates[10].longAcceleration << std::endl;
    CrossingObject_.EgoStates[10].latPosition = 69.30;
    std::cout << "CrossingObject_.EgoStates[10].latPosition(float32): " << CrossingObject_.EgoStates[10].latPosition << std::endl;
    CrossingObject_.EgoStates[10].longPosition = 70.40;
    std::cout << "CrossingObject_.EgoStates[10].longPosition(float32): " << CrossingObject_.EgoStates[10].longPosition << std::endl;
    CrossingObject_.EgoStates[10].latVelocity = 71.50;
    std::cout << "CrossingObject_.EgoStates[10].latVelocity(float32): " << CrossingObject_.EgoStates[10].latVelocity << std::endl;
    CrossingObject_.EgoStates[10].longVelocity = 72.60;
    std::cout << "CrossingObject_.EgoStates[10].longVelocity(float32): " << CrossingObject_.EgoStates[10].longVelocity << std::endl;
    CrossingObject_.EgoStates[11].latAcceleration = 73.70;
    std::cout << "CrossingObject_.EgoStates[11].latAcceleration(float32): " << CrossingObject_.EgoStates[11].latAcceleration << std::endl;
    CrossingObject_.EgoStates[11].longAcceleration = 74.80;
    std::cout << "CrossingObject_.EgoStates[11].longAcceleration(float32): " << CrossingObject_.EgoStates[11].longAcceleration << std::endl;
    CrossingObject_.EgoStates[11].latPosition = 75.90;
    std::cout << "CrossingObject_.EgoStates[11].latPosition(float32): " << CrossingObject_.EgoStates[11].latPosition << std::endl;
    CrossingObject_.EgoStates[11].longPosition = 77.00;
    std::cout << "CrossingObject_.EgoStates[11].longPosition(float32): " << CrossingObject_.EgoStates[11].longPosition << std::endl;
    CrossingObject_.EgoStates[11].latVelocity = 78.10;
    std::cout << "CrossingObject_.EgoStates[11].latVelocity(float32): " << CrossingObject_.EgoStates[11].latVelocity << std::endl;
    CrossingObject_.EgoStates[11].longVelocity = 79.20;
    std::cout << "CrossingObject_.EgoStates[11].longVelocity(float32): " << CrossingObject_.EgoStates[11].longVelocity << std::endl;
    CrossingObject_.EgoStates[12].latAcceleration = 80.30;
    std::cout << "CrossingObject_.EgoStates[12].latAcceleration(float32): " << CrossingObject_.EgoStates[12].latAcceleration << std::endl;
    CrossingObject_.EgoStates[12].longAcceleration = 81.40;
    std::cout << "CrossingObject_.EgoStates[12].longAcceleration(float32): " << CrossingObject_.EgoStates[12].longAcceleration << std::endl;
    CrossingObject_.EgoStates[12].latPosition = 82.50;
    std::cout << "CrossingObject_.EgoStates[12].latPosition(float32): " << CrossingObject_.EgoStates[12].latPosition << std::endl;
    CrossingObject_.EgoStates[12].longPosition = 83.60;
    std::cout << "CrossingObject_.EgoStates[12].longPosition(float32): " << CrossingObject_.EgoStates[12].longPosition << std::endl;
    CrossingObject_.EgoStates[12].latVelocity = 84.70;
    std::cout << "CrossingObject_.EgoStates[12].latVelocity(float32): " << CrossingObject_.EgoStates[12].latVelocity << std::endl;
    CrossingObject_.EgoStates[12].longVelocity = 85.80;
    std::cout << "CrossingObject_.EgoStates[12].longVelocity(float32): " << CrossingObject_.EgoStates[12].longVelocity << std::endl;
    CrossingObject_.EgoStates[13].latAcceleration = 86.90;
    std::cout << "CrossingObject_.EgoStates[13].latAcceleration(float32): " << CrossingObject_.EgoStates[13].latAcceleration << std::endl;
    CrossingObject_.EgoStates[13].longAcceleration = 88.00;
    std::cout << "CrossingObject_.EgoStates[13].longAcceleration(float32): " << CrossingObject_.EgoStates[13].longAcceleration << std::endl;
    CrossingObject_.EgoStates[13].latPosition = 89.10;
    std::cout << "CrossingObject_.EgoStates[13].latPosition(float32): " << CrossingObject_.EgoStates[13].latPosition << std::endl;
    CrossingObject_.EgoStates[13].longPosition = 90.20;
    std::cout << "CrossingObject_.EgoStates[13].longPosition(float32): " << CrossingObject_.EgoStates[13].longPosition << std::endl;
    CrossingObject_.EgoStates[13].latVelocity = 91.30;
    std::cout << "CrossingObject_.EgoStates[13].latVelocity(float32): " << CrossingObject_.EgoStates[13].latVelocity << std::endl;
    CrossingObject_.EgoStates[13].longVelocity = 92.40;
    std::cout << "CrossingObject_.EgoStates[13].longVelocity(float32): " << CrossingObject_.EgoStates[13].longVelocity << std::endl;
    CrossingObject_.EgoStates[14].latAcceleration = 93.50;
    std::cout << "CrossingObject_.EgoStates[14].latAcceleration(float32): " << CrossingObject_.EgoStates[14].latAcceleration << std::endl;
    CrossingObject_.EgoStates[14].longAcceleration = 94.60;
    std::cout << "CrossingObject_.EgoStates[14].longAcceleration(float32): " << CrossingObject_.EgoStates[14].longAcceleration << std::endl;
    CrossingObject_.EgoStates[14].latPosition = 95.70;
    std::cout << "CrossingObject_.EgoStates[14].latPosition(float32): " << CrossingObject_.EgoStates[14].latPosition << std::endl;
    CrossingObject_.EgoStates[14].longPosition = 96.80;
    std::cout << "CrossingObject_.EgoStates[14].longPosition(float32): " << CrossingObject_.EgoStates[14].longPosition << std::endl;
    CrossingObject_.EgoStates[14].latVelocity = 97.90;
    std::cout << "CrossingObject_.EgoStates[14].latVelocity(float32): " << CrossingObject_.EgoStates[14].latVelocity << std::endl;
    CrossingObject_.EgoStates[14].longVelocity = 99.00;
    std::cout << "CrossingObject_.EgoStates[14].longVelocity(float32): " << CrossingObject_.EgoStates[14].longVelocity << std::endl;
    CrossingObject_.EgoStates[15].latAcceleration = 100.10;
    std::cout << "CrossingObject_.EgoStates[15].latAcceleration(float32): " << CrossingObject_.EgoStates[15].latAcceleration << std::endl;
    CrossingObject_.EgoStates[15].longAcceleration = 101.20;
    std::cout << "CrossingObject_.EgoStates[15].longAcceleration(float32): " << CrossingObject_.EgoStates[15].longAcceleration << std::endl;
    CrossingObject_.EgoStates[15].latPosition = 102.30;
    std::cout << "CrossingObject_.EgoStates[15].latPosition(float32): " << CrossingObject_.EgoStates[15].latPosition << std::endl;
    CrossingObject_.EgoStates[15].longPosition = 103.40;
    std::cout << "CrossingObject_.EgoStates[15].longPosition(float32): " << CrossingObject_.EgoStates[15].longPosition << std::endl;
    CrossingObject_.EgoStates[15].latVelocity = 104.50;
    std::cout << "CrossingObject_.EgoStates[15].latVelocity(float32): " << CrossingObject_.EgoStates[15].latVelocity << std::endl;
    CrossingObject_.EgoStates[15].longVelocity = 105.60;
    std::cout << "CrossingObject_.EgoStates[15].longVelocity(float32): " << CrossingObject_.EgoStates[15].longVelocity << std::endl;
    CrossingObject_.EgoStates[16].latAcceleration = 106.70;
    std::cout << "CrossingObject_.EgoStates[16].latAcceleration(float32): " << CrossingObject_.EgoStates[16].latAcceleration << std::endl;
    CrossingObject_.EgoStates[16].longAcceleration = 107.80;
    std::cout << "CrossingObject_.EgoStates[16].longAcceleration(float32): " << CrossingObject_.EgoStates[16].longAcceleration << std::endl;
    CrossingObject_.EgoStates[16].latPosition = 108.90;
    std::cout << "CrossingObject_.EgoStates[16].latPosition(float32): " << CrossingObject_.EgoStates[16].latPosition << std::endl;
    CrossingObject_.EgoStates[16].longPosition = 110.00;
    std::cout << "CrossingObject_.EgoStates[16].longPosition(float32): " << CrossingObject_.EgoStates[16].longPosition << std::endl;
    CrossingObject_.EgoStates[16].latVelocity = 111.10;
    std::cout << "CrossingObject_.EgoStates[16].latVelocity(float32): " << CrossingObject_.EgoStates[16].latVelocity << std::endl;
    CrossingObject_.EgoStates[16].longVelocity = 112.20;
    std::cout << "CrossingObject_.EgoStates[16].longVelocity(float32): " << CrossingObject_.EgoStates[16].longVelocity << std::endl;
    CrossingObject_.EgoStates[17].latAcceleration = 113.30;
    std::cout << "CrossingObject_.EgoStates[17].latAcceleration(float32): " << CrossingObject_.EgoStates[17].latAcceleration << std::endl;
    CrossingObject_.EgoStates[17].longAcceleration = 114.40;
    std::cout << "CrossingObject_.EgoStates[17].longAcceleration(float32): " << CrossingObject_.EgoStates[17].longAcceleration << std::endl;
    CrossingObject_.EgoStates[17].latPosition = 115.50;
    std::cout << "CrossingObject_.EgoStates[17].latPosition(float32): " << CrossingObject_.EgoStates[17].latPosition << std::endl;
    CrossingObject_.EgoStates[17].longPosition = 116.60;
    std::cout << "CrossingObject_.EgoStates[17].longPosition(float32): " << CrossingObject_.EgoStates[17].longPosition << std::endl;
    CrossingObject_.EgoStates[17].latVelocity = 117.70;
    std::cout << "CrossingObject_.EgoStates[17].latVelocity(float32): " << CrossingObject_.EgoStates[17].latVelocity << std::endl;
    CrossingObject_.EgoStates[17].longVelocity = 118.80;
    std::cout << "CrossingObject_.EgoStates[17].longVelocity(float32): " << CrossingObject_.EgoStates[17].longVelocity << std::endl;
    CrossingObject_.EgoStates[18].latAcceleration = 119.90;
    std::cout << "CrossingObject_.EgoStates[18].latAcceleration(float32): " << CrossingObject_.EgoStates[18].latAcceleration << std::endl;
    CrossingObject_.EgoStates[18].longAcceleration = 121.00;
    std::cout << "CrossingObject_.EgoStates[18].longAcceleration(float32): " << CrossingObject_.EgoStates[18].longAcceleration << std::endl;
    CrossingObject_.EgoStates[18].latPosition = 122.10;
    std::cout << "CrossingObject_.EgoStates[18].latPosition(float32): " << CrossingObject_.EgoStates[18].latPosition << std::endl;
    CrossingObject_.EgoStates[18].longPosition = 123.20;
    std::cout << "CrossingObject_.EgoStates[18].longPosition(float32): " << CrossingObject_.EgoStates[18].longPosition << std::endl;
    CrossingObject_.EgoStates[18].latVelocity = 124.30;
    std::cout << "CrossingObject_.EgoStates[18].latVelocity(float32): " << CrossingObject_.EgoStates[18].latVelocity << std::endl;
    CrossingObject_.EgoStates[18].longVelocity = 125.40;
    std::cout << "CrossingObject_.EgoStates[18].longVelocity(float32): " << CrossingObject_.EgoStates[18].longVelocity << std::endl;
    CrossingObject_.EgoStates[19].latAcceleration = 126.50;
    std::cout << "CrossingObject_.EgoStates[19].latAcceleration(float32): " << CrossingObject_.EgoStates[19].latAcceleration << std::endl;
    CrossingObject_.EgoStates[19].longAcceleration = 127.60;
    std::cout << "CrossingObject_.EgoStates[19].longAcceleration(float32): " << CrossingObject_.EgoStates[19].longAcceleration << std::endl;
    CrossingObject_.EgoStates[19].latPosition = 128.70;
    std::cout << "CrossingObject_.EgoStates[19].latPosition(float32): " << CrossingObject_.EgoStates[19].latPosition << std::endl;
    CrossingObject_.EgoStates[19].longPosition = 129.80;
    std::cout << "CrossingObject_.EgoStates[19].longPosition(float32): " << CrossingObject_.EgoStates[19].longPosition << std::endl;
    CrossingObject_.EgoStates[19].latVelocity = 130.90;
    std::cout << "CrossingObject_.EgoStates[19].latVelocity(float32): " << CrossingObject_.EgoStates[19].latVelocity << std::endl;
    CrossingObject_.EgoStates[19].longVelocity = 132.00;
    std::cout << "CrossingObject_.EgoStates[19].longVelocity(float32): " << CrossingObject_.EgoStates[19].longVelocity << std::endl;
    CrossingObject_.EgoStates[20].latAcceleration = 133.10;
    std::cout << "CrossingObject_.EgoStates[20].latAcceleration(float32): " << CrossingObject_.EgoStates[20].latAcceleration << std::endl;
    CrossingObject_.EgoStates[20].longAcceleration = 134.20;
    std::cout << "CrossingObject_.EgoStates[20].longAcceleration(float32): " << CrossingObject_.EgoStates[20].longAcceleration << std::endl;
    CrossingObject_.EgoStates[20].latPosition = 135.30;
    std::cout << "CrossingObject_.EgoStates[20].latPosition(float32): " << CrossingObject_.EgoStates[20].latPosition << std::endl;
    CrossingObject_.EgoStates[20].longPosition = 136.40;
    std::cout << "CrossingObject_.EgoStates[20].longPosition(float32): " << CrossingObject_.EgoStates[20].longPosition << std::endl;
    CrossingObject_.EgoStates[20].latVelocity = 137.50;
    std::cout << "CrossingObject_.EgoStates[20].latVelocity(float32): " << CrossingObject_.EgoStates[20].latVelocity << std::endl;
    CrossingObject_.EgoStates[20].longVelocity = 138.60;
    std::cout << "CrossingObject_.EgoStates[20].longVelocity(float32): " << CrossingObject_.EgoStates[20].longVelocity << std::endl;
    CrossingObject_.EgoStates[21].latAcceleration = 139.70;
    std::cout << "CrossingObject_.EgoStates[21].latAcceleration(float32): " << CrossingObject_.EgoStates[21].latAcceleration << std::endl;
    CrossingObject_.EgoStates[21].longAcceleration = 140.80;
    std::cout << "CrossingObject_.EgoStates[21].longAcceleration(float32): " << CrossingObject_.EgoStates[21].longAcceleration << std::endl;
    CrossingObject_.EgoStates[21].latPosition = 141.90;
    std::cout << "CrossingObject_.EgoStates[21].latPosition(float32): " << CrossingObject_.EgoStates[21].latPosition << std::endl;
    CrossingObject_.EgoStates[21].longPosition = 143.00;
    std::cout << "CrossingObject_.EgoStates[21].longPosition(float32): " << CrossingObject_.EgoStates[21].longPosition << std::endl;
    CrossingObject_.EgoStates[21].latVelocity = 144.10;
    std::cout << "CrossingObject_.EgoStates[21].latVelocity(float32): " << CrossingObject_.EgoStates[21].latVelocity << std::endl;
    CrossingObject_.EgoStates[21].longVelocity = 145.20;
    std::cout << "CrossingObject_.EgoStates[21].longVelocity(float32): " << CrossingObject_.EgoStates[21].longVelocity << std::endl;
    CrossingObject_.EgoStates[22].latAcceleration = 146.30;
    std::cout << "CrossingObject_.EgoStates[22].latAcceleration(float32): " << CrossingObject_.EgoStates[22].latAcceleration << std::endl;
    CrossingObject_.EgoStates[22].longAcceleration = 147.40;
    std::cout << "CrossingObject_.EgoStates[22].longAcceleration(float32): " << CrossingObject_.EgoStates[22].longAcceleration << std::endl;
    CrossingObject_.EgoStates[22].latPosition = 148.50;
    std::cout << "CrossingObject_.EgoStates[22].latPosition(float32): " << CrossingObject_.EgoStates[22].latPosition << std::endl;
    CrossingObject_.EgoStates[22].longPosition = 149.60;
    std::cout << "CrossingObject_.EgoStates[22].longPosition(float32): " << CrossingObject_.EgoStates[22].longPosition << std::endl;
    CrossingObject_.EgoStates[22].latVelocity = 150.70;
    std::cout << "CrossingObject_.EgoStates[22].latVelocity(float32): " << CrossingObject_.EgoStates[22].latVelocity << std::endl;
    CrossingObject_.EgoStates[22].longVelocity = 151.80;
    std::cout << "CrossingObject_.EgoStates[22].longVelocity(float32): " << CrossingObject_.EgoStates[22].longVelocity << std::endl;
    CrossingObject_.EgoStates[23].latAcceleration = 152.90;
    std::cout << "CrossingObject_.EgoStates[23].latAcceleration(float32): " << CrossingObject_.EgoStates[23].latAcceleration << std::endl;
    CrossingObject_.EgoStates[23].longAcceleration = 154.00;
    std::cout << "CrossingObject_.EgoStates[23].longAcceleration(float32): " << CrossingObject_.EgoStates[23].longAcceleration << std::endl;
    CrossingObject_.EgoStates[23].latPosition = 155.10;
    std::cout << "CrossingObject_.EgoStates[23].latPosition(float32): " << CrossingObject_.EgoStates[23].latPosition << std::endl;
    CrossingObject_.EgoStates[23].longPosition = 156.20;
    std::cout << "CrossingObject_.EgoStates[23].longPosition(float32): " << CrossingObject_.EgoStates[23].longPosition << std::endl;
    CrossingObject_.EgoStates[23].latVelocity = 157.30;
    std::cout << "CrossingObject_.EgoStates[23].latVelocity(float32): " << CrossingObject_.EgoStates[23].latVelocity << std::endl;
    CrossingObject_.EgoStates[23].longVelocity = 158.40;
    std::cout << "CrossingObject_.EgoStates[23].longVelocity(float32): " << CrossingObject_.EgoStates[23].longVelocity << std::endl;
    CrossingObject_.EgoStates[24].latAcceleration = 159.50;
    std::cout << "CrossingObject_.EgoStates[24].latAcceleration(float32): " << CrossingObject_.EgoStates[24].latAcceleration << std::endl;
    CrossingObject_.EgoStates[24].longAcceleration = 160.60;
    std::cout << "CrossingObject_.EgoStates[24].longAcceleration(float32): " << CrossingObject_.EgoStates[24].longAcceleration << std::endl;
    CrossingObject_.EgoStates[24].latPosition = 161.70;
    std::cout << "CrossingObject_.EgoStates[24].latPosition(float32): " << CrossingObject_.EgoStates[24].latPosition << std::endl;
    CrossingObject_.EgoStates[24].longPosition = 162.80;
    std::cout << "CrossingObject_.EgoStates[24].longPosition(float32): " << CrossingObject_.EgoStates[24].longPosition << std::endl;
    CrossingObject_.EgoStates[24].latVelocity = 163.90;
    std::cout << "CrossingObject_.EgoStates[24].latVelocity(float32): " << CrossingObject_.EgoStates[24].latVelocity << std::endl;
    CrossingObject_.EgoStates[24].longVelocity = 165.00;
    std::cout << "CrossingObject_.EgoStates[24].longVelocity(float32): " << CrossingObject_.EgoStates[24].longVelocity << std::endl;
    CrossingObject_.EgoStates[25].latAcceleration = 166.10;
    std::cout << "CrossingObject_.EgoStates[25].latAcceleration(float32): " << CrossingObject_.EgoStates[25].latAcceleration << std::endl;
    CrossingObject_.EgoStates[25].longAcceleration = 167.20;
    std::cout << "CrossingObject_.EgoStates[25].longAcceleration(float32): " << CrossingObject_.EgoStates[25].longAcceleration << std::endl;
    CrossingObject_.EgoStates[25].latPosition = 168.30;
    std::cout << "CrossingObject_.EgoStates[25].latPosition(float32): " << CrossingObject_.EgoStates[25].latPosition << std::endl;
    CrossingObject_.EgoStates[25].longPosition = 169.40;
    std::cout << "CrossingObject_.EgoStates[25].longPosition(float32): " << CrossingObject_.EgoStates[25].longPosition << std::endl;
    CrossingObject_.EgoStates[25].latVelocity = 170.50;
    std::cout << "CrossingObject_.EgoStates[25].latVelocity(float32): " << CrossingObject_.EgoStates[25].latVelocity << std::endl;
    CrossingObject_.EgoStates[25].longVelocity = 171.60;
    std::cout << "CrossingObject_.EgoStates[25].longVelocity(float32): " << CrossingObject_.EgoStates[25].longVelocity << std::endl;
    CrossingObject_.EgoStates[26].latAcceleration = 172.70;
    std::cout << "CrossingObject_.EgoStates[26].latAcceleration(float32): " << CrossingObject_.EgoStates[26].latAcceleration << std::endl;
    CrossingObject_.EgoStates[26].longAcceleration = 173.80;
    std::cout << "CrossingObject_.EgoStates[26].longAcceleration(float32): " << CrossingObject_.EgoStates[26].longAcceleration << std::endl;
    CrossingObject_.EgoStates[26].latPosition = 174.90;
    std::cout << "CrossingObject_.EgoStates[26].latPosition(float32): " << CrossingObject_.EgoStates[26].latPosition << std::endl;
    CrossingObject_.EgoStates[26].longPosition = 176.00;
    std::cout << "CrossingObject_.EgoStates[26].longPosition(float32): " << CrossingObject_.EgoStates[26].longPosition << std::endl;
    CrossingObject_.EgoStates[26].latVelocity = 177.10;
    std::cout << "CrossingObject_.EgoStates[26].latVelocity(float32): " << CrossingObject_.EgoStates[26].latVelocity << std::endl;
    CrossingObject_.EgoStates[26].longVelocity = 178.20;
    std::cout << "CrossingObject_.EgoStates[26].longVelocity(float32): " << CrossingObject_.EgoStates[26].longVelocity << std::endl;
    CrossingObject_.EgoStates[27].latAcceleration = 179.30;
    std::cout << "CrossingObject_.EgoStates[27].latAcceleration(float32): " << CrossingObject_.EgoStates[27].latAcceleration << std::endl;
    CrossingObject_.EgoStates[27].longAcceleration = 180.40;
    std::cout << "CrossingObject_.EgoStates[27].longAcceleration(float32): " << CrossingObject_.EgoStates[27].longAcceleration << std::endl;
    CrossingObject_.EgoStates[27].latPosition = 181.50;
    std::cout << "CrossingObject_.EgoStates[27].latPosition(float32): " << CrossingObject_.EgoStates[27].latPosition << std::endl;
    CrossingObject_.EgoStates[27].longPosition = 182.60;
    std::cout << "CrossingObject_.EgoStates[27].longPosition(float32): " << CrossingObject_.EgoStates[27].longPosition << std::endl;
    CrossingObject_.EgoStates[27].latVelocity = 183.70;
    std::cout << "CrossingObject_.EgoStates[27].latVelocity(float32): " << CrossingObject_.EgoStates[27].latVelocity << std::endl;
    CrossingObject_.EgoStates[27].longVelocity = 184.80;
    std::cout << "CrossingObject_.EgoStates[27].longVelocity(float32): " << CrossingObject_.EgoStates[27].longVelocity << std::endl;
    CrossingObject_.EgoStates[28].latAcceleration = 185.90;
    std::cout << "CrossingObject_.EgoStates[28].latAcceleration(float32): " << CrossingObject_.EgoStates[28].latAcceleration << std::endl;
    CrossingObject_.EgoStates[28].longAcceleration = 187.00;
    std::cout << "CrossingObject_.EgoStates[28].longAcceleration(float32): " << CrossingObject_.EgoStates[28].longAcceleration << std::endl;
    CrossingObject_.EgoStates[28].latPosition = 188.10;
    std::cout << "CrossingObject_.EgoStates[28].latPosition(float32): " << CrossingObject_.EgoStates[28].latPosition << std::endl;
    CrossingObject_.EgoStates[28].longPosition = 189.20;
    std::cout << "CrossingObject_.EgoStates[28].longPosition(float32): " << CrossingObject_.EgoStates[28].longPosition << std::endl;
    CrossingObject_.EgoStates[28].latVelocity = 190.30;
    std::cout << "CrossingObject_.EgoStates[28].latVelocity(float32): " << CrossingObject_.EgoStates[28].latVelocity << std::endl;
    CrossingObject_.EgoStates[28].longVelocity = 191.40;
    std::cout << "CrossingObject_.EgoStates[28].longVelocity(float32): " << CrossingObject_.EgoStates[28].longVelocity << std::endl;
    CrossingObject_.EgoStates[29].latAcceleration = 192.50;
    std::cout << "CrossingObject_.EgoStates[29].latAcceleration(float32): " << CrossingObject_.EgoStates[29].latAcceleration << std::endl;
    CrossingObject_.EgoStates[29].longAcceleration = 193.60;
    std::cout << "CrossingObject_.EgoStates[29].longAcceleration(float32): " << CrossingObject_.EgoStates[29].longAcceleration << std::endl;
    CrossingObject_.EgoStates[29].latPosition = 194.70;
    std::cout << "CrossingObject_.EgoStates[29].latPosition(float32): " << CrossingObject_.EgoStates[29].latPosition << std::endl;
    CrossingObject_.EgoStates[29].longPosition = 195.80;
    std::cout << "CrossingObject_.EgoStates[29].longPosition(float32): " << CrossingObject_.EgoStates[29].longPosition << std::endl;
    CrossingObject_.EgoStates[29].latVelocity = 196.90;
    std::cout << "CrossingObject_.EgoStates[29].latVelocity(float32): " << CrossingObject_.EgoStates[29].latVelocity << std::endl;
    CrossingObject_.EgoStates[29].longVelocity = 198.00;
    std::cout << "CrossingObject_.EgoStates[29].longVelocity(float32): " << CrossingObject_.EgoStates[29].longVelocity << std::endl;
    CrossingObject_.Properties[0].id = 1;
    std::cout << "CrossingObject_.Properties[0].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[0].id) << std::dec  << std::endl;
    CrossingObject_.Properties[0].motionPattern = 1;
    std::cout << "CrossingObject_.Properties[0].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[0].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[0].referencePoint = 2;
    std::cout << "CrossingObject_.Properties[0].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[0].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[0].type = 3;
    std::cout << "CrossingObject_.Properties[0].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[0].type) << std::dec  << std::endl;
    CrossingObject_.Properties[0].width = 199.10;
    std::cout << "CrossingObject_.Properties[0].width(float32): " << CrossingObject_.Properties[0].width << std::endl;
    CrossingObject_.Properties[0].length = 200.20;
    std::cout << "CrossingObject_.Properties[0].length(float32): " << CrossingObject_.Properties[0].length << std::endl;
    CrossingObject_.Properties[0].cmbbQly = 4;
    std::cout << "CrossingObject_.Properties[0].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[0].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[1].id = 2;
    std::cout << "CrossingObject_.Properties[1].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[1].id) << std::dec  << std::endl;
    CrossingObject_.Properties[1].motionPattern = 5;
    std::cout << "CrossingObject_.Properties[1].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[1].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[1].referencePoint = 6;
    std::cout << "CrossingObject_.Properties[1].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[1].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[1].type = 7;
    std::cout << "CrossingObject_.Properties[1].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[1].type) << std::dec  << std::endl;
    CrossingObject_.Properties[1].width = 201.30;
    std::cout << "CrossingObject_.Properties[1].width(float32): " << CrossingObject_.Properties[1].width << std::endl;
    CrossingObject_.Properties[1].length = 202.40;
    std::cout << "CrossingObject_.Properties[1].length(float32): " << CrossingObject_.Properties[1].length << std::endl;
    CrossingObject_.Properties[1].cmbbQly = 8;
    std::cout << "CrossingObject_.Properties[1].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[1].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[2].id = 3;
    std::cout << "CrossingObject_.Properties[2].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[2].id) << std::dec  << std::endl;
    CrossingObject_.Properties[2].motionPattern = 9;
    std::cout << "CrossingObject_.Properties[2].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[2].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[2].referencePoint = 10;
    std::cout << "CrossingObject_.Properties[2].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[2].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[2].type = 11;
    std::cout << "CrossingObject_.Properties[2].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[2].type) << std::dec  << std::endl;
    CrossingObject_.Properties[2].width = 203.50;
    std::cout << "CrossingObject_.Properties[2].width(float32): " << CrossingObject_.Properties[2].width << std::endl;
    CrossingObject_.Properties[2].length = 204.60;
    std::cout << "CrossingObject_.Properties[2].length(float32): " << CrossingObject_.Properties[2].length << std::endl;
    CrossingObject_.Properties[2].cmbbQly = 12;
    std::cout << "CrossingObject_.Properties[2].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[2].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[3].id = 4;
    std::cout << "CrossingObject_.Properties[3].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[3].id) << std::dec  << std::endl;
    CrossingObject_.Properties[3].motionPattern = 13;
    std::cout << "CrossingObject_.Properties[3].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[3].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[3].referencePoint = 14;
    std::cout << "CrossingObject_.Properties[3].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[3].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[3].type = 15;
    std::cout << "CrossingObject_.Properties[3].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[3].type) << std::dec  << std::endl;
    CrossingObject_.Properties[3].width = 205.70;
    std::cout << "CrossingObject_.Properties[3].width(float32): " << CrossingObject_.Properties[3].width << std::endl;
    CrossingObject_.Properties[3].length = 206.80;
    std::cout << "CrossingObject_.Properties[3].length(float32): " << CrossingObject_.Properties[3].length << std::endl;
    CrossingObject_.Properties[3].cmbbQly = 16;
    std::cout << "CrossingObject_.Properties[3].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[3].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[4].id = 5;
    std::cout << "CrossingObject_.Properties[4].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[4].id) << std::dec  << std::endl;
    CrossingObject_.Properties[4].motionPattern = 17;
    std::cout << "CrossingObject_.Properties[4].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[4].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[4].referencePoint = 18;
    std::cout << "CrossingObject_.Properties[4].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[4].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[4].type = 19;
    std::cout << "CrossingObject_.Properties[4].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[4].type) << std::dec  << std::endl;
    CrossingObject_.Properties[4].width = 207.90;
    std::cout << "CrossingObject_.Properties[4].width(float32): " << CrossingObject_.Properties[4].width << std::endl;
    CrossingObject_.Properties[4].length = 209.00;
    std::cout << "CrossingObject_.Properties[4].length(float32): " << CrossingObject_.Properties[4].length << std::endl;
    CrossingObject_.Properties[4].cmbbQly = 20;
    std::cout << "CrossingObject_.Properties[4].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[4].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[5].id = 6;
    std::cout << "CrossingObject_.Properties[5].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[5].id) << std::dec  << std::endl;
    CrossingObject_.Properties[5].motionPattern = 21;
    std::cout << "CrossingObject_.Properties[5].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[5].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[5].referencePoint = 22;
    std::cout << "CrossingObject_.Properties[5].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[5].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[5].type = 23;
    std::cout << "CrossingObject_.Properties[5].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[5].type) << std::dec  << std::endl;
    CrossingObject_.Properties[5].width = 210.10;
    std::cout << "CrossingObject_.Properties[5].width(float32): " << CrossingObject_.Properties[5].width << std::endl;
    CrossingObject_.Properties[5].length = 211.20;
    std::cout << "CrossingObject_.Properties[5].length(float32): " << CrossingObject_.Properties[5].length << std::endl;
    CrossingObject_.Properties[5].cmbbQly = 24;
    std::cout << "CrossingObject_.Properties[5].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[5].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[6].id = 7;
    std::cout << "CrossingObject_.Properties[6].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[6].id) << std::dec  << std::endl;
    CrossingObject_.Properties[6].motionPattern = 25;
    std::cout << "CrossingObject_.Properties[6].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[6].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[6].referencePoint = 26;
    std::cout << "CrossingObject_.Properties[6].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[6].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[6].type = 27;
    std::cout << "CrossingObject_.Properties[6].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[6].type) << std::dec  << std::endl;
    CrossingObject_.Properties[6].width = 212.30;
    std::cout << "CrossingObject_.Properties[6].width(float32): " << CrossingObject_.Properties[6].width << std::endl;
    CrossingObject_.Properties[6].length = 213.40;
    std::cout << "CrossingObject_.Properties[6].length(float32): " << CrossingObject_.Properties[6].length << std::endl;
    CrossingObject_.Properties[6].cmbbQly = 28;
    std::cout << "CrossingObject_.Properties[6].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[6].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[7].id = 8;
    std::cout << "CrossingObject_.Properties[7].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[7].id) << std::dec  << std::endl;
    CrossingObject_.Properties[7].motionPattern = 29;
    std::cout << "CrossingObject_.Properties[7].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[7].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[7].referencePoint = 30;
    std::cout << "CrossingObject_.Properties[7].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[7].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[7].type = 31;
    std::cout << "CrossingObject_.Properties[7].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[7].type) << std::dec  << std::endl;
    CrossingObject_.Properties[7].width = 214.50;
    std::cout << "CrossingObject_.Properties[7].width(float32): " << CrossingObject_.Properties[7].width << std::endl;
    CrossingObject_.Properties[7].length = 215.60;
    std::cout << "CrossingObject_.Properties[7].length(float32): " << CrossingObject_.Properties[7].length << std::endl;
    CrossingObject_.Properties[7].cmbbQly = 32;
    std::cout << "CrossingObject_.Properties[7].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[7].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[8].id = 9;
    std::cout << "CrossingObject_.Properties[8].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[8].id) << std::dec  << std::endl;
    CrossingObject_.Properties[8].motionPattern = 33;
    std::cout << "CrossingObject_.Properties[8].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[8].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[8].referencePoint = 34;
    std::cout << "CrossingObject_.Properties[8].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[8].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[8].type = 35;
    std::cout << "CrossingObject_.Properties[8].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[8].type) << std::dec  << std::endl;
    CrossingObject_.Properties[8].width = 216.70;
    std::cout << "CrossingObject_.Properties[8].width(float32): " << CrossingObject_.Properties[8].width << std::endl;
    CrossingObject_.Properties[8].length = 217.80;
    std::cout << "CrossingObject_.Properties[8].length(float32): " << CrossingObject_.Properties[8].length << std::endl;
    CrossingObject_.Properties[8].cmbbQly = 36;
    std::cout << "CrossingObject_.Properties[8].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[8].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[9].id = 10;
    std::cout << "CrossingObject_.Properties[9].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[9].id) << std::dec  << std::endl;
    CrossingObject_.Properties[9].motionPattern = 37;
    std::cout << "CrossingObject_.Properties[9].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[9].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[9].referencePoint = 38;
    std::cout << "CrossingObject_.Properties[9].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[9].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[9].type = 39;
    std::cout << "CrossingObject_.Properties[9].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[9].type) << std::dec  << std::endl;
    CrossingObject_.Properties[9].width = 218.90;
    std::cout << "CrossingObject_.Properties[9].width(float32): " << CrossingObject_.Properties[9].width << std::endl;
    CrossingObject_.Properties[9].length = 220.00;
    std::cout << "CrossingObject_.Properties[9].length(float32): " << CrossingObject_.Properties[9].length << std::endl;
    CrossingObject_.Properties[9].cmbbQly = 40;
    std::cout << "CrossingObject_.Properties[9].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[9].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[10].id = 11;
    std::cout << "CrossingObject_.Properties[10].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[10].id) << std::dec  << std::endl;
    CrossingObject_.Properties[10].motionPattern = 41;
    std::cout << "CrossingObject_.Properties[10].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[10].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[10].referencePoint = 42;
    std::cout << "CrossingObject_.Properties[10].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[10].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[10].type = 43;
    std::cout << "CrossingObject_.Properties[10].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[10].type) << std::dec  << std::endl;
    CrossingObject_.Properties[10].width = 221.10;
    std::cout << "CrossingObject_.Properties[10].width(float32): " << CrossingObject_.Properties[10].width << std::endl;
    CrossingObject_.Properties[10].length = 222.20;
    std::cout << "CrossingObject_.Properties[10].length(float32): " << CrossingObject_.Properties[10].length << std::endl;
    CrossingObject_.Properties[10].cmbbQly = 44;
    std::cout << "CrossingObject_.Properties[10].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[10].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[11].id = 12;
    std::cout << "CrossingObject_.Properties[11].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[11].id) << std::dec  << std::endl;
    CrossingObject_.Properties[11].motionPattern = 45;
    std::cout << "CrossingObject_.Properties[11].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[11].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[11].referencePoint = 46;
    std::cout << "CrossingObject_.Properties[11].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[11].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[11].type = 47;
    std::cout << "CrossingObject_.Properties[11].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[11].type) << std::dec  << std::endl;
    CrossingObject_.Properties[11].width = 223.30;
    std::cout << "CrossingObject_.Properties[11].width(float32): " << CrossingObject_.Properties[11].width << std::endl;
    CrossingObject_.Properties[11].length = 224.40;
    std::cout << "CrossingObject_.Properties[11].length(float32): " << CrossingObject_.Properties[11].length << std::endl;
    CrossingObject_.Properties[11].cmbbQly = 48;
    std::cout << "CrossingObject_.Properties[11].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[11].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[12].id = 13;
    std::cout << "CrossingObject_.Properties[12].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[12].id) << std::dec  << std::endl;
    CrossingObject_.Properties[12].motionPattern = 49;
    std::cout << "CrossingObject_.Properties[12].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[12].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[12].referencePoint = 50;
    std::cout << "CrossingObject_.Properties[12].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[12].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[12].type = 51;
    std::cout << "CrossingObject_.Properties[12].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[12].type) << std::dec  << std::endl;
    CrossingObject_.Properties[12].width = 225.50;
    std::cout << "CrossingObject_.Properties[12].width(float32): " << CrossingObject_.Properties[12].width << std::endl;
    CrossingObject_.Properties[12].length = 226.60;
    std::cout << "CrossingObject_.Properties[12].length(float32): " << CrossingObject_.Properties[12].length << std::endl;
    CrossingObject_.Properties[12].cmbbQly = 52;
    std::cout << "CrossingObject_.Properties[12].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[12].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[13].id = 14;
    std::cout << "CrossingObject_.Properties[13].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[13].id) << std::dec  << std::endl;
    CrossingObject_.Properties[13].motionPattern = 53;
    std::cout << "CrossingObject_.Properties[13].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[13].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[13].referencePoint = 54;
    std::cout << "CrossingObject_.Properties[13].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[13].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[13].type = 55;
    std::cout << "CrossingObject_.Properties[13].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[13].type) << std::dec  << std::endl;
    CrossingObject_.Properties[13].width = 227.70;
    std::cout << "CrossingObject_.Properties[13].width(float32): " << CrossingObject_.Properties[13].width << std::endl;
    CrossingObject_.Properties[13].length = 228.80;
    std::cout << "CrossingObject_.Properties[13].length(float32): " << CrossingObject_.Properties[13].length << std::endl;
    CrossingObject_.Properties[13].cmbbQly = 56;
    std::cout << "CrossingObject_.Properties[13].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[13].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[14].id = 15;
    std::cout << "CrossingObject_.Properties[14].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[14].id) << std::dec  << std::endl;
    CrossingObject_.Properties[14].motionPattern = 57;
    std::cout << "CrossingObject_.Properties[14].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[14].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[14].referencePoint = 58;
    std::cout << "CrossingObject_.Properties[14].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[14].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[14].type = 59;
    std::cout << "CrossingObject_.Properties[14].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[14].type) << std::dec  << std::endl;
    CrossingObject_.Properties[14].width = 229.90;
    std::cout << "CrossingObject_.Properties[14].width(float32): " << CrossingObject_.Properties[14].width << std::endl;
    CrossingObject_.Properties[14].length = 231.00;
    std::cout << "CrossingObject_.Properties[14].length(float32): " << CrossingObject_.Properties[14].length << std::endl;
    CrossingObject_.Properties[14].cmbbQly = 60;
    std::cout << "CrossingObject_.Properties[14].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[14].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[15].id = 16;
    std::cout << "CrossingObject_.Properties[15].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[15].id) << std::dec  << std::endl;
    CrossingObject_.Properties[15].motionPattern = 61;
    std::cout << "CrossingObject_.Properties[15].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[15].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[15].referencePoint = 62;
    std::cout << "CrossingObject_.Properties[15].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[15].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[15].type = 63;
    std::cout << "CrossingObject_.Properties[15].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[15].type) << std::dec  << std::endl;
    CrossingObject_.Properties[15].width = 232.10;
    std::cout << "CrossingObject_.Properties[15].width(float32): " << CrossingObject_.Properties[15].width << std::endl;
    CrossingObject_.Properties[15].length = 233.20;
    std::cout << "CrossingObject_.Properties[15].length(float32): " << CrossingObject_.Properties[15].length << std::endl;
    CrossingObject_.Properties[15].cmbbQly = 64;
    std::cout << "CrossingObject_.Properties[15].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[15].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[16].id = 17;
    std::cout << "CrossingObject_.Properties[16].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[16].id) << std::dec  << std::endl;
    CrossingObject_.Properties[16].motionPattern = 65;
    std::cout << "CrossingObject_.Properties[16].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[16].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[16].referencePoint = 66;
    std::cout << "CrossingObject_.Properties[16].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[16].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[16].type = 67;
    std::cout << "CrossingObject_.Properties[16].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[16].type) << std::dec  << std::endl;
    CrossingObject_.Properties[16].width = 234.30;
    std::cout << "CrossingObject_.Properties[16].width(float32): " << CrossingObject_.Properties[16].width << std::endl;
    CrossingObject_.Properties[16].length = 235.40;
    std::cout << "CrossingObject_.Properties[16].length(float32): " << CrossingObject_.Properties[16].length << std::endl;
    CrossingObject_.Properties[16].cmbbQly = 68;
    std::cout << "CrossingObject_.Properties[16].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[16].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[17].id = 18;
    std::cout << "CrossingObject_.Properties[17].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[17].id) << std::dec  << std::endl;
    CrossingObject_.Properties[17].motionPattern = 69;
    std::cout << "CrossingObject_.Properties[17].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[17].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[17].referencePoint = 70;
    std::cout << "CrossingObject_.Properties[17].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[17].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[17].type = 71;
    std::cout << "CrossingObject_.Properties[17].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[17].type) << std::dec  << std::endl;
    CrossingObject_.Properties[17].width = 236.50;
    std::cout << "CrossingObject_.Properties[17].width(float32): " << CrossingObject_.Properties[17].width << std::endl;
    CrossingObject_.Properties[17].length = 237.60;
    std::cout << "CrossingObject_.Properties[17].length(float32): " << CrossingObject_.Properties[17].length << std::endl;
    CrossingObject_.Properties[17].cmbbQly = 72;
    std::cout << "CrossingObject_.Properties[17].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[17].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[18].id = 19;
    std::cout << "CrossingObject_.Properties[18].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[18].id) << std::dec  << std::endl;
    CrossingObject_.Properties[18].motionPattern = 73;
    std::cout << "CrossingObject_.Properties[18].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[18].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[18].referencePoint = 74;
    std::cout << "CrossingObject_.Properties[18].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[18].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[18].type = 75;
    std::cout << "CrossingObject_.Properties[18].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[18].type) << std::dec  << std::endl;
    CrossingObject_.Properties[18].width = 238.70;
    std::cout << "CrossingObject_.Properties[18].width(float32): " << CrossingObject_.Properties[18].width << std::endl;
    CrossingObject_.Properties[18].length = 239.80;
    std::cout << "CrossingObject_.Properties[18].length(float32): " << CrossingObject_.Properties[18].length << std::endl;
    CrossingObject_.Properties[18].cmbbQly = 76;
    std::cout << "CrossingObject_.Properties[18].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[18].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[19].id = 20;
    std::cout << "CrossingObject_.Properties[19].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[19].id) << std::dec  << std::endl;
    CrossingObject_.Properties[19].motionPattern = 77;
    std::cout << "CrossingObject_.Properties[19].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[19].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[19].referencePoint = 78;
    std::cout << "CrossingObject_.Properties[19].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[19].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[19].type = 79;
    std::cout << "CrossingObject_.Properties[19].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[19].type) << std::dec  << std::endl;
    CrossingObject_.Properties[19].width = 240.90;
    std::cout << "CrossingObject_.Properties[19].width(float32): " << CrossingObject_.Properties[19].width << std::endl;
    CrossingObject_.Properties[19].length = 242.00;
    std::cout << "CrossingObject_.Properties[19].length(float32): " << CrossingObject_.Properties[19].length << std::endl;
    CrossingObject_.Properties[19].cmbbQly = 80;
    std::cout << "CrossingObject_.Properties[19].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[19].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[20].id = 21;
    std::cout << "CrossingObject_.Properties[20].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[20].id) << std::dec  << std::endl;
    CrossingObject_.Properties[20].motionPattern = 81;
    std::cout << "CrossingObject_.Properties[20].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[20].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[20].referencePoint = 82;
    std::cout << "CrossingObject_.Properties[20].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[20].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[20].type = 83;
    std::cout << "CrossingObject_.Properties[20].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[20].type) << std::dec  << std::endl;
    CrossingObject_.Properties[20].width = 243.10;
    std::cout << "CrossingObject_.Properties[20].width(float32): " << CrossingObject_.Properties[20].width << std::endl;
    CrossingObject_.Properties[20].length = 244.20;
    std::cout << "CrossingObject_.Properties[20].length(float32): " << CrossingObject_.Properties[20].length << std::endl;
    CrossingObject_.Properties[20].cmbbQly = 84;
    std::cout << "CrossingObject_.Properties[20].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[20].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[21].id = 22;
    std::cout << "CrossingObject_.Properties[21].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[21].id) << std::dec  << std::endl;
    CrossingObject_.Properties[21].motionPattern = 85;
    std::cout << "CrossingObject_.Properties[21].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[21].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[21].referencePoint = 86;
    std::cout << "CrossingObject_.Properties[21].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[21].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[21].type = 87;
    std::cout << "CrossingObject_.Properties[21].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[21].type) << std::dec  << std::endl;
    CrossingObject_.Properties[21].width = 245.30;
    std::cout << "CrossingObject_.Properties[21].width(float32): " << CrossingObject_.Properties[21].width << std::endl;
    CrossingObject_.Properties[21].length = 246.40;
    std::cout << "CrossingObject_.Properties[21].length(float32): " << CrossingObject_.Properties[21].length << std::endl;
    CrossingObject_.Properties[21].cmbbQly = 88;
    std::cout << "CrossingObject_.Properties[21].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[21].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[22].id = 23;
    std::cout << "CrossingObject_.Properties[22].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[22].id) << std::dec  << std::endl;
    CrossingObject_.Properties[22].motionPattern = 89;
    std::cout << "CrossingObject_.Properties[22].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[22].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[22].referencePoint = 90;
    std::cout << "CrossingObject_.Properties[22].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[22].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[22].type = 91;
    std::cout << "CrossingObject_.Properties[22].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[22].type) << std::dec  << std::endl;
    CrossingObject_.Properties[22].width = 247.50;
    std::cout << "CrossingObject_.Properties[22].width(float32): " << CrossingObject_.Properties[22].width << std::endl;
    CrossingObject_.Properties[22].length = 248.60;
    std::cout << "CrossingObject_.Properties[22].length(float32): " << CrossingObject_.Properties[22].length << std::endl;
    CrossingObject_.Properties[22].cmbbQly = 92;
    std::cout << "CrossingObject_.Properties[22].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[22].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[23].id = 24;
    std::cout << "CrossingObject_.Properties[23].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[23].id) << std::dec  << std::endl;
    CrossingObject_.Properties[23].motionPattern = 93;
    std::cout << "CrossingObject_.Properties[23].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[23].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[23].referencePoint = 94;
    std::cout << "CrossingObject_.Properties[23].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[23].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[23].type = 95;
    std::cout << "CrossingObject_.Properties[23].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[23].type) << std::dec  << std::endl;
    CrossingObject_.Properties[23].width = 249.70;
    std::cout << "CrossingObject_.Properties[23].width(float32): " << CrossingObject_.Properties[23].width << std::endl;
    CrossingObject_.Properties[23].length = 250.80;
    std::cout << "CrossingObject_.Properties[23].length(float32): " << CrossingObject_.Properties[23].length << std::endl;
    CrossingObject_.Properties[23].cmbbQly = 96;
    std::cout << "CrossingObject_.Properties[23].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[23].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[24].id = 25;
    std::cout << "CrossingObject_.Properties[24].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[24].id) << std::dec  << std::endl;
    CrossingObject_.Properties[24].motionPattern = 97;
    std::cout << "CrossingObject_.Properties[24].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[24].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[24].referencePoint = 98;
    std::cout << "CrossingObject_.Properties[24].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[24].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[24].type = 99;
    std::cout << "CrossingObject_.Properties[24].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[24].type) << std::dec  << std::endl;
    CrossingObject_.Properties[24].width = 251.90;
    std::cout << "CrossingObject_.Properties[24].width(float32): " << CrossingObject_.Properties[24].width << std::endl;
    CrossingObject_.Properties[24].length = 253.00;
    std::cout << "CrossingObject_.Properties[24].length(float32): " << CrossingObject_.Properties[24].length << std::endl;
    CrossingObject_.Properties[24].cmbbQly = 100;
    std::cout << "CrossingObject_.Properties[24].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[24].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[25].id = 26;
    std::cout << "CrossingObject_.Properties[25].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[25].id) << std::dec  << std::endl;
    CrossingObject_.Properties[25].motionPattern = 101;
    std::cout << "CrossingObject_.Properties[25].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[25].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[25].referencePoint = 102;
    std::cout << "CrossingObject_.Properties[25].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[25].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[25].type = 103;
    std::cout << "CrossingObject_.Properties[25].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[25].type) << std::dec  << std::endl;
    CrossingObject_.Properties[25].width = 254.10;
    std::cout << "CrossingObject_.Properties[25].width(float32): " << CrossingObject_.Properties[25].width << std::endl;
    CrossingObject_.Properties[25].length = 255.20;
    std::cout << "CrossingObject_.Properties[25].length(float32): " << CrossingObject_.Properties[25].length << std::endl;
    CrossingObject_.Properties[25].cmbbQly = 104;
    std::cout << "CrossingObject_.Properties[25].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[25].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[26].id = 27;
    std::cout << "CrossingObject_.Properties[26].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[26].id) << std::dec  << std::endl;
    CrossingObject_.Properties[26].motionPattern = 105;
    std::cout << "CrossingObject_.Properties[26].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[26].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[26].referencePoint = 106;
    std::cout << "CrossingObject_.Properties[26].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[26].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[26].type = 107;
    std::cout << "CrossingObject_.Properties[26].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[26].type) << std::dec  << std::endl;
    CrossingObject_.Properties[26].width = 256.30;
    std::cout << "CrossingObject_.Properties[26].width(float32): " << CrossingObject_.Properties[26].width << std::endl;
    CrossingObject_.Properties[26].length = 257.40;
    std::cout << "CrossingObject_.Properties[26].length(float32): " << CrossingObject_.Properties[26].length << std::endl;
    CrossingObject_.Properties[26].cmbbQly = 108;
    std::cout << "CrossingObject_.Properties[26].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[26].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[27].id = 28;
    std::cout << "CrossingObject_.Properties[27].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[27].id) << std::dec  << std::endl;
    CrossingObject_.Properties[27].motionPattern = 109;
    std::cout << "CrossingObject_.Properties[27].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[27].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[27].referencePoint = 110;
    std::cout << "CrossingObject_.Properties[27].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[27].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[27].type = 111;
    std::cout << "CrossingObject_.Properties[27].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[27].type) << std::dec  << std::endl;
    CrossingObject_.Properties[27].width = 258.50;
    std::cout << "CrossingObject_.Properties[27].width(float32): " << CrossingObject_.Properties[27].width << std::endl;
    CrossingObject_.Properties[27].length = 259.60;
    std::cout << "CrossingObject_.Properties[27].length(float32): " << CrossingObject_.Properties[27].length << std::endl;
    CrossingObject_.Properties[27].cmbbQly = 112;
    std::cout << "CrossingObject_.Properties[27].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[27].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[28].id = 29;
    std::cout << "CrossingObject_.Properties[28].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[28].id) << std::dec  << std::endl;
    CrossingObject_.Properties[28].motionPattern = 113;
    std::cout << "CrossingObject_.Properties[28].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[28].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[28].referencePoint = 114;
    std::cout << "CrossingObject_.Properties[28].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[28].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[28].type = 115;
    std::cout << "CrossingObject_.Properties[28].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[28].type) << std::dec  << std::endl;
    CrossingObject_.Properties[28].width = 260.70;
    std::cout << "CrossingObject_.Properties[28].width(float32): " << CrossingObject_.Properties[28].width << std::endl;
    CrossingObject_.Properties[28].length = 261.80;
    std::cout << "CrossingObject_.Properties[28].length(float32): " << CrossingObject_.Properties[28].length << std::endl;
    CrossingObject_.Properties[28].cmbbQly = 116;
    std::cout << "CrossingObject_.Properties[28].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[28].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.Properties[29].id = 30;
    std::cout << "CrossingObject_.Properties[29].id(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[29].id) << std::dec  << std::endl;
    CrossingObject_.Properties[29].motionPattern = 117;
    std::cout << "CrossingObject_.Properties[29].motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[29].motionPattern) << std::dec  << std::endl;
    CrossingObject_.Properties[29].referencePoint = 118;
    std::cout << "CrossingObject_.Properties[29].referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[29].referencePoint) << std::dec  << std::endl;
    CrossingObject_.Properties[29].type = 119;
    std::cout << "CrossingObject_.Properties[29].type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[29].type) << std::dec  << std::endl;
    CrossingObject_.Properties[29].width = 262.90;
    std::cout << "CrossingObject_.Properties[29].width(float32): " << CrossingObject_.Properties[29].width << std::endl;
    CrossingObject_.Properties[29].length = 264.00;
    std::cout << "CrossingObject_.Properties[29].length(float32): " << CrossingObject_.Properties[29].length << std::endl;
    CrossingObject_.Properties[29].cmbbQly = 120;
    std::cout << "CrossingObject_.Properties[29].cmbbQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CrossingObject_.Properties[29].cmbbQly) << std::dec  << std::endl;
    CrossingObject_.SequenceID = 1;
    std::cout << "CrossingObject_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(CrossingObject_.SequenceID) << std::dec  << std::endl;
};
#endif



#ifdef TRAFFICFLOW_H
/* Set and Print struct TrafficFlow initial value */
void setIntialValue_TrafficFlow(TrafficFlow& TrafficFlow_){
    std::cout << "Set struct TrafficFlow variable and Publish:" << std::endl;
    TrafficFlow_.DrvgSideQly = 1;
    std::cout << "TrafficFlow_.DrvgSideQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.DrvgSideQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst = 1.10;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[0].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd = 2.20;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[0].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowNormDir = 2;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[0].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[0].TrfcFlowQly = 3;
    std::cout << "TrafficFlow_.LaneProperties[0].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[0].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst = 3.30;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[1].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd = 4.40;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[1].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowNormDir = 4;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[1].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[1].TrfcFlowQly = 5;
    std::cout << "TrafficFlow_.LaneProperties[1].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[1].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst = 5.50;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst(float32): " << TrafficFlow_.LaneProperties[2].TrfcFlowAvrgDst << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd = 6.60;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd(float32): " << TrafficFlow_.LaneProperties[2].TrfcFlowAvrgSpd << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowNormDir = 6;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowNormDir(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[2].TrfcFlowNormDir) << std::dec  << std::endl;
    TrafficFlow_.LaneProperties[2].TrfcFlowQly = 7;
    std::cout << "TrafficFlow_.LaneProperties[2].TrfcFlowQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.LaneProperties[2].TrfcFlowQly) << std::dec  << std::endl;
    TrafficFlow_.SequenceID = 1;
    std::cout << "TrafficFlow_.SequenceID(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(TrafficFlow_.SequenceID) << std::dec  << std::endl;
    TrafficFlow_.DrivingSide = 8;
    std::cout << "TrafficFlow_.DrivingSide(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(TrafficFlow_.DrivingSide) << std::dec  << std::endl;
};
#endif



#ifdef ROADPATH_H
/* Set and Print struct RoadPath initial value */
void setIntialValue_RoadPath(RoadPath& RoadPath_){
    std::cout << "Set struct RoadPath variable and Publish:" << std::endl;
    RoadPath_.LaneWidth = 1.10;
    std::cout << "RoadPath_.LaneWidth(float32): " << RoadPath_.LaneWidth << std::endl;
    RoadPath_.OffsLat = 2.20;
    std::cout << "RoadPath_.OffsLat(float32): " << RoadPath_.OffsLat << std::endl;
    RoadPath_.DstLgtToEndLaneMrk = 3.30;
    std::cout << "RoadPath_.DstLgtToEndLaneMrk(float32): " << RoadPath_.DstLgtToEndLaneMrk << std::endl;
    RoadPath_.DstLgtToEndEhCrvt = 4.40;
    std::cout << "RoadPath_.DstLgtToEndEhCrvt(float32): " << RoadPath_.DstLgtToEndEhCrvt << std::endl;
    RoadPath_.DstLgtToEndObj = 5.50;
    std::cout << "RoadPath_.DstLgtToEndObj(float32): " << RoadPath_.DstLgtToEndObj << std::endl;
    RoadPath_.AgDir = 6.60;
    std::cout << "RoadPath_.AgDir(float32): " << RoadPath_.AgDir << std::endl;
    RoadPath_.Crvt = 7.70;
    std::cout << "RoadPath_.Crvt(float32): " << RoadPath_.Crvt << std::endl;
    RoadPath_.CrvtRate[0] = 8.80;
    std::cout << "RoadPath_.CrvtRate[0](float32): " << RoadPath_.CrvtRate[0] << std::endl;
    RoadPath_.CrvtRate[1] = 9.90;
    std::cout << "RoadPath_.CrvtRate[1](float32): " << RoadPath_.CrvtRate[1] << std::endl;
    RoadPath_.CrvtRate[2] = 11.00;
    std::cout << "RoadPath_.CrvtRate[2](float32): " << RoadPath_.CrvtRate[2] << std::endl;
    RoadPath_.SegLen[0] = 12.10;
    std::cout << "RoadPath_.SegLen[0](float32): " << RoadPath_.SegLen[0] << std::endl;
    RoadPath_.SegLen[1] = 13.20;
    std::cout << "RoadPath_.SegLen[1](float32): " << RoadPath_.SegLen[1] << std::endl;
    RoadPath_.SegLen[2] = 14.30;
    std::cout << "RoadPath_.SegLen[2](float32): " << RoadPath_.SegLen[2] << std::endl;
    RoadPath_.Strtd = 1;
    std::cout << "RoadPath_.Strtd(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.Strtd) << std::dec  << std::endl;
    RoadPath_.Vld = 2;
    std::cout << "RoadPath_.Vld(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.Vld) << std::dec  << std::endl;
    RoadPath_.VldForELKA = 3;
    std::cout << "RoadPath_.VldForELKA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForELKA) << std::dec  << std::endl;
    RoadPath_.VldForTrfcAssi = 4;
    std::cout << "RoadPath_.VldForTrfcAssi(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForTrfcAssi) << std::dec  << std::endl;
    RoadPath_.LaneChange = 5;
    std::cout << "RoadPath_.LaneChange(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneChange) << std::dec  << std::endl;
    RoadPath_.VldForCSA = 6;
    std::cout << "RoadPath_.VldForCSA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForCSA) << std::dec  << std::endl;
    RoadPath_.VldForOncomingBraking = 7;
    std::cout << "RoadPath_.VldForOncomingBraking(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForOncomingBraking) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0] = 8;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1] = 9;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2] = 10;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3] = 11;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4] = 12;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5] = 13;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6] = 14;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7] = 15;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8] = 16;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9] = 17;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10] = 18;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11] = 19;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12] = 20;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13] = 21;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14] = 22;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15] = 23;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16] = 24;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17] = 25;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18] = 26;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19] = 27;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20] = 28;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21] = 29;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22] = 30;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23] = 31;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24] = 32;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25] = 33;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26] = 34;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27] = 35;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28] = 36;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29] = 37;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30] = 38;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31] = 39;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdAgDir[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[0] = 40;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[1] = 41;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[2] = 42;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[3] = 43;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[4] = 44;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[5] = 45;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[6] = 46;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[7] = 47;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[8] = 48;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[9] = 49;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[10] = 50;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[11] = 51;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[12] = 52;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[13] = 53;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[14] = 54;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[15] = 55;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[16] = 56;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[17] = 57;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[18] = 58;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[19] = 59;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[20] = 60;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[21] = 61;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[22] = 62;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[23] = 63;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[24] = 64;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[25] = 65;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[26] = 66;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[27] = 67;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[28] = 68;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[29] = 69;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[30] = 70;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjUseForUpdPosn[31] = 71;
    std::cout << "RoadPath_.ObjectInfo.ObjUseForUpdPosn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjUseForUpdPosn[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[0] = 72;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[1] = 73;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[2] = 74;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[3] = 75;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[4] = 76;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[5] = 77;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[6] = 78;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[7] = 79;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[8] = 80;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[9] = 81;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[10] = 82;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[11] = 83;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[12] = 84;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[13] = 85;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[14] = 86;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[15] = 87;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[16] = 88;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[17] = 89;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[18] = 90;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[19] = 91;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[20] = 92;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[21] = 93;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[22] = 94;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[23] = 95;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[24] = 96;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[25] = 97;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[26] = 98;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[27] = 99;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[28] = 100;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[29] = 101;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[30] = 102;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForExtrpn[31] = 103;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForExtrpn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForExtrpn[31]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[0] = 104;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[0](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[0]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[1] = 105;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[1](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[1]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[2] = 106;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[2](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[2]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[3] = 107;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[3](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[3]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[4] = 108;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[4](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[4]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[5] = 109;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[5](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[5]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[6] = 110;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[6](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[6]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[7] = 111;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[7](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[7]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[8] = 112;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[8](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[8]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[9] = 113;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[9](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[9]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[10] = 114;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[10](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[10]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[11] = 115;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[11](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[11]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[12] = 116;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[12](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[12]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[13] = 117;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[13](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[13]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[14] = 118;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[14](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[14]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[15] = 119;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[15](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[15]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[16] = 120;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[16](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[16]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[17] = 121;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[17](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[17]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[18] = 122;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[18](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[18]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[19] = 123;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[19](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[19]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[20] = 124;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[20](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[20]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[21] = 125;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[21](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[21]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[22] = 126;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[22](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[22]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[23] = 127;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[23](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[23]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[24] = 128;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[24](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[24]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[25] = 129;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[25](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[25]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[26] = 130;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[26](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[26]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[27] = 131;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[27](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[27]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[28] = 132;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[28](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[28]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[29] = 133;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[29](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[29]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[30] = 134;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[30](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[30]) << std::dec  << std::endl;
    RoadPath_.ObjectInfo.ObjVldForUpdPosn[31] = 135;
    std::cout << "RoadPath_.ObjectInfo.ObjVldForUpdPosn[31](uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.ObjectInfo.ObjVldForUpdPosn[31]) << std::dec  << std::endl;
    RoadPath_.LowConfDist = 15.40;
    std::cout << "RoadPath_.LowConfDist(float32): " << RoadPath_.LowConfDist << std::endl;
    RoadPath_.HighConfDist = 16.50;
    std::cout << "RoadPath_.HighConfDist(float32): " << RoadPath_.HighConfDist << std::endl;
    RoadPath_.CrvtQly = 136;
    std::cout << "RoadPath_.CrvtQly(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.CrvtQly) << std::dec  << std::endl;
    RoadPath_.VldForACC = 137;
    std::cout << "RoadPath_.VldForACC(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.VldForACC) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Left.Type = 138;
    std::cout << "RoadPath_.LaneMarkerInfo.Left.Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Left.Type) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF = 139;
    std::cout << "RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Left.IsUsedByRGF) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Right.Type = 140;
    std::cout << "RoadPath_.LaneMarkerInfo.Right.Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Right.Type) << std::dec  << std::endl;
    RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF = 141;
    std::cout << "RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RoadPath_.LaneMarkerInfo.Right.IsUsedByRGF) << std::dec  << std::endl;
    RoadPath_.Timestamp = 1;
    std::cout << "RoadPath_.Timestamp(uint64): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(RoadPath_.Timestamp) << std::dec  << std::endl;
};
#endif



#ifdef LONGCTRLOBJINFO_H
/* Set and Print struct LongCtrlObjInfo initial value */
void setIntialValue_LongCtrlObjInfo(LongCtrlObjInfo& LongCtrlObjInfo_){
    std::cout << "Set struct LongCtrlObjInfo variable and Publish:" << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A = 1.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading = 2.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn = 1;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index = 2;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad = 3.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence = 3;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory = 4;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern = 5;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat = 4.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt = 5.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd = 6.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts = 6;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator = 7;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type = 8;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLLeft.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A = 7.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading = 8.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn = 9;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index = 10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad = 9.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence = 11;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory = 12;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern = 13;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat = 11.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt = 12.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd = 13.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts = 14;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator = 15;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type = 16;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtAdjLRight.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A = 14.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading = 15.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn = 17;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index = 18;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad = 16.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence = 19;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory = 20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern = 21;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat = 17.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt = 18.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd = 19.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts = 22;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator = 23;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type = 24;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtCutIn.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A = 20.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading = 22.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn = 25;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index = 26;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad = 23.10;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence = 27;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory = 28;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern = 29;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat = 24.20;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt = 25.30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd = 26.40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts = 30;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator = 31;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type = 32;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtFrstClstLane.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A = 27.50;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.A << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading = 28.60;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Heading << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn = 33;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Idn) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index = 34;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Index) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad = 29.70;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoad << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence = 35;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.latPositionRoadConfidence) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory = 36;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionHistory) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern = 37;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.motionPattern) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat = 30.80;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLat << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt = 31.90;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.PosnLgt << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd = 33.00;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd(float32): " << LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.Spd << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts = 38;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.TgtLaneSts) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator = 39;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.turnIndicator) << std::dec  << std::endl;
    LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type = 40;
    std::cout << "LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LongCtrlObjInfo_.TargetsSelectedForACC.AccTgtSecClstLane.type) << std::dec  << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1 = 34.10;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal1 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2 = 35.20;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal2 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3 = 36.30;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal3 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4 = 37.40;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal4 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5 = 38.50;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal5 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6 = 39.60;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal6 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7 = 40.70;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal7 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8 = 41.80;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal8 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9 = 42.90;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal9 << std::endl;
    LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10 = 44.00;
    std::cout << "LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10(float32): " << LongCtrlObjInfo_.AdditionalTarSelnSignals.Signal10 << std::endl;
};
#endif



#ifdef LDP_OBJ_H
/* Set and Print struct LDP_Obj initial value */
void setIntialValue_LDP_Obj(LDP_Obj& LDP_Obj_){
    std::cout << "Set struct LDP_Obj variable and Publish:" << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType = 1;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.interventionType) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.objectType = 2;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.objectType(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.objectType) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint = 3;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.referencePoint) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.length = 1.10;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.length(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.length << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.width = 2.20;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.width(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.width << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.heading = 3.30;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.heading(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.heading << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision = 4.40;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeToCollision << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach = 5.50;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeToReach << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane = 6.60;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.timeOutsideEgoLane << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision = 7.70;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latPositionAtTimeToCollision << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition = 8.80;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtPosition << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition = 9.90;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latPosition << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity = 11.00;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtVelocity << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity = 12.10;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.latVelocity << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane = 4;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.targetInEgoLane) << std::dec  << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf = 13.20;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.lgtAccRqrdForPrimTarToAvdSelf << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance = 14.30;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance(float32): " << LDP_Obj_.CritLatCdn_PrimaryTarget.egoLatAccRequiredForAvoidance << std::endl;
    LDP_Obj_.CritLatCdn_PrimaryTarget.exists = 5;
    std::cout << "LDP_Obj_.CritLatCdn_PrimaryTarget.exists(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.CritLatCdn_PrimaryTarget.exists) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInEgoLane = 6;
    std::cout << "LDP_Obj_.secondaryObstacleInEgoLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInEgoLane) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInLeftLane = 7;
    std::cout << "LDP_Obj_.secondaryObstacleInLeftLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInLeftLane) << std::dec  << std::endl;
    LDP_Obj_.secondaryObstacleInRightLane = 8;
    std::cout << "LDP_Obj_.secondaryObstacleInRightLane(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDP_Obj_.secondaryObstacleInRightLane) << std::dec  << std::endl;
};
#endif



#ifdef OBJFRNTCDNFORSUPP_H
/* Set and Print struct ObjFrntCdnForSupp initial value */
void setIntialValue_ObjFrntCdnForSupp(ObjFrntCdnForSupp& ObjFrntCdnForSupp_){
    std::cout << "Set struct ObjFrntCdnForSupp variable and Publish:" << std::endl;
    ObjFrntCdnForSupp_.ObjIdx = 1;
    std::cout << "ObjFrntCdnForSupp_.ObjIdx(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.ObjIdx) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.ObjPosnLat = 1.10;
    std::cout << "ObjFrntCdnForSupp_.ObjPosnLat(float32): " << ObjFrntCdnForSupp_.ObjPosnLat << std::endl;
    ObjFrntCdnForSupp_.ObjPosnLgt = 2.20;
    std::cout << "ObjFrntCdnForSupp_.ObjPosnLgt(float32): " << ObjFrntCdnForSupp_.ObjPosnLgt << std::endl;
    ObjFrntCdnForSupp_.ObjSpdLgt = 3.30;
    std::cout << "ObjFrntCdnForSupp_.ObjSpdLgt(float32): " << ObjFrntCdnForSupp_.ObjSpdLgt << std::endl;
    ObjFrntCdnForSupp_.ObjTyp = 2;
    std::cout << "ObjFrntCdnForSupp_.ObjTyp(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.ObjTyp) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.ObjWidth = 4.40;
    std::cout << "ObjFrntCdnForSupp_.ObjWidth(float32): " << ObjFrntCdnForSupp_.ObjWidth << std::endl;
    ObjFrntCdnForSupp_.TiToCllsn = 5.50;
    std::cout << "ObjFrntCdnForSupp_.TiToCllsn(float32): " << ObjFrntCdnForSupp_.TiToCllsn << std::endl;
    ObjFrntCdnForSupp_.VisnIdx = 3;
    std::cout << "ObjFrntCdnForSupp_.VisnIdx(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.VisnIdx) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.SuppressSideRoadEdge = 4;
    std::cout << "ObjFrntCdnForSupp_.SuppressSideRoadEdge(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.SuppressSideRoadEdge) << std::dec  << std::endl;
    ObjFrntCdnForSupp_.SuppressSideSolidLKA = 5;
    std::cout << "ObjFrntCdnForSupp_.SuppressSideSolidLKA(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ObjFrntCdnForSupp_.SuppressSideSolidLKA) << std::dec  << std::endl;
};
#endif



#ifdef LDPPATH_H
/* Set and Print struct LDPPath initial value */
void setIntialValue_LDPPath(LDPPath& LDPPath_){
    std::cout << "Set struct LDPPath variable and Publish:" << std::endl;
    LDPPath_.ITC.nrOfSegments = 1;
    std::cout << "LDPPath_.ITC.nrOfSegments(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDPPath_.ITC.nrOfSegments) << std::dec  << std::endl;
    LDPPath_.ITC.coefficientVector[0] = 1.10;
    std::cout << "LDPPath_.ITC.coefficientVector[0](float32): " << LDPPath_.ITC.coefficientVector[0] << std::endl;
    LDPPath_.ITC.coefficientVector[1] = 2.20;
    std::cout << "LDPPath_.ITC.coefficientVector[1](float32): " << LDPPath_.ITC.coefficientVector[1] << std::endl;
    LDPPath_.ITC.coefficientVector[2] = 3.30;
    std::cout << "LDPPath_.ITC.coefficientVector[2](float32): " << LDPPath_.ITC.coefficientVector[2] << std::endl;
    LDPPath_.ITC.coefficientVector[3] = 4.40;
    std::cout << "LDPPath_.ITC.coefficientVector[3](float32): " << LDPPath_.ITC.coefficientVector[3] << std::endl;
    LDPPath_.ITC.coefficientVector[4] = 5.50;
    std::cout << "LDPPath_.ITC.coefficientVector[4](float32): " << LDPPath_.ITC.coefficientVector[4] << std::endl;
    LDPPath_.ITC.coefficientVector[5] = 6.60;
    std::cout << "LDPPath_.ITC.coefficientVector[5](float32): " << LDPPath_.ITC.coefficientVector[5] << std::endl;
    LDPPath_.ITC.coefficientVector[6] = 7.70;
    std::cout << "LDPPath_.ITC.coefficientVector[6](float32): " << LDPPath_.ITC.coefficientVector[6] << std::endl;
    LDPPath_.ITC.coefficientVector[7] = 8.80;
    std::cout << "LDPPath_.ITC.coefficientVector[7](float32): " << LDPPath_.ITC.coefficientVector[7] << std::endl;
    LDPPath_.ITC.coefficientVector[8] = 9.90;
    std::cout << "LDPPath_.ITC.coefficientVector[8](float32): " << LDPPath_.ITC.coefficientVector[8] << std::endl;
    LDPPath_.ITC.coefficientVector[9] = 11.00;
    std::cout << "LDPPath_.ITC.coefficientVector[9](float32): " << LDPPath_.ITC.coefficientVector[9] << std::endl;
    LDPPath_.ITC.coefficientVector[10] = 12.10;
    std::cout << "LDPPath_.ITC.coefficientVector[10](float32): " << LDPPath_.ITC.coefficientVector[10] << std::endl;
    LDPPath_.ITC.coefficientVector[11] = 13.20;
    std::cout << "LDPPath_.ITC.coefficientVector[11](float32): " << LDPPath_.ITC.coefficientVector[11] << std::endl;
    LDPPath_.ITC.coefficientVector[12] = 14.30;
    std::cout << "LDPPath_.ITC.coefficientVector[12](float32): " << LDPPath_.ITC.coefficientVector[12] << std::endl;
    LDPPath_.ITC.coefficientVector[13] = 15.40;
    std::cout << "LDPPath_.ITC.coefficientVector[13](float32): " << LDPPath_.ITC.coefficientVector[13] << std::endl;
    LDPPath_.ITC.coefficientVector[14] = 16.50;
    std::cout << "LDPPath_.ITC.coefficientVector[14](float32): " << LDPPath_.ITC.coefficientVector[14] << std::endl;
    LDPPath_.ITC.coefficientVector[15] = 17.60;
    std::cout << "LDPPath_.ITC.coefficientVector[15](float32): " << LDPPath_.ITC.coefficientVector[15] << std::endl;
    LDPPath_.ITC.coefficientVector[16] = 18.70;
    std::cout << "LDPPath_.ITC.coefficientVector[16](float32): " << LDPPath_.ITC.coefficientVector[16] << std::endl;
    LDPPath_.ITC.coefficientVector[17] = 19.80;
    std::cout << "LDPPath_.ITC.coefficientVector[17](float32): " << LDPPath_.ITC.coefficientVector[17] << std::endl;
    LDPPath_.ITC.coefficientVector[18] = 20.90;
    std::cout << "LDPPath_.ITC.coefficientVector[18](float32): " << LDPPath_.ITC.coefficientVector[18] << std::endl;
    LDPPath_.ITC.coefficientVector[19] = 22.00;
    std::cout << "LDPPath_.ITC.coefficientVector[19](float32): " << LDPPath_.ITC.coefficientVector[19] << std::endl;
    LDPPath_.ITC.coefficientVector[20] = 23.10;
    std::cout << "LDPPath_.ITC.coefficientVector[20](float32): " << LDPPath_.ITC.coefficientVector[20] << std::endl;
    LDPPath_.ITC.coefficientVector[21] = 24.20;
    std::cout << "LDPPath_.ITC.coefficientVector[21](float32): " << LDPPath_.ITC.coefficientVector[21] << std::endl;
    LDPPath_.ITC.coefficientVector[22] = 25.30;
    std::cout << "LDPPath_.ITC.coefficientVector[22](float32): " << LDPPath_.ITC.coefficientVector[22] << std::endl;
    LDPPath_.ITC.coefficientVector[23] = 26.40;
    std::cout << "LDPPath_.ITC.coefficientVector[23](float32): " << LDPPath_.ITC.coefficientVector[23] << std::endl;
    LDPPath_.ITC.timeVector[0] = 27.50;
    std::cout << "LDPPath_.ITC.timeVector[0](float32): " << LDPPath_.ITC.timeVector[0] << std::endl;
    LDPPath_.ITC.timeVector[1] = 28.60;
    std::cout << "LDPPath_.ITC.timeVector[1](float32): " << LDPPath_.ITC.timeVector[1] << std::endl;
    LDPPath_.ITC.timeVector[2] = 29.70;
    std::cout << "LDPPath_.ITC.timeVector[2](float32): " << LDPPath_.ITC.timeVector[2] << std::endl;
    LDPPath_.ITC.timeVector[3] = 30.80;
    std::cout << "LDPPath_.ITC.timeVector[3](float32): " << LDPPath_.ITC.timeVector[3] << std::endl;
    LDPPath_.ITC.timeVector[4] = 31.90;
    std::cout << "LDPPath_.ITC.timeVector[4](float32): " << LDPPath_.ITC.timeVector[4] << std::endl;
    LDPPath_.ITC.timeVector[5] = 33.00;
    std::cout << "LDPPath_.ITC.timeVector[5](float32): " << LDPPath_.ITC.timeVector[5] << std::endl;
    LDPPath_.initialLatPosition = 34.10;
    std::cout << "LDPPath_.initialLatPosition(float32): " << LDPPath_.initialLatPosition << std::endl;
    LDPPath_.initialLatVelocity = 35.20;
    std::cout << "LDPPath_.initialLatVelocity(float32): " << LDPPath_.initialLatVelocity << std::endl;
    LDPPath_.initialLatAcceleration = 36.30;
    std::cout << "LDPPath_.initialLatAcceleration(float32): " << LDPPath_.initialLatAcceleration << std::endl;
    LDPPath_.initialLongVelocity = 37.40;
    std::cout << "LDPPath_.initialLongVelocity(float32): " << LDPPath_.initialLongVelocity << std::endl;
    LDPPath_.initialLongAcceleration = 38.50;
    std::cout << "LDPPath_.initialLongAcceleration(float32): " << LDPPath_.initialLongAcceleration << std::endl;
    LDPPath_.latAccRequiredForAvoidance = 39.60;
    std::cout << "LDPPath_.latAccRequiredForAvoidance(float32): " << LDPPath_.latAccRequiredForAvoidance << std::endl;
    LDPPath_.latAccRequiredForAlignment = 40.70;
    std::cout << "LDPPath_.latAccRequiredForAlignment(float32): " << LDPPath_.latAccRequiredForAlignment << std::endl;
    LDPPath_.pathInfo = 1;
    std::cout << "LDPPath_.pathInfo(uint16): " << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(LDPPath_.pathInfo) << std::dec  << std::endl;
    LDPPath_.nrOfSegments = 2;
    std::cout << "LDPPath_.nrOfSegments(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(LDPPath_.nrOfSegments) << std::dec  << std::endl;
    LDPPath_.timeVector[0] = 41.80;
    std::cout << "LDPPath_.timeVector[0](float32): " << LDPPath_.timeVector[0] << std::endl;
    LDPPath_.timeVector[1] = 42.90;
    std::cout << "LDPPath_.timeVector[1](float32): " << LDPPath_.timeVector[1] << std::endl;
    LDPPath_.timeVector[2] = 44.00;
    std::cout << "LDPPath_.timeVector[2](float32): " << LDPPath_.timeVector[2] << std::endl;
    LDPPath_.timeVector[3] = 45.10;
    std::cout << "LDPPath_.timeVector[3](float32): " << LDPPath_.timeVector[3] << std::endl;
    LDPPath_.timeVector[4] = 46.20;
    std::cout << "LDPPath_.timeVector[4](float32): " << LDPPath_.timeVector[4] << std::endl;
    LDPPath_.timeVector[5] = 47.30;
    std::cout << "LDPPath_.timeVector[5](float32): " << LDPPath_.timeVector[5] << std::endl;
    LDPPath_.timeVector[6] = 48.40;
    std::cout << "LDPPath_.timeVector[6](float32): " << LDPPath_.timeVector[6] << std::endl;
    LDPPath_.timeVector[7] = 49.50;
    std::cout << "LDPPath_.timeVector[7](float32): " << LDPPath_.timeVector[7] << std::endl;
    LDPPath_.coefficientVector[0] = 50.60;
    std::cout << "LDPPath_.coefficientVector[0](float32): " << LDPPath_.coefficientVector[0] << std::endl;
    LDPPath_.coefficientVector[1] = 51.70;
    std::cout << "LDPPath_.coefficientVector[1](float32): " << LDPPath_.coefficientVector[1] << std::endl;
    LDPPath_.coefficientVector[2] = 52.80;
    std::cout << "LDPPath_.coefficientVector[2](float32): " << LDPPath_.coefficientVector[2] << std::endl;
    LDPPath_.coefficientVector[3] = 53.90;
    std::cout << "LDPPath_.coefficientVector[3](float32): " << LDPPath_.coefficientVector[3] << std::endl;
    LDPPath_.coefficientVector[4] = 55.00;
    std::cout << "LDPPath_.coefficientVector[4](float32): " << LDPPath_.coefficientVector[4] << std::endl;
    LDPPath_.coefficientVector[5] = 56.10;
    std::cout << "LDPPath_.coefficientVector[5](float32): " << LDPPath_.coefficientVector[5] << std::endl;
    LDPPath_.coefficientVector[6] = 57.20;
    std::cout << "LDPPath_.coefficientVector[6](float32): " << LDPPath_.coefficientVector[6] << std::endl;
    LDPPath_.coefficientVector[7] = 58.30;
    std::cout << "LDPPath_.coefficientVector[7](float32): " << LDPPath_.coefficientVector[7] << std::endl;
    LDPPath_.coefficientVector[8] = 59.40;
    std::cout << "LDPPath_.coefficientVector[8](float32): " << LDPPath_.coefficientVector[8] << std::endl;
    LDPPath_.coefficientVector[9] = 60.50;
    std::cout << "LDPPath_.coefficientVector[9](float32): " << LDPPath_.coefficientVector[9] << std::endl;
    LDPPath_.coefficientVector[10] = 61.60;
    std::cout << "LDPPath_.coefficientVector[10](float32): " << LDPPath_.coefficientVector[10] << std::endl;
    LDPPath_.coefficientVector[11] = 62.70;
    std::cout << "LDPPath_.coefficientVector[11](float32): " << LDPPath_.coefficientVector[11] << std::endl;
    LDPPath_.coefficientVector[12] = 63.80;
    std::cout << "LDPPath_.coefficientVector[12](float32): " << LDPPath_.coefficientVector[12] << std::endl;
    LDPPath_.coefficientVector[13] = 64.90;
    std::cout << "LDPPath_.coefficientVector[13](float32): " << LDPPath_.coefficientVector[13] << std::endl;
    LDPPath_.coefficientVector[14] = 66.00;
    std::cout << "LDPPath_.coefficientVector[14](float32): " << LDPPath_.coefficientVector[14] << std::endl;
    LDPPath_.coefficientVector[15] = 67.10;
    std::cout << "LDPPath_.coefficientVector[15](float32): " << LDPPath_.coefficientVector[15] << std::endl;
    LDPPath_.coefficientVector[16] = 68.20;
    std::cout << "LDPPath_.coefficientVector[16](float32): " << LDPPath_.coefficientVector[16] << std::endl;
    LDPPath_.coefficientVector[17] = 69.30;
    std::cout << "LDPPath_.coefficientVector[17](float32): " << LDPPath_.coefficientVector[17] << std::endl;
    LDPPath_.coefficientVector[18] = 70.40;
    std::cout << "LDPPath_.coefficientVector[18](float32): " << LDPPath_.coefficientVector[18] << std::endl;
    LDPPath_.coefficientVector[19] = 71.50;
    std::cout << "LDPPath_.coefficientVector[19](float32): " << LDPPath_.coefficientVector[19] << std::endl;
    LDPPath_.coefficientVector[20] = 72.60;
    std::cout << "LDPPath_.coefficientVector[20](float32): " << LDPPath_.coefficientVector[20] << std::endl;
    LDPPath_.coefficientVector[21] = 73.70;
    std::cout << "LDPPath_.coefficientVector[21](float32): " << LDPPath_.coefficientVector[21] << std::endl;
    LDPPath_.coefficientVector[22] = 74.80;
    std::cout << "LDPPath_.coefficientVector[22](float32): " << LDPPath_.coefficientVector[22] << std::endl;
    LDPPath_.coefficientVector[23] = 75.90;
    std::cout << "LDPPath_.coefficientVector[23](float32): " << LDPPath_.coefficientVector[23] << std::endl;
    LDPPath_.coefficientVector[24] = 77.00;
    std::cout << "LDPPath_.coefficientVector[24](float32): " << LDPPath_.coefficientVector[24] << std::endl;
    LDPPath_.coefficientVector[25] = 78.10;
    std::cout << "LDPPath_.coefficientVector[25](float32): " << LDPPath_.coefficientVector[25] << std::endl;
    LDPPath_.coefficientVector[26] = 79.20;
    std::cout << "LDPPath_.coefficientVector[26](float32): " << LDPPath_.coefficientVector[26] << std::endl;
    LDPPath_.coefficientVector[27] = 80.30;
    std::cout << "LDPPath_.coefficientVector[27](float32): " << LDPPath_.coefficientVector[27] << std::endl;
    LDPPath_.coefficientVector[28] = 81.40;
    std::cout << "LDPPath_.coefficientVector[28](float32): " << LDPPath_.coefficientVector[28] << std::endl;
    LDPPath_.coefficientVector[29] = 82.50;
    std::cout << "LDPPath_.coefficientVector[29](float32): " << LDPPath_.coefficientVector[29] << std::endl;
    LDPPath_.coefficientVector[30] = 83.60;
    std::cout << "LDPPath_.coefficientVector[30](float32): " << LDPPath_.coefficientVector[30] << std::endl;
    LDPPath_.coefficientVector[31] = 84.70;
    std::cout << "LDPPath_.coefficientVector[31](float32): " << LDPPath_.coefficientVector[31] << std::endl;
    LDPPath_.coefficientVector[32] = 85.80;
    std::cout << "LDPPath_.coefficientVector[32](float32): " << LDPPath_.coefficientVector[32] << std::endl;
    LDPPath_.coefficientVector[33] = 86.90;
    std::cout << "LDPPath_.coefficientVector[33](float32): " << LDPPath_.coefficientVector[33] << std::endl;
    LDPPath_.coefficientVector[34] = 88.00;
    std::cout << "LDPPath_.coefficientVector[34](float32): " << LDPPath_.coefficientVector[34] << std::endl;
    LDPPath_.coefficientVector[35] = 89.10;
    std::cout << "LDPPath_.coefficientVector[35](float32): " << LDPPath_.coefficientVector[35] << std::endl;
    LDPPath_.coefficientVector[36] = 90.20;
    std::cout << "LDPPath_.coefficientVector[36](float32): " << LDPPath_.coefficientVector[36] << std::endl;
    LDPPath_.coefficientVector[37] = 91.30;
    std::cout << "LDPPath_.coefficientVector[37](float32): " << LDPPath_.coefficientVector[37] << std::endl;
    LDPPath_.coefficientVector[38] = 92.40;
    std::cout << "LDPPath_.coefficientVector[38](float32): " << LDPPath_.coefficientVector[38] << std::endl;
    LDPPath_.coefficientVector[39] = 93.50;
    std::cout << "LDPPath_.coefficientVector[39](float32): " << LDPPath_.coefficientVector[39] << std::endl;
    LDPPath_.coefficientVector[40] = 94.60;
    std::cout << "LDPPath_.coefficientVector[40](float32): " << LDPPath_.coefficientVector[40] << std::endl;
    LDPPath_.coefficientVector[41] = 95.70;
    std::cout << "LDPPath_.coefficientVector[41](float32): " << LDPPath_.coefficientVector[41] << std::endl;
};
#endif



#ifdef CAH_ACTIVESAFETY_H
/* Set and Print struct CAH_ActiveSafety initial value */
void setIntialValue_CAH_ActiveSafety(CAH_ActiveSafety& CAH_ActiveSafety_){
    std::cout << "Set struct CAH_ActiveSafety variable and Publish:" << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_State = 1;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_State(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_State) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_WarningState = 2;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_WarningState(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_WarningState) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning = 3;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_LatentWarning) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState = 4;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_AcuteWarningState) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_flag.FCW_TargetId = 5;
    std::cout << "CAH_ActiveSafety_.fcw_flag.FCW_TargetId(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.fcw_flag.FCW_TargetId) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_FunConfig = 1;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_FunConfig(uint32): " << std::hex << std::setw(8) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_FunConfig) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_State = 6;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_State(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_State) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq = 7;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_PrefillReq) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_Level = 8;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_Level(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_Level) << std::dec  << std::endl;
    CAH_ActiveSafety_.aeb_flag.AEB_Type = 9;
    std::cout << "CAH_ActiveSafety_.aeb_flag.AEB_Type(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.aeb_flag.AEB_Type) << std::dec  << std::endl;
    CAH_ActiveSafety_.drt.DRT_TTC = 1.10;
    std::cout << "CAH_ActiveSafety_.drt.DRT_TTC(float32): " << CAH_ActiveSafety_.drt.DRT_TTC << std::endl;
    CAH_ActiveSafety_.drt.DRT_ID = 10;
    std::cout << "CAH_ActiveSafety_.drt.DRT_ID(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.drt.DRT_ID) << std::dec  << std::endl;
    CAH_ActiveSafety_.drt.DRT_ObjectClass = 11;
    std::cout << "CAH_ActiveSafety_.drt.DRT_ObjectClass(uint8): " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(CAH_ActiveSafety_.drt.DRT_ObjectClass) << std::dec  << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime = 2.20;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_ReactionTime << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_TTCThres = 3.30;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_TTCThres(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_TTCThres << std::endl;
    CAH_ActiveSafety_.fcw_dev.FCW_TTC = 4.40;
    std::cout << "CAH_ActiveSafety_.fcw_dev.FCW_TTC(float32): " << CAH_ActiveSafety_.fcw_dev.FCW_TTC << std::endl;
};
#endif

#ifdef VEHBUSIN_H
std::map<std::string, VariableVariant > VehBusIn_Map = {
//[VehBusIn]
{"VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB(float32)" , &VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUS_ActualTorqueFB},
{"VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB(float32)" , &VehBusIn_.SysSigGrp_HPCGW_CB_APP.GW_MCUM_ActualTorqueFB},
{"VehBusIn_.SysSigGrp_HPCGW_CB_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_HPCGW_CB_APP.Message_QF},
{"VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected(float32)" , &VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTemp_Corrected},
{"VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD(uint8)" , &VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_OutsideTempVD},
{"VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts(uint8)" , &VehBusIn_.SysSigGrp_HPCGW_3B0_APP.GW_TMS_FrontDefrostSts},
{"VehBusIn_.SysSigGrp_HPCGW_3B0_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_HPCGW_3B0_APP.Message_QF},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLeverValid},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_GearLevelPosSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalStsValid},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakePedalSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_BrakeSysFault},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_AccPedalValid},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition(float32)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_ThrottlePosition},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque(float32)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_FDrvRequestTorque},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque(float32)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.VCU_RDrvRequestTorque},
{"VehBusIn_.SysSigGrp_HPCVCU_B6_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B6_APP.Message_QF},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap_VD},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap(uint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ADASPosTorqueCap},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB_VD},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB(sint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_AllMotWhlTrqFB},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag(uint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueTarget_Drag},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag(uint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueActDrag},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt(uint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_HydraTorqTgt},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalValid},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct(uint16)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_RecuBrakeTorqueAct},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent(float32)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_TheoAccePedalPercent},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_DriverAssistProhibitSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VlcHoldSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCActiveSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_ACCAvailable},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_VehicleSts},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_Vehicle_Warning},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Active},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.VCU_MAI_Available},
{"VehBusIn_.SysSigGrp_HPCVCU_B7_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_HPCVCU_B7_APP.Message_QF},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BackDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BackDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_BrakeFluidLevel},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_DriverDoorSts_VD},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FLOC_PPD_Status},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FRDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FRDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FROC_PPD_Status},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontCoverSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_FrontWiperSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtOpenSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HazardSwtSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_HighBeamCtrl},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1A},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON1B},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2A},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON2B},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_KeyPosition_ON3},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_LowBeamCtrl},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2BRelaySts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_ON2CRelaySts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RearFogCtrl},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RLOC_PPD_Status},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RMOC_PPD_Status},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RRDoorSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RRDoorSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_RROC_PPD_Status},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftCtrl},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnLeftSwtSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightCtrl},
{"VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.BCM_TurnRightSwtSts},
{"VehBusIn_.SysSigGrp_HPCBCM_290.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_290.MessageQf},
{"VehBusIn_.SysSigGrp_ICU_24D.ICU_AccelerationMode(uint8)" , &VehBusIn_.SysSigGrp_ICU_24D.ICU_AccelerationMode},
{"VehBusIn_.SysSigGrp_ICU_24D.ICU_TotalOdometerkm(uint32)" , &VehBusIn_.SysSigGrp_ICU_24D.ICU_TotalOdometerkm},
{"VehBusIn_.SysSigGrp_ICU_24D.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_ICU_24D.MessageQf},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_shake_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_levelSet},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SPEEDCHANGEVOICE_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Offset_Unit},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Upper_Offset},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset(sint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IACC_Lower_Offset},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_Cloud_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_Cloud_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLWF_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_SLIF_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_controlSW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ISA_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ELK_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_ELK_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LKA_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LKA_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LDW_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LCC_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_LCC_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_levelSet},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IHBC_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_IHBC_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_FCW_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_AEB_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_AEB_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTB_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTB_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCW_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCW_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTA_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_RCTA_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_DOW_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_DOW_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.ICU_BSD_SW(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.ICU_BSD_SW},
{"VehBusIn_.SysSigGrp_ICU_332_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_ICU_332_APP.Message_QF},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Data(uint8)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Data},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Hour(uint8)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Hour},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Minute(uint8)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Minute},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Month(uint8)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Month},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Second(uint8)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Second},
{"VehBusIn_.SysSigGrp_IVI_5CE.IVI_Year(uint16)" , &VehBusIn_.SysSigGrp_IVI_5CE.IVI_Year},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_CrashOutputSts(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_CrashOutputSts},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_FLSeatBeltRSt},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_FRSeatBeltRSt},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_RLSeatBeltRSt},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_RMSeatBeltRSt},
{"VehBusIn_.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.ACU_RRSeatBeltRSt},
{"VehBusIn_.SysSigGrp_ACU_2A3.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_ACU_2A3.MessageQf},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcce(float32)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcce},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcce(float32)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcce},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid(uint8)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LateralAcceValid},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid(uint8)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_LongitAcceValid},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRate(float32)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRate},
{"VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRateValid(uint8)" , &VehBusIn_.SysSigGrp_ACU_233_APP.ACU_YawRateValid},
{"VehBusIn_.SysSigGrp_ACU_233_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_ACU_233_APP.Message_QF},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_PulseActiveSts},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq(float32)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralCtrlHandTorq},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_MotorTorque(float32)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_MotorTorque},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralNotAvailableReason},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralActive(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralActive},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_LateralAvailable},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringModeStsFB},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque(float32)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorque},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_(uint16)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelRotSpd_},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle(float32)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteerWheelAngle},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringTorqueValidData},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleSpeedValidData},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_SteeringAngleValidData},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_FailureSts(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_FailureSts},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_CalibrationSts},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_Fault(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.EPS_Fault},
{"VehBusIn_.SysSigGrp_EPS_18D_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_EPS_18D_APP.Message_QF},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCFailed(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCFailed},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCActive(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VDCActive},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_EBDFailed(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_EBDFailed},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_Vehiclestandstill},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ESCFunctionStatus},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSFailed(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSFailed},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSActive(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_TCSActive},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSFailed(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSFailed},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSActive(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_ABSActive},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeedVD},
{"VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed(float32)" , &VehBusIn_.SysSigGrp_IBC_182_APP.IBC_VehicleSpeed},
{"VehBusIn_.SysSigGrp_IBC_182_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_IBC_182_APP.Message_QF},
{"VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied(uint8)" , &VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied},
{"VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q(uint8)" , &VehBusIn_.SysSigGrp_IBC_183.IBC_BrakePedalApplied_Q},
{"VehBusIn_.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct(uint16)" , &VehBusIn_.SysSigGrp_IBC_183.IBC_HydraTorqTgtAct},
{"VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer(uint8)" , &VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer},
{"VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q(uint8)" , &VehBusIn_.SysSigGrp_IBC_183.IBC_sOutputRodDriverPer_Q},
{"VehBusIn_.SysSigGrp_IBC_183.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_IBC_183.MessageQf},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC(uint16)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedRC},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC(uint16)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedRC},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC(uint16)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedRC},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC(uint16)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedRC},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedVD},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedVD},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedVD},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedVD},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelDirection},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelDirection},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelDirection},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelDirection},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH(float32)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RRWheelSpeedKPH},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH(float32)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_RLWheelSpeedKPH},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH(float32)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FRWheelSpeedKPH},
{"VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH(float32)" , &VehBusIn_.SysSigGrp_IBC_184_APP.IBC_FLWheelSpeedKPH},
{"VehBusIn_.SysSigGrp_IBC_184_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_IBC_184_APP.Message_QF},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VLC_Available_ACC},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_HDCStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_HDCStatus},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCACC(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCACC},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Active_ACC},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_CDD_Available_ACC},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCAEB(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_QDCAEB},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHAvailable(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHAvailable},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AVHStatus},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEBdecAvailable},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEB_active(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AEB_active},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LdmBLC(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LdmBLC},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABP_active(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABP_active},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABPAviliable(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABPAviliable},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWB_active(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWB_active},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_AWBAvaliable},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABA_active(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABA_active},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABAAvaliable},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceActValid},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct(float32)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_LongitAcceAct},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ABS_FailureLamp},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_ESP_FailureLamp},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure_Q},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure(float32)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_PlungerPressure},
{"VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.IBC_VehicleHoldStatus},
{"VehBusIn_.SysSigGrp_IBC_185_APP.Message_QF(uint8)" , &VehBusIn_.SysSigGrp_IBC_185_APP.Message_QF},
{"VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available(uint8)" , &VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_APAACCRequest_Available},
{"VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_FailStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_FailStatus},
{"VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_Status(uint8)" , &VehBusIn_.SysSigGrp_IBC_227.IBC_EPB_Status},
{"VehBusIn_.SysSigGrp_IBC_227.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_IBC_227.MessageQf},
{"VehBusIn_.SysSigGrp_IBC_3CE.iTPMS_SystemStatus(uint8)" , &VehBusIn_.SysSigGrp_IBC_3CE.iTPMS_SystemStatus},
{"VehBusIn_.SysSigGrp_IBC_3CE.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_IBC_3CE.MessageQf},
{"VehBusIn_.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq(uint8)" , &VehBusIn_.SysSigGrp_IVI_3E3.IVI_GetRidAlarmCancelReq},
{"VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_A},
{"VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_B},
{"VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_C},
{"VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B4.HPCBCM_challenge_D},
{"VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_A},
{"VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_B},
{"VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_C},
{"VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D(uint8)" , &VehBusIn_.SysSigGrp_HPCBCM_1B7.HPCBCM_response_D},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW1(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW1},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW10(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW10},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW2(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW2},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW3(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW3},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW4(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW4},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW5(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW5},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW6(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW6},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW7(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW7},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW8(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW8},
{"VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW9(uint8)" , &VehBusIn_.SysSigGrp_IBC_6F3.IBC_UDS_SW9},
{"VehBusIn_.SysSigGrp_LMC_25A.LMC_LSFCActiveSt(uint8)" , &VehBusIn_.SysSigGrp_LMC_25A.LMC_LSFCActiveSt},
{"VehBusIn_.SysSigGrp_LMC_25A.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_LMC_25A.MessageQf},
{"VehBusIn_.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt(uint8)" , &VehBusIn_.SysSigGrp_LMC_FD.LMC_FTSC_ActiveSt},
{"VehBusIn_.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt(uint8)" , &VehBusIn_.SysSigGrp_LMC_FD.LMC_HSPC_ActiveSt},
{"VehBusIn_.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt(uint8)" , &VehBusIn_.SysSigGrp_LMC_FD.LMC_WBDC_ActiveSt},
{"VehBusIn_.SysSigGrp_LMC_FD.MessageQf(uint8)" , &VehBusIn_.SysSigGrp_LMC_FD.MessageQf},
};
#endif



#ifdef VEHPARAM_TX_H
std::map<std::string, VariableVariant > VehParam_Tx_Map = {
//[VehParam_Tx]
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
{"VehParam_Tx_.WhlRadius(float32)" , &VehParam_Tx_.WhlRadius}
};
#endif



#ifdef IDT_FUNCTGTVISNID_H
std::map<std::string, VariableVariant > IDT_FuncTgtVisnID_Map = {
//[IDT_FuncTgtVisnID]
{"IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID},
{"IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID},
{"IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID},
{"IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID},
{"IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID},
};
#endif



#ifdef VEHBUSIN_H
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
#endif


#ifdef VEHPARAM_TX_H
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
#endif


#ifdef IDT_FUNCTGTVISNID_H
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
#endif


