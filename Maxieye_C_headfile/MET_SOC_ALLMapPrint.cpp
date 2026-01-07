#include "MET_SOC_ALLMapPrint.h"


std::queue<std::string> inputQueue;
int flag=true;
std::string data_in={0x0};
std::atomic_bool stop{ false };



void print_memory(const void* ptr, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(ptr);
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

void printVariableVariant(const std::string& name, VariableVariant var) {
    std::visit([&name](auto&& ptr) {
        using T = std::decay_t<decltype(*ptr)>;
        
        // 根据类型决定打印格式
        if constexpr (std::is_same_v<T, uint8_t> ) {
            // 8位类型：宽度2
            std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        // if constexpr (std::is_same_v<T, unsigned char> ) {
        //     // 8位类型：宽度2
        //     std::cout << name << ": 0x" << std::hex << std::setw(2) << std::setfill('0') 
        //               << static_cast<int>(*ptr) << std::dec << std::endl;
        // } 
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




#ifdef MET_SOC_SWP1_PARAM_H

#include "Adapter_SWP1.h"


std::map<std::string, VariableVariant > MET_SOC_SWP1_Param_Map = {
//[MET_SOC_SWP1_Param]
{"MET_SOC_SWP1_Param_.frameId(uint32_t)" , &MET_SOC_SWP1_Param_.frameId},
{"MET_SOC_SWP1_Param_.timestamps(uint64_t)" , &MET_SOC_SWP1_Param_.timestamps},
{"MET_SOC_SWP1_Param_.k_LongRangRadarType(uint8_t)" , &MET_SOC_SWP1_Param_.k_LongRangRadarType},
{"MET_SOC_SWP1_Param_.veh_config_type(uint8_t)" , &MET_SOC_SWP1_Param_.veh_config_type},
{"MET_SOC_SWP1_Param_.Platform_Type(uint8_t)" , &MET_SOC_SWP1_Param_.Platform_Type},
{"MET_SOC_SWP1_Param_.k_SideRadarType(uint8_t)" , &MET_SOC_SWP1_Param_.k_SideRadarType},
{"MET_SOC_SWP1_Param_.k_BSW_DistFrontToRearAxle(uint16_t)" , &MET_SOC_SWP1_Param_.k_BSW_DistFrontToRearAxle},
{"MET_SOC_SWP1_Param_.k_WheelBase(uint16_t)" , &MET_SOC_SWP1_Param_.k_WheelBase},
{"MET_SOC_SWP1_Param_.k_BSW_HostVehLength(uint16_t)" , &MET_SOC_SWP1_Param_.k_BSW_HostVehLength},
{"MET_SOC_SWP1_Param_.k_Vehicle_Weight(uint16_t)" , &MET_SOC_SWP1_Param_.k_Vehicle_Weight},
{"MET_SOC_SWP1_Param_.k_front_cornering_compliance(float)" , &MET_SOC_SWP1_Param_.k_front_cornering_compliance},
{"MET_SOC_SWP1_Param_.k_YawInertiaAdjustFac(float)" , &MET_SOC_SWP1_Param_.k_YawInertiaAdjustFac},
{"MET_SOC_SWP1_Param_.k_RearAxleLoadRatio(float)" , &MET_SOC_SWP1_Param_.k_RearAxleLoadRatio},
{"MET_SOC_SWP1_Param_.k_rear_compliance(float)" , &MET_SOC_SWP1_Param_.k_rear_compliance},
{"MET_SOC_SWP1_Param_.k_SteeringGearRatio(float)" , &MET_SOC_SWP1_Param_.k_SteeringGearRatio},
{"MET_SOC_SWP1_Param_.k_VehWidth_Min(uint16_t)" , &MET_SOC_SWP1_Param_.k_VehWidth_Min},
{"MET_SOC_SWP1_Param_.k_VehWidth_Max(uint16_t)" , &MET_SOC_SWP1_Param_.k_VehWidth_Max},
{"MET_SOC_SWP1_Param_.k_WheelRadius(uint16_t)" , &MET_SOC_SWP1_Param_.k_WheelRadius},
{"MET_SOC_SWP1_Param_.k_WheelWidthAve(uint16_t)" , &MET_SOC_SWP1_Param_.k_WheelWidthAve},
{"MET_SOC_SWP1_Param_.k_vehicle_speed_characteristic(uint8_t)" , &MET_SOC_SWP1_Param_.k_vehicle_speed_characteristic},
{"MET_SOC_SWP1_Param_.k_radar_VertPos_mm_MRR(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_VertPos_mm_MRR},
{"MET_SOC_SWP1_Param_.k_radar_LongPos_mm_MRR(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_LongPos_mm_MRR},
{"MET_SOC_SWP1_Param_.k_radar_LatPos_mm_MRR(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_LatPos_mm_MRR},
{"MET_SOC_SWP1_Param_.k_Radar_VKI_Major_Version_MRR(uint8_t)" , &MET_SOC_SWP1_Param_.k_Radar_VKI_Major_Version_MRR},
{"MET_SOC_SWP1_Param_.k_Radar_VKI_Minor_Version_MRR(uint8_t)" , &MET_SOC_SWP1_Param_.k_Radar_VKI_Minor_Version_MRR},
{"MET_SOC_SWP1_Param_.k_VKI_Request_dDxv_Rear_Bumper(uint16_t)" , &MET_SOC_SWP1_Param_.k_VKI_Request_dDxv_Rear_Bumper},
{"MET_SOC_SWP1_Param_.k_VKI_Request_dxv_Front_Bumper(uint16_t)" , &MET_SOC_SWP1_Param_.k_VKI_Request_dxv_Front_Bumper},
{"MET_SOC_SWP1_Param_.k_VKI_Request_CoverDamping_MRR(uint8_t)" , &MET_SOC_SWP1_Param_.k_VKI_Request_CoverDamping_MRR},
{"MET_SOC_SWP1_Param_.k_VKI_Request_aXMountPosX(uint16_t)" , &MET_SOC_SWP1_Param_.k_VKI_Request_aXMountPosX},
{"MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_MRR(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_MRR},
{"MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFL(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFL},
{"MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFL(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFL},
{"MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFL(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFL},
{"MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFL(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFL},
{"MET_SOC_SWP1_Param_.Reserved11(uint8_t)" , &MET_SOC_SWP1_Param_.Reserved11},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFL(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFL},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFL(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFL},
{"MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFR(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFR},
{"MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFR(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFR},
{"MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFR(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFR},
{"MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFR(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFR},
{"MET_SOC_SWP1_Param_.Reserved12(uint8_t)" , &MET_SOC_SWP1_Param_.Reserved12},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFR(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFR},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFR(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFR},
{"MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRL(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRL},
{"MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRL(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRL},
{"MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRL(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRL},
{"MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRL(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRL},
{"MET_SOC_SWP1_Param_.Reserved13(uint8_t)" , &MET_SOC_SWP1_Param_.Reserved13},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRL(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRL},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRL(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRL},
{"MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRR(uint16_t)" , &MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRR},
{"MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRR(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRR},
{"MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRR(int16_t)" , &MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRR},
{"MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRR(int8_t)" , &MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRR},
{"MET_SOC_SWP1_Param_.Reserved14(uint8_t)" , &MET_SOC_SWP1_Param_.Reserved14},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRR(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRR},
{"MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRR(float)" , &MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRR},
{"MET_SOC_SWP1_Param_.k_origin_point_of_coordinate_SRR(uint8_t)" , &MET_SOC_SWP1_Param_.k_origin_point_of_coordinate_SRR},
{"MET_SOC_SWP1_Param_.k_orientation_of_coordinate_SRR(uint8_t)" , &MET_SOC_SWP1_Param_.k_orientation_of_coordinate_SRR},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_x(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_x},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_y(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_y},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_z},
{"MET_SOC_SWP1_Param_.Reserved15(uint16_t)" , &MET_SOC_SWP1_Param_.Reserved15},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_x(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_x},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_y(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_y},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_z},
{"MET_SOC_SWP1_Param_.Reserved16(uint16_t)" , &MET_SOC_SWP1_Param_.Reserved16},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_x(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_x},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_y(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_y},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_z},
{"MET_SOC_SWP1_Param_.Reserved17(uint16_t)" , &MET_SOC_SWP1_Param_.Reserved17},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_x(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_x},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_y(int16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_y},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_z},
{"MET_SOC_SWP1_Param_.Reserved18(uint16_t)" , &MET_SOC_SWP1_Param_.Reserved18},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_BackCAM_x(int16_t)" , &MET_SOC_SWP1_Param_.k_BackCAM_x},
{"MET_SOC_SWP1_Param_.k_BackCAM_y(int16_t)" , &MET_SOC_SWP1_Param_.k_BackCAM_y},
{"MET_SOC_SWP1_Param_.k_BackCAM_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_BackCAM_z},
{"MET_SOC_SWP1_Param_.Reserved19(uint16_t)" , &MET_SOC_SWP1_Param_.Reserved19},
{"MET_SOC_SWP1_Param_.k_BackCAM_Yaw(float)" , &MET_SOC_SWP1_Param_.k_BackCAM_Yaw},
{"MET_SOC_SWP1_Param_.k_BackCAM_Pitch(float)" , &MET_SOC_SWP1_Param_.k_BackCAM_Pitch},
{"MET_SOC_SWP1_Param_.k_BackCAM_Roll(float)" , &MET_SOC_SWP1_Param_.k_BackCAM_Roll},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_x(int16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_x},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_y(int16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_y},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_z},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_x(int16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_x},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_y(int16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_y},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_z(uint16_t)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_z},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta_Online},
{"MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta_Online(float)" , &MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta_Online},
{"MET_SOC_SWP1_Param_.k_IMU_x(int16_t)" , &MET_SOC_SWP1_Param_.k_IMU_x},
{"MET_SOC_SWP1_Param_.k_IMU_y(int16_t)" , &MET_SOC_SWP1_Param_.k_IMU_y},
{"MET_SOC_SWP1_Param_.k_IMU_z(int16_t)" , &MET_SOC_SWP1_Param_.k_IMU_z},
{"MET_SOC_SWP1_Param_.k_IMU_z_Rotaion(float)" , &MET_SOC_SWP1_Param_.k_IMU_z_Rotaion},
{"MET_SOC_SWP1_Param_.k_IMU_x_Rotaion(float)" , &MET_SOC_SWP1_Param_.k_IMU_x_Rotaion},
{"MET_SOC_SWP1_Param_.k_IMU_y_Rotaion(float)" , &MET_SOC_SWP1_Param_.k_IMU_y_Rotaion},
{"MET_SOC_SWP1_Param_.k_GNSS_type(uint8_t)" , &MET_SOC_SWP1_Param_.k_GNSS_type},
};

/* Print struct MET_SOC_SWP1_Param changed value */
void print_MET_SOC_SWP1_Param(MET_SOC_SWP1_Param& MET_SOC_SWP1_Param_,MET_SOC_SWP1_Param& MET_SOC_SWP1_Param_old){
// std::cout << "MET_SOC_SWP1_Param all variable:" << std::endl;
    if(flag)
    {
    if(MET_SOC_SWP1_Param_.frameId != MET_SOC_SWP1_Param_old.frameId){
        std::cout << "MET_SOC_SWP1_Param_.frameId(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_SWP1_Param_.frameId << std::dec  << std::endl;
        }

    if(MET_SOC_SWP1_Param_.timestamps != MET_SOC_SWP1_Param_old.timestamps){
        std::cout << "MET_SOC_SWP1_Param_.timestamps(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_SWP1_Param_.timestamps << std::dec  << std::endl;
        }
    }
    if(MET_SOC_SWP1_Param_.k_LongRangRadarType != MET_SOC_SWP1_Param_old.k_LongRangRadarType){
        std::cout << "MET_SOC_SWP1_Param_.k_LongRangRadarType(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_LongRangRadarType) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.veh_config_type != MET_SOC_SWP1_Param_old.veh_config_type){
        std::cout << "MET_SOC_SWP1_Param_.veh_config_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.veh_config_type) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Platform_Type != MET_SOC_SWP1_Param_old.Platform_Type){
        std::cout << "MET_SOC_SWP1_Param_.Platform_Type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.Platform_Type) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_SideRadarType != MET_SOC_SWP1_Param_old.k_SideRadarType){
        std::cout << "MET_SOC_SWP1_Param_.k_SideRadarType(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_SideRadarType) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BSW_DistFrontToRearAxle != MET_SOC_SWP1_Param_old.k_BSW_DistFrontToRearAxle){
        std::cout << "MET_SOC_SWP1_Param_.k_BSW_DistFrontToRearAxle(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_BSW_DistFrontToRearAxle << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_WheelBase != MET_SOC_SWP1_Param_old.k_WheelBase){
        std::cout << "MET_SOC_SWP1_Param_.k_WheelBase(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_WheelBase << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BSW_HostVehLength != MET_SOC_SWP1_Param_old.k_BSW_HostVehLength){
        std::cout << "MET_SOC_SWP1_Param_.k_BSW_HostVehLength(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_BSW_HostVehLength << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_Vehicle_Weight != MET_SOC_SWP1_Param_old.k_Vehicle_Weight){
        std::cout << "MET_SOC_SWP1_Param_.k_Vehicle_Weight(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_Vehicle_Weight << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_front_cornering_compliance != MET_SOC_SWP1_Param_old.k_front_cornering_compliance){
        std::cout << "MET_SOC_SWP1_Param_.k_front_cornering_compliance(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_front_cornering_compliance << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_YawInertiaAdjustFac != MET_SOC_SWP1_Param_old.k_YawInertiaAdjustFac){
        std::cout << "MET_SOC_SWP1_Param_.k_YawInertiaAdjustFac(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_YawInertiaAdjustFac << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_RearAxleLoadRatio != MET_SOC_SWP1_Param_old.k_RearAxleLoadRatio){
        std::cout << "MET_SOC_SWP1_Param_.k_RearAxleLoadRatio(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_RearAxleLoadRatio << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_rear_compliance != MET_SOC_SWP1_Param_old.k_rear_compliance){
        std::cout << "MET_SOC_SWP1_Param_.k_rear_compliance(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_rear_compliance << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_SteeringGearRatio != MET_SOC_SWP1_Param_old.k_SteeringGearRatio){
        std::cout << "MET_SOC_SWP1_Param_.k_SteeringGearRatio(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_SteeringGearRatio << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VehWidth_Min != MET_SOC_SWP1_Param_old.k_VehWidth_Min){
        std::cout << "MET_SOC_SWP1_Param_.k_VehWidth_Min(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_VehWidth_Min << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VehWidth_Max != MET_SOC_SWP1_Param_old.k_VehWidth_Max){
        std::cout << "MET_SOC_SWP1_Param_.k_VehWidth_Max(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_VehWidth_Max << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_WheelRadius != MET_SOC_SWP1_Param_old.k_WheelRadius){
        std::cout << "MET_SOC_SWP1_Param_.k_WheelRadius(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_WheelRadius << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_WheelWidthAve != MET_SOC_SWP1_Param_old.k_WheelWidthAve){
        std::cout << "MET_SOC_SWP1_Param_.k_WheelWidthAve(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_WheelWidthAve << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_vehicle_speed_characteristic != MET_SOC_SWP1_Param_old.k_vehicle_speed_characteristic){
        std::cout << "MET_SOC_SWP1_Param_.k_vehicle_speed_characteristic(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_vehicle_speed_characteristic) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_VertPos_mm_MRR != MET_SOC_SWP1_Param_old.k_radar_VertPos_mm_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_VertPos_mm_MRR(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_VertPos_mm_MRR << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_MRR != MET_SOC_SWP1_Param_old.k_radar_LongPos_mm_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LongPos_mm_MRR(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_LongPos_mm_MRR << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_MRR != MET_SOC_SWP1_Param_old.k_radar_LatPos_mm_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LatPos_mm_MRR(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_MRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_Radar_VKI_Major_Version_MRR != MET_SOC_SWP1_Param_old.k_Radar_VKI_Major_Version_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_Radar_VKI_Major_Version_MRR(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_Radar_VKI_Major_Version_MRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_Radar_VKI_Minor_Version_MRR != MET_SOC_SWP1_Param_old.k_Radar_VKI_Minor_Version_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_Radar_VKI_Minor_Version_MRR(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_Radar_VKI_Minor_Version_MRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VKI_Request_dDxv_Rear_Bumper != MET_SOC_SWP1_Param_old.k_VKI_Request_dDxv_Rear_Bumper){
        std::cout << "MET_SOC_SWP1_Param_.k_VKI_Request_dDxv_Rear_Bumper(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_VKI_Request_dDxv_Rear_Bumper << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VKI_Request_dxv_Front_Bumper != MET_SOC_SWP1_Param_old.k_VKI_Request_dxv_Front_Bumper){
        std::cout << "MET_SOC_SWP1_Param_.k_VKI_Request_dxv_Front_Bumper(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_VKI_Request_dxv_Front_Bumper << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VKI_Request_CoverDamping_MRR != MET_SOC_SWP1_Param_old.k_VKI_Request_CoverDamping_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_VKI_Request_CoverDamping_MRR(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_VKI_Request_CoverDamping_MRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_VKI_Request_aXMountPosX != MET_SOC_SWP1_Param_old.k_VKI_Request_aXMountPosX){
        std::cout << "MET_SOC_SWP1_Param_.k_VKI_Request_aXMountPosX(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_VKI_Request_aXMountPosX << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_MRR != MET_SOC_SWP1_Param_old.k_radar_azimuth_polarity_MRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_MRR(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_MRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFL != MET_SOC_SWP1_Param_old.k_radar_VertPos_mm_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFL(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFL << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFL != MET_SOC_SWP1_Param_old.k_radar_LongPos_mm_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFL(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFL != MET_SOC_SWP1_Param_old.k_radar_LatPos_mm_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFL(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFL != MET_SOC_SWP1_Param_old.k_radar_azimuth_polarity_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFL(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved11 != MET_SOC_SWP1_Param_old.Reserved11){
        std::cout << "MET_SOC_SWP1_Param_.Reserved11(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.Reserved11) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFL != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Pitch_deg_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFL(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFL << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFL != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Yaw_deg_SRRFL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFL(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFL << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFR != MET_SOC_SWP1_Param_old.k_radar_VertPos_mm_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFR(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRFR << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFR != MET_SOC_SWP1_Param_old.k_radar_LongPos_mm_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFR(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRFR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFR != MET_SOC_SWP1_Param_old.k_radar_LatPos_mm_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFR(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRFR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFR != MET_SOC_SWP1_Param_old.k_radar_azimuth_polarity_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFR(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRFR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved12 != MET_SOC_SWP1_Param_old.Reserved12){
        std::cout << "MET_SOC_SWP1_Param_.Reserved12(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.Reserved12) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFR != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Pitch_deg_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFR(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRFR << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFR != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Yaw_deg_SRRFR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFR(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRFR << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRL != MET_SOC_SWP1_Param_old.k_radar_VertPos_mm_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRL(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRL << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRL != MET_SOC_SWP1_Param_old.k_radar_LongPos_mm_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRL(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRL != MET_SOC_SWP1_Param_old.k_radar_LatPos_mm_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRL(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRL != MET_SOC_SWP1_Param_old.k_radar_azimuth_polarity_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRL(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRL) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved13 != MET_SOC_SWP1_Param_old.Reserved13){
        std::cout << "MET_SOC_SWP1_Param_.Reserved13(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.Reserved13) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRL != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Pitch_deg_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRL(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRL << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRL != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Yaw_deg_SRRRL){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRL(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRL << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRR != MET_SOC_SWP1_Param_old.k_radar_VertPos_mm_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRR(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_radar_VertPos_mm_SRRRR << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRR != MET_SOC_SWP1_Param_old.k_radar_LongPos_mm_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRR(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LongPos_mm_SRRRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRR != MET_SOC_SWP1_Param_old.k_radar_LatPos_mm_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRR(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_LatPos_mm_SRRRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRR != MET_SOC_SWP1_Param_old.k_radar_azimuth_polarity_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRR(int8_t): 0x" << static_cast<int>(MET_SOC_SWP1_Param_.k_radar_azimuth_polarity_SRRRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved14 != MET_SOC_SWP1_Param_old.Reserved14){
        std::cout << "MET_SOC_SWP1_Param_.Reserved14(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.Reserved14) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRR != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Pitch_deg_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRR(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Pitch_deg_SRRRR << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRR != MET_SOC_SWP1_Param_old.k_radar_boresight_angle_Yaw_deg_SRRRR){
        std::cout << "MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRR(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_radar_boresight_angle_Yaw_deg_SRRRR << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_origin_point_of_coordinate_SRR != MET_SOC_SWP1_Param_old.k_origin_point_of_coordinate_SRR){
        std::cout << "MET_SOC_SWP1_Param_.k_origin_point_of_coordinate_SRR(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_origin_point_of_coordinate_SRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_orientation_of_coordinate_SRR != MET_SOC_SWP1_Param_old.k_orientation_of_coordinate_SRR){
        std::cout << "MET_SOC_SWP1_Param_.k_orientation_of_coordinate_SRR(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_orientation_of_coordinate_SRR) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_x != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_x){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_y != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_y){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_z != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_z){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved15 != MET_SOC_SWP1_Param_old.Reserved15){
        std::cout << "MET_SOC_SWP1_Param_.Reserved15(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.Reserved15 << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_FrontCAM_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_FrontCAM_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_x != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_x){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_y != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_y){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_z != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_z){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved16 != MET_SOC_SWP1_Param_old.Reserved16){
        std::cout << "MET_SOC_SWP1_Param_.Reserved16(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.Reserved16 << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_LeftCAM_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_LeftCAM_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_x != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_x){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_RightCAM_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_y != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_y){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_RightCAM_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_z != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_z){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_AVM_RightCAM_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved17 != MET_SOC_SWP1_Param_old.Reserved17){
        std::cout << "MET_SOC_SWP1_Param_.Reserved17(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.Reserved17 << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RightCAM_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RightCAM_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_x != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_x){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_RearCAM_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_y != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_y){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_AVM_RearCAM_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_z != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_z){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_AVM_RearCAM_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved18 != MET_SOC_SWP1_Param_old.Reserved18){
        std::cout << "MET_SOC_SWP1_Param_.Reserved18(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.Reserved18 << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_RearCAM_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_RearCAM_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_x != MET_SOC_SWP1_Param_old.k_BackCAM_x){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_BackCAM_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_y != MET_SOC_SWP1_Param_old.k_BackCAM_y){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_BackCAM_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_z != MET_SOC_SWP1_Param_old.k_BackCAM_z){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_BackCAM_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.Reserved19 != MET_SOC_SWP1_Param_old.Reserved19){
        std::cout << "MET_SOC_SWP1_Param_.Reserved19(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.Reserved19 << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_Yaw != MET_SOC_SWP1_Param_old.k_BackCAM_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_BackCAM_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_Pitch != MET_SOC_SWP1_Param_old.k_BackCAM_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_BackCAM_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_BackCAM_Roll != MET_SOC_SWP1_Param_old.k_BackCAM_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_BackCAM_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_BackCAM_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_AVM_BackCAM_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_AVM_BackCAM_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_x != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_x){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_y != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_y){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_z != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_z){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Narrow_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Narrow_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_x != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_x){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_y != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_y){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_z != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_z){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_z(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_z << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Yaw){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Pitch){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Roll){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Pitch_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Yaw_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Roll_Delta){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Pitch_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Pitch_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Yaw_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Yaw_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta_Online != MET_SOC_SWP1_Param_old.k_FrontCAM_Wide_Roll_Delta_Online){
        std::cout << "MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta_Online(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_FrontCAM_Wide_Roll_Delta_Online << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_x != MET_SOC_SWP1_Param_old.k_IMU_x){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_x(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_IMU_x) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_y != MET_SOC_SWP1_Param_old.k_IMU_y){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_y(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_IMU_y) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_z != MET_SOC_SWP1_Param_old.k_IMU_z){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_z(int16_t): " << static_cast<int>(MET_SOC_SWP1_Param_.k_IMU_z) << std::dec  << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_z_Rotaion != MET_SOC_SWP1_Param_old.k_IMU_z_Rotaion){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_z_Rotaion(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_IMU_z_Rotaion << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_x_Rotaion != MET_SOC_SWP1_Param_old.k_IMU_x_Rotaion){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_x_Rotaion(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_IMU_x_Rotaion << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_IMU_y_Rotaion != MET_SOC_SWP1_Param_old.k_IMU_y_Rotaion){
        std::cout << "MET_SOC_SWP1_Param_.k_IMU_y_Rotaion(float): " << std::fixed << std::setprecision(2) << MET_SOC_SWP1_Param_.k_IMU_y_Rotaion << std::endl;
        }
    if(MET_SOC_SWP1_Param_.k_GNSS_type != MET_SOC_SWP1_Param_old.k_GNSS_type){
        std::cout << "MET_SOC_SWP1_Param_.k_GNSS_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_SWP1_Param_.k_GNSS_type) << std::dec  << std::endl;
        }
}



#endif





#ifdef MET_SOC_CAMSPARAM_H

#include"Adapter_CamsParam.h"


std::map<std::string, VariableVariant > MET_SOC_CamsParam_Map = {
//[MET_SOC_CamsParam]
{"MET_SOC_CamsParam_.update_count(uint32_t)" , &MET_SOC_CamsParam_.update_count},
{"MET_SOC_CamsParam_.update_type(uint8_t)" , &MET_SOC_CamsParam_.update_type},
{"MET_SOC_CamsParam_.cams_num(uint16_t)" , &MET_SOC_CamsParam_.cams_num},
{"MET_SOC_CamsParam_.cams[0].support_update(uint8_t)" , &MET_SOC_CamsParam_.cams[0].support_update},
{"MET_SOC_CamsParam_.cams[0].distort_type(uint8_t)" , &MET_SOC_CamsParam_.cams[0].distort_type},
{"MET_SOC_CamsParam_.cams[0].cam_idx(uint8_t)" , &MET_SOC_CamsParam_.cams[0].cam_idx},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.position_x(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.position_x},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.position_y(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.position_y},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.position_z(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.position_z},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.yaw(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.yaw},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.roll(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.roll},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.pitch(float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.pitch},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][3]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][0](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][0]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][1](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][1]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][2](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][2]},
{"MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][3](float)" , &MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][3]},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_X(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_X},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_Y(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_Y},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_X(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_X},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_Y(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_Y},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_X(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_X},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_Y(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_Y},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgWidth(uint16_t)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgWidth},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgHeight(uint16_t)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgHeight},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_X(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_X},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_Y(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_Y},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_H(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_H},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_V(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_V},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgWidth(uint16_t)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgWidth},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgHeight(uint16_t)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgHeight},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_X(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_X},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_Y(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_Y},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_H(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_H},
{"MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_V(float)" , &MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_V},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k1(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k1},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k2(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k2},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k3(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k3},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k4(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k4},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k5(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k5},
{"MET_SOC_CamsParam_.cams[0].data.distortion.k6(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.k6},
{"MET_SOC_CamsParam_.cams[0].data.distortion.p1(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.p1},
{"MET_SOC_CamsParam_.cams[0].data.distortion.p2(float)" , &MET_SOC_CamsParam_.cams[0].data.distortion.p2},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Cint[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Cint[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.K[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.K[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[9](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[9]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[10](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[10]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[11](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[11]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[12](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[12]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[13](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[13]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[14](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[14]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[15](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[15]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.H[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.H[8]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[0](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[0]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[1](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[1]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[2](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[2]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[3](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[3]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[4](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[4]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[5](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[5]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[6](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[6]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[7](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[7]},
{"MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[8](float)" , &MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[8]},
};



/* Print struct MET_SOC_CamsParam changed value */
void print_MET_SOC_CamsParam(MET_SOC_CamsParam& MET_SOC_CamsParam_,MET_SOC_CamsParam& MET_SOC_CamsParam_old){
// std::cout << "MET_SOC_CamsParam all variable:" << std::endl;
    if(MET_SOC_CamsParam_.update_count != MET_SOC_CamsParam_old.update_count){
        std::cout << "MET_SOC_CamsParam_.update_count(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_CamsParam_.update_count << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.update_type != MET_SOC_CamsParam_old.update_type){
        std::cout << "MET_SOC_CamsParam_.update_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_CamsParam_.update_type) << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams_num != MET_SOC_CamsParam_old.cams_num){
        std::cout << "MET_SOC_CamsParam_.cams_num(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_CamsParam_.cams_num << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].support_update != MET_SOC_CamsParam_old.cams[0].support_update){
        std::cout << "MET_SOC_CamsParam_.cams[0].support_update(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_CamsParam_.cams[0].support_update) << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].distort_type != MET_SOC_CamsParam_old.cams[0].distort_type){
        std::cout << "MET_SOC_CamsParam_.cams[0].distort_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_CamsParam_.cams[0].distort_type) << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].cam_idx != MET_SOC_CamsParam_old.cams[0].cam_idx){
        std::cout << "MET_SOC_CamsParam_.cams[0].cam_idx(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_CamsParam_.cams[0].cam_idx) << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.position_x != MET_SOC_CamsParam_old.cams[0].data.extrinsic.position_x){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.position_x(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.position_x << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.position_y != MET_SOC_CamsParam_old.cams[0].data.extrinsic.position_y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.position_y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.position_y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.position_z != MET_SOC_CamsParam_old.cams[0].data.extrinsic.position_z){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.position_z(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.position_z << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.yaw != MET_SOC_CamsParam_old.cams[0].data.extrinsic.yaw){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.yaw << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.roll != MET_SOC_CamsParam_old.cams[0].data.extrinsic.roll){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.roll << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.pitch != MET_SOC_CamsParam_old.cams[0].data.extrinsic.pitch){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.pitch << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[0][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[0][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[0][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[0][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[0][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[1][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[1][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[1][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[1][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[1][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[2][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[2][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[2][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[2][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[2][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[3][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[3][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[3][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.cam2Body[3][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.cam2Body[3][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[0][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[0][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[0][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[0][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[0][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[1][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[1][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[1][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[1][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[1][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[2][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[2][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[2][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[2][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[2][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][0] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[3][0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][1] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[3][1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][2] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[3][2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][3] != MET_SOC_CamsParam_old.cams[0].data.extrinsic.body2Cam[3][3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.extrinsic.body2Cam[3][3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_X != MET_SOC_CamsParam_old.cams[0].data.intrinsic.pixelPitch_X){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_X(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_X << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_Y != MET_SOC_CamsParam_old.cams[0].data.intrinsic.pixelPitch_Y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_Y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.pixelPitch_Y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength != MET_SOC_CamsParam_old.cams[0].data.intrinsic.focalLength){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_X != MET_SOC_CamsParam_old.cams[0].data.intrinsic.focalLength_X){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_X(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_X << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_Y != MET_SOC_CamsParam_old.cams[0].data.intrinsic.focalLength_Y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_Y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.focalLength_Y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_focalLength){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_X != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_focalLength_X){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_X(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_X << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_Y != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_focalLength_Y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_Y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_focalLength_Y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgWidth != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_imgWidth){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgWidth(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgWidth << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgHeight != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_imgHeight){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgHeight(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_imgHeight << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_X != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_principalPoint_X){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_X(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_X << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_Y != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_principalPoint_Y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_Y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_principalPoint_Y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_H != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_camFov_H){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_H(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_H << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_V != MET_SOC_CamsParam_old.cams[0].data.intrinsic.distort_camFov_V){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_V(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.distort_camFov_V << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgWidth != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_imgWidth){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgWidth(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgWidth << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgHeight != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_imgHeight){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgHeight(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_imgHeight << std::dec  << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_X != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_principalPoint_X){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_X(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_X << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_Y != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_principalPoint_Y){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_Y(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_principalPoint_Y << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_H != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_camFov_H){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_H(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_H << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_V != MET_SOC_CamsParam_old.cams[0].data.intrinsic.undistort_camFov_V){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_V(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.intrinsic.undistort_camFov_V << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k1 != MET_SOC_CamsParam_old.cams[0].data.distortion.k1){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k1(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k1 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k2 != MET_SOC_CamsParam_old.cams[0].data.distortion.k2){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k2(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k2 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k3 != MET_SOC_CamsParam_old.cams[0].data.distortion.k3){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k3(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k3 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k4 != MET_SOC_CamsParam_old.cams[0].data.distortion.k4){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k4(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k4 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k5 != MET_SOC_CamsParam_old.cams[0].data.distortion.k5){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k5(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k5 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.k6 != MET_SOC_CamsParam_old.cams[0].data.distortion.k6){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.k6(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.k6 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.p1 != MET_SOC_CamsParam_old.cams[0].data.distortion.p1){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.p1(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.p1 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.distortion.p2 != MET_SOC_CamsParam_old.cams[0].data.distortion.p2){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.distortion.p2(float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.distortion.p2 << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Cint[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.Cint[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Cint[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Cint[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.CintInv[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.CintInv[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.Rrot[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Rrot[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.RrotInv[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.RrotInv[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.Ttrs[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Ttrs[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.TtrsInv[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.TtrsInv[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.K[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.K[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.K[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.K[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[9] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[9]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[9] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[10] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[10]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[10] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[11] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[11]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[11] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[12] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[12]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[12] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[13] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[13]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[13] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[14] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[14]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[14] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[15] != MET_SOC_CamsParam_old.cams[0].data.camParam.Kinv[15]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Kinv[15] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.H[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.H[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.H[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.H[8] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[0] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[0]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[0] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[1] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[1]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[1] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[2] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[2]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[2] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[3] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[3]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[3] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[4] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[4]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[4] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[5] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[5]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[5] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[6] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[6]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[6] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[7] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[7]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[7] << std::endl;
        }
    if(MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[8] != MET_SOC_CamsParam_old.cams[0].data.camParam.Hinv[8]){
        std::cout << "MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_CamsParam_.cams[0].data.camParam.Hinv[8] << std::endl;
        }
}
#endif



#ifdef MET_SOC_BEVAUTOFIXRESULT_H
std::map<std::string, VariableVariant > MET_SOC_BEVAutofixResult_Map = {
//[MET_SOC_BEVAutofixResult]
{"MET_SOC_BEVAutofixResult_.frameID(uint32_t)" , &MET_SOC_BEVAutofixResult_.frameID},
{"MET_SOC_BEVAutofixResult_.pic_time_stamp(uint64_t)" , &MET_SOC_BEVAutofixResult_.pic_time_stamp},
{"MET_SOC_BEVAutofixResult_.result_time_stamp(uint64_t)" , &MET_SOC_BEVAutofixResult_.result_time_stamp},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.roll(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.roll},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.yaw(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.yaw},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.pitch(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.pitch},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.percentage(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.percentage},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.confidence(float)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.confidence},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_is_in_autofix_process(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_is_in_autofix_process},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_valid},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_timeout(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_timeout},
{"MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_current_frame_valid(bool)" , &MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_current_frame_valid},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_vp_error(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_vp_error},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_insufficient_lane(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_insufficient_lane},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_yawrate(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_yawrate},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_low_speed(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_low_speed},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_heading(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_heading},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_line_type(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_line_type},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_wz(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_wz},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_vp_error(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_vp_error},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_vp_error(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_vp_error},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_vp_error(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_vp_error},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_vp_error(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_vp_error},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[19]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_not_single_line(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_not_single_line},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_front(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_front},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch(bool)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[0](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[0]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[1](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[1]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[2](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[2]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[3](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[3]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[4](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[4]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[5](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[5]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[6](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[6]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[7](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[7]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[8](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[8]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[9](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[9]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[10](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[10]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[11](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[11]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[12](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[12]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[13](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[13]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[14](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[14]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[15](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[15]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[16](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[16]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[17](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[17]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[18](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[18]},
{"MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[19](float)" , &MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[19]},
};




/* Print struct MET_SOC_BEVAutofixResult changed value */
void print_MET_SOC_BEVAutofixResult(MET_SOC_BEVAutofixResult& MET_SOC_BEVAutofixResult_,MET_SOC_BEVAutofixResult& MET_SOC_BEVAutofixResult_old){
// std::cout << "MET_SOC_BEVAutofixResult all variable:" << std::endl;
    if(MET_SOC_BEVAutofixResult_.frameID != MET_SOC_BEVAutofixResult_old.frameID){
        std::cout << "MET_SOC_BEVAutofixResult_.frameID(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_BEVAutofixResult_.frameID << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.pic_time_stamp != MET_SOC_BEVAutofixResult_old.pic_time_stamp){
        std::cout << "MET_SOC_BEVAutofixResult_.pic_time_stamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_BEVAutofixResult_.pic_time_stamp << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.result_time_stamp != MET_SOC_BEVAutofixResult_old.result_time_stamp){
        std::cout << "MET_SOC_BEVAutofixResult_.result_time_stamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_BEVAutofixResult_.result_time_stamp << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camF_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camF_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camFN_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camFN_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camB_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camB_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLF_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLF_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRF_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRF_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camLB_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camLB_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.camRB_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.camRB_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMF_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMF_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMB_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMB_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVML_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVML_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.roll != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.roll(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.roll << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.yaw != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.yaw){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.yaw(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.yaw << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.pitch != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.pitch(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.pitch << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.percentage != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.percentage){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.percentage(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.percentage << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.confidence != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.confidence){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.confidence(float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.confidence << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_is_in_autofix_process != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.f_is_in_autofix_process){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_is_in_autofix_process(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_is_in_autofix_process) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.f_calib_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_timeout != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.f_calib_timeout){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_timeout(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_calib_timeout) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_current_frame_valid != MET_SOC_BEVAutofixResult_old.autofixBuffer.AVMR_info.f_current_frame_valid){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_current_frame_valid(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixBuffer.AVMR_info.f_current_frame_valid) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_vp_error != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_vp_error){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_vp_error(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_vp_error) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_insufficient_lane != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_insufficient_lane){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_insufficient_lane(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_insufficient_lane) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_yawrate != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_large_yawrate){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_yawrate(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_yawrate) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_low_speed != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_low_speed){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_low_speed(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_low_speed) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_heading != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_large_heading){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_heading(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_heading) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_line_type != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_invalid_line_type){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_line_type(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_line_type) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_wz != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_large_wz){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_wz(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_large_wz) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camF_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camF_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_vp_error != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.f_vp_error){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_vp_error(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_vp_error) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camFN_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camFN_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_vp_error != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.f_vp_error){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_vp_error(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_vp_error) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camB_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camB_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLF_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLF_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRF_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRF_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camLB_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camLB_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.camRB_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.camRB_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_vp_error != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.f_vp_error){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_vp_error(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_vp_error) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMF_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMF_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_vp_error != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.f_vp_error){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_vp_error(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_vp_error) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMB_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMB_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVML_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVML_debug_info.reserve[19] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_not_single_line != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.f_not_single_line){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_not_single_line(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_not_single_line) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_point_num) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_front != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.f_invalid_front){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_front(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_front) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_roll) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch(bool): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.f_invalid_pitch) << std::dec  << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[0] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[0]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[0](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[0] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[1] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[1]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[1](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[1] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[2] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[2]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[2](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[2] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[3] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[3]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[3](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[3] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[4] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[4]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[4](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[4] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[5] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[5]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[5](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[5] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[6] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[6]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[6](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[6] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[7] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[7]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[7](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[7] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[8] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[8]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[8](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[8] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[9] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[9]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[9](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[9] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[10] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[10]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[10](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[10] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[11] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[11]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[11](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[11] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[12] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[12]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[12](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[12] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[13] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[13]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[13](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[13] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[14] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[14]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[14](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[14] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[15] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[15]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[15](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[15] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[16] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[16]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[16](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[16] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[17] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[17]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[17](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[17] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[18] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[18]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[18](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[18] << std::endl;
        }
    if(MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[19] != MET_SOC_BEVAutofixResult_old.autofixDebugBuffer.AVMR_debug_info.reserve[19]){
        std::cout << "MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[19](float): " << std::fixed << std::setprecision(2) << MET_SOC_BEVAutofixResult_.autofixDebugBuffer.AVMR_debug_info.reserve[19] << std::endl;
        }
}
#endif







#ifdef MET_SOC_VEHINFO_H
std::map<std::string, VariableVariant > MET_SOC_VehInfo_Map = {
//[MET_SOC_VehInfo]
{"MET_SOC_VehInfo_.frameID(uint32_t)" , &MET_SOC_VehInfo_.frameID},
{"MET_SOC_VehInfo_.timestamp(uint64_t)" , &MET_SOC_VehInfo_.timestamp},
{"MET_SOC_VehInfo_.Speed_Mode(uint8_t)" , &MET_SOC_VehInfo_.Speed_Mode},
{"MET_SOC_VehInfo_.VehInfo.vehicle_speed(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_speed},
{"MET_SOC_VehInfo_.VehInfo.steering_wheel_angle(float)" , &MET_SOC_VehInfo_.VehInfo.steering_wheel_angle},
{"MET_SOC_VehInfo_.VehInfo.accelerator_pedal_pos(float)" , &MET_SOC_VehInfo_.VehInfo.accelerator_pedal_pos},
{"MET_SOC_VehInfo_.VehInfo.accelerator_pedal_speed(short)" , &MET_SOC_VehInfo_.VehInfo.accelerator_pedal_speed},
{"MET_SOC_VehInfo_.VehInfo.brake_pedal_pos(float)" , &MET_SOC_VehInfo_.VehInfo.brake_pedal_pos},
{"MET_SOC_VehInfo_.VehInfo.vehicle_yaw_angle(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_yaw_angle},
{"MET_SOC_VehInfo_.VehInfo.vehicle_yaw_rate(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_yaw_rate},
{"MET_SOC_VehInfo_.VehInfo.vehicle_pitch_angle(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_pitch_angle},
{"MET_SOC_VehInfo_.VehInfo.vehicle_pitch_rate(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_pitch_rate},
{"MET_SOC_VehInfo_.VehInfo.Lft_turnlight_active(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.Lft_turnlight_active},
{"MET_SOC_VehInfo_.VehInfo.Rght_turnlight_active(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.Rght_turnlight_active},
{"MET_SOC_VehInfo_.VehInfo.windWiperStatus(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.windWiperStatus},
{"MET_SOC_VehInfo_.VehInfo.light_sensor_val(short)" , &MET_SOC_VehInfo_.VehInfo.light_sensor_val},
{"MET_SOC_VehInfo_.VehInfo.rain_sensor_val(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.rain_sensor_val},
{"MET_SOC_VehInfo_.VehInfo.lowBeamStatus(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.lowBeamStatus},
{"MET_SOC_VehInfo_.VehInfo.highBeamStatus(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.highBeamStatus},
{"MET_SOC_VehInfo_.VehInfo.envirLightStatus(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.envirLightStatus},
{"MET_SOC_VehInfo_.VehInfo.brakePedalGrd(short)" , &MET_SOC_VehInfo_.VehInfo.brakePedalGrd},
{"MET_SOC_VehInfo_.VehInfo.brakePedalFlag(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.brakePedalFlag},
{"MET_SOC_VehInfo_.VehInfo.temperatureValue(float)" , &MET_SOC_VehInfo_.VehInfo.temperatureValue},
{"MET_SOC_VehInfo_.VehInfo.steeringWheelSpeed(short)" , &MET_SOC_VehInfo_.VehInfo.steeringWheelSpeed},
{"MET_SOC_VehInfo_.VehInfo.gearInfo(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.gearInfo},
{"MET_SOC_VehInfo_.VehInfo.estimated_gear(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.estimated_gear},
{"MET_SOC_VehInfo_.VehInfo.mileage(int)" , &MET_SOC_VehInfo_.VehInfo.mileage},
{"MET_SOC_VehInfo_.VehInfo.vehicle_roll_angle(float)" , &MET_SOC_VehInfo_.VehInfo.vehicle_roll_angle},
{"MET_SOC_VehInfo_.VehInfo.vehicel_roll_rate(float)" , &MET_SOC_VehInfo_.VehInfo.vehicel_roll_rate},
{"MET_SOC_VehInfo_.VehInfo.gpsInfo.longitude(int32_t)" , &MET_SOC_VehInfo_.VehInfo.gpsInfo.longitude},
{"MET_SOC_VehInfo_.VehInfo.gpsInfo.latitude(int32_t)" , &MET_SOC_VehInfo_.VehInfo.gpsInfo.latitude},
{"MET_SOC_VehInfo_.VehInfo.gpsInfo.height(float)" , &MET_SOC_VehInfo_.VehInfo.gpsInfo.height},
{"MET_SOC_VehInfo_.VehInfo.aeb_brake_failed(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.aeb_brake_failed},
{"MET_SOC_VehInfo_.VehInfo.engine_min_torque_value(short)" , &MET_SOC_VehInfo_.VehInfo.engine_min_torque_value},
{"MET_SOC_VehInfo_.VehInfo.engine_max_torque_value(short)" , &MET_SOC_VehInfo_.VehInfo.engine_max_torque_value},
{"MET_SOC_VehInfo_.VehInfo.engine_cur_torque_value(short)" , &MET_SOC_VehInfo_.VehInfo.engine_cur_torque_value},
{"MET_SOC_VehInfo_.VehInfo.acc_brake_failed(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.acc_brake_failed},
{"MET_SOC_VehInfo_.VehInfo.cruise_control_enable(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.cruise_control_enable},
{"MET_SOC_VehInfo_.VehInfo.cruise_gap_switch_activation(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.cruise_gap_switch_activation},
{"MET_SOC_VehInfo_.VehInfo.cruise_secondary_switch_status(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.cruise_secondary_switch_status},
{"MET_SOC_VehInfo_.VehInfo.cruise_speed_limit_switch_status(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.cruise_speed_limit_switch_status},
{"MET_SOC_VehInfo_.VehInfo.lka_overlay_torque_status(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.lka_overlay_torque_status},
{"MET_SOC_VehInfo_.VehInfo.lka_steering_total_torque(float)" , &MET_SOC_VehInfo_.VehInfo.lka_steering_total_torque},
{"MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_mode(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_mode},
{"MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_stat(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_stat},
{"MET_SOC_VehInfo_.VehInfo.lka_steering_delta_torque(float)" , &MET_SOC_VehInfo_.VehInfo.lka_steering_delta_torque},
{"MET_SOC_VehInfo_.VehInfo.lka_driver_applied_torque(float)" , &MET_SOC_VehInfo_.VehInfo.lka_driver_applied_torque},
{"MET_SOC_VehInfo_.VehInfo.lateral_acceleration_primary(float)" , &MET_SOC_VehInfo_.VehInfo.lateral_acceleration_primary},
{"MET_SOC_VehInfo_.VehInfo.lateral_acceleration_secondary(float)" , &MET_SOC_VehInfo_.VehInfo.lateral_acceleration_secondary},
{"MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_primary(float)" , &MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_primary},
{"MET_SOC_VehInfo_.VehInfo.filter_longitudinal_acceleration_primary(float)" , &MET_SOC_VehInfo_.VehInfo.filter_longitudinal_acceleration_primary},
{"MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_secondary(float)" , &MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_secondary},
{"MET_SOC_VehInfo_.VehInfo.filter_yawrate_primary(float)" , &MET_SOC_VehInfo_.VehInfo.filter_yawrate_primary},
{"MET_SOC_VehInfo_.VehInfo.yawrate_primary(float)" , &MET_SOC_VehInfo_.VehInfo.yawrate_primary},
{"MET_SOC_VehInfo_.VehInfo.yawrate_secondary(float)" , &MET_SOC_VehInfo_.VehInfo.yawrate_secondary},
{"MET_SOC_VehInfo_.VehInfo.wheel_brake_pressure_estimated(float)" , &MET_SOC_VehInfo_.VehInfo.wheel_brake_pressure_estimated},
{"MET_SOC_VehInfo_.VehInfo.braking_actual_deceleration(float)" , &MET_SOC_VehInfo_.VehInfo.braking_actual_deceleration},
{"MET_SOC_VehInfo_.VehInfo.braking_target_deceleration(float)" , &MET_SOC_VehInfo_.VehInfo.braking_target_deceleration},
{"MET_SOC_VehInfo_.VehInfo.gateway_button(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.gateway_button},
{"MET_SOC_VehInfo_.VehInfo.wheel_status_feback_zcsd(uint8_t)" , &MET_SOC_VehInfo_.VehInfo.wheel_status_feback_zcsd},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLatAccelValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLatAccelValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLongAccelValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLongAccelValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.abs_active(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.abs_active},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.steeringWheelAngleValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.steeringWheelAngleValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.accelPedPosPctValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.accelPedPosPctValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.mainBeamIndication(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.mainBeamIndication},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.wiperFrontCmd(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.wiperFrontCmd},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.reverseGear(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.reverseGear},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.brakePedalPressed(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.brakePedalPressed},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleLatAccel(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleLatAccel},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleLongAccel(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleLongAccel},
{"MET_SOC_VehInfo_.ToVseVehInfo.YawRateTimeStamp_ms(uint32_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.YawRateTimeStamp_ms},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocity(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocity},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleVerticalAccel(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleVerticalAccel},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRate(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRate},
{"MET_SOC_VehInfo_.ToVseVehInfo.steeringWheelAngle(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.steeringWheelAngle},
{"MET_SOC_VehInfo_.ToVseVehInfo.accelPedPosPct(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.accelPedPosPct},
{"MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpd(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpd},
{"MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpd(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpd},
{"MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpd(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpd},
{"MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpd(float)" , &MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpd},
{"MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_high(uint32_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_high},
{"MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_low(uint32_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_low},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehIndex(uint32_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehIndex},
{"MET_SOC_VehInfo_.ToVseVehInfo.odometer(uint32_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.odometer},
{"MET_SOC_VehInfo_.ToVseVehInfo.countryIdentification(uint16_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.countryIdentification},
{"MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirection(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirection},
{"MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirectionValid(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirectionValid},
{"MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirection(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirection},
{"MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirectionValid(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirectionValid},
{"MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Year(uint16_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Year},
{"MET_SOC_VehInfo_.ToVseVehInfo.crc_16(uint16_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.crc_16},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_turnIndicator(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_turnIndicator},
{"MET_SOC_VehInfo_.ToVseVehInfo.met_wiperSpeedInfo(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.met_wiperSpeedInfo},
{"MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirection(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirection},
{"MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirectionValid(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirectionValid},
{"MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMajor(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMajor},
{"MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMinor(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMinor},
{"MET_SOC_VehInfo_.ToVseVehInfo.currentGear(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.currentGear},
{"MET_SOC_VehInfo_.ToVseVehInfo.fcwWarningSensitivityLevel(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.fcwWarningSensitivityLevel},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocityValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocityValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.WheelSlipEvent(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.WheelSlipEvent},
{"MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRateValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRateValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Month(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Month},
{"MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Day(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Day},
{"MET_SOC_VehInfo_.ToVseVehInfo.Hours(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Hours},
{"MET_SOC_VehInfo_.ToVseVehInfo.Minutes(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Minutes},
{"MET_SOC_VehInfo_.ToVseVehInfo.Seconds(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.Seconds},
{"MET_SOC_VehInfo_.ToVseVehInfo.kl15_state(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.kl15_state},
{"MET_SOC_VehInfo_.ToVseVehInfo.brakePedalPosPctValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.brakePedalPosPctValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpdValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpdValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpdValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpdValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpdValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpdValidity},
{"MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpdValidity(uint8_t)" , &MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpdValidity},
};



/* Print struct MET_SOC_VehInfo changed value */
void print_MET_SOC_VehInfo(MET_SOC_VehInfo& MET_SOC_VehInfo_,MET_SOC_VehInfo& MET_SOC_VehInfo_old){
// std::cout << "MET_SOC_VehInfo all variable:" << std::endl;
    if(MET_SOC_VehInfo_.frameID != MET_SOC_VehInfo_old.frameID){
        std::cout << "MET_SOC_VehInfo_.frameID(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.frameID << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.timestamp != MET_SOC_VehInfo_old.timestamp){
        std::cout << "MET_SOC_VehInfo_.timestamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_VehInfo_.timestamp << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.Speed_Mode != MET_SOC_VehInfo_old.Speed_Mode){
        std::cout << "MET_SOC_VehInfo_.Speed_Mode(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.Speed_Mode) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_speed != MET_SOC_VehInfo_old.VehInfo.vehicle_speed){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_speed(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_speed << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.steering_wheel_angle != MET_SOC_VehInfo_old.VehInfo.steering_wheel_angle){
        std::cout << "MET_SOC_VehInfo_.VehInfo.steering_wheel_angle(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.steering_wheel_angle << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.accelerator_pedal_pos != MET_SOC_VehInfo_old.VehInfo.accelerator_pedal_pos){
        std::cout << "MET_SOC_VehInfo_.VehInfo.accelerator_pedal_pos(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.accelerator_pedal_pos << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.accelerator_pedal_speed != MET_SOC_VehInfo_old.VehInfo.accelerator_pedal_speed){
        std::cout << "MET_SOC_VehInfo_.VehInfo.accelerator_pedal_speed(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.accelerator_pedal_speed) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.brake_pedal_pos != MET_SOC_VehInfo_old.VehInfo.brake_pedal_pos){
        std::cout << "MET_SOC_VehInfo_.VehInfo.brake_pedal_pos(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.brake_pedal_pos << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_yaw_angle != MET_SOC_VehInfo_old.VehInfo.vehicle_yaw_angle){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_yaw_angle(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_yaw_angle << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_yaw_rate != MET_SOC_VehInfo_old.VehInfo.vehicle_yaw_rate){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_yaw_rate(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_yaw_rate << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_pitch_angle != MET_SOC_VehInfo_old.VehInfo.vehicle_pitch_angle){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_pitch_angle(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_pitch_angle << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_pitch_rate != MET_SOC_VehInfo_old.VehInfo.vehicle_pitch_rate){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_pitch_rate(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_pitch_rate << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.Lft_turnlight_active != MET_SOC_VehInfo_old.VehInfo.Lft_turnlight_active){
        std::cout << "MET_SOC_VehInfo_.VehInfo.Lft_turnlight_active(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.Lft_turnlight_active) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.Rght_turnlight_active != MET_SOC_VehInfo_old.VehInfo.Rght_turnlight_active){
        std::cout << "MET_SOC_VehInfo_.VehInfo.Rght_turnlight_active(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.Rght_turnlight_active) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.windWiperStatus != MET_SOC_VehInfo_old.VehInfo.windWiperStatus){
        std::cout << "MET_SOC_VehInfo_.VehInfo.windWiperStatus(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.windWiperStatus) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.light_sensor_val != MET_SOC_VehInfo_old.VehInfo.light_sensor_val){
        std::cout << "MET_SOC_VehInfo_.VehInfo.light_sensor_val(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.light_sensor_val) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.rain_sensor_val != MET_SOC_VehInfo_old.VehInfo.rain_sensor_val){
        std::cout << "MET_SOC_VehInfo_.VehInfo.rain_sensor_val(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.rain_sensor_val) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lowBeamStatus != MET_SOC_VehInfo_old.VehInfo.lowBeamStatus){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lowBeamStatus(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.lowBeamStatus) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.highBeamStatus != MET_SOC_VehInfo_old.VehInfo.highBeamStatus){
        std::cout << "MET_SOC_VehInfo_.VehInfo.highBeamStatus(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.highBeamStatus) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.envirLightStatus != MET_SOC_VehInfo_old.VehInfo.envirLightStatus){
        std::cout << "MET_SOC_VehInfo_.VehInfo.envirLightStatus(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.envirLightStatus) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.brakePedalGrd != MET_SOC_VehInfo_old.VehInfo.brakePedalGrd){
        std::cout << "MET_SOC_VehInfo_.VehInfo.brakePedalGrd(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.brakePedalGrd) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.brakePedalFlag != MET_SOC_VehInfo_old.VehInfo.brakePedalFlag){
        std::cout << "MET_SOC_VehInfo_.VehInfo.brakePedalFlag(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.brakePedalFlag) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.temperatureValue != MET_SOC_VehInfo_old.VehInfo.temperatureValue){
        std::cout << "MET_SOC_VehInfo_.VehInfo.temperatureValue(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.temperatureValue << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.steeringWheelSpeed != MET_SOC_VehInfo_old.VehInfo.steeringWheelSpeed){
        std::cout << "MET_SOC_VehInfo_.VehInfo.steeringWheelSpeed(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.steeringWheelSpeed) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.gearInfo != MET_SOC_VehInfo_old.VehInfo.gearInfo){
        std::cout << "MET_SOC_VehInfo_.VehInfo.gearInfo(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.gearInfo) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.estimated_gear != MET_SOC_VehInfo_old.VehInfo.estimated_gear){
        std::cout << "MET_SOC_VehInfo_.VehInfo.estimated_gear(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.estimated_gear) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.mileage != MET_SOC_VehInfo_old.VehInfo.mileage){
        std::cout << "MET_SOC_VehInfo_.VehInfo.mileage(int): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.mileage) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicle_roll_angle != MET_SOC_VehInfo_old.VehInfo.vehicle_roll_angle){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicle_roll_angle(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicle_roll_angle << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.vehicel_roll_rate != MET_SOC_VehInfo_old.VehInfo.vehicel_roll_rate){
        std::cout << "MET_SOC_VehInfo_.VehInfo.vehicel_roll_rate(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.vehicel_roll_rate << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.gpsInfo.longitude != MET_SOC_VehInfo_old.VehInfo.gpsInfo.longitude){
        std::cout << "MET_SOC_VehInfo_.VehInfo.gpsInfo.longitude(int32_t): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.gpsInfo.longitude) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.gpsInfo.latitude != MET_SOC_VehInfo_old.VehInfo.gpsInfo.latitude){
        std::cout << "MET_SOC_VehInfo_.VehInfo.gpsInfo.latitude(int32_t): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.gpsInfo.latitude) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.gpsInfo.height != MET_SOC_VehInfo_old.VehInfo.gpsInfo.height){
        std::cout << "MET_SOC_VehInfo_.VehInfo.gpsInfo.height(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.gpsInfo.height << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.aeb_brake_failed != MET_SOC_VehInfo_old.VehInfo.aeb_brake_failed){
        std::cout << "MET_SOC_VehInfo_.VehInfo.aeb_brake_failed(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.aeb_brake_failed) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.engine_min_torque_value != MET_SOC_VehInfo_old.VehInfo.engine_min_torque_value){
        std::cout << "MET_SOC_VehInfo_.VehInfo.engine_min_torque_value(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.engine_min_torque_value) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.engine_max_torque_value != MET_SOC_VehInfo_old.VehInfo.engine_max_torque_value){
        std::cout << "MET_SOC_VehInfo_.VehInfo.engine_max_torque_value(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.engine_max_torque_value) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.engine_cur_torque_value != MET_SOC_VehInfo_old.VehInfo.engine_cur_torque_value){
        std::cout << "MET_SOC_VehInfo_.VehInfo.engine_cur_torque_value(short): " << static_cast<int>(MET_SOC_VehInfo_.VehInfo.engine_cur_torque_value) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.acc_brake_failed != MET_SOC_VehInfo_old.VehInfo.acc_brake_failed){
        std::cout << "MET_SOC_VehInfo_.VehInfo.acc_brake_failed(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.acc_brake_failed) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.cruise_control_enable != MET_SOC_VehInfo_old.VehInfo.cruise_control_enable){
        std::cout << "MET_SOC_VehInfo_.VehInfo.cruise_control_enable(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.cruise_control_enable) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.cruise_gap_switch_activation != MET_SOC_VehInfo_old.VehInfo.cruise_gap_switch_activation){
        std::cout << "MET_SOC_VehInfo_.VehInfo.cruise_gap_switch_activation(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.cruise_gap_switch_activation) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.cruise_secondary_switch_status != MET_SOC_VehInfo_old.VehInfo.cruise_secondary_switch_status){
        std::cout << "MET_SOC_VehInfo_.VehInfo.cruise_secondary_switch_status(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.cruise_secondary_switch_status) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.cruise_speed_limit_switch_status != MET_SOC_VehInfo_old.VehInfo.cruise_speed_limit_switch_status){
        std::cout << "MET_SOC_VehInfo_.VehInfo.cruise_speed_limit_switch_status(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.cruise_speed_limit_switch_status) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_overlay_torque_status != MET_SOC_VehInfo_old.VehInfo.lka_overlay_torque_status){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_overlay_torque_status(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.lka_overlay_torque_status) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_steering_total_torque != MET_SOC_VehInfo_old.VehInfo.lka_steering_total_torque){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_steering_total_torque(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.lka_steering_total_torque << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_mode != MET_SOC_VehInfo_old.VehInfo.lka_handoff_steering_wheel_mode){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_mode(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_mode) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_stat != MET_SOC_VehInfo_old.VehInfo.lka_handoff_steering_wheel_stat){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_stat(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.lka_handoff_steering_wheel_stat) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_steering_delta_torque != MET_SOC_VehInfo_old.VehInfo.lka_steering_delta_torque){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_steering_delta_torque(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.lka_steering_delta_torque << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lka_driver_applied_torque != MET_SOC_VehInfo_old.VehInfo.lka_driver_applied_torque){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lka_driver_applied_torque(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.lka_driver_applied_torque << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lateral_acceleration_primary != MET_SOC_VehInfo_old.VehInfo.lateral_acceleration_primary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lateral_acceleration_primary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.lateral_acceleration_primary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.lateral_acceleration_secondary != MET_SOC_VehInfo_old.VehInfo.lateral_acceleration_secondary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.lateral_acceleration_secondary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.lateral_acceleration_secondary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_primary != MET_SOC_VehInfo_old.VehInfo.longitudinal_acceleration_primary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_primary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_primary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.filter_longitudinal_acceleration_primary != MET_SOC_VehInfo_old.VehInfo.filter_longitudinal_acceleration_primary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.filter_longitudinal_acceleration_primary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.filter_longitudinal_acceleration_primary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_secondary != MET_SOC_VehInfo_old.VehInfo.longitudinal_acceleration_secondary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_secondary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.longitudinal_acceleration_secondary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.filter_yawrate_primary != MET_SOC_VehInfo_old.VehInfo.filter_yawrate_primary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.filter_yawrate_primary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.filter_yawrate_primary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.yawrate_primary != MET_SOC_VehInfo_old.VehInfo.yawrate_primary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.yawrate_primary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.yawrate_primary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.yawrate_secondary != MET_SOC_VehInfo_old.VehInfo.yawrate_secondary){
        std::cout << "MET_SOC_VehInfo_.VehInfo.yawrate_secondary(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.yawrate_secondary << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.wheel_brake_pressure_estimated != MET_SOC_VehInfo_old.VehInfo.wheel_brake_pressure_estimated){
        std::cout << "MET_SOC_VehInfo_.VehInfo.wheel_brake_pressure_estimated(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.wheel_brake_pressure_estimated << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.braking_actual_deceleration != MET_SOC_VehInfo_old.VehInfo.braking_actual_deceleration){
        std::cout << "MET_SOC_VehInfo_.VehInfo.braking_actual_deceleration(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.braking_actual_deceleration << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.braking_target_deceleration != MET_SOC_VehInfo_old.VehInfo.braking_target_deceleration){
        std::cout << "MET_SOC_VehInfo_.VehInfo.braking_target_deceleration(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.VehInfo.braking_target_deceleration << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.gateway_button != MET_SOC_VehInfo_old.VehInfo.gateway_button){
        std::cout << "MET_SOC_VehInfo_.VehInfo.gateway_button(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.gateway_button) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.VehInfo.wheel_status_feback_zcsd != MET_SOC_VehInfo_old.VehInfo.wheel_status_feback_zcsd){
        std::cout << "MET_SOC_VehInfo_.VehInfo.wheel_status_feback_zcsd(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.VehInfo.wheel_status_feback_zcsd) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLatAccelValidity != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.vehicleLatAccelValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLatAccelValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLatAccelValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLongAccelValidity != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.vehicleLongAccelValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLongAccelValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleLongAccelValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.abs_active != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.abs_active){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.abs_active(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.abs_active) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.vehicleVerticalAccelValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.steeringWheelAngleValidity != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.steeringWheelAngleValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.steeringWheelAngleValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.steeringWheelAngleValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.accelPedPosPctValidity != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.accelPedPosPctValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.accelPedPosPctValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.accelPedPosPctValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirection) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid != MET_SOC_VehInfo_old.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_Validity.RR_WheelRotatedDirectionValid) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.mainBeamIndication != MET_SOC_VehInfo_old.ToVseVehInfo.met_mwrb.mainBeamIndication){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.mainBeamIndication(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.mainBeamIndication) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.wiperFrontCmd != MET_SOC_VehInfo_old.ToVseVehInfo.met_mwrb.wiperFrontCmd){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.wiperFrontCmd(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.wiperFrontCmd) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.reverseGear != MET_SOC_VehInfo_old.ToVseVehInfo.met_mwrb.reverseGear){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.reverseGear(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.reverseGear) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.brakePedalPressed != MET_SOC_VehInfo_old.ToVseVehInfo.met_mwrb.brakePedalPressed){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.brakePedalPressed(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_mwrb.brakePedalPressed) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleLatAccel != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleLatAccel){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleLatAccel(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.vehicleLatAccel << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleLongAccel != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleLongAccel){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleLongAccel(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.vehicleLongAccel << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.YawRateTimeStamp_ms != MET_SOC_VehInfo_old.ToVseVehInfo.YawRateTimeStamp_ms){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.YawRateTimeStamp_ms(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.YawRateTimeStamp_ms << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocity != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleVelocity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocity(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocity << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleVerticalAccel != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleVerticalAccel){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleVerticalAccel(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.vehicleVerticalAccel << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRate != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleYawRate){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRate(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRate << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.steeringWheelAngle != MET_SOC_VehInfo_old.ToVseVehInfo.steeringWheelAngle){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.steeringWheelAngle(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.steeringWheelAngle << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.accelPedPosPct != MET_SOC_VehInfo_old.ToVseVehInfo.accelPedPosPct){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.accelPedPosPct(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.accelPedPosPct << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpd != MET_SOC_VehInfo_old.ToVseVehInfo.LF_WheelSpd){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpd(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpd << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpd != MET_SOC_VehInfo_old.ToVseVehInfo.RF_WheelSpd){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpd(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpd << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpd != MET_SOC_VehInfo_old.ToVseVehInfo.LR_WheelSpd){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpd(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpd << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpd != MET_SOC_VehInfo_old.ToVseVehInfo.RR_WheelSpd){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpd(float): " << std::fixed << std::setprecision(2) << MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpd << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_high != MET_SOC_VehInfo_old.ToVseVehInfo.mcu_pts_value_high){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_high(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_high << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_low != MET_SOC_VehInfo_old.ToVseVehInfo.mcu_pts_value_low){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_low(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.mcu_pts_value_low << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehIndex != MET_SOC_VehInfo_old.ToVseVehInfo.vehIndex){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehIndex(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.vehIndex << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.odometer != MET_SOC_VehInfo_old.ToVseVehInfo.odometer){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.odometer(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.odometer << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.countryIdentification != MET_SOC_VehInfo_old.ToVseVehInfo.countryIdentification){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.countryIdentification(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.countryIdentification << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirection != MET_SOC_VehInfo_old.ToVseVehInfo.LF_WheelRotatedDirection){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirection(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirection) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirectionValid != MET_SOC_VehInfo_old.ToVseVehInfo.LF_WheelRotatedDirectionValid){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirectionValid(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelRotatedDirectionValid) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirection != MET_SOC_VehInfo_old.ToVseVehInfo.RF_WheelRotatedDirection){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirection(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirection) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirectionValid != MET_SOC_VehInfo_old.ToVseVehInfo.RF_WheelRotatedDirectionValid){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirectionValid(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelRotatedDirectionValid) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Year != MET_SOC_VehInfo_old.ToVseVehInfo.Calendar_Year){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Year(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Year << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.crc_16 != MET_SOC_VehInfo_old.ToVseVehInfo.crc_16){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.crc_16(uint16_t): 0x" << std::hex << std::setw(4) << std::setfill('0') << MET_SOC_VehInfo_.ToVseVehInfo.crc_16 << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_turnIndicator != MET_SOC_VehInfo_old.ToVseVehInfo.met_turnIndicator){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_turnIndicator(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_turnIndicator) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.met_wiperSpeedInfo != MET_SOC_VehInfo_old.ToVseVehInfo.met_wiperSpeedInfo){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.met_wiperSpeedInfo(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.met_wiperSpeedInfo) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirection != MET_SOC_VehInfo_old.ToVseVehInfo.LR_WheelRotatedDirection){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirection(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirection) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirectionValid != MET_SOC_VehInfo_old.ToVseVehInfo.LR_WheelRotatedDirectionValid){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirectionValid(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelRotatedDirectionValid) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMajor != MET_SOC_VehInfo_old.ToVseVehInfo.apiVersionMajor){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMajor(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMajor) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMinor != MET_SOC_VehInfo_old.ToVseVehInfo.apiVersionMinor){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMinor(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.apiVersionMinor) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.currentGear != MET_SOC_VehInfo_old.ToVseVehInfo.currentGear){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.currentGear(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.currentGear) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.fcwWarningSensitivityLevel != MET_SOC_VehInfo_old.ToVseVehInfo.fcwWarningSensitivityLevel){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.fcwWarningSensitivityLevel(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.fcwWarningSensitivityLevel) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocityValidity != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleVelocityValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocityValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.vehicleVelocityValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.WheelSlipEvent != MET_SOC_VehInfo_old.ToVseVehInfo.WheelSlipEvent){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.WheelSlipEvent(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.WheelSlipEvent) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRateValidity != MET_SOC_VehInfo_old.ToVseVehInfo.vehicleYawRateValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRateValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.vehicleYawRateValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Month != MET_SOC_VehInfo_old.ToVseVehInfo.Calendar_Month){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Month(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Month) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Day != MET_SOC_VehInfo_old.ToVseVehInfo.Calendar_Day){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Day(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.Calendar_Day) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Hours != MET_SOC_VehInfo_old.ToVseVehInfo.Hours){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Hours(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.Hours) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Minutes != MET_SOC_VehInfo_old.ToVseVehInfo.Minutes){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Minutes(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.Minutes) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.Seconds != MET_SOC_VehInfo_old.ToVseVehInfo.Seconds){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.Seconds(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.Seconds) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.kl15_state != MET_SOC_VehInfo_old.ToVseVehInfo.kl15_state){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.kl15_state(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.kl15_state) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.brakePedalPosPctValidity != MET_SOC_VehInfo_old.ToVseVehInfo.brakePedalPosPctValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.brakePedalPosPctValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.brakePedalPosPctValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpdValidity != MET_SOC_VehInfo_old.ToVseVehInfo.LF_WheelSpdValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpdValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LF_WheelSpdValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpdValidity != MET_SOC_VehInfo_old.ToVseVehInfo.RF_WheelSpdValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpdValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.RF_WheelSpdValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpdValidity != MET_SOC_VehInfo_old.ToVseVehInfo.LR_WheelSpdValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpdValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.LR_WheelSpdValidity) << std::dec  << std::endl;
        }
    if(MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpdValidity != MET_SOC_VehInfo_old.ToVseVehInfo.RR_WheelSpdValidity){
        std::cout << "MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpdValidity(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MET_SOC_VehInfo_.ToVseVehInfo.RR_WheelSpdValidity) << std::dec  << std::endl;
        }
}
#endif









