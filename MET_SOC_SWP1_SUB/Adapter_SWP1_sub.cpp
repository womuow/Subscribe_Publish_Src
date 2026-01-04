#include"Adapter_SWP1.h"



using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*>;


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





int config_async_sub(std::string json_file) {
    

    Adapter_SWP1 Adapter_SWP1_;
    Adapter_SWP1_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libSWP1", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    MET_SOC_SWP1_Param MET_SOC_SWP1_Param_;
    MET_SOC_SWP1_Param MET_SOC_SWP1_Param_old;

        std::map<std::string, VariableVariant > variableMap = {
            

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
    auto sub = MOS::communication::Subscriber::New(
        Adapter_SWP1_.domain_id,
        Adapter_SWP1_.topic, 
        proto_info, 
        [&data_in](MOS::message::spMsg tmp) {

        auto data_vec = tmp->GetDataRef()->GetDataVec();
        auto data_size_vec = tmp->GetDataRef()->GetDataSizeVec();
        auto size = data_size_vec.size();

        for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);

        }
    }
);


    // std::thread inputThread(asyncInputThread);
    std::thread inputThread2(asyncInputThreadTTY);

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::memcpy(&MET_SOC_SWP1_Param_, data_in.data(), sizeof(MET_SOC_SWP1_Param));


        print_MET_SOC_SWP1_Param(MET_SOC_SWP1_Param_,MET_SOC_SWP1_Param_old);


        MET_SOC_SWP1_Param_old = MET_SOC_SWP1_Param_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&SWP1_old, data_in.data(), sizeof(SWP1));
    }
    return 0;
}

void Adapter_SWP1::run()
{
    config_async_sub(json_file);
}
Adapter_SWP1::Adapter_SWP1()
{
}
Adapter_SWP1::~Adapter_SWP1()
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
    std::cout << "Running on Linux(SWP1_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_SWP1 objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}


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
    flag = false;
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











