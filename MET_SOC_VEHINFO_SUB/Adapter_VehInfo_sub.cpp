#include"Adapter_VehInfo.h"



using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,float*,short*,int*,METGearInfo*>;
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
        
        else if constexpr (std::is_same_v<T, METGearInfo>) {
            // 8位类型：宽度2
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
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
    std::cout<<"data length: "<<data_in.length()<<std::endl<<"MET_SOC_VehInfo length: "<<sizeof(MET_SOC_VehInfo)<<std::endl;

    // if (flag)
    {
        std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
        print_memory(data_in.data(),data_in.size());  
        // flag=false;
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
    

    Adapter_VEHINFO Adapter_VEHINFO_;
    Adapter_VEHINFO_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libVEHINFO", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    data_in ="";
    MET_SOC_VehInfo MET_SOC_VehInfo_;
    MET_SOC_VehInfo MET_SOC_VehInfo_old;

        std::map<std::string, VariableVariant > variableMap = {
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
    auto sub = MOS::communication::Subscriber::New(
        Adapter_VEHINFO_.domain_id,
        Adapter_VEHINFO_.topic, 
        proto_info, 
        [](MOS::message::spMsg tmp) {

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

        std::memcpy(&MET_SOC_VehInfo_, data_in.data(), sizeof(MET_SOC_VehInfo));


        print_MET_SOC_VehInfo(MET_SOC_VehInfo_,MET_SOC_VehInfo_old);


        MET_SOC_VehInfo_old = MET_SOC_VehInfo_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&VEHINFO_old, data_in.data(), sizeof(VEHINFO));
    }
    return 0;
}

void Adapter_VEHINFO::run()
{
    config_async_sub(json_file);
}
Adapter_VEHINFO::Adapter_VEHINFO()
{
}
Adapter_VEHINFO::~Adapter_VEHINFO()
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
    std::cout << "Running on Linux(VEHINFO_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_VEHINFO objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}




/* Print struct MET_SOC_VehInfo changed value */
void print_MET_SOC_VehInfo(MET_SOC_VehInfo& MET_SOC_VehInfo_,MET_SOC_VehInfo& MET_SOC_VehInfo_old){
// std::cout << "MET_SOC_VehInfo all variable:" << std::endl;
    if(flag)
    {
    // if(MET_SOC_VehInfo_.frameID != MET_SOC_VehInfo_old.frameID){
        std::cout << "MET_SOC_VehInfo_.frameID(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_VehInfo_.frameID << std::dec  << std::endl;
        // }
    // if(MET_SOC_VehInfo_.timestamp != MET_SOC_VehInfo_old.timestamp){
        std::cout << "MET_SOC_VehInfo_.timestamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_VehInfo_.timestamp << std::dec  << std::endl;
        // }
        flag=false;
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














