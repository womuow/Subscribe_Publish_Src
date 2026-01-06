#include"Adapter_CamsParam.h"
// typedef bool uint8_t;


using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,int8_t*,int16_t*,int32_t*,int64_t*,float*,CamParamUpdateType*,CameraDistortType*>;
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
    std::cout<<"data length: "<<data_in.length()<<std::endl<<"MET_SOC_CamsParam length: "<<sizeof(MET_SOC_CamsParam)<<std::endl;

    // if (flag)
    {
        std::cout << "data_in.data() size="<<data_in.size()<< std::endl;
        //print_memory(data_in.data(),data_in.size());  
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
                
               tcflush(serial_fd, TCIOFLUSH);
            // close(serial_fd);
            // break;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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
    

    Adapter_CamsParam Adapter_CamsParam_;
    Adapter_CamsParam_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libCamsParam", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    // std::string data_in = Adapter_CamsParam_.data_in;
    data_in = "";
    MET_SOC_CamsParam MET_SOC_CamsParam_;
    MET_SOC_CamsParam MET_SOC_CamsParam_old;

std::map<std::string, VariableVariant > variableMap = {
            
            
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
    auto sub = MOS::communication::Subscriber::New(
        Adapter_CamsParam_.domain_id,
        Adapter_CamsParam_.topic, 
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

    


    std::thread inputThread2(asyncInputThreadTTY);
    

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::memcpy(&MET_SOC_CamsParam_, data_in.data(), data_in.length());


        print_MET_SOC_CamsParam(MET_SOC_CamsParam_,MET_SOC_CamsParam_old);


        MET_SOC_CamsParam_old = MET_SOC_CamsParam_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&CamsParam_old, data_in.data(), sizeof(CamsParam));
    }
    return 0;
}

void Adapter_CamsParam::run()
{
    config_async_sub(json_file);
}
Adapter_CamsParam::Adapter_CamsParam()
{
}
Adapter_CamsParam::~Adapter_CamsParam()
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
    std::cout << "Running on Linux(CamsParam_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_CamsParam objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}





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















