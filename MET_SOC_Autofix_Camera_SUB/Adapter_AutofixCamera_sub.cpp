#include"Adapter_AutofixCamera.h"



using VariableVariant = std::variant<uint8_t*, uint16_t* ,uint32_t*,uint64_t*,float*,short*,int*,bool*>;


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
        
        else if constexpr (std::is_same_v<T, bool>) {
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
    

    Adapter_AutofixCamera Adapter_AutofixCamera_;
    Adapter_AutofixCamera_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libAutofixCamera", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    MET_SOC_BEVAutofixResult MET_SOC_BEVAutofixResult_;
    MET_SOC_BEVAutofixResult MET_SOC_BEVAutofixResult_old;

        std::map<std::string, VariableVariant > variableMap = {
            

            

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
    auto sub = MOS::communication::Subscriber::New(
        Adapter_AutofixCamera_.domain_id,
        Adapter_AutofixCamera_.topic, 
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

        std::memcpy(&MET_SOC_BEVAutofixResult_, data_in.data(), sizeof(MET_SOC_BEVAutofixResult));


        print_MET_SOC_BEVAutofixResult(MET_SOC_BEVAutofixResult_,MET_SOC_BEVAutofixResult_old);


        MET_SOC_BEVAutofixResult_old = MET_SOC_BEVAutofixResult_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&AutofixCamera_old, data_in.data(), sizeof(AutofixCamera));
    }
    return 0;
}

void Adapter_AutofixCamera::run()
{
    config_async_sub(json_file);
}
Adapter_AutofixCamera::Adapter_AutofixCamera()
{
}
Adapter_AutofixCamera::~Adapter_AutofixCamera()
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
    std::cout << "Running on Linux(Autofix/Camera_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_AutofixCamera objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}




/* Print struct MET_SOC_BEVAutofixResult changed value */
void print_MET_SOC_BEVAutofixResult(MET_SOC_BEVAutofixResult& MET_SOC_BEVAutofixResult_,MET_SOC_BEVAutofixResult& MET_SOC_BEVAutofixResult_old){
// std::cout << "MET_SOC_BEVAutofixResult all variable:" << std::endl;
    if (flag){
    // if(MET_SOC_BEVAutofixResult_.frameID != MET_SOC_BEVAutofixResult_old.frameID){
        std::cout << "MET_SOC_BEVAutofixResult_.frameID(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << MET_SOC_BEVAutofixResult_.frameID << std::dec  << std::endl;
        // }
    // if(MET_SOC_BEVAutofixResult_.pic_time_stamp != MET_SOC_BEVAutofixResult_old.pic_time_stamp){
        std::cout << "MET_SOC_BEVAutofixResult_.pic_time_stamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_BEVAutofixResult_.pic_time_stamp << std::dec  << std::endl;
        // }
    // if(MET_SOC_BEVAutofixResult_.result_time_stamp != MET_SOC_BEVAutofixResult_old.result_time_stamp){
        std::cout << "MET_SOC_BEVAutofixResult_.result_time_stamp(uint64_t): 0x" << std::hex << std::setw(16) << std::setfill('0') << MET_SOC_BEVAutofixResult_.result_time_stamp << std::dec  << std::endl;
        // }
        flag = false;
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
















