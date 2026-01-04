#include"Adapter_EM_CMD.h"



using VariableVariant = std::variant<uint8_t*, uint16_t*, uint32_t*, uint64_t*, int8_t*, int16_t*, int32_t*, int64_t*, float*, bool*,char *,RecoveryAction*>;


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
        

        else if constexpr ( std::is_same_v<T, int8_t>) {
            // 8位类型：宽度2
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec  << std::endl;
        }

        else if constexpr ( std::is_same_v<T, int16_t>) {
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
        std::cout << "error: '" << input << "' is too int16_t" << std::endl;
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
    

    Adapter_EM_CMD Adapter_EM_CMD_;
    Adapter_EM_CMD_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libEM_CMD", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    RecoveryCommand RecoveryCommand_;
    RecoveryCommand RecoveryCommand_old;

    RecoveryResponse RecoveryResponse_;
    RecoveryResponse RecoveryResponse_old;

    



    std::map<std::string, VariableVariant > variableMap = {
 
        // [RecoveryCommand]
{"RecoveryCommand_.process_name[0](int8_t)" , &RecoveryCommand_.process_name[0]},
{"RecoveryCommand_.process_name[1](int8_t)" , &RecoveryCommand_.process_name[1]},
{"RecoveryCommand_.process_name[2](int8_t)" , &RecoveryCommand_.process_name[2]},
{"RecoveryCommand_.process_name[3](int8_t)" , &RecoveryCommand_.process_name[3]},
{"RecoveryCommand_.process_name[4](int8_t)" , &RecoveryCommand_.process_name[4]},
{"RecoveryCommand_.process_name[5](int8_t)" , &RecoveryCommand_.process_name[5]},
{"RecoveryCommand_.process_name[6](int8_t)" , &RecoveryCommand_.process_name[6]},
{"RecoveryCommand_.process_name[7](int8_t)" , &RecoveryCommand_.process_name[7]},
{"RecoveryCommand_.process_name[8](int8_t)" , &RecoveryCommand_.process_name[8]},
{"RecoveryCommand_.process_name[9](int8_t)" , &RecoveryCommand_.process_name[9]},
{"RecoveryCommand_.process_name[10](int8_t)" , &RecoveryCommand_.process_name[10]},
{"RecoveryCommand_.process_name[11](int8_t)" , &RecoveryCommand_.process_name[11]},
{"RecoveryCommand_.process_name[12](int8_t)" , &RecoveryCommand_.process_name[12]},
{"RecoveryCommand_.process_name[13](int8_t)" , &RecoveryCommand_.process_name[13]},
{"RecoveryCommand_.process_name[14](int8_t)" , &RecoveryCommand_.process_name[14]},
{"RecoveryCommand_.process_name[15](int8_t)" , &RecoveryCommand_.process_name[15]},
{"RecoveryCommand_.process_name[16](int8_t)" , &RecoveryCommand_.process_name[16]},
{"RecoveryCommand_.process_name[17](int8_t)" , &RecoveryCommand_.process_name[17]},
{"RecoveryCommand_.process_name[18](int8_t)" , &RecoveryCommand_.process_name[18]},
{"RecoveryCommand_.process_name[19](int8_t)" , &RecoveryCommand_.process_name[19]},
{"RecoveryCommand_.process_name[20](int8_t)" , &RecoveryCommand_.process_name[20]},
{"RecoveryCommand_.process_name[21](int8_t)" , &RecoveryCommand_.process_name[21]},
{"RecoveryCommand_.process_name[22](int8_t)" , &RecoveryCommand_.process_name[22]},
{"RecoveryCommand_.process_name[23](int8_t)" , &RecoveryCommand_.process_name[23]},
{"RecoveryCommand_.process_name[24](int8_t)" , &RecoveryCommand_.process_name[24]},
{"RecoveryCommand_.process_name[25](int8_t)" , &RecoveryCommand_.process_name[25]},
{"RecoveryCommand_.process_name[26](int8_t)" , &RecoveryCommand_.process_name[26]},
{"RecoveryCommand_.process_name[27](int8_t)" , &RecoveryCommand_.process_name[27]},
{"RecoveryCommand_.process_name[28](int8_t)" , &RecoveryCommand_.process_name[28]},
{"RecoveryCommand_.process_name[29](int8_t)" , &RecoveryCommand_.process_name[29]},
{"RecoveryCommand_.process_name[30](int8_t)" , &RecoveryCommand_.process_name[30]},
{"RecoveryCommand_.process_name[31](int8_t)" , &RecoveryCommand_.process_name[31]},
{"RecoveryCommand_.action{}(uint8_t)" , &RecoveryCommand_.action},
{"RecoveryCommand_.error_code{}(uint32_t)" , &RecoveryCommand_.error_code},




// [RecoveryResponse]
{"RecoveryResponse_.success{}(uint8_t)" , &RecoveryResponse_.success},



        };


    auto sub = MOS::communication::Subscriber::New(
        Adapter_EM_CMD_.domain_id,
        Adapter_EM_CMD_.topic, 
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
        if(data_in.length()>0)
        {
            switch (data_in.length())
            {
                case sizeof(RecoveryCommand):
                {
                    std::memcpy(&RecoveryCommand_, data_in.data(), sizeof(RecoveryCommand));
                    print_RecoveryCommand(RecoveryCommand_,RecoveryCommand_old);
                    RecoveryCommand_old = RecoveryCommand_;                
                    break;
                }
                case sizeof(RecoveryResponse):
                {
                    std::memcpy(&RecoveryResponse_, data_in.data(), sizeof(RecoveryResponse));
                    print_RecoveryResponse(RecoveryResponse_,RecoveryResponse_old);
                    RecoveryResponse_old = RecoveryResponse_;                
                    break;
                }
                

            }
        }

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        data_in.clear();
    }
    return 0;
}

void Adapter_EM_CMD::run()
{
    config_async_sub(json_file);
}
Adapter_EM_CMD::Adapter_EM_CMD()
{
}
Adapter_EM_CMD::~Adapter_EM_CMD()
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
    std::cout << "Running on Linux(EM_CMD_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_EM_CMD objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}









/* Print struct RecoveryCommand changed value */
void print_RecoveryCommand(RecoveryCommand& RecoveryCommand_,RecoveryCommand& RecoveryCommand_old){
// std::cout << "RecoveryCommand all variable:" << std::endl;
    if(RecoveryCommand_.process_name[0] != RecoveryCommand_old.process_name[0]){
        std::cout << "RecoveryCommand_.process_name[0](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[0]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[1] != RecoveryCommand_old.process_name[1]){
        std::cout << "RecoveryCommand_.process_name[1](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[1]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[2] != RecoveryCommand_old.process_name[2]){
        std::cout << "RecoveryCommand_.process_name[2](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[2]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[3] != RecoveryCommand_old.process_name[3]){
        std::cout << "RecoveryCommand_.process_name[3](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[3]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[4] != RecoveryCommand_old.process_name[4]){
        std::cout << "RecoveryCommand_.process_name[4](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[4]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[5] != RecoveryCommand_old.process_name[5]){
        std::cout << "RecoveryCommand_.process_name[5](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[5]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[6] != RecoveryCommand_old.process_name[6]){
        std::cout << "RecoveryCommand_.process_name[6](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[6]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[7] != RecoveryCommand_old.process_name[7]){
        std::cout << "RecoveryCommand_.process_name[7](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[7]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[8] != RecoveryCommand_old.process_name[8]){
        std::cout << "RecoveryCommand_.process_name[8](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[8]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[9] != RecoveryCommand_old.process_name[9]){
        std::cout << "RecoveryCommand_.process_name[9](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[9]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[10] != RecoveryCommand_old.process_name[10]){
        std::cout << "RecoveryCommand_.process_name[10](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[10]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[11] != RecoveryCommand_old.process_name[11]){
        std::cout << "RecoveryCommand_.process_name[11](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[11]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[12] != RecoveryCommand_old.process_name[12]){
        std::cout << "RecoveryCommand_.process_name[12](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[12]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[13] != RecoveryCommand_old.process_name[13]){
        std::cout << "RecoveryCommand_.process_name[13](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[13]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[14] != RecoveryCommand_old.process_name[14]){
        std::cout << "RecoveryCommand_.process_name[14](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[14]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[15] != RecoveryCommand_old.process_name[15]){
        std::cout << "RecoveryCommand_.process_name[15](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[15]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[16] != RecoveryCommand_old.process_name[16]){
        std::cout << "RecoveryCommand_.process_name[16](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[16]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[17] != RecoveryCommand_old.process_name[17]){
        std::cout << "RecoveryCommand_.process_name[17](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[17]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[18] != RecoveryCommand_old.process_name[18]){
        std::cout << "RecoveryCommand_.process_name[18](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[18]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[19] != RecoveryCommand_old.process_name[19]){
        std::cout << "RecoveryCommand_.process_name[19](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[19]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[20] != RecoveryCommand_old.process_name[20]){
        std::cout << "RecoveryCommand_.process_name[20](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[20]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[21] != RecoveryCommand_old.process_name[21]){
        std::cout << "RecoveryCommand_.process_name[21](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[21]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[22] != RecoveryCommand_old.process_name[22]){
        std::cout << "RecoveryCommand_.process_name[22](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[22]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[23] != RecoveryCommand_old.process_name[23]){
        std::cout << "RecoveryCommand_.process_name[23](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[23]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[24] != RecoveryCommand_old.process_name[24]){
        std::cout << "RecoveryCommand_.process_name[24](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[24]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[25] != RecoveryCommand_old.process_name[25]){
        std::cout << "RecoveryCommand_.process_name[25](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[25]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[26] != RecoveryCommand_old.process_name[26]){
        std::cout << "RecoveryCommand_.process_name[26](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[26]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[27] != RecoveryCommand_old.process_name[27]){
        std::cout << "RecoveryCommand_.process_name[27](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[27]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[28] != RecoveryCommand_old.process_name[28]){
        std::cout << "RecoveryCommand_.process_name[28](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[28]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[29] != RecoveryCommand_old.process_name[29]){
        std::cout << "RecoveryCommand_.process_name[29](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[29]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[30] != RecoveryCommand_old.process_name[30]){
        std::cout << "RecoveryCommand_.process_name[30](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[30]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.process_name[31] != RecoveryCommand_old.process_name[31]){
        std::cout << "RecoveryCommand_.process_name[31](int8_t): 0x" << static_cast<int>(RecoveryCommand_.process_name[31]) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.action != RecoveryCommand_old.action){
        std::cout << "RecoveryCommand_.action{}(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RecoveryCommand_.action) << std::dec  << std::endl;
        }
    if(RecoveryCommand_.error_code != RecoveryCommand_old.error_code){
        std::cout << "RecoveryCommand_.error_code{}(uint32_t): 0x" << std::hex << std::setw(8) << std::setfill('0') << RecoveryCommand_.error_code << std::dec  << std::endl;
        }
}



/* Print struct RecoveryResponse changed value */
void print_RecoveryResponse(RecoveryResponse& RecoveryResponse_,RecoveryResponse& RecoveryResponse_old){
// std::cout << "RecoveryResponse all variable:" << std::endl;
    if(RecoveryResponse_.success != RecoveryResponse_old.success){
        std::cout << "RecoveryResponse_.success{}(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(RecoveryResponse_.success) << std::dec  << std::endl;
        }
}










