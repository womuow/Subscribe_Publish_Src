#include "AdApter_FuncTgtVisnID_sub.h"
#include <termios.h>
#include <unistd.h>
#include <cstring>

using namespace std::chrono_literals;
using VariableVariant = std::variant<uint8*, uint16* ,uint32*,float32*,sint8*,sint16*,sint32*>;



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
        else
        {
            // inputQueue.push("input is empty");
        }
    }
}


void print_flags(struct termios *options) {
    printf("输入模式标志 (c_iflag):\n");
    printf("  IGNBRK: %s\n", (options->c_iflag & IGNBRK) ? "设置" : "未设置");
    printf("  BRKINT: %s\n", (options->c_iflag & BRKINT) ? "设置" : "未设置");
    printf("  IGNPAR: %s\n", (options->c_iflag & IGNPAR) ? "设置" : "未设置");
    printf("  PARMRK: %s\n", (options->c_iflag & PARMRK) ? "设置" : "未设置");
    printf("  INPCK:  %s\n", (options->c_iflag & INPCK) ? "设置" : "未设置");
    printf("  ISTRIP: %s\n", (options->c_iflag & ISTRIP) ? "设置" : "未设置");
    printf("  INLCR:  %s\n", (options->c_iflag & INLCR) ? "设置" : "未设置");
    printf("  IGNCR:  %s\n", (options->c_iflag & IGNCR) ? "设置" : "未设置");
    printf("  ICRNL:  %s\n", (options->c_iflag & ICRNL) ? "设置" : "未设置");
    printf("  IXON:   %s\n", (options->c_iflag & IXON) ? "设置" : "未设置");
    printf("  IXOFF:  %s\n", (options->c_iflag & IXOFF) ? "设置" : "未设置");
    printf("  IXANY:  %s\n", (options->c_iflag & IXANY) ? "设置" : "未设置");
    printf("  IMAXBEL:%s\n", (options->c_iflag & IMAXBEL) ? "设置" : "未设置");

    printf("\n输出模式标志 (c_oflag):\n");
    printf("  OPOST:  %s\n", (options->c_oflag & OPOST) ? "设置" : "未设置");
    printf("  ONLCR:  %s\n", (options->c_oflag & ONLCR) ? "设置" : "未设置");
    printf("  OCRNL:  %s\n", (options->c_oflag & OCRNL) ? "设置" : "未设置");
    printf("  ONOCR:  %s\n", (options->c_oflag & ONOCR) ? "设置" : "未设置");
    printf("  ONLRET: %s\n", (options->c_oflag & ONLRET) ? "设置" : "未设置");
    // 其他输出模式标志...

    printf("\n控制模式标志 (c_cflag):\n");
    printf("  CREAD:  %s\n", (options->c_cflag & CREAD) ? "设置" : "未设置");
    printf("  PARENB: %s\n", (options->c_cflag & PARENB) ? "设置" : "未设置");
    printf("  PARODD: %s\n", (options->c_cflag & PARODD) ? "设置" : "未设置");
    printf("  CSTOPB: %s\n", (options->c_cflag & CSTOPB) ? "设置" : "未设置");
    printf("  CSIZE:  ");
    if ((options->c_cflag & CSIZE) == CS5)
        printf("5\n");
    else if ((options->c_cflag & CSIZE) == CS6)
        printf("6\n");
    else if ((options->c_cflag & CSIZE) == CS7)
        printf("7\n");
    else if ((options->c_cflag & CSIZE) == CS8)
        printf("8\n");
    else
        printf("未知\n");
    printf("  CRTSCTS:%s\n", (options->c_cflag & CRTSCTS) ? "设置" : "未设置");
    printf("  CLOCAL: %s\n", (options->c_cflag & CLOCAL) ? "设置" : "未设置");

    printf("\n本地模式标志 (c_lflag):\n");
    printf("  ISIG:   %s\n", (options->c_lflag & ISIG) ? "设置" : "未设置");
    printf("  ICANON: %s\n", (options->c_lflag & ICANON) ? "设置" : "未设置");
    printf("  ECHO:   %s\n", (options->c_lflag & ECHO) ? "设置" : "未设置");
    printf("  ECHOE:  %s\n", (options->c_lflag & ECHOE) ? "设置" : "未设置");
    printf("  ECHOK:  %s\n", (options->c_lflag & ECHOK) ? "设置" : "未设置");
    printf("  ECHONL: %s\n", (options->c_lflag & ECHONL) ? "设置" : "未设置");
    printf("  NOFLSH: %s\n", (options->c_lflag & NOFLSH) ? "设置" : "未设置");
    printf("  TOSTOP: %s\n", (options->c_lflag & TOSTOP) ? "设置" : "未设置");
    printf("  IEXTEN: %s\n", (options->c_lflag & IEXTEN) ? "设置" : "未设置");

    printf("\n控制字符 (c_cc):\n");
    printf("  VINTR:  0x%02X\n", options->c_cc[VINTR]);
    printf("  VQUIT:  0x%02X\n", options->c_cc[VQUIT]);
    printf("  VERASE: 0x%02X\n", options->c_cc[VERASE]);
    printf("  VKILL:  0x%02X\n", options->c_cc[VKILL]);
    printf("  VEOF:   0x%02X\n", options->c_cc[VEOF]);
    printf("  VTIME:  0x%02X\n", options->c_cc[VTIME]);
    printf("  VMIN:   0x%02X\n", options->c_cc[VMIN]);
    printf("  VSTART: 0x%02X\n", options->c_cc[VSTART]);
    printf("  VSTOP:  0x%02X\n", options->c_cc[VSTOP]);
    printf("  VSUSP:  0x%02X\n", options->c_cc[VSUSP]);
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
 
        
        // 短暂休眠
        // usleep(500000);  // 500ms
    // }
    
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
                      << *ptr << std::dec  << std::endl;
        }
        else if constexpr (std::is_same_v<T, uint32> ) {
            // 32位类型：宽度8
            std::cout << name << ": 0x" << std::hex << std::setw(8) << std::setfill('0') 
                      << *ptr << std::dec  << std::endl;
        }
        
        else if constexpr (std::is_same_v<T, sint8>) {
            // 8位类型：宽度2
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec << std::endl;
        } 
        else if constexpr ( std::is_same_v<T, sint16>) {
            // 16位类型：宽度4
            std::cout << name << ": " << std::dec << std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec  << std::endl;
        }
        else if constexpr ( std::is_same_v<T, sint32>) {
            // 32位类型：宽度8
            std::cout << name << ": " << std::dec<< std::setfill('0') 
                      << static_cast<int>(*ptr) << std::dec  << std::endl;
        }

        else if constexpr (std::is_same_v<T, float32>) {
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


bool g_flag = true;

int config_async_sub(std::string json_file) {
    

    AdApter_FuncTgtVisnID AdApter_FuncTgtVisnID_;
    AdApter_FuncTgtVisnID_.json_file = json_file;
    int flag=true;

    
    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;


    MOS::communication::Init(json_file);
    MOS::utils::Register::get().register_version("libFuncTgtVisnID", "1.1.0");
    MOS::communication::ProtocolInfo proto_info;
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    std::string data_in ="";
    IDT_FuncTgtVisnID IDT_FuncTgtVisnID_;
    IDT_FuncTgtVisnID IDT_FuncTgtVisnID_old;

        std::map<std::string, VariableVariant > variableMap = {
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.ACCTgtVisnID},
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.CutInTgtVisnID},
        {"IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.LongTgtVisnID.FDWTgtVisnID},
        {"IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.AEBTgtVisnID},
        {"IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID(uint8)" , &IDT_FuncTgtVisnID_.CATgtVisnID.FCWTgtVisnID},


    };



    auto sub = MOS::communication::Subscriber::New(
        AdApter_FuncTgtVisnID_.domain_id,
        AdApter_FuncTgtVisnID_.topic, 
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


    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::memcpy(&IDT_FuncTgtVisnID_, data_in.data(), sizeof(IDT_FuncTgtVisnID));
    print_IDT_FuncTgtVisnID(IDT_FuncTgtVisnID_,IDT_FuncTgtVisnID_old);
    IDT_FuncTgtVisnID_old = IDT_FuncTgtVisnID_;
    // std::thread inputThread(asyncInputThread);
    std::thread inputThread2(asyncInputThreadTTY);

    while (true) {//while (!stop.load()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::memcpy(&IDT_FuncTgtVisnID_, data_in.data(), sizeof(IDT_FuncTgtVisnID));

        // std::cout<< "Print FuncTgtVisnID changed value"<< std::endl;
        //print_FuncTgtVisnID(IDT_FuncTgtVisnID_,FuncTgtVisnID_old);
        // std::cout<< "----------------------------------End"<< std::endl;
        print_IDT_FuncTgtVisnID(IDT_FuncTgtVisnID_,IDT_FuncTgtVisnID_old);


        IDT_FuncTgtVisnID_old = IDT_FuncTgtVisnID_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        //std::memcpy(&FuncTgtVisnID_old, data_in.data(), sizeof(FuncTgtVisnID));
    }
    return 0;
}

void AdApter_FuncTgtVisnID::run()
{
    config_async_sub(json_file);
}
AdApter_FuncTgtVisnID::AdApter_FuncTgtVisnID()
{
}
AdApter_FuncTgtVisnID::~AdApter_FuncTgtVisnID()
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
    std::cout << "Running on Linux(FuncTgtVisnID_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    AdApter_FuncTgtVisnID objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
return 0;
}


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









