#include"Adapter_PARAM_CS.h"


using VariableVariant = std::variant<uint8_t*, uint16_t*, uint32_t*, uint64_t*, int8_t*, int16_t*, int32_t*, int64_t*, float*, bool*,MagnaParamReqType*,MagnaStatusCode*,MagnaClientStatusCode*,MagnaCalibResultCode*>;


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








void TestServiceCallback(MOS::message::spMsg request, MOS::message::spMsg & response) {
    auto request_gen_ts = request->GetGenTimestamp();
    auto request_first_data = static_cast<MagnaParamCSReq*>(request->GetDataRef()->GetDataVec()[0]);
    MagnaParamCSReq_ = *request_first_data;

    std::cout<<"receive client request, gen_ts = "<< request_gen_ts<<", req.meta_req_data[0] = "<< (int)(request_first_data->meta_req_data[0]) <<std::endl;
  

//   LOG_DEBUG("receive client request, gen_ts = %ld, event = %d", request_gen_ts, request_first_data->event);

    //  TestResponse res;
//   res.rev = request_first_data->event + 1;

//   auto res_data_ref = std::make_shared<MOS::message::DataRef>(&res, sizeof(res));
//   response->SetDataRef(res_data_ref);
//   response->SetGenTimestamp(MOS::TimeUtils::NowNsec());

//   LOG_DEBUG("send response, gen_ts = %ld, rev = %d", response->GetGenTimestamp(), res.rev);
}

void set_proto_info(MOS::communication::ProtocolInfo& proto_info, bool type, int port, const std::string& ip,
                    int block_size, int block_count = 10) {
//   if (type) {
//     proto_info.protocol_type = MOS::communication::kProtocolNet;
//     proto_info.net_info.local_addr.ip = ip.c_str();
//     proto_info.net_info.local_addr.port = port;
//   } else {
    proto_info.protocol_type = MOS::communication::kProtocolShm;
    proto_info.shm_info.block_count = block_count;
    proto_info.shm_info.block_size = block_size;
    proto_info.shm_info.fast_mode = false;
//   }
}



int config_client(int domain_id, std::string service_name, bool type, std::string discovery_json_file, std::string ip,
                  uint32_t port) {
  // 初始化Communication中间件
  MOS::communication::Init(discovery_json_file);

  // 配置通信属性
  MOS::communication::ProtocolInfo proto_info;
  set_proto_info(proto_info, type, port, ip, sizeof(MagnaParamCSReq));

  // 创建Client
  auto client = MOS::communication::Client::New(domain_id, service_name, proto_info);
  uint8_t count = 0;
  int timeout_ms = 1000;
  while (true) {
    MagnaParamCSReq req;
    count++;
    req.meta_req_data[0] = count;
    std::cout<<"output req.meta_req_data[0] = "<<req.meta_req_data[0]<<std::endl;
    
    auto request_msg = std::make_shared<MOS::message::Message>();
    auto response_msg = std::make_shared<MOS::message::Message>();
    auto data_ref = std::make_shared<MOS::message::DataRef>(&req, sizeof(MagnaParamCSReq));
    request_msg->SetDataRef(data_ref);

    // 发送 request
    auto ret = client->SendRequest(request_msg, response_msg, timeout_ms);
    if (ret == MOS::communication::COMM_CODE_OK) {
      // 处理 response
    //   auto response = static_cast<MagnaParamCSRes*>(response_msg->GetDataRef()->GetDataVec()[0]);
    //   LOG_DEBUG("get response success, gen_ts = %ld, rev = %d", response_msg->GetGenTimestamp(), response->rev);

    } else {
    //   LOG_WARNING("get response failed, ret = %d", ret);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
    





void asyncClientThread() {

    config_client(0, "CS/Calib", false, "discovery_config.json", "127.0.0.1",12352);


}


int config_async_service(std::string json_file) {
    

    Adapter_PARAM_CS Adapter_PARAM_CS_;
    Adapter_PARAM_CS_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;

    // 初始化Communication中间件
    MOS::communication::Init(json_file);
    // 配置通信属性
    MOS::communication::ProtocolInfo proto_info;
    set_proto_info(proto_info, Adapter_PARAM_CS_.type, Adapter_PARAM_CS_.port, Adapter_PARAM_CS_.ip, sizeof(MagnaParamCSRes));


    
    MagnaParamCSReq MagnaParamCSReq_;
    MagnaParamCSReq MagnaParamCSReq_old;

    MagnaParamCSRes MagnaParamCSRes_;
    MagnaParamCSRes MagnaParamCSRes_old;



    std::map<std::string, VariableVariant > variableMap = {};
    


    // 创建Service
    auto service = MOS::communication::Service::New(Adapter_PARAM_CS_.domain_id, Adapter_PARAM_CS_.topic, proto_info, TestServiceCallback);


    std::thread inputThread(asyncClientThread);
    // std::thread inputThread2(asyncInputThreadTTY);

    while (true) {

        print_MagnaParamCSReq(MagnaParamCSReq_, MagnaParamCSReq_old);
        MagnaParamCSReq_old = MagnaParamCSReq_;

        if (!inputQueue.empty())
        {
            getVariableValue(variableMap,inputQueue.front());
            inputQueue.pop();
        }

    }
    return 0;
}

void Adapter_PARAM_CS::run()
{
    config_async_service(json_file);
}
Adapter_PARAM_CS::Adapter_PARAM_CS()
{
}
Adapter_PARAM_CS::~Adapter_PARAM_CS()
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
    std::cout << "Running on Linux(PARAM_CS_Sub)"<< fullPath << std::endl;
#endif
    size_t pos = fullPath.find_last_of("\\/");
    std::string configPath = fullPath.substr(0, pos) + "/discovery_config.json";
    Adapter_PARAM_CS objtest;
    objtest.json_file = configPath;
    objtest.run();
    // config_async_sub(configPath);
    return 0;
}



/* Print struct MagnaParamCSReq changed value */
void print_MagnaParamCSReq(MagnaParamCSReq& MagnaParamCSReq_,MagnaParamCSReq& MagnaParamCSReq_old){
// std::cout << "MagnaParamCSReq all variable:" << std::endl;
    if(MagnaParamCSReq_.req_type != MagnaParamCSReq_old.req_type){
        std::cout << "MagnaParamCSReq_.req_type(uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.req_type) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[0] != MagnaParamCSReq_old.meta_req_data[0]){
        std::cout << "MagnaParamCSReq_.meta_req_data[0](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[0]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[1] != MagnaParamCSReq_old.meta_req_data[1]){
        std::cout << "MagnaParamCSReq_.meta_req_data[1](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[1]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[2] != MagnaParamCSReq_old.meta_req_data[2]){
        std::cout << "MagnaParamCSReq_.meta_req_data[2](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[2]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[3] != MagnaParamCSReq_old.meta_req_data[3]){
        std::cout << "MagnaParamCSReq_.meta_req_data[3](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[3]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[4] != MagnaParamCSReq_old.meta_req_data[4]){
        std::cout << "MagnaParamCSReq_.meta_req_data[4](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[4]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[5] != MagnaParamCSReq_old.meta_req_data[5]){
        std::cout << "MagnaParamCSReq_.meta_req_data[5](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[5]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[6] != MagnaParamCSReq_old.meta_req_data[6]){
        std::cout << "MagnaParamCSReq_.meta_req_data[6](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[6]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[7] != MagnaParamCSReq_old.meta_req_data[7]){
        std::cout << "MagnaParamCSReq_.meta_req_data[7](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[7]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[8] != MagnaParamCSReq_old.meta_req_data[8]){
        std::cout << "MagnaParamCSReq_.meta_req_data[8](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[8]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[9] != MagnaParamCSReq_old.meta_req_data[9]){
        std::cout << "MagnaParamCSReq_.meta_req_data[9](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[9]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[10] != MagnaParamCSReq_old.meta_req_data[10]){
        std::cout << "MagnaParamCSReq_.meta_req_data[10](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[10]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[11] != MagnaParamCSReq_old.meta_req_data[11]){
        std::cout << "MagnaParamCSReq_.meta_req_data[11](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[11]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[12] != MagnaParamCSReq_old.meta_req_data[12]){
        std::cout << "MagnaParamCSReq_.meta_req_data[12](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[12]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[13] != MagnaParamCSReq_old.meta_req_data[13]){
        std::cout << "MagnaParamCSReq_.meta_req_data[13](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[13]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[14] != MagnaParamCSReq_old.meta_req_data[14]){
        std::cout << "MagnaParamCSReq_.meta_req_data[14](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[14]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[15] != MagnaParamCSReq_old.meta_req_data[15]){
        std::cout << "MagnaParamCSReq_.meta_req_data[15](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[15]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[16] != MagnaParamCSReq_old.meta_req_data[16]){
        std::cout << "MagnaParamCSReq_.meta_req_data[16](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[16]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[17] != MagnaParamCSReq_old.meta_req_data[17]){
        std::cout << "MagnaParamCSReq_.meta_req_data[17](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[17]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[18] != MagnaParamCSReq_old.meta_req_data[18]){
        std::cout << "MagnaParamCSReq_.meta_req_data[18](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[18]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[19] != MagnaParamCSReq_old.meta_req_data[19]){
        std::cout << "MagnaParamCSReq_.meta_req_data[19](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[19]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[20] != MagnaParamCSReq_old.meta_req_data[20]){
        std::cout << "MagnaParamCSReq_.meta_req_data[20](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[20]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[21] != MagnaParamCSReq_old.meta_req_data[21]){
        std::cout << "MagnaParamCSReq_.meta_req_data[21](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[21]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[22] != MagnaParamCSReq_old.meta_req_data[22]){
        std::cout << "MagnaParamCSReq_.meta_req_data[22](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[22]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[23] != MagnaParamCSReq_old.meta_req_data[23]){
        std::cout << "MagnaParamCSReq_.meta_req_data[23](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[23]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[24] != MagnaParamCSReq_old.meta_req_data[24]){
        std::cout << "MagnaParamCSReq_.meta_req_data[24](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[24]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[25] != MagnaParamCSReq_old.meta_req_data[25]){
        std::cout << "MagnaParamCSReq_.meta_req_data[25](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[25]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[26] != MagnaParamCSReq_old.meta_req_data[26]){
        std::cout << "MagnaParamCSReq_.meta_req_data[26](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[26]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[27] != MagnaParamCSReq_old.meta_req_data[27]){
        std::cout << "MagnaParamCSReq_.meta_req_data[27](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[27]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[28] != MagnaParamCSReq_old.meta_req_data[28]){
        std::cout << "MagnaParamCSReq_.meta_req_data[28](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[28]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[29] != MagnaParamCSReq_old.meta_req_data[29]){
        std::cout << "MagnaParamCSReq_.meta_req_data[29](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[29]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[30] != MagnaParamCSReq_old.meta_req_data[30]){
        std::cout << "MagnaParamCSReq_.meta_req_data[30](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[30]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[31] != MagnaParamCSReq_old.meta_req_data[31]){
        std::cout << "MagnaParamCSReq_.meta_req_data[31](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[31]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[32] != MagnaParamCSReq_old.meta_req_data[32]){
        std::cout << "MagnaParamCSReq_.meta_req_data[32](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[32]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[33] != MagnaParamCSReq_old.meta_req_data[33]){
        std::cout << "MagnaParamCSReq_.meta_req_data[33](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[33]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[34] != MagnaParamCSReq_old.meta_req_data[34]){
        std::cout << "MagnaParamCSReq_.meta_req_data[34](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[34]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[35] != MagnaParamCSReq_old.meta_req_data[35]){
        std::cout << "MagnaParamCSReq_.meta_req_data[35](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[35]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[36] != MagnaParamCSReq_old.meta_req_data[36]){
        std::cout << "MagnaParamCSReq_.meta_req_data[36](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[36]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[37] != MagnaParamCSReq_old.meta_req_data[37]){
        std::cout << "MagnaParamCSReq_.meta_req_data[37](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[37]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[38] != MagnaParamCSReq_old.meta_req_data[38]){
        std::cout << "MagnaParamCSReq_.meta_req_data[38](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[38]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[39] != MagnaParamCSReq_old.meta_req_data[39]){
        std::cout << "MagnaParamCSReq_.meta_req_data[39](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[39]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[40] != MagnaParamCSReq_old.meta_req_data[40]){
        std::cout << "MagnaParamCSReq_.meta_req_data[40](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[40]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[41] != MagnaParamCSReq_old.meta_req_data[41]){
        std::cout << "MagnaParamCSReq_.meta_req_data[41](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[41]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[42] != MagnaParamCSReq_old.meta_req_data[42]){
        std::cout << "MagnaParamCSReq_.meta_req_data[42](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[42]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[43] != MagnaParamCSReq_old.meta_req_data[43]){
        std::cout << "MagnaParamCSReq_.meta_req_data[43](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[43]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[44] != MagnaParamCSReq_old.meta_req_data[44]){
        std::cout << "MagnaParamCSReq_.meta_req_data[44](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[44]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[45] != MagnaParamCSReq_old.meta_req_data[45]){
        std::cout << "MagnaParamCSReq_.meta_req_data[45](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[45]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[46] != MagnaParamCSReq_old.meta_req_data[46]){
        std::cout << "MagnaParamCSReq_.meta_req_data[46](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[46]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[47] != MagnaParamCSReq_old.meta_req_data[47]){
        std::cout << "MagnaParamCSReq_.meta_req_data[47](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[47]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[48] != MagnaParamCSReq_old.meta_req_data[48]){
        std::cout << "MagnaParamCSReq_.meta_req_data[48](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[48]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[49] != MagnaParamCSReq_old.meta_req_data[49]){
        std::cout << "MagnaParamCSReq_.meta_req_data[49](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[49]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[50] != MagnaParamCSReq_old.meta_req_data[50]){
        std::cout << "MagnaParamCSReq_.meta_req_data[50](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[50]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[51] != MagnaParamCSReq_old.meta_req_data[51]){
        std::cout << "MagnaParamCSReq_.meta_req_data[51](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[51]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[52] != MagnaParamCSReq_old.meta_req_data[52]){
        std::cout << "MagnaParamCSReq_.meta_req_data[52](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[52]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[53] != MagnaParamCSReq_old.meta_req_data[53]){
        std::cout << "MagnaParamCSReq_.meta_req_data[53](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[53]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[54] != MagnaParamCSReq_old.meta_req_data[54]){
        std::cout << "MagnaParamCSReq_.meta_req_data[54](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[54]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[55] != MagnaParamCSReq_old.meta_req_data[55]){
        std::cout << "MagnaParamCSReq_.meta_req_data[55](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[55]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[56] != MagnaParamCSReq_old.meta_req_data[56]){
        std::cout << "MagnaParamCSReq_.meta_req_data[56](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[56]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[57] != MagnaParamCSReq_old.meta_req_data[57]){
        std::cout << "MagnaParamCSReq_.meta_req_data[57](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[57]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[58] != MagnaParamCSReq_old.meta_req_data[58]){
        std::cout << "MagnaParamCSReq_.meta_req_data[58](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[58]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[59] != MagnaParamCSReq_old.meta_req_data[59]){
        std::cout << "MagnaParamCSReq_.meta_req_data[59](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[59]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[60] != MagnaParamCSReq_old.meta_req_data[60]){
        std::cout << "MagnaParamCSReq_.meta_req_data[60](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[60]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[61] != MagnaParamCSReq_old.meta_req_data[61]){
        std::cout << "MagnaParamCSReq_.meta_req_data[61](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[61]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[62] != MagnaParamCSReq_old.meta_req_data[62]){
        std::cout << "MagnaParamCSReq_.meta_req_data[62](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[62]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[63] != MagnaParamCSReq_old.meta_req_data[63]){
        std::cout << "MagnaParamCSReq_.meta_req_data[63](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[63]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[64] != MagnaParamCSReq_old.meta_req_data[64]){
        std::cout << "MagnaParamCSReq_.meta_req_data[64](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[64]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[65] != MagnaParamCSReq_old.meta_req_data[65]){
        std::cout << "MagnaParamCSReq_.meta_req_data[65](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[65]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[66] != MagnaParamCSReq_old.meta_req_data[66]){
        std::cout << "MagnaParamCSReq_.meta_req_data[66](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[66]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[67] != MagnaParamCSReq_old.meta_req_data[67]){
        std::cout << "MagnaParamCSReq_.meta_req_data[67](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[67]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[68] != MagnaParamCSReq_old.meta_req_data[68]){
        std::cout << "MagnaParamCSReq_.meta_req_data[68](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[68]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[69] != MagnaParamCSReq_old.meta_req_data[69]){
        std::cout << "MagnaParamCSReq_.meta_req_data[69](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[69]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[70] != MagnaParamCSReq_old.meta_req_data[70]){
        std::cout << "MagnaParamCSReq_.meta_req_data[70](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[70]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[71] != MagnaParamCSReq_old.meta_req_data[71]){
        std::cout << "MagnaParamCSReq_.meta_req_data[71](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[71]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[72] != MagnaParamCSReq_old.meta_req_data[72]){
        std::cout << "MagnaParamCSReq_.meta_req_data[72](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[72]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[73] != MagnaParamCSReq_old.meta_req_data[73]){
        std::cout << "MagnaParamCSReq_.meta_req_data[73](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[73]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[74] != MagnaParamCSReq_old.meta_req_data[74]){
        std::cout << "MagnaParamCSReq_.meta_req_data[74](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[74]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[75] != MagnaParamCSReq_old.meta_req_data[75]){
        std::cout << "MagnaParamCSReq_.meta_req_data[75](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[75]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[76] != MagnaParamCSReq_old.meta_req_data[76]){
        std::cout << "MagnaParamCSReq_.meta_req_data[76](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[76]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[77] != MagnaParamCSReq_old.meta_req_data[77]){
        std::cout << "MagnaParamCSReq_.meta_req_data[77](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[77]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[78] != MagnaParamCSReq_old.meta_req_data[78]){
        std::cout << "MagnaParamCSReq_.meta_req_data[78](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[78]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[79] != MagnaParamCSReq_old.meta_req_data[79]){
        std::cout << "MagnaParamCSReq_.meta_req_data[79](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[79]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[80] != MagnaParamCSReq_old.meta_req_data[80]){
        std::cout << "MagnaParamCSReq_.meta_req_data[80](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[80]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[81] != MagnaParamCSReq_old.meta_req_data[81]){
        std::cout << "MagnaParamCSReq_.meta_req_data[81](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[81]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[82] != MagnaParamCSReq_old.meta_req_data[82]){
        std::cout << "MagnaParamCSReq_.meta_req_data[82](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[82]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[83] != MagnaParamCSReq_old.meta_req_data[83]){
        std::cout << "MagnaParamCSReq_.meta_req_data[83](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[83]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[84] != MagnaParamCSReq_old.meta_req_data[84]){
        std::cout << "MagnaParamCSReq_.meta_req_data[84](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[84]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[85] != MagnaParamCSReq_old.meta_req_data[85]){
        std::cout << "MagnaParamCSReq_.meta_req_data[85](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[85]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[86] != MagnaParamCSReq_old.meta_req_data[86]){
        std::cout << "MagnaParamCSReq_.meta_req_data[86](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[86]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[87] != MagnaParamCSReq_old.meta_req_data[87]){
        std::cout << "MagnaParamCSReq_.meta_req_data[87](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[87]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[88] != MagnaParamCSReq_old.meta_req_data[88]){
        std::cout << "MagnaParamCSReq_.meta_req_data[88](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[88]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[89] != MagnaParamCSReq_old.meta_req_data[89]){
        std::cout << "MagnaParamCSReq_.meta_req_data[89](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[89]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[90] != MagnaParamCSReq_old.meta_req_data[90]){
        std::cout << "MagnaParamCSReq_.meta_req_data[90](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[90]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[91] != MagnaParamCSReq_old.meta_req_data[91]){
        std::cout << "MagnaParamCSReq_.meta_req_data[91](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[91]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[92] != MagnaParamCSReq_old.meta_req_data[92]){
        std::cout << "MagnaParamCSReq_.meta_req_data[92](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[92]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[93] != MagnaParamCSReq_old.meta_req_data[93]){
        std::cout << "MagnaParamCSReq_.meta_req_data[93](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[93]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[94] != MagnaParamCSReq_old.meta_req_data[94]){
        std::cout << "MagnaParamCSReq_.meta_req_data[94](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[94]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[95] != MagnaParamCSReq_old.meta_req_data[95]){
        std::cout << "MagnaParamCSReq_.meta_req_data[95](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[95]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[96] != MagnaParamCSReq_old.meta_req_data[96]){
        std::cout << "MagnaParamCSReq_.meta_req_data[96](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[96]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[97] != MagnaParamCSReq_old.meta_req_data[97]){
        std::cout << "MagnaParamCSReq_.meta_req_data[97](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[97]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[98] != MagnaParamCSReq_old.meta_req_data[98]){
        std::cout << "MagnaParamCSReq_.meta_req_data[98](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[98]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[99] != MagnaParamCSReq_old.meta_req_data[99]){
        std::cout << "MagnaParamCSReq_.meta_req_data[99](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[99]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[100] != MagnaParamCSReq_old.meta_req_data[100]){
        std::cout << "MagnaParamCSReq_.meta_req_data[100](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[100]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[101] != MagnaParamCSReq_old.meta_req_data[101]){
        std::cout << "MagnaParamCSReq_.meta_req_data[101](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[101]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[102] != MagnaParamCSReq_old.meta_req_data[102]){
        std::cout << "MagnaParamCSReq_.meta_req_data[102](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[102]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[103] != MagnaParamCSReq_old.meta_req_data[103]){
        std::cout << "MagnaParamCSReq_.meta_req_data[103](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[103]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[104] != MagnaParamCSReq_old.meta_req_data[104]){
        std::cout << "MagnaParamCSReq_.meta_req_data[104](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[104]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[105] != MagnaParamCSReq_old.meta_req_data[105]){
        std::cout << "MagnaParamCSReq_.meta_req_data[105](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[105]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[106] != MagnaParamCSReq_old.meta_req_data[106]){
        std::cout << "MagnaParamCSReq_.meta_req_data[106](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[106]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[107] != MagnaParamCSReq_old.meta_req_data[107]){
        std::cout << "MagnaParamCSReq_.meta_req_data[107](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[107]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[108] != MagnaParamCSReq_old.meta_req_data[108]){
        std::cout << "MagnaParamCSReq_.meta_req_data[108](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[108]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[109] != MagnaParamCSReq_old.meta_req_data[109]){
        std::cout << "MagnaParamCSReq_.meta_req_data[109](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[109]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[110] != MagnaParamCSReq_old.meta_req_data[110]){
        std::cout << "MagnaParamCSReq_.meta_req_data[110](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[110]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[111] != MagnaParamCSReq_old.meta_req_data[111]){
        std::cout << "MagnaParamCSReq_.meta_req_data[111](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[111]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[112] != MagnaParamCSReq_old.meta_req_data[112]){
        std::cout << "MagnaParamCSReq_.meta_req_data[112](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[112]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[113] != MagnaParamCSReq_old.meta_req_data[113]){
        std::cout << "MagnaParamCSReq_.meta_req_data[113](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[113]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[114] != MagnaParamCSReq_old.meta_req_data[114]){
        std::cout << "MagnaParamCSReq_.meta_req_data[114](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[114]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[115] != MagnaParamCSReq_old.meta_req_data[115]){
        std::cout << "MagnaParamCSReq_.meta_req_data[115](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[115]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[116] != MagnaParamCSReq_old.meta_req_data[116]){
        std::cout << "MagnaParamCSReq_.meta_req_data[116](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[116]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[117] != MagnaParamCSReq_old.meta_req_data[117]){
        std::cout << "MagnaParamCSReq_.meta_req_data[117](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[117]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[118] != MagnaParamCSReq_old.meta_req_data[118]){
        std::cout << "MagnaParamCSReq_.meta_req_data[118](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[118]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[119] != MagnaParamCSReq_old.meta_req_data[119]){
        std::cout << "MagnaParamCSReq_.meta_req_data[119](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[119]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[120] != MagnaParamCSReq_old.meta_req_data[120]){
        std::cout << "MagnaParamCSReq_.meta_req_data[120](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[120]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[121] != MagnaParamCSReq_old.meta_req_data[121]){
        std::cout << "MagnaParamCSReq_.meta_req_data[121](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[121]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[122] != MagnaParamCSReq_old.meta_req_data[122]){
        std::cout << "MagnaParamCSReq_.meta_req_data[122](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[122]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[123] != MagnaParamCSReq_old.meta_req_data[123]){
        std::cout << "MagnaParamCSReq_.meta_req_data[123](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[123]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[124] != MagnaParamCSReq_old.meta_req_data[124]){
        std::cout << "MagnaParamCSReq_.meta_req_data[124](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[124]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[125] != MagnaParamCSReq_old.meta_req_data[125]){
        std::cout << "MagnaParamCSReq_.meta_req_data[125](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[125]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[126] != MagnaParamCSReq_old.meta_req_data[126]){
        std::cout << "MagnaParamCSReq_.meta_req_data[126](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[126]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[127] != MagnaParamCSReq_old.meta_req_data[127]){
        std::cout << "MagnaParamCSReq_.meta_req_data[127](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[127]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[128] != MagnaParamCSReq_old.meta_req_data[128]){
        std::cout << "MagnaParamCSReq_.meta_req_data[128](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[128]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[129] != MagnaParamCSReq_old.meta_req_data[129]){
        std::cout << "MagnaParamCSReq_.meta_req_data[129](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[129]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[130] != MagnaParamCSReq_old.meta_req_data[130]){
        std::cout << "MagnaParamCSReq_.meta_req_data[130](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[130]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[131] != MagnaParamCSReq_old.meta_req_data[131]){
        std::cout << "MagnaParamCSReq_.meta_req_data[131](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[131]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[132] != MagnaParamCSReq_old.meta_req_data[132]){
        std::cout << "MagnaParamCSReq_.meta_req_data[132](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[132]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[133] != MagnaParamCSReq_old.meta_req_data[133]){
        std::cout << "MagnaParamCSReq_.meta_req_data[133](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[133]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[134] != MagnaParamCSReq_old.meta_req_data[134]){
        std::cout << "MagnaParamCSReq_.meta_req_data[134](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[134]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[135] != MagnaParamCSReq_old.meta_req_data[135]){
        std::cout << "MagnaParamCSReq_.meta_req_data[135](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[135]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[136] != MagnaParamCSReq_old.meta_req_data[136]){
        std::cout << "MagnaParamCSReq_.meta_req_data[136](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[136]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[137] != MagnaParamCSReq_old.meta_req_data[137]){
        std::cout << "MagnaParamCSReq_.meta_req_data[137](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[137]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[138] != MagnaParamCSReq_old.meta_req_data[138]){
        std::cout << "MagnaParamCSReq_.meta_req_data[138](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[138]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[139] != MagnaParamCSReq_old.meta_req_data[139]){
        std::cout << "MagnaParamCSReq_.meta_req_data[139](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[139]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[140] != MagnaParamCSReq_old.meta_req_data[140]){
        std::cout << "MagnaParamCSReq_.meta_req_data[140](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[140]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[141] != MagnaParamCSReq_old.meta_req_data[141]){
        std::cout << "MagnaParamCSReq_.meta_req_data[141](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[141]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[142] != MagnaParamCSReq_old.meta_req_data[142]){
        std::cout << "MagnaParamCSReq_.meta_req_data[142](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[142]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[143] != MagnaParamCSReq_old.meta_req_data[143]){
        std::cout << "MagnaParamCSReq_.meta_req_data[143](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[143]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[144] != MagnaParamCSReq_old.meta_req_data[144]){
        std::cout << "MagnaParamCSReq_.meta_req_data[144](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[144]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[145] != MagnaParamCSReq_old.meta_req_data[145]){
        std::cout << "MagnaParamCSReq_.meta_req_data[145](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[145]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[146] != MagnaParamCSReq_old.meta_req_data[146]){
        std::cout << "MagnaParamCSReq_.meta_req_data[146](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[146]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[147] != MagnaParamCSReq_old.meta_req_data[147]){
        std::cout << "MagnaParamCSReq_.meta_req_data[147](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[147]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[148] != MagnaParamCSReq_old.meta_req_data[148]){
        std::cout << "MagnaParamCSReq_.meta_req_data[148](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[148]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[149] != MagnaParamCSReq_old.meta_req_data[149]){
        std::cout << "MagnaParamCSReq_.meta_req_data[149](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[149]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[150] != MagnaParamCSReq_old.meta_req_data[150]){
        std::cout << "MagnaParamCSReq_.meta_req_data[150](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[150]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[151] != MagnaParamCSReq_old.meta_req_data[151]){
        std::cout << "MagnaParamCSReq_.meta_req_data[151](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[151]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[152] != MagnaParamCSReq_old.meta_req_data[152]){
        std::cout << "MagnaParamCSReq_.meta_req_data[152](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[152]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[153] != MagnaParamCSReq_old.meta_req_data[153]){
        std::cout << "MagnaParamCSReq_.meta_req_data[153](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[153]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[154] != MagnaParamCSReq_old.meta_req_data[154]){
        std::cout << "MagnaParamCSReq_.meta_req_data[154](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[154]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[155] != MagnaParamCSReq_old.meta_req_data[155]){
        std::cout << "MagnaParamCSReq_.meta_req_data[155](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[155]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[156] != MagnaParamCSReq_old.meta_req_data[156]){
        std::cout << "MagnaParamCSReq_.meta_req_data[156](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[156]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[157] != MagnaParamCSReq_old.meta_req_data[157]){
        std::cout << "MagnaParamCSReq_.meta_req_data[157](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[157]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[158] != MagnaParamCSReq_old.meta_req_data[158]){
        std::cout << "MagnaParamCSReq_.meta_req_data[158](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[158]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[159] != MagnaParamCSReq_old.meta_req_data[159]){
        std::cout << "MagnaParamCSReq_.meta_req_data[159](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[159]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[160] != MagnaParamCSReq_old.meta_req_data[160]){
        std::cout << "MagnaParamCSReq_.meta_req_data[160](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[160]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[161] != MagnaParamCSReq_old.meta_req_data[161]){
        std::cout << "MagnaParamCSReq_.meta_req_data[161](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[161]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[162] != MagnaParamCSReq_old.meta_req_data[162]){
        std::cout << "MagnaParamCSReq_.meta_req_data[162](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[162]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[163] != MagnaParamCSReq_old.meta_req_data[163]){
        std::cout << "MagnaParamCSReq_.meta_req_data[163](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[163]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[164] != MagnaParamCSReq_old.meta_req_data[164]){
        std::cout << "MagnaParamCSReq_.meta_req_data[164](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[164]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[165] != MagnaParamCSReq_old.meta_req_data[165]){
        std::cout << "MagnaParamCSReq_.meta_req_data[165](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[165]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[166] != MagnaParamCSReq_old.meta_req_data[166]){
        std::cout << "MagnaParamCSReq_.meta_req_data[166](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[166]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[167] != MagnaParamCSReq_old.meta_req_data[167]){
        std::cout << "MagnaParamCSReq_.meta_req_data[167](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[167]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[168] != MagnaParamCSReq_old.meta_req_data[168]){
        std::cout << "MagnaParamCSReq_.meta_req_data[168](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[168]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[169] != MagnaParamCSReq_old.meta_req_data[169]){
        std::cout << "MagnaParamCSReq_.meta_req_data[169](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[169]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[170] != MagnaParamCSReq_old.meta_req_data[170]){
        std::cout << "MagnaParamCSReq_.meta_req_data[170](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[170]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[171] != MagnaParamCSReq_old.meta_req_data[171]){
        std::cout << "MagnaParamCSReq_.meta_req_data[171](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[171]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[172] != MagnaParamCSReq_old.meta_req_data[172]){
        std::cout << "MagnaParamCSReq_.meta_req_data[172](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[172]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[173] != MagnaParamCSReq_old.meta_req_data[173]){
        std::cout << "MagnaParamCSReq_.meta_req_data[173](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[173]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[174] != MagnaParamCSReq_old.meta_req_data[174]){
        std::cout << "MagnaParamCSReq_.meta_req_data[174](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[174]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[175] != MagnaParamCSReq_old.meta_req_data[175]){
        std::cout << "MagnaParamCSReq_.meta_req_data[175](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[175]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[176] != MagnaParamCSReq_old.meta_req_data[176]){
        std::cout << "MagnaParamCSReq_.meta_req_data[176](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[176]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[177] != MagnaParamCSReq_old.meta_req_data[177]){
        std::cout << "MagnaParamCSReq_.meta_req_data[177](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[177]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[178] != MagnaParamCSReq_old.meta_req_data[178]){
        std::cout << "MagnaParamCSReq_.meta_req_data[178](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[178]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[179] != MagnaParamCSReq_old.meta_req_data[179]){
        std::cout << "MagnaParamCSReq_.meta_req_data[179](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[179]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[180] != MagnaParamCSReq_old.meta_req_data[180]){
        std::cout << "MagnaParamCSReq_.meta_req_data[180](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[180]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[181] != MagnaParamCSReq_old.meta_req_data[181]){
        std::cout << "MagnaParamCSReq_.meta_req_data[181](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[181]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[182] != MagnaParamCSReq_old.meta_req_data[182]){
        std::cout << "MagnaParamCSReq_.meta_req_data[182](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[182]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[183] != MagnaParamCSReq_old.meta_req_data[183]){
        std::cout << "MagnaParamCSReq_.meta_req_data[183](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[183]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[184] != MagnaParamCSReq_old.meta_req_data[184]){
        std::cout << "MagnaParamCSReq_.meta_req_data[184](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[184]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[185] != MagnaParamCSReq_old.meta_req_data[185]){
        std::cout << "MagnaParamCSReq_.meta_req_data[185](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[185]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[186] != MagnaParamCSReq_old.meta_req_data[186]){
        std::cout << "MagnaParamCSReq_.meta_req_data[186](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[186]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[187] != MagnaParamCSReq_old.meta_req_data[187]){
        std::cout << "MagnaParamCSReq_.meta_req_data[187](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[187]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[188] != MagnaParamCSReq_old.meta_req_data[188]){
        std::cout << "MagnaParamCSReq_.meta_req_data[188](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[188]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[189] != MagnaParamCSReq_old.meta_req_data[189]){
        std::cout << "MagnaParamCSReq_.meta_req_data[189](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[189]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[190] != MagnaParamCSReq_old.meta_req_data[190]){
        std::cout << "MagnaParamCSReq_.meta_req_data[190](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[190]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[191] != MagnaParamCSReq_old.meta_req_data[191]){
        std::cout << "MagnaParamCSReq_.meta_req_data[191](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[191]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[192] != MagnaParamCSReq_old.meta_req_data[192]){
        std::cout << "MagnaParamCSReq_.meta_req_data[192](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[192]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[193] != MagnaParamCSReq_old.meta_req_data[193]){
        std::cout << "MagnaParamCSReq_.meta_req_data[193](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[193]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[194] != MagnaParamCSReq_old.meta_req_data[194]){
        std::cout << "MagnaParamCSReq_.meta_req_data[194](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[194]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[195] != MagnaParamCSReq_old.meta_req_data[195]){
        std::cout << "MagnaParamCSReq_.meta_req_data[195](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[195]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[196] != MagnaParamCSReq_old.meta_req_data[196]){
        std::cout << "MagnaParamCSReq_.meta_req_data[196](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[196]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[197] != MagnaParamCSReq_old.meta_req_data[197]){
        std::cout << "MagnaParamCSReq_.meta_req_data[197](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[197]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[198] != MagnaParamCSReq_old.meta_req_data[198]){
        std::cout << "MagnaParamCSReq_.meta_req_data[198](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[198]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[199] != MagnaParamCSReq_old.meta_req_data[199]){
        std::cout << "MagnaParamCSReq_.meta_req_data[199](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[199]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[200] != MagnaParamCSReq_old.meta_req_data[200]){
        std::cout << "MagnaParamCSReq_.meta_req_data[200](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[200]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[201] != MagnaParamCSReq_old.meta_req_data[201]){
        std::cout << "MagnaParamCSReq_.meta_req_data[201](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[201]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[202] != MagnaParamCSReq_old.meta_req_data[202]){
        std::cout << "MagnaParamCSReq_.meta_req_data[202](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[202]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[203] != MagnaParamCSReq_old.meta_req_data[203]){
        std::cout << "MagnaParamCSReq_.meta_req_data[203](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[203]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[204] != MagnaParamCSReq_old.meta_req_data[204]){
        std::cout << "MagnaParamCSReq_.meta_req_data[204](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[204]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[205] != MagnaParamCSReq_old.meta_req_data[205]){
        std::cout << "MagnaParamCSReq_.meta_req_data[205](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[205]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[206] != MagnaParamCSReq_old.meta_req_data[206]){
        std::cout << "MagnaParamCSReq_.meta_req_data[206](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[206]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[207] != MagnaParamCSReq_old.meta_req_data[207]){
        std::cout << "MagnaParamCSReq_.meta_req_data[207](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[207]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[208] != MagnaParamCSReq_old.meta_req_data[208]){
        std::cout << "MagnaParamCSReq_.meta_req_data[208](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[208]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[209] != MagnaParamCSReq_old.meta_req_data[209]){
        std::cout << "MagnaParamCSReq_.meta_req_data[209](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[209]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[210] != MagnaParamCSReq_old.meta_req_data[210]){
        std::cout << "MagnaParamCSReq_.meta_req_data[210](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[210]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[211] != MagnaParamCSReq_old.meta_req_data[211]){
        std::cout << "MagnaParamCSReq_.meta_req_data[211](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[211]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[212] != MagnaParamCSReq_old.meta_req_data[212]){
        std::cout << "MagnaParamCSReq_.meta_req_data[212](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[212]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[213] != MagnaParamCSReq_old.meta_req_data[213]){
        std::cout << "MagnaParamCSReq_.meta_req_data[213](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[213]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[214] != MagnaParamCSReq_old.meta_req_data[214]){
        std::cout << "MagnaParamCSReq_.meta_req_data[214](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[214]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[215] != MagnaParamCSReq_old.meta_req_data[215]){
        std::cout << "MagnaParamCSReq_.meta_req_data[215](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[215]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[216] != MagnaParamCSReq_old.meta_req_data[216]){
        std::cout << "MagnaParamCSReq_.meta_req_data[216](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[216]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[217] != MagnaParamCSReq_old.meta_req_data[217]){
        std::cout << "MagnaParamCSReq_.meta_req_data[217](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[217]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[218] != MagnaParamCSReq_old.meta_req_data[218]){
        std::cout << "MagnaParamCSReq_.meta_req_data[218](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[218]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[219] != MagnaParamCSReq_old.meta_req_data[219]){
        std::cout << "MagnaParamCSReq_.meta_req_data[219](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[219]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[220] != MagnaParamCSReq_old.meta_req_data[220]){
        std::cout << "MagnaParamCSReq_.meta_req_data[220](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[220]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[221] != MagnaParamCSReq_old.meta_req_data[221]){
        std::cout << "MagnaParamCSReq_.meta_req_data[221](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[221]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[222] != MagnaParamCSReq_old.meta_req_data[222]){
        std::cout << "MagnaParamCSReq_.meta_req_data[222](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[222]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[223] != MagnaParamCSReq_old.meta_req_data[223]){
        std::cout << "MagnaParamCSReq_.meta_req_data[223](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[223]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[224] != MagnaParamCSReq_old.meta_req_data[224]){
        std::cout << "MagnaParamCSReq_.meta_req_data[224](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[224]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[225] != MagnaParamCSReq_old.meta_req_data[225]){
        std::cout << "MagnaParamCSReq_.meta_req_data[225](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[225]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[226] != MagnaParamCSReq_old.meta_req_data[226]){
        std::cout << "MagnaParamCSReq_.meta_req_data[226](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[226]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[227] != MagnaParamCSReq_old.meta_req_data[227]){
        std::cout << "MagnaParamCSReq_.meta_req_data[227](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[227]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[228] != MagnaParamCSReq_old.meta_req_data[228]){
        std::cout << "MagnaParamCSReq_.meta_req_data[228](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[228]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[229] != MagnaParamCSReq_old.meta_req_data[229]){
        std::cout << "MagnaParamCSReq_.meta_req_data[229](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[229]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[230] != MagnaParamCSReq_old.meta_req_data[230]){
        std::cout << "MagnaParamCSReq_.meta_req_data[230](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[230]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[231] != MagnaParamCSReq_old.meta_req_data[231]){
        std::cout << "MagnaParamCSReq_.meta_req_data[231](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[231]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[232] != MagnaParamCSReq_old.meta_req_data[232]){
        std::cout << "MagnaParamCSReq_.meta_req_data[232](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[232]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[233] != MagnaParamCSReq_old.meta_req_data[233]){
        std::cout << "MagnaParamCSReq_.meta_req_data[233](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[233]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[234] != MagnaParamCSReq_old.meta_req_data[234]){
        std::cout << "MagnaParamCSReq_.meta_req_data[234](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[234]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[235] != MagnaParamCSReq_old.meta_req_data[235]){
        std::cout << "MagnaParamCSReq_.meta_req_data[235](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[235]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[236] != MagnaParamCSReq_old.meta_req_data[236]){
        std::cout << "MagnaParamCSReq_.meta_req_data[236](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[236]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[237] != MagnaParamCSReq_old.meta_req_data[237]){
        std::cout << "MagnaParamCSReq_.meta_req_data[237](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[237]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[238] != MagnaParamCSReq_old.meta_req_data[238]){
        std::cout << "MagnaParamCSReq_.meta_req_data[238](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[238]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[239] != MagnaParamCSReq_old.meta_req_data[239]){
        std::cout << "MagnaParamCSReq_.meta_req_data[239](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[239]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[240] != MagnaParamCSReq_old.meta_req_data[240]){
        std::cout << "MagnaParamCSReq_.meta_req_data[240](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[240]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[241] != MagnaParamCSReq_old.meta_req_data[241]){
        std::cout << "MagnaParamCSReq_.meta_req_data[241](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[241]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[242] != MagnaParamCSReq_old.meta_req_data[242]){
        std::cout << "MagnaParamCSReq_.meta_req_data[242](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[242]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[243] != MagnaParamCSReq_old.meta_req_data[243]){
        std::cout << "MagnaParamCSReq_.meta_req_data[243](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[243]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[244] != MagnaParamCSReq_old.meta_req_data[244]){
        std::cout << "MagnaParamCSReq_.meta_req_data[244](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[244]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[245] != MagnaParamCSReq_old.meta_req_data[245]){
        std::cout << "MagnaParamCSReq_.meta_req_data[245](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[245]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[246] != MagnaParamCSReq_old.meta_req_data[246]){
        std::cout << "MagnaParamCSReq_.meta_req_data[246](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[246]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[247] != MagnaParamCSReq_old.meta_req_data[247]){
        std::cout << "MagnaParamCSReq_.meta_req_data[247](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[247]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[248] != MagnaParamCSReq_old.meta_req_data[248]){
        std::cout << "MagnaParamCSReq_.meta_req_data[248](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[248]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[249] != MagnaParamCSReq_old.meta_req_data[249]){
        std::cout << "MagnaParamCSReq_.meta_req_data[249](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[249]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[250] != MagnaParamCSReq_old.meta_req_data[250]){
        std::cout << "MagnaParamCSReq_.meta_req_data[250](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[250]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[251] != MagnaParamCSReq_old.meta_req_data[251]){
        std::cout << "MagnaParamCSReq_.meta_req_data[251](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[251]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[252] != MagnaParamCSReq_old.meta_req_data[252]){
        std::cout << "MagnaParamCSReq_.meta_req_data[252](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[252]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[253] != MagnaParamCSReq_old.meta_req_data[253]){
        std::cout << "MagnaParamCSReq_.meta_req_data[253](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[253]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[254] != MagnaParamCSReq_old.meta_req_data[254]){
        std::cout << "MagnaParamCSReq_.meta_req_data[254](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[254]) << std::dec  << std::endl;
        }
    if(MagnaParamCSReq_.meta_req_data[255] != MagnaParamCSReq_old.meta_req_data[255]){
        std::cout << "MagnaParamCSReq_.meta_req_data[255](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSReq_.meta_req_data[255]) << std::dec  << std::endl;
        }
}



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
    if(MagnaParamCSRes_.meta_res_data[10] != MagnaParamCSRes_old.meta_res_data[10]){
        std::cout << "MagnaParamCSRes_.meta_res_data[10](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[10]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[11] != MagnaParamCSRes_old.meta_res_data[11]){
        std::cout << "MagnaParamCSRes_.meta_res_data[11](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[11]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[12] != MagnaParamCSRes_old.meta_res_data[12]){
        std::cout << "MagnaParamCSRes_.meta_res_data[12](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[12]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[13] != MagnaParamCSRes_old.meta_res_data[13]){
        std::cout << "MagnaParamCSRes_.meta_res_data[13](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[13]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[14] != MagnaParamCSRes_old.meta_res_data[14]){
        std::cout << "MagnaParamCSRes_.meta_res_data[14](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[14]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[15] != MagnaParamCSRes_old.meta_res_data[15]){
        std::cout << "MagnaParamCSRes_.meta_res_data[15](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[15]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[16] != MagnaParamCSRes_old.meta_res_data[16]){
        std::cout << "MagnaParamCSRes_.meta_res_data[16](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[16]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[17] != MagnaParamCSRes_old.meta_res_data[17]){
        std::cout << "MagnaParamCSRes_.meta_res_data[17](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[17]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[18] != MagnaParamCSRes_old.meta_res_data[18]){
        std::cout << "MagnaParamCSRes_.meta_res_data[18](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[18]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[19] != MagnaParamCSRes_old.meta_res_data[19]){
        std::cout << "MagnaParamCSRes_.meta_res_data[19](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[19]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[20] != MagnaParamCSRes_old.meta_res_data[20]){
        std::cout << "MagnaParamCSRes_.meta_res_data[20](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[20]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[21] != MagnaParamCSRes_old.meta_res_data[21]){
        std::cout << "MagnaParamCSRes_.meta_res_data[21](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[21]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[22] != MagnaParamCSRes_old.meta_res_data[22]){
        std::cout << "MagnaParamCSRes_.meta_res_data[22](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[22]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[23] != MagnaParamCSRes_old.meta_res_data[23]){
        std::cout << "MagnaParamCSRes_.meta_res_data[23](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[23]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[24] != MagnaParamCSRes_old.meta_res_data[24]){
        std::cout << "MagnaParamCSRes_.meta_res_data[24](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[24]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[25] != MagnaParamCSRes_old.meta_res_data[25]){
        std::cout << "MagnaParamCSRes_.meta_res_data[25](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[25]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[26] != MagnaParamCSRes_old.meta_res_data[26]){
        std::cout << "MagnaParamCSRes_.meta_res_data[26](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[26]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[27] != MagnaParamCSRes_old.meta_res_data[27]){
        std::cout << "MagnaParamCSRes_.meta_res_data[27](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[27]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[28] != MagnaParamCSRes_old.meta_res_data[28]){
        std::cout << "MagnaParamCSRes_.meta_res_data[28](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[28]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[29] != MagnaParamCSRes_old.meta_res_data[29]){
        std::cout << "MagnaParamCSRes_.meta_res_data[29](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[29]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[30] != MagnaParamCSRes_old.meta_res_data[30]){
        std::cout << "MagnaParamCSRes_.meta_res_data[30](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[30]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[31] != MagnaParamCSRes_old.meta_res_data[31]){
        std::cout << "MagnaParamCSRes_.meta_res_data[31](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[31]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[32] != MagnaParamCSRes_old.meta_res_data[32]){
        std::cout << "MagnaParamCSRes_.meta_res_data[32](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[32]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[33] != MagnaParamCSRes_old.meta_res_data[33]){
        std::cout << "MagnaParamCSRes_.meta_res_data[33](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[33]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[34] != MagnaParamCSRes_old.meta_res_data[34]){
        std::cout << "MagnaParamCSRes_.meta_res_data[34](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[34]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[35] != MagnaParamCSRes_old.meta_res_data[35]){
        std::cout << "MagnaParamCSRes_.meta_res_data[35](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[35]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[36] != MagnaParamCSRes_old.meta_res_data[36]){
        std::cout << "MagnaParamCSRes_.meta_res_data[36](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[36]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[37] != MagnaParamCSRes_old.meta_res_data[37]){
        std::cout << "MagnaParamCSRes_.meta_res_data[37](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[37]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[38] != MagnaParamCSRes_old.meta_res_data[38]){
        std::cout << "MagnaParamCSRes_.meta_res_data[38](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[38]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[39] != MagnaParamCSRes_old.meta_res_data[39]){
        std::cout << "MagnaParamCSRes_.meta_res_data[39](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[39]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[40] != MagnaParamCSRes_old.meta_res_data[40]){
        std::cout << "MagnaParamCSRes_.meta_res_data[40](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[40]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[41] != MagnaParamCSRes_old.meta_res_data[41]){
        std::cout << "MagnaParamCSRes_.meta_res_data[41](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[41]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[42] != MagnaParamCSRes_old.meta_res_data[42]){
        std::cout << "MagnaParamCSRes_.meta_res_data[42](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[42]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[43] != MagnaParamCSRes_old.meta_res_data[43]){
        std::cout << "MagnaParamCSRes_.meta_res_data[43](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[43]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[44] != MagnaParamCSRes_old.meta_res_data[44]){
        std::cout << "MagnaParamCSRes_.meta_res_data[44](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[44]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[45] != MagnaParamCSRes_old.meta_res_data[45]){
        std::cout << "MagnaParamCSRes_.meta_res_data[45](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[45]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[46] != MagnaParamCSRes_old.meta_res_data[46]){
        std::cout << "MagnaParamCSRes_.meta_res_data[46](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[46]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[47] != MagnaParamCSRes_old.meta_res_data[47]){
        std::cout << "MagnaParamCSRes_.meta_res_data[47](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[47]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[48] != MagnaParamCSRes_old.meta_res_data[48]){
        std::cout << "MagnaParamCSRes_.meta_res_data[48](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[48]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[49] != MagnaParamCSRes_old.meta_res_data[49]){
        std::cout << "MagnaParamCSRes_.meta_res_data[49](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[49]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[50] != MagnaParamCSRes_old.meta_res_data[50]){
        std::cout << "MagnaParamCSRes_.meta_res_data[50](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[50]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[51] != MagnaParamCSRes_old.meta_res_data[51]){
        std::cout << "MagnaParamCSRes_.meta_res_data[51](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[51]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[52] != MagnaParamCSRes_old.meta_res_data[52]){
        std::cout << "MagnaParamCSRes_.meta_res_data[52](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[52]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[53] != MagnaParamCSRes_old.meta_res_data[53]){
        std::cout << "MagnaParamCSRes_.meta_res_data[53](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[53]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[54] != MagnaParamCSRes_old.meta_res_data[54]){
        std::cout << "MagnaParamCSRes_.meta_res_data[54](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[54]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[55] != MagnaParamCSRes_old.meta_res_data[55]){
        std::cout << "MagnaParamCSRes_.meta_res_data[55](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[55]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[56] != MagnaParamCSRes_old.meta_res_data[56]){
        std::cout << "MagnaParamCSRes_.meta_res_data[56](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[56]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[57] != MagnaParamCSRes_old.meta_res_data[57]){
        std::cout << "MagnaParamCSRes_.meta_res_data[57](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[57]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[58] != MagnaParamCSRes_old.meta_res_data[58]){
        std::cout << "MagnaParamCSRes_.meta_res_data[58](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[58]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[59] != MagnaParamCSRes_old.meta_res_data[59]){
        std::cout << "MagnaParamCSRes_.meta_res_data[59](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[59]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[60] != MagnaParamCSRes_old.meta_res_data[60]){
        std::cout << "MagnaParamCSRes_.meta_res_data[60](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[60]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[61] != MagnaParamCSRes_old.meta_res_data[61]){
        std::cout << "MagnaParamCSRes_.meta_res_data[61](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[61]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[62] != MagnaParamCSRes_old.meta_res_data[62]){
        std::cout << "MagnaParamCSRes_.meta_res_data[62](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[62]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[63] != MagnaParamCSRes_old.meta_res_data[63]){
        std::cout << "MagnaParamCSRes_.meta_res_data[63](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[63]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[64] != MagnaParamCSRes_old.meta_res_data[64]){
        std::cout << "MagnaParamCSRes_.meta_res_data[64](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[64]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[65] != MagnaParamCSRes_old.meta_res_data[65]){
        std::cout << "MagnaParamCSRes_.meta_res_data[65](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[65]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[66] != MagnaParamCSRes_old.meta_res_data[66]){
        std::cout << "MagnaParamCSRes_.meta_res_data[66](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[66]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[67] != MagnaParamCSRes_old.meta_res_data[67]){
        std::cout << "MagnaParamCSRes_.meta_res_data[67](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[67]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[68] != MagnaParamCSRes_old.meta_res_data[68]){
        std::cout << "MagnaParamCSRes_.meta_res_data[68](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[68]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[69] != MagnaParamCSRes_old.meta_res_data[69]){
        std::cout << "MagnaParamCSRes_.meta_res_data[69](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[69]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[70] != MagnaParamCSRes_old.meta_res_data[70]){
        std::cout << "MagnaParamCSRes_.meta_res_data[70](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[70]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[71] != MagnaParamCSRes_old.meta_res_data[71]){
        std::cout << "MagnaParamCSRes_.meta_res_data[71](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[71]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[72] != MagnaParamCSRes_old.meta_res_data[72]){
        std::cout << "MagnaParamCSRes_.meta_res_data[72](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[72]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[73] != MagnaParamCSRes_old.meta_res_data[73]){
        std::cout << "MagnaParamCSRes_.meta_res_data[73](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[73]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[74] != MagnaParamCSRes_old.meta_res_data[74]){
        std::cout << "MagnaParamCSRes_.meta_res_data[74](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[74]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[75] != MagnaParamCSRes_old.meta_res_data[75]){
        std::cout << "MagnaParamCSRes_.meta_res_data[75](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[75]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[76] != MagnaParamCSRes_old.meta_res_data[76]){
        std::cout << "MagnaParamCSRes_.meta_res_data[76](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[76]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[77] != MagnaParamCSRes_old.meta_res_data[77]){
        std::cout << "MagnaParamCSRes_.meta_res_data[77](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[77]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[78] != MagnaParamCSRes_old.meta_res_data[78]){
        std::cout << "MagnaParamCSRes_.meta_res_data[78](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[78]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[79] != MagnaParamCSRes_old.meta_res_data[79]){
        std::cout << "MagnaParamCSRes_.meta_res_data[79](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[79]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[80] != MagnaParamCSRes_old.meta_res_data[80]){
        std::cout << "MagnaParamCSRes_.meta_res_data[80](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[80]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[81] != MagnaParamCSRes_old.meta_res_data[81]){
        std::cout << "MagnaParamCSRes_.meta_res_data[81](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[81]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[82] != MagnaParamCSRes_old.meta_res_data[82]){
        std::cout << "MagnaParamCSRes_.meta_res_data[82](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[82]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[83] != MagnaParamCSRes_old.meta_res_data[83]){
        std::cout << "MagnaParamCSRes_.meta_res_data[83](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[83]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[84] != MagnaParamCSRes_old.meta_res_data[84]){
        std::cout << "MagnaParamCSRes_.meta_res_data[84](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[84]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[85] != MagnaParamCSRes_old.meta_res_data[85]){
        std::cout << "MagnaParamCSRes_.meta_res_data[85](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[85]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[86] != MagnaParamCSRes_old.meta_res_data[86]){
        std::cout << "MagnaParamCSRes_.meta_res_data[86](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[86]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[87] != MagnaParamCSRes_old.meta_res_data[87]){
        std::cout << "MagnaParamCSRes_.meta_res_data[87](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[87]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[88] != MagnaParamCSRes_old.meta_res_data[88]){
        std::cout << "MagnaParamCSRes_.meta_res_data[88](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[88]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[89] != MagnaParamCSRes_old.meta_res_data[89]){
        std::cout << "MagnaParamCSRes_.meta_res_data[89](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[89]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[90] != MagnaParamCSRes_old.meta_res_data[90]){
        std::cout << "MagnaParamCSRes_.meta_res_data[90](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[90]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[91] != MagnaParamCSRes_old.meta_res_data[91]){
        std::cout << "MagnaParamCSRes_.meta_res_data[91](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[91]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[92] != MagnaParamCSRes_old.meta_res_data[92]){
        std::cout << "MagnaParamCSRes_.meta_res_data[92](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[92]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[93] != MagnaParamCSRes_old.meta_res_data[93]){
        std::cout << "MagnaParamCSRes_.meta_res_data[93](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[93]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[94] != MagnaParamCSRes_old.meta_res_data[94]){
        std::cout << "MagnaParamCSRes_.meta_res_data[94](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[94]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[95] != MagnaParamCSRes_old.meta_res_data[95]){
        std::cout << "MagnaParamCSRes_.meta_res_data[95](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[95]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[96] != MagnaParamCSRes_old.meta_res_data[96]){
        std::cout << "MagnaParamCSRes_.meta_res_data[96](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[96]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[97] != MagnaParamCSRes_old.meta_res_data[97]){
        std::cout << "MagnaParamCSRes_.meta_res_data[97](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[97]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[98] != MagnaParamCSRes_old.meta_res_data[98]){
        std::cout << "MagnaParamCSRes_.meta_res_data[98](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[98]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[99] != MagnaParamCSRes_old.meta_res_data[99]){
        std::cout << "MagnaParamCSRes_.meta_res_data[99](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[99]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[100] != MagnaParamCSRes_old.meta_res_data[100]){
        std::cout << "MagnaParamCSRes_.meta_res_data[100](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[100]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[101] != MagnaParamCSRes_old.meta_res_data[101]){
        std::cout << "MagnaParamCSRes_.meta_res_data[101](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[101]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[102] != MagnaParamCSRes_old.meta_res_data[102]){
        std::cout << "MagnaParamCSRes_.meta_res_data[102](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[102]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[103] != MagnaParamCSRes_old.meta_res_data[103]){
        std::cout << "MagnaParamCSRes_.meta_res_data[103](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[103]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[104] != MagnaParamCSRes_old.meta_res_data[104]){
        std::cout << "MagnaParamCSRes_.meta_res_data[104](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[104]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[105] != MagnaParamCSRes_old.meta_res_data[105]){
        std::cout << "MagnaParamCSRes_.meta_res_data[105](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[105]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[106] != MagnaParamCSRes_old.meta_res_data[106]){
        std::cout << "MagnaParamCSRes_.meta_res_data[106](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[106]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[107] != MagnaParamCSRes_old.meta_res_data[107]){
        std::cout << "MagnaParamCSRes_.meta_res_data[107](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[107]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[108] != MagnaParamCSRes_old.meta_res_data[108]){
        std::cout << "MagnaParamCSRes_.meta_res_data[108](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[108]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[109] != MagnaParamCSRes_old.meta_res_data[109]){
        std::cout << "MagnaParamCSRes_.meta_res_data[109](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[109]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[110] != MagnaParamCSRes_old.meta_res_data[110]){
        std::cout << "MagnaParamCSRes_.meta_res_data[110](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[110]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[111] != MagnaParamCSRes_old.meta_res_data[111]){
        std::cout << "MagnaParamCSRes_.meta_res_data[111](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[111]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[112] != MagnaParamCSRes_old.meta_res_data[112]){
        std::cout << "MagnaParamCSRes_.meta_res_data[112](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[112]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[113] != MagnaParamCSRes_old.meta_res_data[113]){
        std::cout << "MagnaParamCSRes_.meta_res_data[113](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[113]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[114] != MagnaParamCSRes_old.meta_res_data[114]){
        std::cout << "MagnaParamCSRes_.meta_res_data[114](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[114]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[115] != MagnaParamCSRes_old.meta_res_data[115]){
        std::cout << "MagnaParamCSRes_.meta_res_data[115](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[115]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[116] != MagnaParamCSRes_old.meta_res_data[116]){
        std::cout << "MagnaParamCSRes_.meta_res_data[116](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[116]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[117] != MagnaParamCSRes_old.meta_res_data[117]){
        std::cout << "MagnaParamCSRes_.meta_res_data[117](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[117]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[118] != MagnaParamCSRes_old.meta_res_data[118]){
        std::cout << "MagnaParamCSRes_.meta_res_data[118](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[118]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[119] != MagnaParamCSRes_old.meta_res_data[119]){
        std::cout << "MagnaParamCSRes_.meta_res_data[119](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[119]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[120] != MagnaParamCSRes_old.meta_res_data[120]){
        std::cout << "MagnaParamCSRes_.meta_res_data[120](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[120]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[121] != MagnaParamCSRes_old.meta_res_data[121]){
        std::cout << "MagnaParamCSRes_.meta_res_data[121](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[121]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[122] != MagnaParamCSRes_old.meta_res_data[122]){
        std::cout << "MagnaParamCSRes_.meta_res_data[122](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[122]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[123] != MagnaParamCSRes_old.meta_res_data[123]){
        std::cout << "MagnaParamCSRes_.meta_res_data[123](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[123]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[124] != MagnaParamCSRes_old.meta_res_data[124]){
        std::cout << "MagnaParamCSRes_.meta_res_data[124](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[124]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[125] != MagnaParamCSRes_old.meta_res_data[125]){
        std::cout << "MagnaParamCSRes_.meta_res_data[125](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[125]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[126] != MagnaParamCSRes_old.meta_res_data[126]){
        std::cout << "MagnaParamCSRes_.meta_res_data[126](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[126]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[127] != MagnaParamCSRes_old.meta_res_data[127]){
        std::cout << "MagnaParamCSRes_.meta_res_data[127](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[127]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[128] != MagnaParamCSRes_old.meta_res_data[128]){
        std::cout << "MagnaParamCSRes_.meta_res_data[128](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[128]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[129] != MagnaParamCSRes_old.meta_res_data[129]){
        std::cout << "MagnaParamCSRes_.meta_res_data[129](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[129]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[130] != MagnaParamCSRes_old.meta_res_data[130]){
        std::cout << "MagnaParamCSRes_.meta_res_data[130](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[130]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[131] != MagnaParamCSRes_old.meta_res_data[131]){
        std::cout << "MagnaParamCSRes_.meta_res_data[131](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[131]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[132] != MagnaParamCSRes_old.meta_res_data[132]){
        std::cout << "MagnaParamCSRes_.meta_res_data[132](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[132]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[133] != MagnaParamCSRes_old.meta_res_data[133]){
        std::cout << "MagnaParamCSRes_.meta_res_data[133](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[133]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[134] != MagnaParamCSRes_old.meta_res_data[134]){
        std::cout << "MagnaParamCSRes_.meta_res_data[134](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[134]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[135] != MagnaParamCSRes_old.meta_res_data[135]){
        std::cout << "MagnaParamCSRes_.meta_res_data[135](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[135]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[136] != MagnaParamCSRes_old.meta_res_data[136]){
        std::cout << "MagnaParamCSRes_.meta_res_data[136](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[136]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[137] != MagnaParamCSRes_old.meta_res_data[137]){
        std::cout << "MagnaParamCSRes_.meta_res_data[137](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[137]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[138] != MagnaParamCSRes_old.meta_res_data[138]){
        std::cout << "MagnaParamCSRes_.meta_res_data[138](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[138]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[139] != MagnaParamCSRes_old.meta_res_data[139]){
        std::cout << "MagnaParamCSRes_.meta_res_data[139](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[139]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[140] != MagnaParamCSRes_old.meta_res_data[140]){
        std::cout << "MagnaParamCSRes_.meta_res_data[140](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[140]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[141] != MagnaParamCSRes_old.meta_res_data[141]){
        std::cout << "MagnaParamCSRes_.meta_res_data[141](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[141]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[142] != MagnaParamCSRes_old.meta_res_data[142]){
        std::cout << "MagnaParamCSRes_.meta_res_data[142](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[142]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[143] != MagnaParamCSRes_old.meta_res_data[143]){
        std::cout << "MagnaParamCSRes_.meta_res_data[143](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[143]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[144] != MagnaParamCSRes_old.meta_res_data[144]){
        std::cout << "MagnaParamCSRes_.meta_res_data[144](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[144]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[145] != MagnaParamCSRes_old.meta_res_data[145]){
        std::cout << "MagnaParamCSRes_.meta_res_data[145](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[145]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[146] != MagnaParamCSRes_old.meta_res_data[146]){
        std::cout << "MagnaParamCSRes_.meta_res_data[146](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[146]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[147] != MagnaParamCSRes_old.meta_res_data[147]){
        std::cout << "MagnaParamCSRes_.meta_res_data[147](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[147]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[148] != MagnaParamCSRes_old.meta_res_data[148]){
        std::cout << "MagnaParamCSRes_.meta_res_data[148](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[148]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[149] != MagnaParamCSRes_old.meta_res_data[149]){
        std::cout << "MagnaParamCSRes_.meta_res_data[149](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[149]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[150] != MagnaParamCSRes_old.meta_res_data[150]){
        std::cout << "MagnaParamCSRes_.meta_res_data[150](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[150]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[151] != MagnaParamCSRes_old.meta_res_data[151]){
        std::cout << "MagnaParamCSRes_.meta_res_data[151](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[151]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[152] != MagnaParamCSRes_old.meta_res_data[152]){
        std::cout << "MagnaParamCSRes_.meta_res_data[152](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[152]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[153] != MagnaParamCSRes_old.meta_res_data[153]){
        std::cout << "MagnaParamCSRes_.meta_res_data[153](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[153]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[154] != MagnaParamCSRes_old.meta_res_data[154]){
        std::cout << "MagnaParamCSRes_.meta_res_data[154](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[154]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[155] != MagnaParamCSRes_old.meta_res_data[155]){
        std::cout << "MagnaParamCSRes_.meta_res_data[155](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[155]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[156] != MagnaParamCSRes_old.meta_res_data[156]){
        std::cout << "MagnaParamCSRes_.meta_res_data[156](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[156]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[157] != MagnaParamCSRes_old.meta_res_data[157]){
        std::cout << "MagnaParamCSRes_.meta_res_data[157](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[157]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[158] != MagnaParamCSRes_old.meta_res_data[158]){
        std::cout << "MagnaParamCSRes_.meta_res_data[158](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[158]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[159] != MagnaParamCSRes_old.meta_res_data[159]){
        std::cout << "MagnaParamCSRes_.meta_res_data[159](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[159]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[160] != MagnaParamCSRes_old.meta_res_data[160]){
        std::cout << "MagnaParamCSRes_.meta_res_data[160](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[160]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[161] != MagnaParamCSRes_old.meta_res_data[161]){
        std::cout << "MagnaParamCSRes_.meta_res_data[161](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[161]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[162] != MagnaParamCSRes_old.meta_res_data[162]){
        std::cout << "MagnaParamCSRes_.meta_res_data[162](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[162]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[163] != MagnaParamCSRes_old.meta_res_data[163]){
        std::cout << "MagnaParamCSRes_.meta_res_data[163](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[163]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[164] != MagnaParamCSRes_old.meta_res_data[164]){
        std::cout << "MagnaParamCSRes_.meta_res_data[164](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[164]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[165] != MagnaParamCSRes_old.meta_res_data[165]){
        std::cout << "MagnaParamCSRes_.meta_res_data[165](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[165]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[166] != MagnaParamCSRes_old.meta_res_data[166]){
        std::cout << "MagnaParamCSRes_.meta_res_data[166](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[166]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[167] != MagnaParamCSRes_old.meta_res_data[167]){
        std::cout << "MagnaParamCSRes_.meta_res_data[167](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[167]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[168] != MagnaParamCSRes_old.meta_res_data[168]){
        std::cout << "MagnaParamCSRes_.meta_res_data[168](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[168]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[169] != MagnaParamCSRes_old.meta_res_data[169]){
        std::cout << "MagnaParamCSRes_.meta_res_data[169](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[169]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[170] != MagnaParamCSRes_old.meta_res_data[170]){
        std::cout << "MagnaParamCSRes_.meta_res_data[170](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[170]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[171] != MagnaParamCSRes_old.meta_res_data[171]){
        std::cout << "MagnaParamCSRes_.meta_res_data[171](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[171]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[172] != MagnaParamCSRes_old.meta_res_data[172]){
        std::cout << "MagnaParamCSRes_.meta_res_data[172](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[172]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[173] != MagnaParamCSRes_old.meta_res_data[173]){
        std::cout << "MagnaParamCSRes_.meta_res_data[173](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[173]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[174] != MagnaParamCSRes_old.meta_res_data[174]){
        std::cout << "MagnaParamCSRes_.meta_res_data[174](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[174]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[175] != MagnaParamCSRes_old.meta_res_data[175]){
        std::cout << "MagnaParamCSRes_.meta_res_data[175](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[175]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[176] != MagnaParamCSRes_old.meta_res_data[176]){
        std::cout << "MagnaParamCSRes_.meta_res_data[176](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[176]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[177] != MagnaParamCSRes_old.meta_res_data[177]){
        std::cout << "MagnaParamCSRes_.meta_res_data[177](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[177]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[178] != MagnaParamCSRes_old.meta_res_data[178]){
        std::cout << "MagnaParamCSRes_.meta_res_data[178](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[178]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[179] != MagnaParamCSRes_old.meta_res_data[179]){
        std::cout << "MagnaParamCSRes_.meta_res_data[179](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[179]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[180] != MagnaParamCSRes_old.meta_res_data[180]){
        std::cout << "MagnaParamCSRes_.meta_res_data[180](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[180]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[181] != MagnaParamCSRes_old.meta_res_data[181]){
        std::cout << "MagnaParamCSRes_.meta_res_data[181](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[181]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[182] != MagnaParamCSRes_old.meta_res_data[182]){
        std::cout << "MagnaParamCSRes_.meta_res_data[182](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[182]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[183] != MagnaParamCSRes_old.meta_res_data[183]){
        std::cout << "MagnaParamCSRes_.meta_res_data[183](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[183]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[184] != MagnaParamCSRes_old.meta_res_data[184]){
        std::cout << "MagnaParamCSRes_.meta_res_data[184](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[184]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[185] != MagnaParamCSRes_old.meta_res_data[185]){
        std::cout << "MagnaParamCSRes_.meta_res_data[185](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[185]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[186] != MagnaParamCSRes_old.meta_res_data[186]){
        std::cout << "MagnaParamCSRes_.meta_res_data[186](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[186]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[187] != MagnaParamCSRes_old.meta_res_data[187]){
        std::cout << "MagnaParamCSRes_.meta_res_data[187](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[187]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[188] != MagnaParamCSRes_old.meta_res_data[188]){
        std::cout << "MagnaParamCSRes_.meta_res_data[188](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[188]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[189] != MagnaParamCSRes_old.meta_res_data[189]){
        std::cout << "MagnaParamCSRes_.meta_res_data[189](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[189]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[190] != MagnaParamCSRes_old.meta_res_data[190]){
        std::cout << "MagnaParamCSRes_.meta_res_data[190](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[190]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[191] != MagnaParamCSRes_old.meta_res_data[191]){
        std::cout << "MagnaParamCSRes_.meta_res_data[191](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[191]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[192] != MagnaParamCSRes_old.meta_res_data[192]){
        std::cout << "MagnaParamCSRes_.meta_res_data[192](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[192]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[193] != MagnaParamCSRes_old.meta_res_data[193]){
        std::cout << "MagnaParamCSRes_.meta_res_data[193](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[193]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[194] != MagnaParamCSRes_old.meta_res_data[194]){
        std::cout << "MagnaParamCSRes_.meta_res_data[194](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[194]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[195] != MagnaParamCSRes_old.meta_res_data[195]){
        std::cout << "MagnaParamCSRes_.meta_res_data[195](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[195]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[196] != MagnaParamCSRes_old.meta_res_data[196]){
        std::cout << "MagnaParamCSRes_.meta_res_data[196](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[196]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[197] != MagnaParamCSRes_old.meta_res_data[197]){
        std::cout << "MagnaParamCSRes_.meta_res_data[197](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[197]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[198] != MagnaParamCSRes_old.meta_res_data[198]){
        std::cout << "MagnaParamCSRes_.meta_res_data[198](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[198]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[199] != MagnaParamCSRes_old.meta_res_data[199]){
        std::cout << "MagnaParamCSRes_.meta_res_data[199](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[199]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[200] != MagnaParamCSRes_old.meta_res_data[200]){
        std::cout << "MagnaParamCSRes_.meta_res_data[200](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[200]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[201] != MagnaParamCSRes_old.meta_res_data[201]){
        std::cout << "MagnaParamCSRes_.meta_res_data[201](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[201]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[202] != MagnaParamCSRes_old.meta_res_data[202]){
        std::cout << "MagnaParamCSRes_.meta_res_data[202](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[202]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[203] != MagnaParamCSRes_old.meta_res_data[203]){
        std::cout << "MagnaParamCSRes_.meta_res_data[203](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[203]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[204] != MagnaParamCSRes_old.meta_res_data[204]){
        std::cout << "MagnaParamCSRes_.meta_res_data[204](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[204]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[205] != MagnaParamCSRes_old.meta_res_data[205]){
        std::cout << "MagnaParamCSRes_.meta_res_data[205](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[205]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[206] != MagnaParamCSRes_old.meta_res_data[206]){
        std::cout << "MagnaParamCSRes_.meta_res_data[206](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[206]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[207] != MagnaParamCSRes_old.meta_res_data[207]){
        std::cout << "MagnaParamCSRes_.meta_res_data[207](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[207]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[208] != MagnaParamCSRes_old.meta_res_data[208]){
        std::cout << "MagnaParamCSRes_.meta_res_data[208](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[208]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[209] != MagnaParamCSRes_old.meta_res_data[209]){
        std::cout << "MagnaParamCSRes_.meta_res_data[209](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[209]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[210] != MagnaParamCSRes_old.meta_res_data[210]){
        std::cout << "MagnaParamCSRes_.meta_res_data[210](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[210]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[211] != MagnaParamCSRes_old.meta_res_data[211]){
        std::cout << "MagnaParamCSRes_.meta_res_data[211](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[211]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[212] != MagnaParamCSRes_old.meta_res_data[212]){
        std::cout << "MagnaParamCSRes_.meta_res_data[212](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[212]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[213] != MagnaParamCSRes_old.meta_res_data[213]){
        std::cout << "MagnaParamCSRes_.meta_res_data[213](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[213]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[214] != MagnaParamCSRes_old.meta_res_data[214]){
        std::cout << "MagnaParamCSRes_.meta_res_data[214](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[214]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[215] != MagnaParamCSRes_old.meta_res_data[215]){
        std::cout << "MagnaParamCSRes_.meta_res_data[215](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[215]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[216] != MagnaParamCSRes_old.meta_res_data[216]){
        std::cout << "MagnaParamCSRes_.meta_res_data[216](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[216]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[217] != MagnaParamCSRes_old.meta_res_data[217]){
        std::cout << "MagnaParamCSRes_.meta_res_data[217](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[217]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[218] != MagnaParamCSRes_old.meta_res_data[218]){
        std::cout << "MagnaParamCSRes_.meta_res_data[218](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[218]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[219] != MagnaParamCSRes_old.meta_res_data[219]){
        std::cout << "MagnaParamCSRes_.meta_res_data[219](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[219]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[220] != MagnaParamCSRes_old.meta_res_data[220]){
        std::cout << "MagnaParamCSRes_.meta_res_data[220](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[220]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[221] != MagnaParamCSRes_old.meta_res_data[221]){
        std::cout << "MagnaParamCSRes_.meta_res_data[221](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[221]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[222] != MagnaParamCSRes_old.meta_res_data[222]){
        std::cout << "MagnaParamCSRes_.meta_res_data[222](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[222]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[223] != MagnaParamCSRes_old.meta_res_data[223]){
        std::cout << "MagnaParamCSRes_.meta_res_data[223](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[223]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[224] != MagnaParamCSRes_old.meta_res_data[224]){
        std::cout << "MagnaParamCSRes_.meta_res_data[224](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[224]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[225] != MagnaParamCSRes_old.meta_res_data[225]){
        std::cout << "MagnaParamCSRes_.meta_res_data[225](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[225]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[226] != MagnaParamCSRes_old.meta_res_data[226]){
        std::cout << "MagnaParamCSRes_.meta_res_data[226](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[226]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[227] != MagnaParamCSRes_old.meta_res_data[227]){
        std::cout << "MagnaParamCSRes_.meta_res_data[227](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[227]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[228] != MagnaParamCSRes_old.meta_res_data[228]){
        std::cout << "MagnaParamCSRes_.meta_res_data[228](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[228]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[229] != MagnaParamCSRes_old.meta_res_data[229]){
        std::cout << "MagnaParamCSRes_.meta_res_data[229](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[229]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[230] != MagnaParamCSRes_old.meta_res_data[230]){
        std::cout << "MagnaParamCSRes_.meta_res_data[230](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[230]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[231] != MagnaParamCSRes_old.meta_res_data[231]){
        std::cout << "MagnaParamCSRes_.meta_res_data[231](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[231]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[232] != MagnaParamCSRes_old.meta_res_data[232]){
        std::cout << "MagnaParamCSRes_.meta_res_data[232](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[232]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[233] != MagnaParamCSRes_old.meta_res_data[233]){
        std::cout << "MagnaParamCSRes_.meta_res_data[233](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[233]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[234] != MagnaParamCSRes_old.meta_res_data[234]){
        std::cout << "MagnaParamCSRes_.meta_res_data[234](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[234]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[235] != MagnaParamCSRes_old.meta_res_data[235]){
        std::cout << "MagnaParamCSRes_.meta_res_data[235](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[235]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[236] != MagnaParamCSRes_old.meta_res_data[236]){
        std::cout << "MagnaParamCSRes_.meta_res_data[236](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[236]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[237] != MagnaParamCSRes_old.meta_res_data[237]){
        std::cout << "MagnaParamCSRes_.meta_res_data[237](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[237]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[238] != MagnaParamCSRes_old.meta_res_data[238]){
        std::cout << "MagnaParamCSRes_.meta_res_data[238](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[238]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[239] != MagnaParamCSRes_old.meta_res_data[239]){
        std::cout << "MagnaParamCSRes_.meta_res_data[239](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[239]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[240] != MagnaParamCSRes_old.meta_res_data[240]){
        std::cout << "MagnaParamCSRes_.meta_res_data[240](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[240]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[241] != MagnaParamCSRes_old.meta_res_data[241]){
        std::cout << "MagnaParamCSRes_.meta_res_data[241](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[241]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[242] != MagnaParamCSRes_old.meta_res_data[242]){
        std::cout << "MagnaParamCSRes_.meta_res_data[242](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[242]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[243] != MagnaParamCSRes_old.meta_res_data[243]){
        std::cout << "MagnaParamCSRes_.meta_res_data[243](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[243]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[244] != MagnaParamCSRes_old.meta_res_data[244]){
        std::cout << "MagnaParamCSRes_.meta_res_data[244](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[244]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[245] != MagnaParamCSRes_old.meta_res_data[245]){
        std::cout << "MagnaParamCSRes_.meta_res_data[245](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[245]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[246] != MagnaParamCSRes_old.meta_res_data[246]){
        std::cout << "MagnaParamCSRes_.meta_res_data[246](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[246]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[247] != MagnaParamCSRes_old.meta_res_data[247]){
        std::cout << "MagnaParamCSRes_.meta_res_data[247](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[247]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[248] != MagnaParamCSRes_old.meta_res_data[248]){
        std::cout << "MagnaParamCSRes_.meta_res_data[248](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[248]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[249] != MagnaParamCSRes_old.meta_res_data[249]){
        std::cout << "MagnaParamCSRes_.meta_res_data[249](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[249]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[250] != MagnaParamCSRes_old.meta_res_data[250]){
        std::cout << "MagnaParamCSRes_.meta_res_data[250](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[250]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[251] != MagnaParamCSRes_old.meta_res_data[251]){
        std::cout << "MagnaParamCSRes_.meta_res_data[251](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[251]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[252] != MagnaParamCSRes_old.meta_res_data[252]){
        std::cout << "MagnaParamCSRes_.meta_res_data[252](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[252]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[253] != MagnaParamCSRes_old.meta_res_data[253]){
        std::cout << "MagnaParamCSRes_.meta_res_data[253](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[253]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[254] != MagnaParamCSRes_old.meta_res_data[254]){
        std::cout << "MagnaParamCSRes_.meta_res_data[254](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[254]) << std::dec  << std::endl;
        }
    if(MagnaParamCSRes_.meta_res_data[255] != MagnaParamCSRes_old.meta_res_data[255]){
        std::cout << "MagnaParamCSRes_.meta_res_data[255](uint8_t): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(MagnaParamCSRes_.meta_res_data[255]) << std::dec  << std::endl;
        }
}







