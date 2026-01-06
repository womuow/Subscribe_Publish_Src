#include"Adapter_PARAM_CS.h"


using VariableVariant = std::variant<uint8_t*, uint16_t*, uint32_t*, uint64_t*, int8_t*, int16_t*, int32_t*, int64_t*, float*, bool*,MagnaParamReqType*,MagnaStatusCode*,MagnaClientStatusCode*,MagnaCalibResultCode*>;

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
                    if(tcsetattr(serial_fd,TCSANOW,&options_old)==0){
                            stop.store(true);
                             std::cout << "串口设置已恢复" << std::endl;}
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

    auto data_vec = request->GetDataRef()->GetDataVec();
    auto data_size_vec = request->GetDataRef()->GetDataSizeVec();
    auto size = data_size_vec.size();

    for (int i = 0; i < size; i++) {
        auto vec_size = data_size_vec[i];
        uint8_t* vec_data = static_cast<uint8_t*>(data_vec[i]);
        data_in = std::string(reinterpret_cast<const char*>(vec_data), vec_size);
        // std::cout<<"vec_size="<< vec_size  <<std::endl;

        }


    // print_memory(data_in.data(), data_in.size());
    // std::cout<<" data_size_vec.size() = "<< size  <<std::endl;
    // std::cout<<" data_in size = "<< data_in.size()  <<std::endl;

    // std::cout<<" data_in[1] = "<< static_cast<int>(data_in.data()[1])  <<std::endl;

    // std::cout<<" data_in.length() = "<< data_in.length()  <<";MagnaParamCSReq_ length="<< sizeof(MagnaParamCSReq_) <<std::endl;


    std::memcpy(&MagnaParamCSReq_, data_in.data(), data_in.length());

    std::cout<<"receive request, gen_ts = "<< request->GetGenTimestamp() <<std::endl;
    print_MagnaParamCSReq(MagnaParamCSReq_, MagnaParamCSReq_old);


    //  MagnaParamCSRes_ res;
//   res.rev = request_first_data->event + 1;
MagnaParamCSRes_.meta_res_data[0] = MagnaParamCSReq_.meta_req_data[0] + 1;
  auto res_data_ref = std::make_shared<MOS::message::DataRef>(&MagnaParamCSRes_, sizeof(MagnaParamCSRes));
  response->SetDataRef(res_data_ref);
  response->SetGenTimestamp(MOS::TimeUtils::NowNsec());

  
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

  std::this_thread::sleep_for(std::chrono::seconds(3));


  MOS::communication::Init(discovery_json_file);

  // 配置通信属性
  MOS::communication::ProtocolInfo proto_info;
  set_proto_info(proto_info, type, port, ip, sizeof(MagnaParamCSReq));

  // 创建Client
  auto client = MOS::communication::Client::New(domain_id, service_name, proto_info);
  uint8_t count = 0;
  int timeout_ms = 10000;
  MagnaParamCSReq req;
  MagnaParamCSRes response_old;
  while (!stop.load()) 
  {
    count++;
    req.meta_req_data[0] = count;
    // std::cout<<"output req.meta_req_data[0] = 0x"<<static_cast<int>(req.meta_req_data[0])<<std::endl;
    
    auto request_msg = std::make_shared<MOS::message::Message>();
    auto response_msg = std::make_shared<MOS::message::Message>();
    auto data_ref = std::make_shared<MOS::message::DataRef>(&req, sizeof(MagnaParamCSReq));
    request_msg->SetDataRef(data_ref);

    
    // print_memory(&req, sizeof(MagnaParamCSReq));

    // 发送 request
    auto ret = client->SendRequest(request_msg, response_msg, timeout_ms);
    if (ret == MOS::communication::COMM_CODE_OK) {
      auto response = static_cast<MagnaParamCSRes*>(response_msg->GetDataRef()->GetDataVec()[0]);
        // std::cout<<"get response success, ret = "<< static_cast<int>(ret) <<std::endl;
        // std::cout<<"response->meta_res_data[0] = "<< static_cast<int>(response->meta_res_data[0]) <<std::endl;
        print_MagnaParamCSRes(*response,response_old);
        response_old =*response;

      // 处理 response
    //   auto response = static_cast<MagnaParamCSRes*>(response_msg->GetDataRef()->GetDataVec()[0]);
    //   LOG_DEBUG("get response success, gen_ts = %ld, rev = %d", response_msg->GetGenTimestamp(), response->rev);
    } 
    else if (ret == MOS::communication::COMM_CODE_CLIENT_TIMEOUT)
    {
        std::cout<<"get response failed, ret = "<< static_cast<int>(ret) <<"Client operation timed out."<<std::endl; 

    }
    else 
    {
        std::cout<<"get response failed, ret = "<< static_cast<int>(ret) <<std::endl;
    //   LOG_WARNING("get response failed, ret = %d", ret);
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  return 0;
}
    





void asyncClientThread(std::string json_file) {
    Adapter_PARAM_CS Adapter_PARAM_CS_;    

    config_client(Adapter_PARAM_CS_.domain_id, Adapter_PARAM_CS_.topic, false, json_file, Adapter_PARAM_CS_.ip,Adapter_PARAM_CS_.port);

}


int config_async_service(std::string json_file) {
    

    Adapter_PARAM_CS Adapter_PARAM_CS_;
    Adapter_PARAM_CS_.json_file = json_file;

    std::cout << "config_async_sub start with "  <<json_file<<"!!!"<< std::endl;

    // 初始化Communication中间件
    MOS::communication::Init(json_file);
    // 配置通信属性
    MOS::communication::ProtocolInfo proto_info;
    set_proto_info(proto_info, Adapter_PARAM_CS_.type, Adapter_PARAM_CS_.port, Adapter_PARAM_CS_.ip, sizeof(MagnaParamCSRes));//block_size 设置为要发出去的


    std::map<std::string, VariableVariant > variableMap = {};
    
    // 创建Service
    auto service = MOS::communication::Service::New(Adapter_PARAM_CS_.domain_id, Adapter_PARAM_CS_.topic, proto_info, TestServiceCallback);


    // std::thread inputThread(asyncClientThread,json_file);
    // std::thread inputThread2(asyncInputThreadTTY);

    while (!stop.load()) {
     std::this_thread::sleep_for(std::chrono::seconds(2));

        // print_MagnaParamCSReq(MagnaParamCSReq_, MagnaParamCSReq_old);
        // MagnaParamCSReq_old = MagnaParamCSReq_;

        // if (!inputQueue.empty())
        // {
        //     getVariableValue(variableMap,inputQueue.front());
        //     inputQueue.pop();
        // }

    }
    // inputThread.join();//阻塞当前线程（通常是主线程），直到 inputThread线程执行完成
    return 0;
}

void Adapter_PARAM_CS::run()
{
    // asyncClientThread(json_file);
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
    
}







