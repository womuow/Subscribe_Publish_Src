#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>

/* Include the C++ DDS API. */
// #include "dds/dds.hpp"

///* Include data type and specific traits to be used with the C++ DDS API. */
// #include "Carla_Objects.hpp"
//#include <argparse/argparse.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <queue>
#include <string>
#include <thread>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <codecvt>
#include <locale>
#else
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <climits>
#include <cstdlib>
#include <cstring>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

#include <mos-com/utils/register.h>
#include <mos-com/utils/time_utils.h>
#include <mos-com/utils/error.h>
#include "mos-com/interface/client.hpp"
#include "mos-com/interface/getter.hpp"
#include "mos-com/interface/publisher.hpp"
#include "mos-com/interface/service.hpp"
#include "mos-com/interface/subscriber.hpp"
#include "mos-com/message/message.h"
#include "mos-com/utils/debug_log.h"
#include"MET_SOC_MAGNA_PARAM_CS.h"
#include"mos-pack/met_pack.h"
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <variant>
#include <iomanip>
#include <map>

using namespace std::chrono_literals;

class Adapter_VEHINFO
{
public:
	Adapter_VEHINFO();
	~Adapter_VEHINFO();
	void run();
	std::string json_file = "";
// private:
	// dds::sub::DataReader<Objects::MSG_Objects> topic_read_init(std::string topicName);
	std::string topicName = "/carla/obj";
	
	int domain_id = 0;
	bool type = false;
	std::string topic = "Veh/vehinfo";
	
	std::string data_in = "";
	uint32_t cycle_ms = 100;
	uint32_t port = 12352;
	std::string ip = "127.0.0.1";
	MOS::communication::ProtocolInfo proto_info;
	std::atomic_bool stop{ false };

	auto maxeye_midware_init();
	void maxeye_data_init();
	// void update_objInfo(const Objects::MSG_Objects msg, std::string& data_str);
};

// 全局变量
std::queue<std::string> inputQueue;
int flag=true;


/* Print struct FuncTgtVisnID changed value */
// void print_MET_SOC_VehInfo(MET_SOC_VehInfo& MET_SOC_VehInfo_,MET_SOC_VehInfo& MET_SOC_VehInfo_old);