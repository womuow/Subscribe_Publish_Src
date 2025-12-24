#include <iostream>
#include <chrono>
#include <thread>
#include "IPC_Pub.h"
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
#include <mos-com/message/message.h>
#include "mos-com/utils/debug_log.h"
using namespace std::chrono_literals;
// using namespace org::eclipse::cyclonedds;
class AdApter_RoadPathPUB
{
public:
	AdApter_RoadPathPUB();
	~AdApter_RoadPathPUB();	void run();
	std::string json_file = "";
	// dds::sub::DataReader<RoadPathPUBModule::RoadPathPUB> topic_read_init(std::string topicName);
	// RoadPathPUBModule::RoadPathPUB topic_out;
	//std::string topicName = "rte2dds/RoadPathPUB";
	int domiain_id = 0;
	bool type = false;
	std::string topic = "PPF/RoadPath";
	std::string data_in = "";
	uint32_t cycle_ms = 25;
	uint32_t port = 12359;
	std::string ip = "127.0.0.1";
	MOS::communication::ProtocolInfo proto_info;
	std::atomic_bool stop{ false };
	auto maxeye_midware_init();
	void maxeye_data_init();
	// void update_objInfo(const RoadPathPUBModule::RoadPathPUB msg);
private:
};



void setIntialValue_RoadPath(RoadPath& RoadPath_);