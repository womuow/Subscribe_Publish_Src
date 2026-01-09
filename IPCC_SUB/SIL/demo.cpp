#include "mos-com/message/message.h"
#include "mos-com/interface/subscriber.hpp"
#include "mos-com/interface/publisher.hpp"
#include "mos-com/utils/register.h"
#include "ipc_msg.h"
 
void main()
{
    MOS::communication::ProtocolInfo protoc_info;
    protoc_info.protocol_type = MOS::communication::kProtocolHybrid;
    protoc_info.shm_info.block_count = 256;
    protoc_info.shm_info.block_size = 1024*6;
    protoc_info.shm_info.fast_mode = false;
    pub_ipc_ = MOS::communication::Publisher::New(0, "ipc_services/msg/subscriber", protoc_info);

    auto pub_msg = std::make_shared<MOS::message::Message>();
    const auto now_time = MOS::TimeUtils::NowNsec();
    const auto data_ref = std::make_shared<MOS::message::DataRef>(msg.get(), sizeof(IPC_MSG) + msg->data_size);
    pub_msg->SetDataRef(data_ref);
    pub_msg->SetGenTimestamp(now_time);
    pub_ipc_->Pub(pub_msg);
}