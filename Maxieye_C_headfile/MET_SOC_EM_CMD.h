// MET_SOC_EM_CMD.h
// version 2
//  C/S架构下，Client端向 EM 模块发送恢复命令的消息定义
//
// - topic:      "MCU/RecoveryCommand" → 作为 Client 
// 向EM发送恢复命令的消息主题名称（Topic)
// - protocol:   ProtocolShm           → 指定消息传输协议类型
// - domain:     0                     → 区分所属的域，Domain 为 0
//
//-----------------------------------------------------------------------------
// ① RecoveryAction 枚举定义了 MCU 可以发送的恢复操作类型
//    - kNone: 无操作
//    - kRebootProcess: 重启指定进程
//    - kKillProcess: 杀死指定进程
//    - kPowerOff: 上电
//    - kPowerOn:  下电
//
//// ② RecoveryCommand 结构体用于描述 client端 向 EM 发送的恢复命令
//    - process_name: 需要控制的进程名称，上下电时可留空
//    - action: 要执行的恢复操作
//    - error_code: 关联的错误代码，上下电时可设为0
//// ③ RecoveryResponse 结构体用于 EM 向 client 端返回恢复操作的结果
//    - success: 操作是否成功
//-----------------------------------------------------------------------------

#ifndef MET_SOC_EM_CMD_H
#define MET_SOC_EM_CMD_H

#include <cstdint>

namespace mos::mgmt::infra {
#define MET_SOC_EM_CMD_VERSION 2

enum class RecoveryAction : uint8_t {
  kNone = 0,
  kRebootProcess,
  kKillProcess,
  kPowerOff,
  kPowerOn
};

struct RecoveryCommand {
  char process_name[32]{};  // Name of the process to control 下电时可留空
  RecoveryAction action{};  // What action to take
  uint32_t
      error_code{};  // The associated error code reported earlier 下电可设为0
};

struct RecoveryResponse {
  bool success{};
};

}  // namespace mos::mgmt::infra

#endif