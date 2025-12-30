// MET_SOC_EM_ERROR_CODES.h
// version 2
//
// - topic:      "EM/ErrCode"       → 作为 EM 模块错误上报的消息主题名称（Topic)
// - protocol:   ProtocolHybrid     → 指定消息传输协议类型
// - domain:     0                  → 区分所属的域，Domain 为 0
//
/**
 * -------------------------------------------------------------------
 * ① MAX_MODULE_COUNT = 64
 *    - 系统最多支持 64 个模块（module）并行进行错误跟踪与上报。
 *    - modules[index] 对应模块 index 的当前错误快照信息。
 *
 * -------------------------------------------------------------------
 * ② EmAppError 结构体字段语义
 *
 *    struct EmAppError {
 *        uint8_t error_flag      错误标志位
 *                               - =1 → 当前模块存在错误信息
 *                               - =0 → 当前模块无错误
 *
 *        uint8_t domain_id       错误来源域 ID
 *                               - 分配策略：
 *                                   1 → EM 系统错误
 *                                   2 → 通信 COMM 组件错误
 *                                   3 → 用户自定义上报（User Report）
 *                               - 0xFF 表示无效或未知来源
 *
 *        uint8_t pipeline_id     流水线 / Pipe 路径标识
 *                               - 取值范围 0~254
 *                               - 0xFF 表示该错误不属于任何 pipeline
 *
 *        uint8_t error_id        具体错误编号
 *                               - 取值含义因 domain_id 不同而变动
 *                               - 当 domain_id=1 时为 EM 错误编码
 *                               - =0xFF 表示无效、未配置或无错误
 *    };
 *
 * -------------------------------------------------------------------
 * ③ EMErrorInfo = 当前所有模块错误快照
 *
 *    struct EMErrorInfo {
 *         uint8_t global_fault{0};
 *        EmAppError modules[MAX_MODULE_COUNT];
 *    };
 * 
 * - global_fault：整体软件故障状态标志
 * - 每个 modules[i] 保存模块 i 现存错误状态（可周期刷新 / 订阅推送）
 *
 * -------------------------------------------------------------------
 * ④ 错误编码规范 — EM_BASE_ERROR_CODE = **1**
 *
 *  EM_BASE_ERROR_CODE = 1
 *  以下 enum 为默认分配的 EM 级错误：
 *
 *      EM_ALIVE_TIMEOUT           → Liveness超时
 *      EM_DEADLINE_TIMEOUT_MIN    → Deadline最小阈值超时
 *      EM_DEADLINE_TIMEOUT_MAX    → Deadline最大阈值超时
 *
 * ============================================================
 **/

#ifndef MET_SOC_EM_ERROR_CODES_H
#define MET_SOC_EM_ERROR_CODES_H

#define MET_SOC_EM_ERROR_CODES_VERSION 2

#include <stdint.h>

#include <csignal>
#include <cstdint>

namespace mos::mgmt::infra {

// 最大模块数
constexpr size_t MAX_MODULE_COUNT = 64;
struct EmAppError {
  uint8_t error_flag{0};      // 错误标志位，1表示有错误，0表示无错误
  uint8_t domain_id{0xFF};    // 1: EM, 2: comm  3:user report
  uint8_t pipeline_id{0xFF};  // 模块所属流水线ID，255表示无效
  uint8_t error_id{0xFF};
};

// EM 错误信息集合
struct EMErrorInfo {
  uint8_t global_fault{0};               // 全局软件故障状态
  EmAppError modules[MAX_MODULE_COUNT];  // 每个模块当前错误信息
};

// Error codes for Execution Management (EM) module.
// All error codes are offset from EM_BASE_ERROR_CODE.
// Domain ID for EM errors is 1.
constexpr uint8_t EM_BASE_ERROR_CODE = 1;
enum EmErrorCode {
  EM_ALIVE_TIMEOUT = EM_BASE_ERROR_CODE,  // Alive timeout (超时)
  EM_DEADLINE_TIMEOUT_MIN,                // Deadline timeout minimum threshold
  EM_DEADLINE_TIMEOUT_MAX,                // Deadline timeout maximum threshold
};

//
//
}  // namespace mos::mgmt::infra
#endif
