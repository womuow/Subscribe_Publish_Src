#ifndef IPC_IPC_MSG_H_
#define IPC_IPC_MSG_H_

#include <string.h>
#ifdef __cplusplus
#include <string>
#endif

#include <cstdint>
/*
id:	id
target: bitmap of target processor.
data_size: valid data size in data[] field
timestamp: unit is microsecond, the time this message generated.
data[0]: payload of this message.
*/


typedef uint16_t uint16_t;
typedef uint64_t uint64_t;
typedef uint8_t uint8_t;

struct IPC_MSG {
	uint16_t id;
	uint16_t version;
	uint16_t data_size;
	uint16_t target;
	uint64_t timestamp;
	uint8_t data[0];
};


#define IPC_MSG_MAX_DATA_SIZE 512
#define IPC_MSG_MAX_SIZE (sizeof(struct IPC_MSG) + IPC_MSG_MAX_DATA_SIZE)
struct IPC_MSG_DATA_SIZE16 {
	struct IPC_MSG header;
	uint8_t data[16];
};

struct IPC_MSG_DATA_SIZE32 {
	struct IPC_MSG header;
	uint8_t data[32];
};

struct IPC_MSG_DATA_SIZE64 {
	struct IPC_MSG header;
	uint8_t data[64];
};

struct IPC_MSG_DATA_SIZE128 {
	struct IPC_MSG header;
	uint8_t data[128];
};

struct IPC_MSG_DATA_SIZE512 {
	struct IPC_MSG header;
	uint8_t data[512];
};

struct IPC_MSG_DATA_SIZE_MAX {
	struct IPC_MSG header;
	uint8_t data[IPC_MSG_MAX_DATA_SIZE];
};

#endif // IPC_IPC_MSG_H_