#ifndef IPC_TRANSPORT_H_
#define IPC_TRANSPORT_H_

#include "ipc_msg.h"

typedef void (*IPC_MSG_RECV_CALLBACK)(const IPC_MSG *msg);

/// @brief ipc send msg to mcu async
/// @param msg IPC_MSG ptr
/// @return True send successfully | False send failed
bool ipc_send_msg(const IPC_MSG *msg);

/// @brief Register to recv ipc msg callbacks based on target、id type
/// @param target target mcu id 
/// @param id  message id 
/// @param callback ipc msg callback
/// @return true/false
bool register_recv_ipc_msg(const unsigned short target,const unsigned short id,IPC_MSG_RECV_CALLBACK callback);

/// @brief UnRegister to recv ipc msg callbacks
/// @param target target mcu id 
/// @param id  message id 
/// @return true/false
bool unregister_recv_ipc_msg(const unsigned short target,const unsigned short id);

/// @brief send ipc msg to mcu sync
/// @param msg IPC_MSG ptr
/// @return response IPC_MSG ptr
// const IPC_MSG* ipc_send_msg_sync(const IPC_MSG *msg);


#endif // IPC_TRANSPORT_H_