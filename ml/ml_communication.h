#ifndef _SOCKET_EXPERIMENT_H
#define _SOCKET_EXPERIMENT_H

#ifdef WIN32
#include <winsock.h>
#else
#include <sys/socket.h>
#endif

#include <sys/types.h>
#include "ml_types.h"

#ifdef __cplusplus
extern "C"{
#endif

#define MAX_COMM_BUFFER_SIZE 512 // max buffer size sent between server and client

// routine which invokes remote functions (client -> server ml_* func, server -> client callback handler).
int invoke_remote_function(int socket, ml_device_handle_t dev_hdl, ml_function_id_t func_id,
			   void* args_in, void* args_out);

// functions which initializes communication with TCP 
int connect_to_server_tcp(in_addr dst_host_address, int port_num);
int accept_connection_from_client_tcp(struct sockaddr_in* destSockAddr, int portnum);

// functions which actually does the communication job with TCP
int send_buffer_tcp(const char* const buffer_send, int dstSocket);
int receive_buffer_tcp(char* buffer_received, int dstSocket);

#ifdef __cplusplus
}
#endif

#endif
