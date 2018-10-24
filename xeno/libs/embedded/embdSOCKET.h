/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#ifndef EMBD_SOCKET_H
#define EMBD_SOCKET_H
/****************************************************************************/
#include "embdCOMMON.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/*Memory Copy*/
#include <string.h>

/* Socket */
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <net/if.h>
/****************************************************************************/
#define SCK_DOMAIN		AF_INET //IPv4 Address Family
#define SCK_TYPE		SOCK_DGRAM // UDP
#define SCK_PROTOCOL	IPPROTO_UDP // can be 0, but IPPROTO_UDP for clarity
#define DGRAM_SIZE	(1464) // optimal size for a UDP datagram in [Bytes]	
/****************************************************************************/
typedef struct embdsocket{

	char* host_ip;
	char* client_ip;
	short host_port;
	short client_port;

	struct sockaddr_in host_address;
	struct sockaddr_in client_address;

	int host_socket;
	int client_socket;

	int send_socket;
	int recv_socket;

	char send_dgram[DGRAM_SIZE];
	char recv_dgram[DGRAM_SIZE];

}SOCKET;
/****************************************************************************/
int sck_init_sockets(SOCKET* foo);
void sck_send_clear_dgrams(SOCKET* foo);
void sck_close_sockets(SOCKET* foo);
void sck_copy_to_dgram(SOCKET* foo, char* message);
char* sck_get_interface_ipadd(char* interface);

/****************************************************************************/
#endif //EMBD_SOCKET_H




