/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#include <embdSOCKET.h>
/****************************************************************************/
int sck_init_sockets(SOCKET* foo)
{
	printf("\nOpening Socket for UDP Communication...");

	if((foo->host_socket = socket(SCK_DOMAIN, SCK_TYPE, SCK_PROTOCOL))<0){
		fprintf(stderr, "Unable to Open UDP Socket for sending!\n");
		return _EMBD_RET_ERR_;
	}

	foo->host_address.sin_family = SCK_DOMAIN;
	foo->host_address.sin_port = htons(foo->host_port);
	foo->host_address.sin_addr.s_addr = inet_addr(foo->host_ip);

	if((foo->client_socket = socket(SCK_DOMAIN, SCK_TYPE, SCK_PROTOCOL))<0){
		fprintf(stderr, "Unable to Open UDP Socket for recieving!\n");
		return _EMBD_RET_ERR_;
	}

	foo->client_address.sin_family = SCK_DOMAIN;
	foo->client_address.sin_port = htons(foo->client_port);
	foo->client_address.sin_addr.s_addr = inet_addr(foo->client_ip);


	if((foo->recv_socket = bind(foo->host_socket,(struct sockaddr *)&foo->host_address,sizeof(foo->host_address)))<0){
		fprintf(stderr, "Unable to bind sockets!\n");
		return _EMBD_RET_ERR_;
	}
	printf("OK\n");
	
	return _EMBD_RET_SCC_;

}
/****************************************************************************/
void sck_send_clear_dgrams(SOCKET* foo)
{
	int i;

	foo->send_socket = sendto(foo->client_socket,foo->send_dgram,DGRAM_SIZE,0,
		(struct sockaddr *)&foo->client_address,sizeof(foo->client_address));

	for(i=0; i<DGRAM_SIZE; ++i)
		foo->send_dgram[i] = 0;

}
/****************************************************************************/
void sck_close_sockets(SOCKET* foo)
{
	printf("Closing UDP sockets...");
	close(foo->host_socket);
	close(foo->client_socket);
	close(foo->send_socket);
	close(foo->recv_socket);
	printf("OK!\n");

}
/****************************************************************************/
void sck_copy_to_dgram(SOCKET* foo, char* message)
{
	memcpy(foo->send_dgram, message, sizeof(message));
}
/****************************************************************************/
char* sck_get_interface_ipadd(char* interface)
{
	int fd;
	struct ifreq ifrq;

	fd = socket(SCK_DOMAIN, SCK_TYPE, 0);
	ifrq.ifr_addr.sa_family = SCK_DOMAIN; // IPv4 
	strncpy(ifrq.ifr_name, interface, IFNAMSIZ-1);
	ioctl(fd, SIOCGIFADDR, &ifrq);
	close(fd);

	return inet_ntoa(((struct sockaddr_in *)&ifrq.ifr_addr)->sin_addr);
}
/****************************************************************************/