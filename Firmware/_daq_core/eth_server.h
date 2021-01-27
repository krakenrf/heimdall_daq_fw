/*
 *
 * Description: Manages the Ethernet server socket for IQ streaming
 *
 * Project: HeIMDALL DAQ Firmware
 * License: GNU GPL V3
 * Authors: Tamas Peto
 * 
 *  Copyright (C) 2018-2020  Tamás Pető
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
*/
#pragma once
#include <strings.h>
// Ethernet server libraries
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "log.h"

#define IQ_SERVER_PORT  	    5000

int iq_stream_con(int * sockets)
{
   /*
	*	Description:
	*		This function opens an Ethernet server and then accepts connections
	*		from IQ sample sinks, such as signal processing or data recorder modules.
	*		Aftef successfull connection, the server waits for the "streaming" command.
	*		When the command has arrived, the descriptor of the client is passed to 
	*		the calling process (generally a double buffered iq streaming process).
	*/ 
	int sock, connected, bytes_recieved, true_value = 1;
	socklen_t sin_size;
	char recv_data[1024];

	struct sockaddr_in server_addr,client_addr; // This structure stores the addresses
	
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) { // Create server socket
		log_fatal("Create server socket failed ");
		return(-1);
	}
	 /* Applying socket options*/
	if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&true_value,sizeof(int)) == -1) {
	    log_fatal("Setting socket options failed");
		return(-1);
	}
	// Set server address parameters
	server_addr.sin_family = AF_INET; // Address family
	server_addr.sin_port = htons(IQ_SERVER_PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY; // Set the server IP address to own
	bzero(&(server_addr.sin_zero),8);

	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
	{
		log_fatal("Unable to bind the server socket");
		return(-1);
	}
	if (listen(sock, 5) == -1)
	{
		log_fatal("Unable to start listenning");
		return(-1);
	}

	log_info("IQ Data TCP Server waiting for client on port %d",IQ_SERVER_PORT);	

    sin_size = sizeof(struct sockaddr_in);
    connected = accept(sock, (struct sockaddr *)&client_addr,&sin_size);
	log_info("New connection from (%s , %d)",  inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
	
	log_info("Waiting for client streaming request");
    bytes_recieved = recv(connected,recv_data,1024,0);
    recv_data[bytes_recieved] = '\0';

    /* User may close the socket unexpectedly */
    if(bytes_recieved == -1)
    {
    	log_error("Unexpected connection close - exiting..");
        close(connected);
        close(sock);
        return -1;
      }
	  /* Streaming start command */
      if (strcmp(recv_data , "streaming") == 0)
      {
        	log_info("Streaming request received");
        	sockets[0] = sock;
        	sockets[1] = connected;
        	return 0;
       }
       close(connected);
       close(sock);
       return -1; //Not expected signaling
}
int iq_stream_close(int* sockets)
{
	/*
	*	Closes the previously opened TCP Server and Client sockets
	*/
	close(sockets[1]);
	close(sockets[0]);
	return 0;
}








