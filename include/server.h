#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define MYPORT "4950" // the port users will be connecting to
#define MAXBUFLEN 100

/**
 * @brief Definition of the Server class
 * 
 */
class Server
{
public:
    // Constructors and Destructors
    Server();
    ~Server();

    // Main Run Method
    void Run( void );

    // --------- Get and Set Methods

    // gTest Friendly Methods

private:
    // Private Functions
    void* get_in_addr(struct sockaddr *sa);
    int GetData(void);
    int SetupServer(void);

    //Other Variables
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
    struct sockaddr_storage their_addr;
    char buf[MAXBUFLEN];
    socklen_t addr_len;
    char s[INET6_ADDRSTRLEN];

};

#endif