#ifndef UDP_UTILS_H_
#define UDP_UTILS_H_

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

namespace udp_utils
{
  class UDPSenderSocket
  {
  public:
  	UDPSenderSocket() : sockfd(0)
  	{
  		slen = sizeof(si_server);
  	}

    UDPSenderSocket(const char ip[], const char port[]) : UDPSenderSocket()
  	{
  		Configure(ip, port);
  	}


    void Configure(const char ip[], const char port[])
  	{
    	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    		perror("UDPSenderSocket - socket creation failed");
    		exit(1);
    	}

  		uint16_t serverPort = static_cast<uint16_t>(stod(std::string(port)));
  		ConfigureSenderSocket(si_server, ip, serverPort);
  	}

  	template<typename TP>
  	int Send(TP dataToSend, std::size_t dataSize)
  	{
  		return sendto(sockfd, dataToSend, dataSize, 0, (struct sockaddr *) &si_server, slen);
  	}

  private:

    bool ConfigureSenderSocket(struct sockaddr_in &si_out, const char *ip, const uint16_t port)
  	{

  		// zero out the structure
  		memset((char *) &si_out, 0, sizeof(si_out));
  		si_out.sin_family = AF_INET;
  		si_out.sin_port = htons(port);

  		if (inet_aton(ip, &si_out.sin_addr) == 0) {
  			fprintf(stderr, "inet_aton() failed\n");
  			exit(1);
  		}

  		return true;
  	}

  	struct sockaddr_in si_server;
  	int sockfd;
  	socklen_t slen;
  };


  //------------------------------------------------------------------------------

  class UDPReceiverSocket
  {
  public:

  	UDPReceiverSocket() : sockfd_(0)
  	{
  		slen_ = sizeof(si_client_);
  	}

    UDPReceiverSocket(const char port[], const bool nonBlocking) : UDPReceiverSocket()
  	{
  		Configure(port, nonBlocking);
  	}

    void Configure(const char port[], const bool nonBlocking)
  	{
  		uint16_t port_ = static_cast<uint16_t>(stod(std::string(port)));
  		ConfigureReceiverSocket(sockfd_, si_client_, port_, nonBlocking);
  	}

  	template<typename TP>
  	int Receive(TP data, std::size_t dataSize)
  	{
  		return recvfrom(sockfd_, data, dataSize, 0, (struct sockaddr *) &si_client_, &slen_);

  	}

  private:

  	bool ConfigureReceiverSocket(int &sockfd, struct sockaddr_in &si_in, uint16_t port, bool nonBlocking)
  	{
  		///
  		/// SOCKET Initialization
  		///
  		if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
  			perror("socket");
  			exit(1);
  		}

  		// zero out the structure
  		memset((char *) &si_in, 0, sizeof(si_in));
  		si_in.sin_family = AF_INET;
  		si_in.sin_port = htons(port);
  		si_in.sin_addr.s_addr = htonl(INADDR_ANY);

  		//bind socket to port
  		if (bind(sockfd, (struct sockaddr*) &si_in, sizeof(si_in)) == -1) {
  			perror("bind error");
  			exit(1);
  		}

  		if (nonBlocking) {
  			// Put the socket in non-blocking mode:
  			if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL) | O_NONBLOCK) < 0) {
  				perror("Error in putting the socket in non blocking mode");
  				exit(1);
  			}
  		}

  		return true;
  	}

  	struct sockaddr_in si_client_;
  	int sockfd_;
  	socklen_t slen_;			// = sizeof(si_tool);
  };

}

#endif /* UDP_UTILS_H_ */
