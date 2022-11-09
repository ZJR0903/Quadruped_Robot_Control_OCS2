#pragma once
#include <sys/select.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <time.h>
#include <string>
 
using namespace std;

namespace udp{
	class Udp
	{
	private:
		static Udp m_Instance;
		int  m_sockClient;
		struct sockaddr_in *addrSrv; 
		socklen_t  addr_len;
	public:
		Udp(){}
		~Udp()
		{
			Close();
		}
		static Udp& GetInstance()
		{
			return m_Instance;
		}
		void Init(const char* ip, int port)
		{
			m_sockClient = socket(AF_INET, SOCK_DGRAM, 0);
			addr_len=sizeof(struct sockaddr_in);
			addrSrv = (struct sockaddr_in *)malloc(addr_len);
			memset(addrSrv, 0, addr_len);
			addrSrv->sin_addr.s_addr = inet_addr(ip);//IP地址
			addrSrv->sin_family = AF_INET;
			addrSrv->sin_port = htons(port);//端口号
			cout<<"bind"<<ip<<endl;
			bind(m_sockClient, (struct sockaddr*)addrSrv, addr_len);
		}
		void Close()
		{
			close(m_sockClient);
			free(addrSrv);
			addrSrv = NULL;
		}
		void Sendmsg(char * buff)
		{
			sendto(m_sockClient, buff, 6*4*6, 0, (struct sockaddr*)addrSrv, addr_len);
		}
		void Recvmsg(char* buff,int len)
		{
			recvfrom(m_sockClient, buff, len, 0, (struct sockaddr*)addrSrv,&addr_len);
		}
	};
}
 