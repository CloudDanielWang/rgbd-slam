#include "acrbslam/wifi.h"
#include "acrbslam/frame.h"

namespace acrbslam
{


//wifi初始化函数
void wifi_comu::wifi_init()
{
	if((pc_sock=socket(AF_INET,SOCK_DGRAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Local_Addr.sin_family = AF_INET;
	Local_Addr.sin_port = htons(LOCALPORT);
	Local_Addr.sin_addr.s_addr = inet_addr(LOCALIP);

	if(bind(pc_sock,(struct sockaddr *)&Local_Addr,sizeof(Local_Addr))<0)
	{
		printf("bind error\n");
		perror("wifi_bind");
	}
	Remote_Addr.sin_family = AF_INET;
	Remote_Addr.sin_port = htons(REMOTEPORT);
	Remote_Addr.sin_addr.s_addr = inet_addr(REMOTEIP);
}

//wifi 发送数据函数
void wifi_comu::send_data(char *data,unsigned int num)
{
	unsigned int data_num=num;
	char *wifi_data=data;

	if(sendto(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,sizeof(Remote_Addr))==-1)
	printf("wifi_send_error\n");	
		
}








}//namespace acrbslam

