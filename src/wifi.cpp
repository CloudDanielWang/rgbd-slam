#include "wifi.h"
#include "slamBase.h"

#include <string> 

using namespace std;

void wifi_comu (SLAM_DATA *slamdata)
{
/*
	static ParameterReader wifipara;
	int LOCALPORT  =   atoi( wifipara.getData( "LOCALPORT" ).c_str() );
	char  LOCALIP =    wifipara.getData( "LOCALIP" ) ;
	int REMOTEPORT  =   atoi( wifipara.getData( "REMOTEPORT" ).c_str() );
	char REMOTEIP  =   wifipara.getData( "REMOTEIP" );
*/

	struct FRAME *frame=&(slamdata->FRAME);
	struct RESULT_OF_PNP *pnpdata=&(slamdata->RESULT_OF_PNP);	

	sigset_t  mask;    //主线程中需要把所有的信号屏蔽掉  
	int signo;         //用来屏蔽信号
	sigemptyset(&mask);
	sigaddset(&mask, SIGRTMAX-6);
	pthread_sigmask(SIG_BLOCK, &mask, NULL);  //主线程中需要把所有的信号屏蔽掉  如果不屏蔽掉 就会按系统的处理函数运行
	
	struct sockaddr_in Local_Addr;
	struct sockaddr_in Remote_Addr;
	
	int Local_sock,Addr_len;

	if((Local_sock = socket(AF_INET, SOCK_DGRAM ,0))<0)
	{
		perror("socket error");
		return;
	}
	Addr_len = sizeof(struct sockaddr_in); 
    	Local_Addr.sin_family = AF_INET; //填充地址与端口的信息
	Local_Addr.sin_port = htons(LOCALPORT); //端口
	Local_Addr.sin_addr.s_addr = inet_addr(LOCALIP);//IP

	if(bind(Local_sock,(struct sockaddr *)&Local_Addr,sizeof(Local_Addr))<0)
	{
		printf("bind error\n");
		perror("wifi_bind");
		return;
	}
	Remote_Addr.sin_family = AF_INET; //填充地址与端口的信息
	Remote_Addr.sin_port = htons(REMOTEPORT); //端口
	Remote_Addr.sin_addr.s_addr = inet_addr(REMOTEIP);//IP


	//WIFI数据赋值
	ACRB_WIFI_DATA Acrb_Wifi_Data;
	while (1)
	{
		sigwait(&mask,&signo);


		Acrb_Wifi_Data.cloudMap= frame->cloudMap;
		Acrb_Wifi_Data.rvec=pnpdata->rvec;
		Acrb_Wifi_Data.tvec=pnpdata->tvec;


		if(sendto(Local_sock,(char *)(&Acrb_Wifi_Data),sizeof(Acrb_Wifi_Data),0,(struct sockaddr*)&Remote_Addr,sizeof(Remote_Addr))==-1)	
		{
				printf("wifi_send_error\n");
				perror("send");
		}
	}



//return;


	
}