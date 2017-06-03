#include "acrbslam/wifi.h"


namespace acrbslam
{


//wifi 参数读取函数
wifi_comu::wifi_comu()
:send_time(0)
{	
	LOCALPORT    	= Config::get<int> ( "LOCALPORT" );
	REMOTEPORT	= Config::get<int> ( "REMOTEPORT" );

	string  localIP=Config::get<string> ( "LOCALIP" );
	LOCALIP=localIP.c_str();
	string  remoteIP=(Config::get<string> ( "REMOTEIP" ));
	REMOTEIP=remoteIP.c_str();
}

//wifi释放函数
wifi_comu::~wifi_comu()
{

}


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
		return ;
	}
	Remote_Addr.sin_family = AF_INET;
	Remote_Addr.sin_port = htons(REMOTEPORT);
	Remote_Addr.sin_addr.s_addr = inet_addr(REMOTEIP);
}

//根据CSDN 博客书写
void wifi_comu::wifi_init_uav()
{
/*
	if (pc_sock=socket(AF_INET,SOCK_STREAM,0)<0)
	{
		printf("socket error\n");
	}
	server = gethostbyname(LOCALIP);
	if (server==NULL)
	{
		fprintf(stderr,"ERROR,no such host\n");
		exit(0);
	}
	bzero((char*)&Local_Addr,sizeof(Local_Addr));
	Local_Addr.sin_family = AF_INET;
	bcopy((char*)server->h_addr, (char *)&Local_Addr.sin_addr.s_addr,server->h_length);
	Local_Addr.sin_port = htons(LOCALPORT);
*/
	if((pc_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Local_Addr.sin_family = AF_INET;
	Local_Addr.sin_port = htons(LOCALPORT);
	Local_Addr.sin_addr.s_addr = inet_addr(LOCALIP);

	if((connect(pc_sock,(struct sockaddr *)&Local_Addr,sizeof(Local_Addr)))<0)
	
		perror("ERROR connecting");
	

}

//wif函数接收端初始化
void wifi_comu::wifi_init_pc()
{
	if (pc_sock=socket(AF_INET,SOCK_STREAM,0)<0)
	{
		printf("socket error\n");
	}

	Local_Addr.sin_family = AF_INET;
	Local_Addr.sin_port = htons(LOCALPORT);
	Local_Addr.sin_addr.s_addr = inet_addr(LOCALIP);

	if(bind(pc_sock, (struct sockaddr *)&Local_Addr,sizeof(Local_Addr))<0)
	{	
		printf("bind error\n");
		perror("wifi_bind");
		return ;
	}	

	if(listen(pc_sock,5)==-1)
	{
		printf("listen error\n");
		return;
	}

	socklen_t addr_len=sizeof(Remote_Addr);


	cout<<"Wait.."<<endl;
	do
	{
		uav_sock=accept(uav_sock,(struct sockaddr*)&Remote_Addr,&addr_len);
	}while(uav_sock<0);


}



//
//wifi 发送数据函数
void wifi_comu::send_data(char *data,unsigned int num)
{
	unsigned int data_num=num;
	char *wifi_data=data;

	if(sendto(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,sizeof(Remote_Addr))==-1)
	printf("wifi_send_error\n");	
		
}

//wifi发送新函数
void wifi_comu::send_data_new(Mat frame)
{
	if(send(pc_sock,frame.data, frame.total()*frame.elemSize(),0)==-1);
	printf("wifi_send_error\n");
}

//

//wifi 接收数据函数
int wifi_comu::receive_data(char *data, long unsigned int num)
{
	long unsigned int data_num=num;
	int temp_data;
	char *wifi_data=data;
	socklen_t addr_len=sizeof(Remote_Addr);

	temp_data=recvfrom(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,&addr_len);
	return temp_data;
}

//wifi pc端接收新函数
void wifi_comu::receive_data_pc(Mat frame)
{
	char receive_data[307200];
	int bytes=0;

	for (int i=0; i<307200; i+=bytes)
	{
		if((bytes=recv(uav_sock,receive_data+i, 307200-i, 0 ))==-1)
		{
			cout<<"Fault"<<endl;
			exit(-1);

		}
	}
	Mat temp_mat(Size(640,480), CV_8UC1, receive_data);
	frame=temp_mat;
	return;
}





}//namespace acrbslam

