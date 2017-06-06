#include "acrbslam/wifi.h"


namespace acrbslam
{


//wifi 参数读取函数
wifi_comu::wifi_comu()
:send_time(0)
{	
	SERVER_PORT    	= Config::get<int> ( "SERVER_PORT" );
	CLIENT_PORT	= Config::get<int> ( "CLIENT_PORT" );

	string  server_IP=Config::get<string> ( "SERVER_IP" );
	SERVER_IP=server_IP.c_str();
	string  client_IP=(Config::get<string> ( "CLIENT_IP" ));
	CLIENT_IP=client_IP.c_str();
}

//wifi释放函数
wifi_comu::~wifi_comu()
{

}


//wifi初始化函数
/*
{	
	if((pc_sock=socket(AF_INET,SOCK_DGRAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Local_Addr.sin_family = AF_INET;
	Local_Addr.sin_port = htons(SERVER_PORT);
	Local_Addr.sin_addr.s_addr = inet_addr(SERVER_IP);

	if(bind(pc_sock,(struct sockaddr *)&Local_Addr,sizeof(Local_Addr))<0)
	{	
		printf("bind error\n");
		perror("wifi_bind");
		return ;
	}
	Remote_Addr.sin_family = AF_INET;
	Remote_Addr.sin_port = htons(CLIENT_PORT);
	Remote_Addr.sin_addr.s_addr = inet_addr(CLIENT_IP);
}
*/
//根据CSDN 博客书写
void wifi_comu::wifi_init_uav()
{

	if((pc_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket	//注意为STREAM类型
	{
		printf("socket error\n");
	}	

	Server_Addr.sin_family = AF_INET;
	Server_Addr.sin_port = htons(SERVER_PORT);		//此处端口为地面站，即接收端的端口号，IP也是
	Server_Addr.sin_addr.s_addr = inet_addr(SERVER_IP);

	if((connect(pc_sock,(struct sockaddr *)&Server_Addr,sizeof(Server_Addr)))<0)
	
		perror("ERROR connecting");
	

}

//wif函数接收端初始化
void wifi_comu::wifi_init_pc()
{
	if((pc_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Client_Addr.sin_family = AF_INET;
	Client_Addr.sin_port = htons(SERVER_PORT);
	Client_Addr.sin_addr.s_addr = inet_addr(SERVER_IP);

	if(bind(pc_sock,(struct sockaddr *)&Client_Addr,sizeof(Client_Addr))<0)		//此处为本地IP与端口
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

	socklen_t addr_len=sizeof(Server_Addr);


	cout<<"Wait.."<<endl;
	do
	{
		uav_sock=accept(pc_sock,(struct sockaddr*)&Server_Addr,&addr_len);
	}while(uav_sock<0);


}



//
//wifi 发送数据函数
/*
void wifi_comu::send_data(char *data,unsigned int num)
{
	unsigned int data_num=num;
	char *wifi_data=data;

	if(sendto(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,sizeof(Remote_Addr))==-1)
	printf("wifi_send_error\n");	
		
}
*/
//wifi发送新函数
void wifi_comu::send_data_new(Mat frame)
{
	if((send(pc_sock,frame.data, frame.total()*frame.elemSize(),0))==-1)
	{
		printf("wifi_send_error\n");
	}
		
	
}

//

//wifi 接收数据函数
/*
int wifi_comu::receive_data(char *data, long unsigned int num)
{
	long unsigned int data_num=num;
	int temp_data;
	char *wifi_data=data;
	socklen_t addr_len=sizeof(Remote_Addr);

	temp_data=recvfrom(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,&addr_len);
	return temp_data;
}
*/

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

