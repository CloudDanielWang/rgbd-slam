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

	if((server_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket	//注意为STREAM类型
	{
		printf("socket error\n");
	}	

	Server_Addr.sin_family = AF_INET;
	Server_Addr.sin_port =htons(SERVER_PORT);		//此处端口为地面站，即接收端的端口号，IP也是
	Server_Addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	SERVER_LEN=sizeof(Server_Addr);

	if((connect(server_sock,(struct sockaddr *)&Server_Addr,SERVER_LEN))<0)
	{
		perror("ERROR connecting");
		close(server_sock);
		exit(1);
	}
	
		

	return;
	

}

//wif函数接收端初始化
void wifi_comu::wifi_init_pc()
{
	if((server_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Server_Addr.sin_family = AF_INET;
	Server_Addr.sin_port = htons(SERVER_PORT);
	Server_Addr.sin_addr.s_addr =htonl(INADDR_ANY);// inet_addr(CLIENT_IP);
	SERVER_LEN=sizeof(Server_Addr);

	if(bind(server_sock,(struct sockaddr *)&Server_Addr,SERVER_LEN)<0)		//此处为本地IP与端口
	{	
		printf("bind error\n");
		perror("wifi_bind");
		return ;
	}	

	if(listen(server_sock,5)==-1)
	{
		printf("listen error\n");
		return;
	}

	socklen_t addr_len=sizeof(Client_Addr);


	cout<<"Wait.."<<endl;
	do
	{
		client_sock=accept(server_sock,(struct sockaddr*)&Client_Addr,&addr_len);
	}while(client_sock<0);

	return;
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
	int imgSize = frame.total()*frame.elemSize();

	if((write(server_sock,frame.data, imgSize))==-1)
	{
		printf("wifi_send_error\n");
	}
	//cout<<"send finishied"<<endl;
	return;
		
}

//


//
void wifi_comu::send_data_client_writev(Mat RGBframe, Mat Depthframe)
{
	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	//cout<<"rgbimgSize"<<rgbimgSize<<endl;
	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	//cout<<"depthSize"<<depthSize<<endl;


	 struct iovec frame_data[2];
           	frame_data[0].iov_base=RGBframe.data;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=Depthframe.data;
           	frame_data[1].iov_len=depthSize;

	if((writev(server_sock,frame_data, 2))==-1)
	{
		printf("wifi_send_error\n");
	}

	return;
		
}

//



//wifi server端接收新函数
cv::Mat  wifi_comu::receive_data_pc(Mat frame)
{
	const int imgSize = frame.total()*frame.elemSize();
	 uchar sockData[imgSize];

	int bytes=0;

	for (int i=0; i<imgSize; i+=bytes)
	{
		if((bytes=recv(client_sock,sockData+i, imgSize-i, 0 ))==-1)
		{
			cout<<"Fault"<<endl;
			close(client_sock);
			exit(-1);

		}
	}
	Mat temp_mat(Size(640,480), CV_8UC3, sockData);
	
	return temp_mat;
}



cv::Mat  wifi_comu::receive_data_server_readv(Mat RGBframe, Mat Depthframe)
{
	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	 uchar RGBData[rgbimgSize];

	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	uchar DepthData[depthSize];

	 struct iovec frame_data[2];
           	frame_data[0].iov_base=RGBData;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=DepthData;
           	frame_data[1].iov_len=depthSize;

           	int bytes=0;


	if((bytes=readv(client_sock,frame_data,2 ))==-1)
	{
		cout<<"Fault"<<endl;
		close(client_sock);
		exit(-1);

	}

	Mat temp_mat(Size(640,480), CV_8UC3, RGBData);
	//Mat temp_depth_mat(Size(640,480), CV_16UC1, DepthData);
	return temp_mat;
	//return temp_depth_mat;
}





}//namespace acrbslam

