
#include "acrbslam/gps.h"

namespace acrbslam
{

GPS::GPS()
{
	string gps_com_addr=Config::get<string> ( "gps_com_addr" );
	GPS_com_addr=gps_com_addr.c_str();
}	

int GPS::gps_com_init(int fd)    //配置串口     
{
   struct termios newtio,oldtio;
   tcgetattr(fd,&oldtio);
   bzero(&newtio,sizeof(newtio));

   newtio.c_cflag |=CLOCAL|CREAD;
   /*8N1*/
   newtio.c_cflag &=~CSIZE;
   newtio.c_cflag |= CS8;

   newtio.c_cflag &=~PARENB;

   newtio.c_cflag &= ~CSTOPB;
   /*9600bit/s*/
   cfsetispeed(&newtio,B38400);
   cfsetospeed(&newtio,B38400);
   /*对接收字符和等待时间没有特别要求，刷清输入输出队列*/
   newtio.c_cc[VTIME] = 0;
   newtio.c_cc[VMIN] = 1;
   tcdrain(fd);
   tcflush(fd,TCIOFLUSH);
   /*激活配置*/
   if((tcsetattr(fd,TCSANOW,&newtio))!=0){
      perror("com set error");
      return 0;
   }   
   return 1;
}

/*
O_RDONLY      只读模式
O_WRONLY      只写模式
O_RDWR        读写模式

打开/创建文件时，至少得使用上述三个常量中的一个。以下常量是选用的：
O_APPEND       每次写操作都写入文件的末尾
O_CREAT        如果指定文件不存在，则创建这个文件
O_EXCL         如果要创建的文件已存在，则返回 -1，并且修改 errno 的值
O_TRUNC        如果文件存在，并且以只写/读写方式打开，则清空文件全部内容
O_NOCTTY       如果路径名指向终端设备，不要把这个设备用作控制终端。
O_NONBLOCK     如果路径名指向 FIFO/块文件/字符文件，则把文件的打开和后继 I/O设置为非阻塞模式（nonblocking mode）
O_NDELAY       标志通知Linux系统，这个程序不关心DCD信号线所处的状态（端口的另一端是否激活或者停止）。如果用户指定了这个标志，则进程将会一直出在睡眠状态，知道DCD信号线被激活。
以下三个常量同样是选用的，它们用于同步输入输出
O_DSYNC        等待物理 I/O 结束后再 write。在不影响读取新写入的数据的前提下，不等待文件属性更新。
O_RSYNC        read 等待所有写入同一区域的写操作完成后再进行
O_SYNC         等待物理 I/O 结束后再 write，包括更新文件属性的 I/O
*/




double GPS::calcumin(double degree)
{
	double temp = degree/10000000.0;
	double min = (temp -(int)temp)*60;	
	return min;	
}

double GPS::calcudeg(double degree)
{
	return (int)degree/10000000;		
}


void GPS::gps_comu(Data *data)
{ 

	sigset_t              mask;    //主线程中需要把所有的信号屏蔽掉  
	int signo;          //用来屏蔽信号
	sigemptyset(&mask);
	sigaddset(&mask, SIGRTMAX-5);
	pthread_sigmask(SIG_BLOCK, &mask, NULL);  //主线程中需要把所有的信号屏蔽掉  如果不屏蔽掉 就会按系统的处理函数运行

	int ttygps;
	/////////////打开串口///////////////////
	if((ttygps = open(GPS_com_addr,O_RDWR|O_NOCTTY|O_NDELAY))==-1){
		perror("打开串口失败");
		//return NULL;
	}   
	if(fcntl(ttygps,F_SETFL,0)<0)
		printf("fcntl=%d\n",fcntl(ttygps,F_SETFL,0));
	//初始化串口
	if(!gps_com_init(ttygps)){
		perror("ttygps串口初始化失败");
		//return NULL;
	} 
	
	/*float lat_deg_base = 4856.3930;
	float lng_deg_base = 12247.4841;*/
	/*double lat_base = 489398830;
	double lng_base = 1227914010;*/
	double lat_base = 391079188;//39.10623110569929
	double lng_base = 1171730089;//117.17011277139659

	double lat;
	double lng;
	char temp[96];
	char gpgga[96];

	long sec_base,nsec_base,sec_now,nsec_now;
	
	long hour_gps,min_gps,sec_gps,temp_gps;

	double time_ms_base = pm_msec();



	while(1)
	{ 
		sigwait(&mask,&signo);           //等待主线程时钟信号
			
		double time_ms_now = pm_msec();
		int time_ms_delta = (int)(time_ms_now - time_ms_base);

		int sec_delta,nsec_delta,ms_gps;
		sec_delta = time_ms_delta/1000;
		ms_gps = time_ms_delta -sec_delta*1000;
		
		hour_gps = sec_delta/3600;
		temp_gps = sec_delta%3600;
		min_gps = temp_gps/60;
		sec_gps = temp_gps%60;

		/*double lat_change = 100.0*(DspData->x_dsp);
		double lng_change = 100.0*(DspData->y_dsp);
		lat = lat_base+lat_change;
		lng = lng_base+lng_change;*/
		//float rads = fabs(lat_base)/10000000.0*0.0174532925;
		//lat = lat_base+100.0*data->x;		//垚哥程序中的100原因未知
		lat = lat_base+125.17*data->x;	//此处的倍数是通过谷歌地球GPS数据在同济大学体育场的篮球场估算可得
		//lng = (lng_base+100.0*data->y);///cos(rads);
		lng = (lng_base+89.9346*data->y);//同为估算
		
		//cout<<"x"<<'\t'<<data->x<<endl;
		//cout<<"y"<<'\t'<<data->y<<endl;
		//cout<<"z"<<'\t'<<data->z<<endl;

		cout<<"lat,deg"<<'\t'<<calcudeg(lat)<<endl;
		
		cout<<"lat,min"<<'\t'<<calcumin(lat)<<endl;
		
		cout<<"lng,deg"<<'\t'<<calcudeg(lng)<<endl;
		
		cout<<"lng,min"<<'\t'<<calcumin(lng)<<endl;
		
		cout<<"h"<<'\t'<<data->z<<endl;

		//printf("DspData->x_dsp:%f,DspData->y_dsp:%f,lat_change,%f,lat lng %f,%f,##,%.6f  %.6f\n",DspData->x_dsp,DspData->y_dsp,lat_change,lat,lng,lat/10000000.0,lng/10000000.0);
		//sprintf(temp,"GPGGA,%c%c%c%c%c%c.000,%9.4f,N,%10.4f,W,1,04,0.5,M,19.7,M,,,0000",hour[0],hour[1],min[0],min[1],sec[0],sec[1],lat,lng);
		//sprintf(temp,"GPGGA,%02ld%02ld%02ld.%03d,%02.0f%08.5f,N,%03.0f%08.5f,E,1,04,0.5,M,19.7,M,,,0000",hour_gps,min_gps,sec_gps,ms_gps,calcudeg(lat),calcumin(lat),calcudeg(lng),calcumin(lng));//未添加高度
		sprintf(temp,"GPGGA,%02ld%02ld%02ld.%03d,%02.0f%08.7f,N,%03.0f%08.7f,E,1,04,0.5,%2.5f,M,%2.5f,M,,,0000",hour_gps,min_gps,sec_gps,ms_gps,calcudeg(lat),calcumin(lat),calcudeg(lng),calcumin(lng),data->z,00.0000f);//添加了高度


		//校验位计算
		int ck = 0;
		int i = 0;
		while(temp[i]!='\0'){
			ck^=temp[i];
			i++;
		}

		sprintf(gpgga,"$%s*%2x\r\n",temp,ck);//信号编写完成
		//sprintf(gpgga,"$GPGGA,092204.999,4856.3930,S,12247.4841,W,1,04,0.5,M,19.7,M,,,0000*7A\r\n");
		int WriteNum = write(ttygps,(char *)gpgga,strlen(gpgga)); 
		//printf("%s",gpgga);
		//printf("write_num:%d\n",WriteNum);	 	  
    }                 
    close(ttygps); 
}


}//namespace acrbslam
