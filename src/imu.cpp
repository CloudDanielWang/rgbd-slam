#include "acrbslam/imu.h"

namespace acrbslam
{

void imu_com::com_init()
{
	if((ttyusb0_fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY))==-1)
	{
		perror("打开串口失败");
	}

	if(fcntl(ttyusb0_fd,F_SETFL,0)<0)
		printf("fcntl error");

	struct termios newtio;
	struct termios oldtio;
	tcgetattr(ttyusb0_fd,&oldtio);
	bzero(&newtio,sizeof(newtio));

	newtio.c_cflag |=CLOCAL|CREAD;
	newtio.c_cflag &=~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &=~PARENB;
	newtio.c_cflag &= ~CSTOPB;
   
	cfsetispeed(&newtio,B115200);
	cfsetospeed(&newtio,B115200);
  
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;
	tcdrain(ttyusb0_fd);
	tcflush(ttyusb0_fd,TCIOFLUSH);
  
	if((tcsetattr(ttyusb0_fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
	} 	
}

void imu_com::get_attitude(double *att1,double *att2,double *att3)
{
	char imu_store;
	char *pos;
	int nread,len;
	float imu_roll,imu_pitch,imu_yaw;
	len=0;

	while(1)
	{
		nread=read(ttyusb0_fd,&imu_store,1);	
		if(imu_store!='s')
		{
			continue;
		}     

		pos = new char[100]; 
		while(1)
			{
				nread=read(ttyusb0_fd,&imu_store,1);
				if(imu_store=='t')
				{
					printf("end\n");
					break;
				}
				*(pos+len)=imu_store;
				len++;
			}
		*(pos+len)='\0';
		len=0;
		sscanf(pos,"%f %f %f",&imu_roll,&imu_pitch,&imu_yaw);
		printf("%f %f %f \n",imu_roll*180.0/3.1415,imu_pitch*180.0/3.1415,imu_yaw*180.0/3.1415);
		*att1=(double)imu_roll;
		*att2=(double)imu_pitch;
		*att3=(double)imu_yaw;
		delete []pos;
	}
	close(ttyusb0_fd);
	
}



}//namespace acrbslam


