#include "acrbslam/common_include.h"
#include "acrbslam/config.h"

namespace acrbslam
{


class imu_com
{
	public:
		imu_com();
		int ttyimu;
		const char* IMU_com_addr;	//imu输入串口文件位置


		void imu_com_init();
		void get_attitude(double *att1,double *att2,double *att3);
};


}//namespace acrbslam


