#include "acrbslam/common_include.h"

namespace acrbslam
{


class imu_com
{
	public:
		imu_com(){};
		int ttyusb0_fd;
		void com_init();
		void get_attitude(double *att1,double *att2,double *att3);
};


}//namespace acrbslam


