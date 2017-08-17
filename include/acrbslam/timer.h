#ifndef TIMER_H
#define TIMER_H

#include "acrbslam/common_include.h"

namespace acrbslam
{

/*定时器 应用部分*/
int acrb_timmer_create(int   mseconds,int   id,int signo);  //创建定时器
double pm_msec(void);



}//namespace acrbslam

#endif//TIMER_H


