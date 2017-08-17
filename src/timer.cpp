#include "acrbslam/timer.h"

namespace acrbslam
{

  int acrb_timmer_create(int   mseconds,int   id,int signo)  //创建定时器
{
    timer_t   tid;
    struct   sigevent   se;
    struct   itimerspec   ts,   ots;
    memset   (&se,   0,   sizeof   (se));
    se.sigev_notify   =   SIGEV_SIGNAL;
    se.sigev_signo = signo;
    se.sigev_value.sival_int   =   id;  //用于区分定时器
    if(timer_create(CLOCK_REALTIME,   &se,   &tid)<0)
    {
      perror( "timer_creat_error ");
      return   -1;
    }
    ts.it_value.tv_sec   =   1;
    ts.it_value.tv_nsec   =   0;
    ts.it_interval.tv_sec   =   0;
    ts.it_interval.tv_nsec   =   mseconds*1000000;
    if(timer_settime(tid,TIMER_ABSTIME,&ts,&ots)   <   0)
    {
      perror   ( "timer_settime ");
      return   -1;
    }
    puts( "timer_create   successfully. ");
    return   0;
} 
/*
CLOCK_REALTIME:系统实时时间,随系统实时时间改变而改变,即从UTC1970-1-1 0:0:0开始计时,中间时刻如果系统时间被用户该成其他,则对应的时间相应改变
CLOCK_MONOTONIC:从系统启动这一刻起开始计时,不受系统时间被用户改变的影响
CLOCK_PROCESS_CPUTIME_ID:本进程到当前代码系统CPU花费的时间
CLOCK_THREAD_CPUTIME_ID:本线程到当前代码系统CPU花费的时间
*/
double pm_msec(void)      
{
  timespec tp;
  double time;  
  clock_gettime(CLOCK_REALTIME, &tp);
  time = tp.tv_sec*1000.0 + ((double)tp.tv_nsec)/1000000.0;
  return time;
}
double pm_msec_CPU(void)      
{
  timespec tp;
  double time;  
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tp);
  time = tp.tv_sec*1000.0 + ((double)tp.tv_nsec)/1000000.0;
  return time;
}


}//namespace acrbslam

