#include <ros/ros.h>
#include <shared_serial/watchdog.h>

bool WatchdogThread::set(double interval)
{
  if (set_)
    return false;

  interval_ = interval;
  pthread_mutex_init(&mutex_, NULL);
  pthread_create(&thread_, NULL, WatchdogThread::watchDelegate, this);
  return set_ = true;
}

WatchdogThread::~WatchdogThread()
{
  if (!set_)
    return;

  pthread_cancel(thread_);
  pthread_join(thread_, NULL);
  pthread_mutex_destroy(&mutex_);
}

void *WatchdogThread::watchDelegate(void *obj)
{
  WatchdogThread *wd = static_cast<WatchdogThread*>(obj);
  wd->watch();
  return NULL;
}

void WatchdogThread::watch()
{
  kicked_ = true;

  ros::Rate loop_rate(1./interval_);
  while (ros::ok())
  {
    pthread_mutex_lock(&mutex_);
    if (!kicked_)
      bark();
    kicked_ = false;
    pthread_mutex_unlock(&mutex_);
    loop_rate.sleep();
    pthread_testcancel();
  }
}

bool WatchdogThread::kick()
{
  if (!set_)
    return false;
    
  pthread_mutex_lock(&mutex_);
  kicked_ = true;
  pthread_mutex_unlock(&mutex_);
  
  return true;
}
 
void WatchdogThread::bark()
{
  ROS_FATAL("Watchdog timed out");
  ROS_ISSUE_BREAK();
} 
