#ifndef __SS_WATCHDOG_H
#define __SS_WATCHDOG_H

#include <pthread.h>

class WatchdogThread
{
  protected:
    pthread_t thread_;
    pthread_mutex_t mutex_;
    double interval_;
    bool kicked_, set_;
    
  public:
    WatchdogThread() : set_(false) { }
    WatchdogThread(double interval) : set_(false) { set(interval); }
    virtual ~WatchdogThread();
    
    bool set(double interval);
    bool kick();
    
  protected:
    virtual void watch();
    virtual void bark();
    
  private:
    static void *watchDelegate(void *obj);
};

#endif /* __SS_WATCHDOG_H */
