#ifndef __SS_SERVER_H_
#define __SS_SERVER_H

#include <stdint.h>
#include <sys/time.h>
#include <pthread.h>

#include <ros/ros.h>
#include <shared_serial/LxSerial.h>
#include <shared_serial/watchdog.h>

#include <shared_serial/Connect.h>
#include <shared_serial/Send.h>
#include <shared_serial/SendTo.h>
#include <shared_serial/Recv.h>
#include <shared_serial/SendRecv.h>
#include <shared_serial/Close.h>
#include <shared_serial/Flush.h>

class SerialServerLock
{
  protected:
    int socket_, last_socket_;
    struct timeval timeout_;
    pthread_mutex_t mutex_;
    pthread_cond_t condition_;
    
  public:
    SerialServerLock();
    
    int lock(int socket, float timeout);
    void unlock();
    void checkTimeout();
};

class SerialServer
{
  protected:
    ros::NodeHandle nh_;
    ros::ServiceServer connect_service_;
    ros::ServiceServer sendto_service_;
    ros::ServiceServer recv_service_;
    ros::ServiceServer sendrecv_service_;
    ros::Subscriber    send_topic_;
    ros::Subscriber    close_topic_;
    ros::Subscriber    flush_topic_;
    
    LxSerial serial_port_;
    SerialServerLock lock_;
    WatchdogThread watchdog_;

  public:
    SerialServer() : nh_("~")
    {
      advertiseServices();
      readParameters();
    }
    ~SerialServer()
    {
      nh_.shutdown();
    }
    
    void checkTimeout()
    {
      lock_.checkTimeout();
    }
    
    void kickWatchdog()
    {
      watchdog_.kick();
    }
    
    void advertiseServices();
    void readParameters();
    
    void callbackSend(const shared_serial::Send::ConstPtr& msg);
    void callbackClose(const shared_serial::Close::ConstPtr& msg);
    void callbackFlush(const shared_serial::Flush::ConstPtr& msg);
    bool callbackConnect(shared_serial::Connect::Request& req, shared_serial::Connect::Response& res);
    bool callbackSendTo(shared_serial::SendTo::Request& req, shared_serial::SendTo::Response& res);
    bool callbackRecv(shared_serial::Recv::Request& req, shared_serial::Recv::Response& res);
    bool callbackSendRecv(shared_serial::SendRecv::Request& req, shared_serial::SendRecv::Response& res);
};

#endif /* __SS_SERVER_H */
