#include <ros/assert.h>
#include <shared_serial/server.h>
#include <shared_serial/watchdog.h>

SerialServerLock::SerialServerLock() :
  socket_(0), last_socket_(0)
{
  pthread_mutex_init(&mutex_, NULL);
  pthread_cond_init(&condition_, NULL);
}

void SerialServerLock::checkTimeout()
{
  pthread_mutex_lock(&mutex_);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // Check timeout
  if (socket_ && (tv.tv_sec > timeout_.tv_sec ||
                  (tv.tv_sec == timeout_.tv_sec && tv.tv_usec > timeout_.tv_usec)))
  {
    ROS_DEBUG_STREAM("Lock " << socket_ << " expired");
    socket_ = 0;

    pthread_cond_signal(&condition_);
  }
  
  pthread_mutex_unlock(&mutex_);
}

int SerialServerLock::lock(int socket, float timeout)
{
  pthread_mutex_lock(&mutex_);

  // Check validity
  if (socket && socket != socket_)
  {
    ROS_ERROR_STREAM("Unknown lock " << socket);
    pthread_mutex_unlock(&mutex_);
    return -1;
  }
  
  if (!socket)
  {
    // Wait for port to become free
    while (socket_)
    {
      ROS_DEBUG_STREAM("Waiting for lock");
      pthread_cond_wait(&condition_, &mutex_);
    }
    
    // New connection
    socket_ = last_socket_ = std::max(last_socket_+1, 1);
    
    ROS_DEBUG_STREAM("New lock " << socket_);
  }

  if (timeout)
  {
    // Set timeout
    struct timeval tv;
    gettimeofday(&tv, NULL);
  
    tv.tv_sec += (int)timeout;
    tv.tv_usec += (int)((timeout-(int)timeout)*1000000);
    if (tv.tv_usec > 1000000)
    {
      tv.tv_sec++;
      tv.tv_usec -= 1000000;
    }
    timeout_ = tv;
    
    ROS_DEBUG_STREAM("Lock " << socket_ << " will expire after " << timeout << " seconds");
  }
  else
  {
    ROS_DEBUG_STREAM("Lock " << socket_ << " expired");

    timeout_.tv_sec = timeout_.tv_usec = socket_ = 0;
  }
  
  return socket_;
}

void SerialServerLock::unlock()
{
  if (!socket_)
    pthread_cond_signal(&condition_);
  pthread_mutex_unlock(&mutex_);
}

void SerialServer::advertiseServices()
{
  ROS_INFO("Registering services");

  send_topic_ = nh_.subscribe("send", 10, &SerialServer::callbackSend, this);
  close_topic_ = nh_.subscribe("close", 1, &SerialServer::callbackClose, this);
  flush_topic_ = nh_.subscribe("flush", 1, &SerialServer::callbackFlush, this);
  connect_service_ = nh_.advertiseService("connect", &SerialServer::callbackConnect, this);
  sendto_service_ = nh_.advertiseService("sendto", &SerialServer::callbackSendTo, this);
  recv_service_ = nh_.advertiseService("recv", &SerialServer::callbackRecv, this);
  sendrecv_service_ = nh_.advertiseService("sendrecv", &SerialServer::callbackSendRecv, this);
}

void SerialServer::readParameters()
{
  std::string port_name, port_type;
  LxSerial::PortType lx_port_type;
  int baud_rate;
  double watchdog_interval;
  
  ROS_INFO("Reading parameters");
  
  ROS_ASSERT(nh_.getParam("port_name", port_name));
  ROS_ASSERT(nh_.getParam("port_type", port_type));
  ROS_ASSERT(nh_.getParam("baud_rate", baud_rate));
  nh_.param<double>("watchdog_interval", watchdog_interval, 10);

  if (port_type == "RS232")
    lx_port_type = LxSerial::RS232;
  else if (port_type == "RS485_FTDI")
    lx_port_type = LxSerial::RS485_FTDI;
  else if (port_type == "RS485_EXAR")
    lx_port_type = LxSerial::RS485_EXAR;
  else if (port_type == "RS485_SMSC")
    lx_port_type = LxSerial::RS485_SMSC;
  else if (port_type == "TCP")
    lx_port_type = LxSerial::TCP;
  else
  {
    ROS_FATAL_STREAM("Unknown port type " << port_type);
    ROS_BREAK();
    return;
  }

  ROS_ASSERT(serial_port_.port_open(port_name, lx_port_type));
  ROS_ASSERT(serial_port_.set_speed_int(baud_rate));
  ROS_ASSERT(watchdog_.set(watchdog_interval));
}

void SerialServer::callbackSend(const shared_serial::Send::ConstPtr& msg)
{
  int socket = lock_.lock(msg->socket, msg->timeout);
  if (socket < 0) return;
    
  unsigned char *buf = new unsigned char[msg->data.size()];
  for (unsigned int ii=0; ii != msg->data.size(); ++ii)
    buf[ii] = msg->data[ii];
  
  int n = serial_port_.port_write(buf, msg->data.size());
  delete buf;
  
  if (n != (int)msg->data.size())
  {
    ROS_ERROR("Truncated send, flushing port");
    serial_port_.flush_buffer();
    
    lock_.unlock();
    return;
  }

  ROS_DEBUG_STREAM("Sent " << n << " bytes");
    
  lock_.unlock();
  return;
}

void SerialServer::callbackClose(const shared_serial::Close::ConstPtr& msg)
{
  int socket = lock_.lock(msg->socket, 0);
  if (socket < 0) return;

  lock_.unlock();
  return;
}

void SerialServer::callbackFlush(const shared_serial::Flush::ConstPtr& msg)
{
  int socket = lock_.lock(msg->socket, msg->timeout);
  if (socket < 0) return;
    
  serial_port_.flush_buffer();
  usleep(10);
  serial_port_.flush_buffer();

  ROS_INFO_STREAM("Flushed port");
    
  lock_.unlock();
  return;
}

bool SerialServer::callbackConnect(shared_serial::Connect::Request& req, shared_serial::Connect::Response& res)
{
  int socket = lock_.lock(0, req.timeout);
  if (socket < 0) return false;
  
  res.socket = socket;

  lock_.unlock();  
  return true;
}

bool SerialServer::callbackSendTo(shared_serial::SendTo::Request& req, shared_serial::SendTo::Response& res)
{
  int socket = lock_.lock(req.socket, req.timeout);
  if (socket < 0) return false;

  unsigned char *buf = new unsigned char[req.data.size()];
  for (unsigned int ii=0; ii != req.data.size(); ++ii)
    buf[ii] = req.data[ii];
    
  int n = serial_port_.port_write(buf, req.data.size());
  delete buf;
  
  if (n != (int)req.data.size())
  {
    ROS_ERROR("Truncated send, flushing port");
    serial_port_.flush_buffer();
    
    lock_.unlock();
    return false;
  }

  ROS_DEBUG_STREAM("Sent " << n << " bytes");
  
  res.socket = socket;

  lock_.unlock();
  return true;
}

bool SerialServer::callbackRecv(shared_serial::Recv::Request& req, shared_serial::Recv::Response& res)
{
  int socket = lock_.lock(req.socket, req.sock_timeout);
  if (socket < 0) return false;

  ROS_ASSERT(req.length <= 65536);

  unsigned char *buf = new unsigned char[req.length];
  int n = serial_port_.port_read(buf, req.length, (int)req.recv_timeout, (int)((req.recv_timeout-(int)req.recv_timeout)*1000000));
  
  if (n < 0)
  {
    ROS_ERROR_STREAM("Error " << n << " while reading serial port");
    delete buf;

    serial_port_.flush_buffer();
    usleep(10);
    serial_port_.flush_buffer();
    
    lock_.unlock();
    return false;
  }
  
  ROS_DEBUG_STREAM("Read " << n << " bytes");
  
  res.data.resize(n);
  for (int ii=0; ii != n; ++ii)
    res.data[ii] = buf[ii];
  delete buf;
  
  res.socket = socket;
    
  lock_.unlock();
  return true;
}

bool SerialServer::callbackSendRecv(shared_serial::SendRecv::Request& req, shared_serial::SendRecv::Response& res)
{
  int socket = lock_.lock(req.socket, req.sock_timeout);
  if (socket < 0) return false;

  // *** Send ***
  unsigned char *buf = new unsigned char[req.send_data.size()];
  for (unsigned int ii=0; ii != req.send_data.size(); ++ii)
    buf[ii] = req.send_data[ii];
    
  int n = serial_port_.port_write(buf, req.send_data.size());
  delete buf;
  
  if (n != (int)req.send_data.size())
  {
    ROS_ERROR("Truncated send, flushing port");
    serial_port_.flush_buffer();
    
    lock_.unlock();
    return false;
  }

  ROS_DEBUG_STREAM("Sent " << n << " bytes");

  // *** Receive ***
  ROS_ASSERT(req.length <= 65536);

  buf = new unsigned char[req.length];
  n = serial_port_.port_read(buf, req.length, (int)req.recv_timeout, (int)((req.recv_timeout-(int)req.recv_timeout)*1000000));
  
  if (n < 0)
  {
    ROS_ERROR("Error while reading serial port");
    delete buf;

    serial_port_.flush_buffer();
    usleep(10);
    serial_port_.flush_buffer();
    
    lock_.unlock();
    return false;
  }
  
  ROS_DEBUG_STREAM("Read " << n << " bytes");
  
  res.recv_data.resize(n);
  for (int ii=0; ii != n; ++ii)
    res.recv_data[ii] = buf[ii];
  delete buf;
  
  res.socket = socket;
    
  lock_.unlock();    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shared_serial_server");
  
  SerialServer serialServer;

  ROS_INFO("Spinning");
  
  ros::AsyncSpinner spinner(8);
  spinner.start();
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    serialServer.checkTimeout();
    serialServer.kickWatchdog();
    loop_rate.sleep();
  }
  
  return 0;
}
