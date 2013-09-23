#include <ros/ros.h>
#include <shared_serial/client.h>

#include <shared_serial/Connect.h>
#include <shared_serial/Send.h>
#include <shared_serial/SendTo.h>
#include <shared_serial/Recv.h>
#include <shared_serial/SendRecv.h>
#include <shared_serial/Close.h>
#include <shared_serial/Flush.h>

void SerialClient::init(const char *path)
{
  nh_ = ros::NodeHandle(path);

  send_topic_ = nh_.advertise<shared_serial::Send>("send", 10);
  close_topic_ = nh_.advertise<shared_serial::Close>("close", 1);
  flush_topic_ = nh_.advertise<shared_serial::Flush>("flush", 1);

  connect_service_ = nh_.serviceClient<shared_serial::Connect>("connect", true);
  sendto_service_ = nh_.serviceClient<shared_serial::SendTo>("sendto", true);
  recv_service_ = nh_.serviceClient<shared_serial::Recv>("recv", true);
  sendrecv_service_ = nh_.serviceClient<shared_serial::SendRecv>("sendrecv", true);

  connect_service_.waitForExistence();
  sendto_service_.waitForExistence();
  recv_service_.waitForExistence();
  sendrecv_service_.waitForExistence();
}

int SerialClient::connect(float timeout)
{
  shared_serial::Connect srv;

  if (!connect_service_.call(srv))
    return -1;

  return srv.response.socket;
}

int SerialClient::sendto(int socket, const unsigned char *data, size_t length, float timeout)
{
  shared_serial::SendTo srv;

  srv.request.socket = socket;
  srv.request.data.resize(length);
  for (size_t ii=0; ii < length; ++ii)
    srv.request.data[ii] = data[ii];
  srv.request.timeout = timeout;

  if (!sendto_service_.call(srv))
    return -1;

  return srv.response.socket;
}

int SerialClient::recv(int socket, int length, float recv_timeout, float sock_timeout, unsigned char *data, size_t *data_length)
{
  shared_serial::Recv srv;

  srv.request.socket = socket;
  srv.request.length = length;
  srv.request.recv_timeout = recv_timeout;
  srv.request.sock_timeout = sock_timeout;

  if (!recv_service_.call(srv))
    return -1;

  *data_length = srv.response.data.size();
  for (size_t ii=0; ii < *data_length; ++ii)
    data[ii] = srv.response.data[ii];

  return srv.response.socket;
}

int SerialClient::sendrecv(int socket, const unsigned char *send_data, size_t send_length, size_t recv_length, float recv_timeout, float sock_timeout, unsigned char *recv_data, size_t *recv_data_length)
{
  shared_serial::SendRecv srv;

  srv.request.socket = socket;
  srv.request.send_data.resize(send_length);
  for (size_t ii=0; ii < send_length; ++ii)
    srv.request.send_data[ii] = send_data[ii];
  srv.request.length = recv_length;
  srv.request.recv_timeout = recv_timeout;
  srv.request.sock_timeout = sock_timeout;

  if (!sendrecv_service_.call(srv))
    return -1;

  *recv_data_length = srv.response.recv_data.size();
  for (size_t ii=0; ii < *recv_data_length; ++ii)
    recv_data[ii] = srv.response.recv_data[ii];

  return srv.response.socket;
}

void SerialClient::send(int socket, const unsigned char *data, size_t length, float timeout)
{
  shared_serial::Send msg;

  msg.socket = socket;
  msg.data.resize(length);
  for (size_t ii=0; ii < length; ++ii)
    msg.data[ii] = data[ii];
  msg.timeout = timeout;

  send_topic_.publish(msg);
}

void SerialClient::close(int socket)
{
  shared_serial::Close msg;

  msg.socket = socket;

  close_topic_.publish(msg);
}

void SerialClient::flush(int socket, float timeout)
{
  shared_serial::Flush msg;

  msg.socket = socket;
  msg.timeout = timeout;

  flush_topic_.publish(msg);
}
