#ifndef __SS_CLIENT_H_
#define __SS_CLIENT_H_

#include <ros/ros.h>

/// shared_serial client interface class.
class SerialClient
{
  protected:
    ros::NodeHandle nh_;                  ///< ROS node handle.
    ros::ServiceClient connect_service_;  ///< Service client for connect function.
    ros::ServiceClient sendto_service_;   ///< Service client for sendto function.
    ros::ServiceClient recv_service_;     ///< Service client for recv function.
    ros::ServiceClient sendrecv_service_; ///< Service client for sendrecv function.
    ros::Publisher     send_topic_;       ///< Publisher for send function.
    ros::Publisher     close_topic_;      ///< Published for close function.
    ros::Publisher     flush_topic_;      ///< Publisher for flush function.

  public:
    /// Initialize interface.
    /** \param path Path to server node. */
    void init(const char *path="/comm");

    /// Lock serial port.
    /**
     * \param timeout Lock time in [s]
     * \returns Socket identifier.
     */
    int connect(float timeout);
    
    /// Send data over serial port.
    /**
     * \param socket Socket identifier (when 0, the port is locked first).
     * \param data Data to send.
     * \param length Length of \c data parameter.
     * \param timeout Lock time in [s].
     * \returns Socket identifier.
     */
    int sendto(int socket, const unsigned char *data, size_t length, float timeout);
    
    /// Receive data from serial port.
    /**
     * \param sock Socket identifier (when 0, the port is locked first).
     * \param length Length of data to receive.
     * \param recv_timeout Time to wait for data in [s].
     * \param sock_timeout Lock time in [s].
     * \param data Data buffer, at least \c length bytes long.
     * \param[out] data_length Number of bytes received.
     * \returns Socket identifier.
     */
    int recv(int socket, int length, float recv_timeout, float sock_timeout, unsigned char *data, size_t *data_length);
    
    /// Coalesced send and receive.
    /**
     * \param socket Socket identifier (when 0, the port is locked first).
     * \param send_data Data to send.
     * \param send_length Length of \c send_data parameter in bytes.
     * \param recv_length Length of data to receive.
     * \param recv_timeout Time to wait for data in [s].
     * \param sock_timeout Lock time in [s].
     * \param recv_data Receive data buffer, at least \c recv_length bytes long.
     * \param[out] recv_data_length Number of bytes received.
     * \returns Socket identifier.
     */
    int sendrecv(int socket, const unsigned char *send_data, size_t send_length, size_t recv_length, float recv_timeout, float sock_timeout, unsigned char *recv_data, size_t *recv_data_length);
    
    /// Send data over already locked serial port.
    /**
     * \param socket Socket identifier.
     * \param data Data to send.
     * \param length Length of \c data parameter in bytes.
     * \param timeout Lock time in [s].
     * \note This function is asynchronous.
     */
    void send(int socket, const unsigned char *data, size_t length, float timeout);
    
    /// Unlock serial port.
    /**
     * \param socket Socket identifier.
     * \note This function is asynchronous.
     */
    void close(int socket);
    
    /// Flush serial port.
    /**
     * \param socket Socket identifier.
     * \param timeout Lock time in [s].
     * \note This function is asynchronous.
     */
    void flush(int socket, float timeout);
};

#endif /* __SS_CLIENT_H_ */
