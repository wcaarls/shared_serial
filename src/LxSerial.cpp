//============================================================================
// Name        : LxSerial.cpp
// Author      : Eelko van Breda,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2008 LGPL
// Description : serial communicatin class linux
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>

#ifdef __APPLE__
#include <architecture/byte_order.h>
#else
#include <endian.h>
#endif

#include "shared_serial/LxSerial.h"


/*	constructor */
LxSerial::LxSerial()
{
	hPort 			= INVALID_DEVICE_HANDLE;
//	b_initialised 	= false;
	b_clear_echo	= false;
	b_hw_flow_control = false;
	b_socket = false;
}

/* return name of the port that was opened */
std::string& LxSerial::get_port_name()
{
	return s_port_name;
}

void LxSerial::set_port_type(LxSerial::PortType port_type)
{
	switch (port_type)
	{
		case RS232:
			b_rts 				= 	false;
			break;

		case RS485_FTDI:
			b_rts 				= 	false;
			break;

		case RS485_EXAR:
			b_rts 				= 	true;
			b_clear_echo 		= 	true;
			break;

		case RS485_SMSC:
			b_rts 				= 	false;
			b_clear_echo 		= 	true;
			break;
			
		case TCP:
			b_socket = true;
			b_rts = false;
			b_clear_echo = false;
			break;

		default:
			perror("LxSerial: no port type specified");
			break;
	}
}

/* open port */
bool LxSerial::port_open(const std::string& portname, LxSerial::PortType port_type)
{
	// Set port type
	set_port_type(port_type);
	
	if (b_socket)
	{
		int optval = 1;

		struct sockaddr_in addr;
		struct hostent *server;

		hPort = socket(AF_INET, SOCK_STREAM, 0);
		setsockopt(hPort, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

		bzero((char *) &addr, sizeof(addr));
		addr.sin_family = AF_INET;

		std::string host_name   = portname.substr(0, portname.find(':'));
		std::string port_number = portname.substr(portname.find(':')+1);

		server = gethostbyname(host_name.c_str());
		if (!server)
		{
			perror("unknown remote serial host name");
			return false;
		}

		bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);
		addr.sin_port = htons(atoi(port_number.c_str()));

		if (connect(hPort,(struct sockaddr *) &addr,sizeof(addr)) < 0)
		{
			perror("error connecting to remote serial host");
			return false;
		}
	}
	else
	{
		// Open port
		hPort = open(portname.c_str(), O_RDWR | O_NOCTTY);//|O_NDELAY);						// open the serial device
																				// O_RDWR = open read an write
																				// The O_NOCTTY flag tells UNIX that this program doesn't want to be the "controlling terminal" for that 
																				// port. If you don't specify this then any input (such as keyboard abort signals and so forth) will affect 
																				// your process. Programs like getty(1M/8) use this feature when starting the login process, but normally a 
																				// user program does not want this behavior.
																				// The O_NDELAY flag tells UNIX that this program doesn't care what state the DCD signal line is in ­ 
																				// whether the other end of the port is up and running. If you do not specify this flag, your process will be 
																				// put to sleep until the DCD signal line is the space voltage.

		if (hPort < 0) {															//check if port opened correctly
			perror(" Could not open serial port, aborting");
			return false;
		}


		tcgetattr(hPort, &options); 									// get the current termios (com port options struct) from the kernel
		tcgetattr(hPort, &old_options); 								// get the current termios copy to restore on close

		cfsetispeed(&options, B115200);									// set incomming baudrate to standard 9600, this can be changed with the function set_speed()
		cfsetospeed(&options, B115200);									// set outgoing baudrate to standard 9600

																	// c_cflag Control Options
																	// |= (..) means enabled
																	// &= ~(..) means not enabled
		options.c_cflag |= (CLOCAL|CREAD|CS8); 								// CREAD = enanble receiver
																	// CLOCAL = Local Line, do not change owner of port
																	// CS8 = 8 databits
		options.c_cflag &= ~(CRTSCTS|PARENB|CSTOPB); 					// Disable HW flow control
																	// no paritybit, PARENB = paritybit
																	// 1 stop bit, CSTOPB = 2 stop bits (1 otherwise)
		if (b_hw_flow_control){
			options.c_cflag |= (CRTSCTS); 								// Enable HW flow control
		}

																	// c_lflag Line Options ( controls how input charackters are handeled by the serial driver
		options.c_lflag &= ~(ECHO|ECHONL|ECHOE|ICANON|ISIG|IEXTEN);
//	options.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
																	// ~(..) means not enabled
																	// ECHO echoing of input characters
																	// ECHOE echo erease character as BS-SP-BS
																	// ECHONL mimics echo but doesn't move to next line when it ends
																	// ICANON cononical input = line oriented input, else RAW
																	// ISIG ENABLE SIGINTR, SIGSUSP, SIGDSUSP and SIGQUIT signals
																	// IEXTERN = enable extended functions

																	// c_iflag Input Options
		options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF);
//	options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
																	// ~(..) means not enabled
																	// IGNBRK ignore break condition
																	// BRKINT send a SIGINT when a break condition is detected
																	// PARMRK Mark parity errors
																	// ISTRIP strip parity bits
																	// INLCR map new line to CR
																	// IGNCR ignore CR
																	// ICRNL map CR to new line
																	// IXON Enable software flow control (outgoing)
																	// IXOFF Enable software flow control (incomming)

																	// c_oflag output options
		options.c_oflag &= ~(OPOST);
																	// ~(..) means not enabled
																	// OPOST postprocess output (if not set RAW output)
																	// when OPOST is disabled all other other option bits of c-Oflag are ignored

																	// c_cc control character array (control caracter options and timeouts
																	// Timeout 0.005 sec for first byte, read minimum of 0 bytes
																	// timeouts are ignored in cononical input or when NDELAY option is set.
		options.c_cc[VMIN]     = 0; 									// VMIN Minimum number of characters to read
		options.c_cc[VTIME]    = WAIT_FOR_DATA_DSEC; 					// Time to wait for data (tenths of seconds)

//		fcntl(hPort, F_SETFL, FNDELAY); // don't wait for input when reading data

		if (b_rts){														// if we are using RS_485_EXAR type, we clear the RTS signal
			int msc = TIOCM_RTS;
			ioctl(hPort, TIOCMBIC, &msc);								// Clear RTS signal
			usleep(100);												// WAIT FOR SIGNAL TO BE CLEARD
		}



		if (tcsetattr(hPort,TCSANOW, &options)!=0){						// Set the new options for the port now
			perror("Error: Could not set serial port settings");
			return false;
		}

		usleep(100);													// additional wait for correct functionality
		tcflush(hPort, TCIOFLUSH);										// flush terminal data
	}

	// Save port name
	s_port_name = portname;

	return true;
}

bool	LxSerial::is_port_open()
{
	return hPort >= 0;
}

// speedcontrol */
bool	LxSerial::set_speed( LxSerial::PortSpeed baudrate )
{
	if (b_socket)
		return false;

#ifdef __APPLE__
	int speed = baudrate;
	if ( ioctl( hPort, IOSSIOSPEED, &speed ) == -1 )
	{
		perror("Error: Could not set serial port baudrate");
		return false;
	}
#else
	cfsetispeed(&options, baudrate);								//set incoming baud rate
	cfsetospeed(&options, baudrate);								//set outgoing baud rate

	if (tcsetattr(hPort,TCSANOW, &options)!=0){						// Set the new options for the port now
		perror("Error: Could not set serial port baudrate");
		return false;
	}
	usleep(100);													// additional wait for correct functionality
	tcflush(hPort, TCIOFLUSH);										// flush terminal data
#endif
	return true;
}

bool LxSerial::set_speed_int(const int baudrate)
{
	LxSerial::PortSpeed baud;
	// Baud rate conversion from index to real baud rate :(
	switch (baudrate)
	{
		case 4000000:	baud = S4000000;	break;
		case 3500000:	baud = S3500000;	break;
		case 3000000:	baud = S3000000;	break;
		case 2500000:	baud = S2500000;	break;
		case 2000000:	baud = S2000000;	break;
		case 1500000:	baud = S1500000;	break;
		case 1152000:	baud = S1152000;	break;
		case 1000000:	baud = S1000000;	break;
		case 921600:	baud = S921600;	break;
		case 576000:	baud = S576000;	break;
		case 500000:	baud = S500000;	break;
		case 460800:	baud = S460800;	break;
		case 230400:	baud = S230400;	break;
		case 115200:	baud = S115200;	break;
		case 57600:		baud = S57600;	break;
		case 38400:		baud = S38400;	break;
		case 19200:		baud = S19200;	break;
		case 9600:		baud = S9600;	break;
		case 4800:		baud = S4800;	break;
		case 2400:		baud = S2400;	break;
		case 1800:		baud = S1800;	break;
		case 1200:		baud = S1200;	break;
		case 600:		baud = S600;	break;
		case 300:		baud = S300;	break;
		case 200:		baud = S200;	break;
		case 150:		baud = S150;	break;
		case 134:		baud = S134;	break;
		case 110:		baud = S110;	break;
		case 75:		baud = S75;		break;
		case 50:		baud = S50;		break;
		default:
			printf("This is not a legal portspeed!\n");
			return false;
	}

	set_speed(baud);
	return true;
}

// clear echoed characters from input and detect collisions on write, use for EXAR RS485
void	LxSerial::set_clear_echo(bool clear)
{
	b_clear_echo = clear;
}

// close the serial port
bool	LxSerial::port_close()
{
	if (hPort==INVALID_DEVICE_HANDLE)
		return true;

	if (!b_socket)
	{
		if (tcsetattr(hPort,TCSANOW, &old_options)!=0) {				// restore the old port settings
			perror("Warning: Could not restore serial port settings.");
		}
	}

  	if(close(hPort) == -1) {										//close serial port
  		perror("Error: Could not close serial port.");
  		return false;
  	}
  	hPort = INVALID_DEVICE_HANDLE;
  	return true;
}

int		LxSerial::port_read(unsigned char* buffer, int numBytes) const
{
	int nBytesRead = read(hPort, buffer, numBytes);

	#ifdef __DBG__
	printf("read  ");
	for (int i=0;i<nBytesRead;i++)
		{ printf("%02X ",buffer[i]); }
	printf("(%d)\n",nBytesRead);
	#endif

	return nBytesRead;
}

int 	LxSerial::port_read(unsigned char* buffer, int numBytes, int seconds, int microseconds)
{
	// Init time variables (they are decreased by wait_for_input)
	int s = seconds;
	int us = microseconds;
	int nBytesRead = 0;
	while (nBytesRead < numBytes)
	{
		if( wait_for_input( &s, &us) )
		{
			int partialRead = read(hPort, buffer + nBytesRead, numBytes - nBytesRead); // Read data
			nBytesRead += partialRead;
		}
		else if (nBytesRead < 0)
		{
			#ifdef __DBG__
			printf("Read Timeout... \n");
			#endif
			return READ_ERROR;
		}
		else
			break;
	}

	#ifdef __DBG__
	printf("read  ");
	for (int i=0;i<nBytesRead;i++)
		{ printf("%02X ",buffer[i]); }
	printf("(%d)\n",nBytesRead);
	#endif

	return nBytesRead;
}

bool	LxSerial::wait_for_input(int *seconds, int *microseconds)
{
	fd_set readset;
	timeval timeout;
	timeout.tv_sec = *seconds;										// seconds
	timeout.tv_usec = *microseconds;									// microseconds
	FD_ZERO(&readset);     											// clear file discriptor
	FD_SET(hPort, &readset);										// set filediscripter for port
	int res = select(hPort+1, &readset, NULL, NULL, &timeout); 	// wait till readable data is in the buffer
	*seconds = timeout.tv_sec;
	*microseconds = timeout.tv_usec;
	return res == 1;
}

int		LxSerial::port_write(unsigned char* buffer, int numBytes)
{
	int msc = TIOCM_RTS;
	if (b_rts){														// control the RTS signal if needed
		ioctl(hPort, TIOCMBIS, &msc);								// before you write set RTS signal
		usleep(1000);												// wait until the RTS signal is high
	}

	int numBytesWritten = write(hPort, buffer, numBytes);			// write data

	if (numBytes != numBytesWritten){
		perror("Error while writing to serial port");
		assert(numBytes == numBytesWritten);
	}

	#ifdef __DBG__
	printf("write ");
	for (int i=0;i<numBytes;i++)
		{ printf("%02X ",buffer[i]); }
	printf("(%d)",numBytesWritten);
	printf("\n");
	#endif
	
	tcdrain(hPort);		  											// Wait till all the data in the buffer is transmitted

	if (b_rts){														// control the RTS signal if needed
		ioctl(hPort, TIOCMBIC, &msc);  								// Clear Ready To Send signal
	}

	if (b_clear_echo)
	{												// Read echo from line and detect collisions if neened
		unsigned char* echo_buffer = new unsigned char[numBytes];
		int nBytesRead = 0;
		int s = ECHO_WAIT_TIME_SEC;
		int us = ECHO_WAIT_TIME_USEC;
		while (nBytesRead < numBytesWritten)
		{
			if (wait_for_input(&s , &us))
				nBytesRead = read(hPort, echo_buffer + nBytesRead, numBytes - nBytesRead);		// Read back ECHO of sended data
			else
			{
				#ifdef __DBG__
				printf("echo read back timeout\n");
				#endif
				delete[] echo_buffer;
				return ECHO_TIMEOUT_ERROR;
			}
		}


		#ifdef __DBG__
		printf("echo  ");
		for (int i=0;i<nBytesRead;i++)
			{ printf("%02X ",buffer[i]); }
		printf("(%d)\n",nBytesRead);
		#endif

		if (nBytesRead!=numBytesWritten)
		{
			#ifdef __DBG__
			printf("Warning: nr. of characters echoed not correct\n");
			#endif
			delete[] echo_buffer;
			return READ_ERROR;
		}

		if (memcmp(buffer,echo_buffer, numBytes)!=0)
		{
			#ifdef __DBG__
			printf("collition detect\n");
			#endif
			usleep(COLLISION_WAIT_TIME_USEC);							//wait for a while
			tcflush(hPort, TCIFLUSH); 								//empty all data that was in read buffer but has not been read
			delete[] echo_buffer;
			return COLLISION_DETECT_ERROR;
		}
		delete[] echo_buffer;
	}

	return numBytesWritten;
}

void LxSerial::flush_buffer()
{
	tcflush(hPort, TCIOFLUSH);										// flush data buffer
}

LxSerial::~LxSerial()
{
	if (hPort != INVALID_DEVICE_HANDLE)
		port_close();
}
