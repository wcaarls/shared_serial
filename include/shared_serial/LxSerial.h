//============================================================================
// Name        : LxSerial.cpp
// Author      : Eelko van Breda,www.dbl.tudelft.nl
// Version     : 0.1
// Copyright   : Copyright (c) 2008 LGPL
// Description : serial communicatin class linux
//============================================================================

#ifndef LXSERIAL_H_
#define LXSERIAL_H_
//#define __DBG__

#include <fcntl.h>																/* fileio */
#include <termios.h>   															/* terminal i/o system, talks to /dev/tty* ports  */
#include <unistd.h>																/* Read function */
#include <sys/ioctl.h>															/* ioctl function */
#include <iostream>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define INVALID_DEVICE_HANDLE		-1

class LxSerial
{
	public:
		enum PortType { 	RS232, 													// Normal RS232
							RS485_EXAR,												// EXAR XR16C2850
							RS485_FTDI,												// FTDI FT232RL in 485 mode
							RS485_SMSC,												// SMSC SCH311X RS-485 mode (Versalogic Sidewinder board)
							TCP
		};
		enum PortSpeed { 	S50			=	B50,									// Baudrate to use for the port --> see termios.h
							S75			=	B75,
							S110		=	B110,
							S134		=	B134,
							S150		=	B150,
							S200		=	B200,
							S300		=	B300,
							S600		=	B600,
							S1200		=	B1200,
							S1800		=	B1800,
							S2400		=	B2400,
							S4800		=	B4800,
							S9600		=	B9600,
							S19200		=	B19200,
							S38400		=	B38400,
							S57600  	=	B57600,
							S115200 	=	B115200,
							S230400 	=	B230400,
							S460800 	=	B460800,
							S500000 	=	B500000,
							S576000 	=	B576000,
							S921600 	=	B921600,
							S1000000	=	B1000000,
							S1152000	=	B1152000,
							S1500000	=	B1500000,
							S2000000	=	B2000000,
							S2500000	=	B2500000,
							S3000000	=	B3000000,
							S3500000	=	B3500000,
							S4000000	=	B4000000
		};
		/*return values*/
		static const int READ_ERROR 				=	-1;
		static const int COLLISION_DETECT_ERROR		=	-2;
		static const int ECHO_TIMEOUT_ERROR			=	-3;

		/*magic numbers*/
		static const int COLLISION_WAIT_TIME_USEC	=	10000;							// microseconds
		static const int ECHO_WAIT_TIME_SEC			=	1;								// seconds
		static const int ECHO_WAIT_TIME_USEC		=	0;							// microseconds
		static const int WAIT_FOR_DATA_DSEC			=	5;								//

	protected:
		int 			hPort;															// file handle to the port
		std::string		s_port_name;													// name of the port that was opened
//		bool			b_initialised;													//
		bool			b_clear_echo;													// read sended characters from Rx when true
		bool			b_rts;															// this boolean must be set to enforce setting the RTS signal without hardware contol enabled
		bool 			b_hw_flow_control;												//
		bool			b_socket;
		termios			options, old_options; 											//
		bool			wait_for_input(int *seconds, int *microseconds);					// private member function to wait for port. the time variables are modified after return to reflect the time not slept
		void			set_port_type(LxSerial::PortType port_type);

	public:
			LxSerial();
			 ~LxSerial();
		bool	port_open(const std::string& portname, LxSerial::PortType port_type);	// open serial port. If overridden, make sure you set s_port_name!!
		bool	is_port_open();
		std::string&	get_port_name();
		bool	set_speed(LxSerial::PortSpeed baudrate );						// enumerated
		bool	set_speed_int(const int baudrate);	// Set speed by integer value directly - UNPROTECTED!
		void	set_clear_echo(bool clear);										// clear echoed charackters from input and detect collisions on write
		bool	port_close();
		int	port_read(unsigned char* buffer, int numBytes) const;
		int 	port_read(unsigned char* buffer, int numBytes, int seconds, int microseconds);
		int	port_write(unsigned char* buffer, int numBytes);
		void	flush_buffer();													// flush input and output buffers

};


#endif /*LXSERIAL_H_*/
