/*
 * IMSerial.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: makcakoca
 */

#include "../include/IMSerial.h"

bool IMSerial::b_signal_received = false;

IMSerial::IMSerial()
{
	this->str_name = "/dev/ttyAMA0";
	this->baudrate = B115200;
	this->data_bits = CS8;
	this->parity = 0;
	this->stop_bits = 0;
	this->i_serial_fd = -1;
	this->parity_on = 0;
	this->b_read_data_available = false;
	try{
		this->SerialOpen(this->str_name);
		this->SerialConfig();
	}catch(int i){
		throw i;
	}
}

IMSerial::IMSerial(string str_device_name, speed_t baudrate, tcflag_t data_bits, tcflag_t parity,
		tcflag_t parity_on, tcflag_t stop_bits)
{
	this->str_name = str_device_name;
	this->baudrate = baudrate;
	this->data_bits = data_bits;
	this->parity = parity;
	this->stop_bits = stop_bits;
	this->i_serial_fd = -1;
	this->parity_on = parity_on;
	this->b_read_data_available = false;

	try{
		this->SerialOpen(this->str_name);
		this->SerialConfig();
	}catch(int i){
		throw i;
	}

}

int IMSerial::SetBaudrate(speed_t baudrate)
{
	int i_status_value = -1;

	// Get the current options for the port...
	tcgetattr(this->i_serial_fd, &this->port_settings);

	//Set the baud rates to 19200...
	cfsetispeed(&this->port_settings, baudrate);
	cfsetospeed(&this->port_settings, baudrate);

	// Enable the receiver and set local mode...
	this->port_settings.c_cflag |= (CLOCAL | CREAD);

	// Set the new options for the port...
	i_status_value = tcsetattr(this->i_serial_fd, TCSANOW, &this->port_settings);

	if(i_status_value < 0)
	{
		throw -1;
	}


	return i_status_value;
}

int IMSerial::SetDataBits(tcflag_t data_bits)
{
	int i_status_value = -1;

	// Get the current options for the port...
	i_status_value = tcgetattr(this->i_serial_fd, &this->port_settings);
	if(i_status_value < 0)
	{
		throw -1;
	}


	this->port_settings.c_cflag &= ~CSIZE; /* Mask the character size bits */

	this->port_settings.c_cflag |= this->baudrate | CRTSCTS | data_bits | this->stop_bits
    		| parity_on | this->parity | CLOCAL | CREAD;

	// Set the new options for the port...
	i_status_value = tcsetattr(this->i_serial_fd, TCSANOW, &this->port_settings);
	if(i_status_value < 0)
	{
		throw -1;
	}

	this->data_bits = data_bits;

	return i_status_value;
}

int IMSerial::SetParity(tcflag_t parity, tcflag_t parity_on)
{
	int i_status_value = -1;

	// Get the current options for the port...
	i_status_value = tcgetattr(this->i_serial_fd, &this->port_settings);
	
	if(i_status_value < 0)
	{
		throw -1;
	}


	this->port_settings.c_cflag &= ~CSIZE; /* Mask the character size bits */

	this->port_settings.c_cflag |= this->baudrate | CRTSCTS | this->data_bits | this->stop_bits
    		| parity_on | parity | CLOCAL | CREAD;

	// Set the new options for the port...
	i_status_value = tcsetattr(this->i_serial_fd, TCSANOW, &this->port_settings);

	if(i_status_value < 0)
	{
		throw -1;
	}

	this->parity = parity;
	this->parity_on = parity_on;

	return i_status_value;
}

int IMSerial::SetStopBits(tcflag_t stop_bits)
{
	int i_status_value = -1;

	// Get the current options for the port...
	i_status_value = tcgetattr(this->i_serial_fd, &this->port_settings);
	if(i_status_value < 0)
	{
		throw -1;
	}

	this->port_settings.c_cflag &= ~CSIZE; /* Mask the character size bits */

	this->port_settings.c_cflag |= this->baudrate | CRTSCTS | this->data_bits | stop_bits
    		| this->parity_on | this->parity | CLOCAL | CREAD;

	// Set the new options for the port...
	i_status_value = tcsetattr(this->i_serial_fd, TCSANOW, &this->port_settings);

	if(i_status_value < 0)
	{
		throw -1;
	}

	this->stop_bits = stop_bits;

	return i_status_value;
}

speed_t IMSerial::GetBaudrate() const
{
	return this->baudrate;
}

tcflag_t IMSerial::GetDataBits() const
{
	return this->data_bits;
}

tcflag_t IMSerial::GetParity() const
{
	return this->parity;
}

tcflag_t IMSerial::GetStopBits() const
{
	return this->stop_bits;
}

int IMSerial::ReadData(char * p_c_data, unsigned int u_i_size)
{
	int i_status_value = -1;

	this->b_signal_received = false;

	memset(p_c_data, 0, u_i_size);

	i_status_value = read(this->i_serial_fd, p_c_data, u_i_size);


	/*	if(i_status_value < 0)
	{
		throw -2;
	}
	else if(i_status_value == 0)
	{
		throw -3;
		}*/

	return i_status_value;
}

int IMSerial::ReadDataAsync(char * p_c_data, unsigned int u_i_size)
{
	int i_status_value = -1;

	while(1)
	{
		while(!this->isReadyRead());
		//	i_status_value = this->ReadData(p_c_data, u_i_size);
	}

	return i_status_value;
}

int IMSerial::WriteData(const void * p_data, size_t size)
{
	int i_status_value = -1;

	i_status_value = write(this->i_serial_fd, p_data, size);

	if(i_status_value < 0)
	{
		throw -4;
	}

	return i_status_value;
}

bool IMSerial::isReadyRead() const
{
	return b_signal_received;

}

int IMSerial::SerialOpen(std::string _str_name)
{
	int i_status_value = -1;

	i_status_value = this->i_serial_fd = open(_str_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(i_status_value < 0)
	{
		throw -5;
	}

	return i_status_value;
}

int IMSerial::SerialClose()
{
	int i_status_value = -1;

	i_status_value = close(this->i_serial_fd);

	if(i_status_value < 0)
	{
		throw -6;
	}

	this->i_serial_fd = -1;

	return i_status_value;
}


int IMSerial::SerialConfig()
{
	int i_status_value = -1;

    //install the serial handler before making the device asynchronous
	this->signal_action_io.sa_handler = IMSerial::SignalHandlerIO;
    i_status_value = sigemptyset(&this->signal_action_io.sa_mask);   //saio.sa_mask = 0;
    if(i_status_value < 0)
	{
		throw -9;
	}
    this->signal_action_io.sa_flags = 0;
    this->signal_action_io.sa_restorer = NULL;
    i_status_value = sigaction(SIGIO, &this->signal_action_io, NULL);
    if(i_status_value < 0)
	{
		throw -9;
	}

    // allow the process to receive SIGIO
 //   fcntl(this->i_serial_fd, F_SETOWN, getpid());
    // Make the file descriptor asynchronous (the manual page says only
     // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
    i_status_value = fcntl(this->i_serial_fd, F_SETFL, FASYNC);
    if(i_status_value < 0)
		{
			throw -9;
		}
 //   fcntl(this->i_serial_fd, F_SETFL, O_ASYNC);


    this->port_settings.c_cflag = this->baudrate | CRTSCTS | this->data_bits | this->stop_bits
    		| parity_on | this->parity | CLOCAL | CREAD;

    this->port_settings.c_iflag = 0;
    //this->port_settings.c_iflag |= IGNBRK;
    //this->port_settings.c_iflag &= ~(IXON | IXOFF | IXANY | BRKINT);//IGNPAR;

    this->port_settings.c_oflag = 0;
    this->port_settings.c_lflag = 0;//~(ICANON | ECHO | ECHOE | ISIG);  // RAW input
    this->port_settings.c_cc[VMIN]=1;
    this->port_settings.c_cc[VTIME]=0;

    i_status_value = tcflush(this->i_serial_fd, TCIFLUSH);
	if(i_status_value < 0)
	{
		throw -7;
	}

    i_status_value = tcsetattr(this->i_serial_fd, TCSANOW, &this->port_settings);

	if(i_status_value < 0)
	{
		throw -8;
	}

	return i_status_value;
}

void IMSerial::SignalHandlerIO(int i_status)
{
  b_signal_received = true;
}

IMSerial::~IMSerial() {
  SerialClose();
}
