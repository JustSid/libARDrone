//
//  ARSocket.cpp
//  libARDronde
//
//  Created by Sidney Just
//  Copyright (c) 2014 by Sidney Just
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
//  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <unistd.h>
#include <errno.h>
#include <strings.h>
#include "ARSocket.h"

namespace AR
{
	Socket::Socket(const std::string &address, uint16_t port, Type type) :
		_ip(address),
		_port(port),
		_socket(-1),
		_type(type)
	{}
	
	Socket::~Socket()
	{
		Disconnect();
	}
	
	bool Socket::Connect()
	{
		int type = (_type == Type::UDP) ? SOCK_DGRAM : SOCK_STREAM;
		int protocol = (_type == Type::UDP) ? IPPROTO_UDP : IPPROTO_TCP;
		
		if((_socket = socket(PF_INET, type, protocol)) == -1)
			return false;
		
		bzero(&_address, sizeof(struct sockaddr_in));
		bzero(&_sendAddress, sizeof(struct sockaddr_in));
		
		
		uint16_t port = _port;
		
		if(_type == Type::UDP)
		{
			_address.sin_family = AF_INET;
			_address.sin_port   = htons(port);
			_address.sin_addr.s_addr = htonl(INADDR_ANY);
			
			port = _port;
			
			_sendAddress.sin_family = AF_INET;
			_sendAddress.sin_port   = htons(port);
			
			if(inet_aton(_ip.c_str(), &_sendAddress.sin_addr) == 0)
				return false;
			
			
			if(bind(_socket, reinterpret_cast<struct sockaddr *>(&_address), sizeof(_address)) == -1)
				return false;
		}
		if(_type == Type::TCP)
		{
			_address.sin_family = AF_INET;
			_address.sin_port   = htons(port);
			
			if(inet_aton(_ip.c_str(), &_address.sin_addr) == 0)
				return false;
			
			_sendAddress.sin_family = _address.sin_family;
			_sendAddress.sin_port   = _address.sin_port;
			_sendAddress.sin_addr   = _address.sin_addr;
			
			if(connect(_socket, reinterpret_cast<struct sockaddr *>(&_address), sizeof(_address)) == -1)
				return false;
		}
		
		
		struct timeval tv;
		tv.tv_sec  = 2;
		tv.tv_usec = 0;
		
		setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
		setsockopt(_socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(struct timeval));
		
		return true;
	}
	
	void Socket::Disconnect()
	{
		if(_socket != -1)
		{
			close(_socket);
			_socket = -1;
		}
	}
	
	
	Socket::Result Socket::Send(const void *data, size_t length)
	{
		Result retVal = Result::Success;
		
		int error = errno;
		
		ssize_t result = sendto(_socket, data, length, 0, reinterpret_cast<struct sockaddr *>(&_sendAddress), sizeof(_sendAddress));
		if(result == -1)
			retVal = Result::BrokenSocket;
		
		errno = error;
		
		return retVal;
	}
	
	Socket::Result Socket::Receive(void *data, size_t maximum, size_t *actual)
	{
		Result retVal = Result::Success;
		
		int error = errno;
		
		switch(_type)
		{
			case Type::UDP:
			{
				struct sockaddr_in temp;
				socklen_t length = sizeof(_address);
				
				ssize_t result = recvfrom(_socket, data, maximum, 0, reinterpret_cast<struct sockaddr *>(&temp), &length);
				
				if(result == -1)
				{
					retVal = Result::BrokenSocket;
					
					if(errno == EAGAIN || errno == EWOULDBLOCK)
						retVal = Result::Timeout;
				}
				else if(actual)
				{
					*actual = result;
				}
				
				break;
			}
				
			case Type::TCP:
			{
				ssize_t result = recv(_socket, data, maximum, 0);
				
				if(result == -1)
				{
					retVal = Result::BrokenSocket;
					
					if(errno == EAGAIN || errno == EWOULDBLOCK)
						retVal = Result::Timeout;
				}
				else if(actual)
				{
					*actual = result;
				}
				
				break;
			}
		}
		
		
		
		errno = error;
		return retVal;
	}
}
