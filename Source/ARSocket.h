//
//  ARSocket.h
//  libARDrone
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

#ifndef __libARDrone__ARSocket__
#define __libARDrone__ARSocket__

#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>

namespace AR
{
	class Socket
	{
	public:
		enum class Result
		{
			Success,
			Timeout,
			BrokenSocket
		};
		
		enum class Type
		{
			UDP,
			TCP
		};
		
		Socket(const std::string &address, uint16_t port, Type = Type::UDP);
		~Socket();
		
		bool Connect();
		void Disconnect();
		
		Result Send(const void *data, size_t length);
		Result Receive(void *data, size_t maximum, size_t *actual);
		
	private:
		Type _type;
		std::string _ip;
		uint16_t _port;
		
		struct sockaddr_in _address;
		struct sockaddr_in _sendAddress;
		
		int _socket;
	};
}

#endif /* defined(__libARDrone__ARSocket__) */
