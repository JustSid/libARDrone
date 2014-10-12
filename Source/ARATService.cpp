//
//  ARATService.cpp
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

#include <iostream>
#include "ARATService.h"

namespace AR
{
	ATCommand::ATCommand(const std::string &command)
	{
		_stream << "AT*" << command << "=x";
	}
	
	std::string ATCommand::GetCommand(uint32_t sequence) const
	{
		std::string string = _stream.str();
		size_t pos = string.find("=x") + 1;
		
		std::stringstream temp;
		temp << sequence;
		
		string.replace(pos, 1, temp.str());
		string.append("\r");
		
		return string;
	}
	
	
	
	ATService::ATService(Drone *drone, const std::string &address) :
		Service(drone, "AT"),
		_socket(new Socket(address, 5556))
	{}
	
	ATService::~ATService()
	{
		delete _socket;
	}
	
	
	
	Service::State ATService::ConnectInternal()
	{
		_sequence = 1;
		return _socket->Connect() ? State::Connected : State::Disconnected;
	}
	
	void ATService::DisconnectInternal()
	{
		_socket->Disconnect();
	}
	
	
	void ATService::Send(const ATCommand &command)
	{
		std::lock_guard<std::mutex> lock(_mutex);
	
		_queue.push_back(std::move(command.GetCommand(_sequence)));
		_sequence ++;
		
		Wakeup(WakeupReason::Update);
	}
	
	void ATService::Tick(uint32_t reason)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		size_t length = 0;
		std::stringstream stream;
		
		for(const std::string &command : _queue)
		{
			if(length + command.length() > 1000)
			{
				std::string string = stream.str().c_str();
				_socket->Send(string.c_str(), string.length());
				
				length = 0;
				stream = std::stringstream();
			}
			
			stream << command;
			length += command.length();
		}
		
		if(length > 0)
		{
			std::string string = stream.str().c_str();
			_socket->Send(string.c_str(), string.length());
		}
		
		_queue.clear();
	}
}
