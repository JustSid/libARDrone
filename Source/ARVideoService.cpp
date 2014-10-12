//
//  ARVideoService.cpp
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

#include <algorithm>
#include "ARVideoService.h"
#include "ARDrone.h"

namespace AR
{
	VideoService::VideoService(Drone *drone) :
		Service(drone, "Video"),
		_socket(new Socket(drone->GetDroneIP(), 5555, Socket::Type::TCP))
	{
		_bufferSize = 32 * 1024 * 1024;
		_bufferOffset = 0;
		_buffer = new uint8_t[_bufferSize]; // Ought to be enough...
	}
	
	VideoService::~VideoService()
	{
		delete _socket;
		delete [] _buffer;
	}
	
	
	
	Service::State VideoService::ConnectInternal()
	{
		_bufferOffset = 0;
		_frameBegin   = std::string::npos;
		
		return (_socket->Connect()) ? Service::State::Connected : Service::State::Disconnected;
	}
	
	void VideoService::DisconnectInternal()
	{
		_socket->Disconnect();
	}
	
	void VideoService::AddVideoDataSubscriber(std::function<void (PAVE *, const uint8_t *)> &&callback, void *token)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_subscribers.push_back(std::make_pair(std::move(callback), token));
	}
	
	void VideoService::RemoveVideoDataSubscriber(void *token)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		for(auto i = _subscribers.begin(); i != _subscribers.end(); i ++)
		{
			if(i->second == token)
			{
				_subscribers.erase(i);
				return;
			}
		}
	}
	
	size_t VideoService::FindPaveHeader(size_t offset)
	{
		if(_bufferOffset <= 4)
			return std::string::npos;
		
		uint8_t *temp = _buffer + offset;
		
		for(size_t i = offset; i < _bufferOffset - 4; i ++)
		{
			if(*(temp + 0) == 'P' && *(temp + 1) == 'a' && *(temp + 2) == 'V' && *(temp + 3) == 'E')
				return temp - _buffer;
			
			temp ++;
		}
		
		return std::string::npos;
	}
	
	void VideoService::HandleFrame(std::vector<uint8_t> &frame)
	{
		PAVE *header = reinterpret_cast<PAVE *>(frame.data());
		
		if(header->control != PAVEControlTypeData)
			return;
		
		const uint8_t *data = frame.data() + sizeof(PAVE);
		
		std::lock_guard<std::mutex> lock(_mutex);
		
		for(auto &subscriber : _subscribers)
			subscriber.first(header, data);
	}
	
	void VideoService::Tick(uint32_t reason)
	{
		while(1)
		{
			size_t read = 0;
			
			if(_socket->Receive(_buffer + _bufferOffset, _bufferSize - _bufferOffset, &read) == Socket::Result::Success)
			{
				_bufferOffset += read;
				
				size_t offset = (_frameBegin != std::string::npos) ? _frameBegin + 4 : 0;
				size_t header = FindPaveHeader(offset);
				
				if(header != std::string::npos)
				{
					if(_frameBegin != std::string::npos)
					{
						// Copy the frame
						std::vector<uint8_t> frame(header - _frameBegin);
						std::copy(_buffer + _frameBegin, _buffer + header, frame.data());
						
						HandleFrame(frame);
						
						// Move the frame back in the buffer
						std::copy(_buffer + header, _buffer + _bufferOffset, _buffer);
						
						header        = 0;
						_bufferOffset = (_bufferOffset - header);
					}
					
					_frameBegin = header;
					break;
				}
			}
		}
	}
}
