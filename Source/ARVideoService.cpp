//
//  ARVideoService.cpp
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
		SetCanSleep(false);
		
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
	
	void VideoService::Update()
	{
		{
			std::vector<std::vector<uint8_t>> data;
			
			{
				std::lock_guard<std::mutex> lock(_mutex);
				std::swap(data, _data);
			}
			
			for(auto &temp : data)
			{
				std::copy(temp.data(), temp.data() + temp.size(), _buffer + _bufferOffset);
				_bufferOffset += temp.size();
			}
		}
		
		// Get all the frames
		std::vector<size_t> frames;
		size_t frameBegin = std::string::npos;
		
		while(1)
		{
			size_t offset = (frameBegin != std::string::npos) ? frameBegin + 4 : 0;
			size_t header = FindPaveHeader(offset);
			
			if(header != std::string::npos)
			{
				if(frameBegin != std::string::npos)
				{
					PAVE *pave = reinterpret_cast<PAVE *>(_buffer + frameBegin);
					
					if(pave->control == PAVEControlTypeData)
					{
						if(pave->frame_type == PAVEFrameTypeIDRFrame || pave->frame_type == PAVEFrameTypeIFrame)
							frames.clear();
						
						frames.push_back(frameBegin);
					}
				}
				
				frameBegin = header;
			}
			else
				break;
		}
		
		if(frameBegin != std::string::npos)
		{
			for(size_t offset : frames)
			{
				PAVE *pave = reinterpret_cast<PAVE *>(_buffer + offset);
				const uint8_t *data = _buffer + offset + pave->header_size;
				
				for(auto &subscriber : _subscribers)
					subscriber.first(pave, data);
			}
			
			std::copy(_buffer + frameBegin, _buffer + _bufferOffset, _buffer);
			_bufferOffset = _bufferOffset - frameBegin;
		}
	}
	
	void VideoService::Tick(uint32_t reason)
	{
		size_t read = 0;
		std::vector<uint8_t> buffer(32768);
		
		Socket::Result result = _socket->Receive(buffer.data(), buffer.size(), &read);
		
		if(result == Socket::Result::Success)
		{
			buffer.resize(read);
			
			std::lock_guard<std::mutex> lock(_mutex);
			_data.push_back(std::move(buffer));
		}
		else
		{
			_socket->Disconnect();
			_bufferOffset = 0;
			
			SetState((_socket->Connect()) ? Service::State::Connected : Service::State::Disconnected);
		}
	}
}
