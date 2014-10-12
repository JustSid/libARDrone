//
//  ARService.cpp
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

#include <assert.h>
#include "ARService.h"

namespace AR
{
	Service::Service(Drone *drone, const std::string &name) :
		_drone(drone),
		_reason(0),
		_state(State::Disconnected),
		_wakeup(false),
		_name(name),
		_canSleep(true)
	{}
	
	Service::~Service()
	{
		assert(_state == State::Disconnected);
	}
	
	
	void Service::Connect()
	{
		std::unique_lock<std::mutex> lock(_mutex);
		
		if(_state == State::Disconnected)
		{
			_thread = std::move(std::thread(&Service::ThreadHandler, this));
			_state  = State::Connecting;
			lock.unlock();
			
			_state = ConnectInternal();
			
			if(_state == State::Disconnected)
			{
				Wakeup(WakeupReason::Shutdown);
				_thread.join();
			}
		}
	}
	void Service::Disconnect()
	{
		std::unique_lock<std::mutex> lock(_mutex);
		
		if(_state == State::Connected || _state == State::Connecting)
		{
			_state = State::Disconnecting;
			lock.unlock();
			
			Wakeup(WakeupReason::Shutdown);
			_thread.join();
			
			DisconnectInternal();
			_state = State::Disconnected;
		}
	}
	
	
	void Service::Update()
	{}
	
	
	Service::State Service::ConnectInternal()
	{
		return State::Connected;
	}
	
	void Service::DisconnectInternal()
	{
	}
	
	
	
	void Service::SetState(State state)
	{
		_state = state;
	}
	
	void Service::SetCanSleep(bool value)
	{
		_canSleep = value;
	}
	
	
	void Service::Wakeup(WakeupReason reason)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		_reason |= reason;
		
		if(!_wakeup)
		{
			_wakeup = true;
			_signal.notify_one();
		}
	}
	
	void Service::ThreadHandler()
	{
		while(1)
		{
			if(_canSleep)
			{
				std::unique_lock<std::mutex> lock(_mutex);
				_signal.wait_for(lock, std::chrono::milliseconds(10));
				_wakeup = false;
				lock.unlock();
			}
			
			if(_reason & WakeupReason::Shutdown)
				return;
			
			Tick(_reason);
			_reason = 0;
		}
	}
}
