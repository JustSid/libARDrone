//
//  ARDrone.cpp
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

#include "ARDrone.h"

namespace AR
{
	Drone::Drone(const std::string &droneIP) :
		_droneIP(droneIP),
		_state(State::Disconnected),
		_navdata(nullptr),
		_freshData(false)
	{
		_atService = new ATService(this, _droneIP);
		_navdataService = new NavdataService(this, _droneIP);
		_configService  = new ConfigService(this, _droneIP);
		
		_services.push_back(_atService);
		_services.push_back(_navdataService);
		_services.push_back(_configService);
	}
	
	Drone::~Drone()
	{
		Disconnect();
		
		delete _atService;
		delete _navdataService;
		delete _configService;
	}
	
	
	bool Drone::AddService(Service *service)
	{
		std::lock_guard<std::recursive_mutex> lock(_lock);
		
		if(_state == State::Disconnected)
		{
			_services.push_back(service);
			return true;
		}
		
		return false;
	}
	
	Service *Drone::GetService(const std::string &name)
	{
		std::lock_guard<std::recursive_mutex> lock(_lock);
		for(Service *service : _services)
		{
			if(service->GetName() == name)
				return service;
		}
		
		return nullptr;
	}
	
	
	void Drone::ConnectAsync()
	{
		if(_state != State::Disconnected)
			return;
		
		_state       = State::Connecting;
		_lastMessage = std::chrono::steady_clock::now();
		
		_needsNavdataOptionsUpdate = true;
		_options = 0;
		
		_demoFlag   = false;
		
		_atService->Connect();
		_navdataService->Connect();
		
		// These are not documented but send by the AR Drone at startup. So we just copy them
		_atService->Send(ATCommand("PMODE") << 2);
		_atService->Send(ATCommand("MISC") << 2 << 20 << 2000 << 3000);
		
		// Fetch the config
		_configService->Connect();
		
		for(Service *service : _services)
			service->Connect();
	}
	
	bool Drone::Connect()
	{
		ConnectAsync();
		
		while(GetState() == State::Connecting)
			Update();
		
		return (GetState() == State::Connected);
	}
	
	void Drone::Disconnect()
	{
		if(_state == State::Disconnected)
			return;
		
		for(Service *service : _services)
		{
			if(service == _atService || service == _configService || service == _navdataService)
				continue;
			
			service->Disconnect();
		}
		
		_configService->Disconnect();
		_navdataService->Disconnect();
		_atService->Disconnect();
	}
	
	
	
	bool Drone::Update()
	{
		if(_state != State::Connected && _state != State::Connecting)
			return false;
		
		if(_state == State::Connecting)
		{
			bool hasConnecting = false;
			
			for(Service *service : _services)
			{
				Service::State state = service->GetState();
				
				if(state == Service::State::Disconnected || state == Service::State::Disconnecting)
				{
					_state = State::ConnectionFailed;
					return false;
				}
				
				if(state == Service::State::Connecting)
				{
					hasConnecting = true;
					break;
				}
			}
			
			if(hasConnecting)
			{
				long time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _lastMessage).count();
				if(time >= 2100)
				{
					_state = State::ConnectionFailed;
					return false;
				}
			}
			else
			{
				_state = State::Connected;
			}
		}
		
		{
			std::lock_guard<std::recursive_mutex> lock(_lock);
			
			if(_needsNavdataOptionsUpdate)
			{
				uint32_t options = NavdataTagOptions(NavdataTag::Demo);
				
				for(Service *service : _services)
					options |= service->GetNavdataOptions();
				
				if(options != _options)
				{
					_configService->SendConfig("general:navdata_options", options, nullptr);
					_options = options;
				}
				
				_needsNavdataOptionsUpdate = false;
			}
			
			for(Service *service : _services)
				service->Update();
			
			if(_freshData)
			{
				if(_state == State::Connected)
				{
					for(auto &data : _navdataSubscriber)
						data.first(_navdata);
				}
				else
				{
					// Only send navdata to the services
					for(auto &data : _navdataSubscriber)
					{
						for(Service *service : _services)
						{
							if(data.second == service)
							{
								data.first(_navdata);
								break;
							}
						}
					}
				}
				
				_freshData = false;
			}
		}
		
		std::this_thread::yield();
		return true;
	}
	
	
	void Drone::AddNavdataSubscriber(std::function<void (Navdata *)> &&function, void *token)
	{
		std::lock_guard<std::recursive_mutex> lock(_lock);
		_navdataSubscriber.push_back(std::make_pair(std::move(function), token));
	}
	
	void Drone::RemoveNavdataSubscriber(void *token)
	{
		std::lock_guard<std::recursive_mutex> lock(_lock);
		
		for(auto i = _navdataSubscriber.begin(); i != _navdataSubscriber.end(); i ++)
		{
			if(i->second == token)
			{
				_navdataSubscriber.erase(i);
				return;
			}
		}
	}
	
	
	void Drone::SetNeedsNavdataOptionsUpdate()
	{
		std::lock_guard<std::recursive_mutex> lock(_lock);
		_needsNavdataOptionsUpdate = true;
	}
	
	void Drone::PublishNavdata(Navdata *data)
	{
		if(_state == State::Connecting)
		{
			std::atomic_signal_fence(std::memory_order_acquire); // Might actually be better to make _demoFlag atomic...
			
			if(data->state & ARDRONE_NAVDATA_BOOTSTRAP && !_demoFlag)
			{
				_configService->SendConfig("general:navdata_demo", true, [=](bool result) {
					if(!result)
					{
						_demoFlag = false;
						std::atomic_signal_fence(std::memory_order_release);
					}
				});
				
				_demoFlag = true;
			}
		}
		
		std::lock_guard<std::recursive_mutex> lock(_lock);
		delete _navdata;
		
		_navdata   = data;
		_freshData = true;
		
		_lastMessage = std::chrono::steady_clock::now();
	}
}
