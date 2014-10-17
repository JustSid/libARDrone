//
//  ARDrone.h
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

#ifndef __libARDrone__ARDrone__
#define __libARDrone__ARDrone__

#include <iostream>
#include <string>
#include <atomic>
#include <thread>
#include <vector>

#include "ARATService.h"
#include "ARNavdataService.h"
#include "ARControlService.h"
#include "ARConfigService.h"
#include "ARVideoService.h"

namespace AR
{
	class Drone
	{
	public:
		friend class Service;
		friend class NavdataService;
		friend class ATService;
		
		enum class State
		{
			Disconnected,
			Connecting,
			ConnectionFailed,
			Connected
		};
		
		Drone(const std::string &droneIP);
		~Drone();
		
		void ConnectAsync();
		bool Connect();
		void Disconnect();
		
		bool Update();
		void SetNavdataOptions(uint32_t options);
		
		template<class T, class... Args>
		T *AddService(Args&&... args)
		{
			T *service = new T(this, std::forward<Args>(args)...);
			if(AddService(service))
				return service;
			
			delete service;
			return nullptr;
		}
		
		void AddNavdataSubscriber(std::function<void(Navdata *data)> &&function, void *token);
		void RemoveNavdataSubscriber(void *token);
		
		State GetState() const { return _state.load(); }
		
		template<class T>
		T *GetService(const std::string &name)
		{
			return dynamic_cast<T *>(GetService(name));
		}
		
		const std::string &GetDroneIP() const { return _droneIP; }
		
	private:
		bool AddService(Service *service);
		
		Service *GetService(const std::string &name);
		void PublishNavdata(Navdata *data);
		void SetNeedsNavdataOptionsUpdate();
		
		std::atomic<State> _state;
		std::string _droneIP;
		
		std::string _sessionID;
		std::string _applicationID;
		std::string _userID;
		
		std::vector<Service *> _services;
		
		ATService *_atService;
		NavdataService *_navdataService;
		ConfigService *_configService;
		
		std::chrono::steady_clock::time_point _lastMessage;
		
		std::recursive_mutex _lock;
		std::vector<std::pair<std::function<void(Navdata *data)>, void *>> _navdataSubscriber;
		Navdata *_navdata;
		bool _freshData;
		
		bool _demoFlag;
		bool _needsNavdataOptionsUpdate;
		
		uint32_t _options;
		uint32_t _navdataOptions;
	};
}

#endif /* defined(__libARDrone__ARDrone__) */
