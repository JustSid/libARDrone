//
//  ARConfigService.h
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

#ifndef __libARDrone__ARConfigService__
#define __libARDrone__ARConfigService__

#include <list>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

#include "ARService.h"
#include "ARATService.h"
#include "ARVector.h"

namespace AR
{
	class Navdata;
	class ConfigService : public Service
	{
	public:
		friend class Drone;
		
		ConfigService(Drone *drone, const std::string &droneIP);
		~ConfigService() override;
		
		void FetchConfig(std::function<void(bool)> &&callback);
		void SendConfig(const std::string &key, const std::string &value, std::function<void(bool)> &&callback);
		std::string &GetConfig(const std::string &key);
		
	protected:
		void Tick(uint32_t reason) final;
		State ConnectInternal() final;
		void DisconnectInternal() final;
		
	private:
		void ProcessNavdata(Navdata *navdata);
		
		enum class CommandResult
		{
			Success,
			Failed,
			Proceed
		};
		
		struct Command
		{
			Command(int ttype) :
				retries(3),
				type(ttype),
				state(0)
			{}
			
			Command(Command &&other) = default;
			Command &operator =(Command &&other) = default;
			
			size_t retries;
			int type;
			int state;
			
			std::string key;
			std::string value;
			
			std::function<void(bool)> callback;
		};
		
		CommandResult HandleSendConfig(Command &command);
		CommandResult HandleRequestConfig(Command &command);
		void ParseConfig();
		
		ATService *_atService;
		Socket *_socket;
		
		std::mutex _mutex;
		std::list<Command> _queue;
		std::unordered_map<std::string, std::string> _config;
		
		uint32_t _droneState;
		bool _navdataConsumed;
		bool _requestedConfig;
		
		std::string _configBuffer;
	};
}

#endif /* defined(__libARDrone__ARConfigService__) */
