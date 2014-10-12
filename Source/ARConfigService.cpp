//
//  ARConfigService.cpp
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

#include "ARConfigService.h"
#include "ARDrone.h"

typedef enum
{
	NO_CONTROL_MODE = 0,          /*<! Doing nothing */
	ARDRONE_UPDATE_CONTROL_MODE,  /*<! Not used */
	PIC_UPDATE_CONTROL_MODE,      /*<! Not used */
	LOGS_GET_CONTROL_MODE,        /*<! Not used */
	CFG_GET_CONTROL_MODE,         /*<! Send active configuration file to a client through the 'control' socket UDP 5559 */
	ACK_CONTROL_MODE,             /*<! Reset command mask in navdata */
	CUSTOM_CFG_GET_CONTROL_MODE   /*<! Requests the list of custom configuration IDs */
} ARDRONE_CONTROL_MODE;

namespace AR
{
	enum CommandType
	{
		CommandTypeSend,
		CommandTypeRequest
	};
	
	enum CommandSendState
	{
		CommandSendStateSend,
		CommandSendStateAck,
		CommandSendStateAckClear
	};
	
	
	ConfigService::ConfigService(Drone *drone, const std::string &address) :
		Service(drone, "Config"),
		_socket(new Socket(address, 5559, Socket::Type::TCP))
	{
		
	}
	
	ConfigService::~ConfigService()
	{
		delete _socket;
	}
	
	
	Service::State ConfigService::ConnectInternal()
	{
		if(!_socket->Connect())
			return State::Disconnected;
		
		_atService = GetDrone()->GetService<ATService>("AT");
		GetDrone()->AddNavdataSubscriber(std::bind(&ConfigService::ProcessNavdata, this, std::placeholders::_1), this);
		
		_droneState = 0;
		_navdataConsumed = true;
		_requestedConfig = false;
		
		SendConfig("custom:session_id", "-all", [=](bool success) {
			
			if(!success)
			{
				SetState(State::Disconnected);
				return;
			}
			
			FetchConfig([=](bool result) {
				SetState(result ? State::Connected : State::Disconnected);
			});
		});
		
		return State::Connecting;
	}
	
	void ConfigService::DisconnectInternal()
	{
		_socket->Disconnect();
		GetDrone()->RemoveNavdataSubscriber(this);
	}
	
	void ConfigService::ProcessNavdata(Navdata *navdata)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		_droneState = navdata->state;
		_navdataConsumed = false;
		
		Wakeup(WakeupReason::Update);
	}
	
	
	void ConfigService::FetchConfig(std::function<void(bool)> &&callback)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		Command command(CommandTypeRequest);
		command.callback = std::move(callback);
		
		_queue.push_back(std::move(command));
		
		Wakeup(WakeupReason::Update);
	}
	
	void ConfigService::SendConfig(const std::string &key, const std::string &value, std::function<void(bool)> &&callback)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		Command command(CommandTypeSend);
		command.key = key;
		command.value = value;
		command.callback = std::move(callback);
		
		_queue.push_back(std::move(command));
		
		Wakeup(WakeupReason::Update);
	}
	
	std::string &ConfigService::GetConfig(const std::string &key)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _config[key];
	}
	
	
	ConfigService::CommandResult ConfigService::HandleSendConfig(Command &command)
	{
		switch(command.state)
		{
			case CommandSendStateSend:
				_atService->Send(ATCommand("CONFIG_ID") << "252f1910" << "de476a00" << "efc8fd84");
				_atService->Send(ATCommand("CONFIG") << command.key << command.value);
				
				command.state = CommandSendStateAck;
				return CommandResult::Proceed;
				
			case CommandSendStateAck:
				if(!(_droneState & ARDRONE_COMMAND_MASK))
				{
					command.state = CommandSendStateSend;
					return CommandResult::Failed;
				}
				
				_atService->Send(ATCommand("CTRL") << ACK_CONTROL_MODE << 0);
				command.state = CommandSendStateAckClear;
				
				return CommandResult::Proceed;
				
			case CommandSendStateAckClear:
				if((_droneState & ARDRONE_COMMAND_MASK))
				{
					command.state = CommandSendStateAck;
					return CommandResult::Failed;
				}
				
				_config[command.key] = command.value;
				return CommandResult::Success;
		}
		
		command.state = CommandSendStateSend;
		return CommandResult::Failed;
	}
	
	void ConfigService::ParseConfig()
	{
		char *buffer = new char[_configBuffer.length() + 1];
		strcpy(buffer, _configBuffer.c_str());
		
		char *temp = buffer;
		while(*temp)
		{
			char *end = strchr(temp, '\n');
			if(!end)
				end = temp + strlen(temp);
			
			
			char *param = strchr(temp, '=');
			char *key   = param - 1;
			
			do
			{
				param ++;
			} while(*param == ' ');
			
			while(*(key - 1) == ' ')
				key --;
			
			*end = '\0';
			*key = '\0';
			
			_config.insert(std::make_pair(temp, param));
			
			temp = end + 1;
		}
		
		_configBuffer.clear();
	}
	
	ConfigService::CommandResult ConfigService::HandleRequestConfig(Command &command)
	{
		switch(command.state)
		{
			case 0:
			{
				if(_droneState & ARDRONE_COMMAND_MASK)
				{
					_atService->Send(ATCommand("CTRL") << ACK_CONTROL_MODE << 0);
					return CommandResult::Proceed;
				}
				
				_configBuffer.clear();
				_atService->Send(ATCommand("CTRL") << CFG_GET_CONTROL_MODE << 0);
				
				command.state = 1;
				
				return CommandResult::Proceed;
			}
				
			case 1:
			{
				char buffer[1025];
				size_t received;
				
				if(_socket->Receive(buffer, 1024, &received) == Socket::Result::Success)
				{
					buffer[1024] = '\0';
					_configBuffer.append(buffer);
					
					if(buffer[received - 1] == '\0')
					{
						ParseConfig();
						return CommandResult::Success;
					}
				}
				else
				{
					if(!(_droneState & ACK_CONTROL_MODE))
						command.state = 0;
					
					return CommandResult::Failed;
				}
				
				return CommandResult::Proceed;
			}
		}
		
		command.state = 0;
		return CommandResult::Failed;
	}
	
	void ConfigService::Tick(uint32_t reason)
	{
		std::unique_lock<std::mutex> lock(_mutex);
		
		if(_queue.empty())
			return;
		
		if(_navdataConsumed)
			return;
		_navdataConsumed = true;
		
		
		Command &command = _queue.front();
		CommandResult result;
		
		switch(command.type)
		{
			case CommandTypeSend:
				result = HandleSendConfig(command);
				break;
				
			case CommandTypeRequest:
				result = HandleRequestConfig(command);
				break;
				
			default:
				result = CommandResult::Failed;
				break;
		}
		
		switch(result)
		{
			case CommandResult::Failed:
				if((-- command.retries) == 0)
				{
					auto callback = std::move(command.callback);
					_queue.pop_front();
					
					lock.unlock();
					
					if(callback)
						callback(false);
					
					return;
				}
				
				break;
				
			case CommandResult::Success:
			{
				auto callback = std::move(command.callback);
				_queue.pop_front();
				
				lock.unlock();
				
				if(callback)
					callback(true);
				
				return;
			}
				
			default:
				break;
		}
	}
}
