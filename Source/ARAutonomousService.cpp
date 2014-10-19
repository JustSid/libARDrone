//
//  ARAutonomousService.cpp
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

#include "ARAutonomousService.h"
#include "ARDrone.h"

namespace AR
{
	AutonomousService::AutonomousService(Drone *drone) :
		Service(drone, "Autonomous"),
		_wantsRunning(false),
		_state(AutonomyState::Stopped),
		_navdata(nullptr)
	{
		GetDrone()->AddNavdataSubscriber(std::bind(&AutonomousService::ConsumeNavdata, this, std::placeholders::_1), this);
	}
	
	AutonomousService::~AutonomousService()
	{
		GetDrone()->RemoveNavdataSubscriber(this);
		delete _navdata;
	}
	
	
	void AutonomousService::Start()
	{
		_wantsRunning = true;
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void AutonomousService::Stop()
	{
		_wantsRunning = false;
		Wakeup(WakeupReason::DataAvilable);
	}
	
	
	
	
	void AutonomousService::AddWaypointCommand(const Location &location, uint32_t altitude)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		Command command(Command::Type::MoveTo);
		command.InitializeData<Waypoint>(location, altitude);
		
		_commands.push_back(std::move(command));
	}
	
	
	void AutonomousService::ConsumeNavdata(Navdata *navdata)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		delete _navdata;
		_navdata = navdata->CopyWithTags(NavdataOptions(NavdataTag::Demo, NavdataTag::GPS, NavdataTag::Magneto));
		
		_freshNavdata = true;
	}
	
	
	Service::State AutonomousService::ConnectInternal()
	{
		UpdateNavdataOptions(NavdataOptions(NavdataTag::Demo, NavdataTag::GPS, NavdataTag::Magneto));
		
		_atService = GetDrone()->GetService<ATService>("AT");
		
		_control = GetDrone()->GetService<ControlService>("Control");
		if(!_control)
			_control = GetDrone()->AddService<ControlService>();
		
		_cooldown = std::chrono::steady_clock::now();
		
		return State::Connected;
	}
	
	void AutonomousService::SetCooldown(std::chrono::milliseconds duration)
	{
		_cooldown = std::chrono::steady_clock::now() + duration;
	}
	
	
	
	bool AutonomousService::ExecuteMoveToCommand(const Command &command, Waypoint *waypoint)
	{
		NavdataOptionDemo *demo = _navdata->GetOptionWithTag<NavdataOptionDemo>(NavdataTag::Demo);
		NavdataOptionMagneto *magneto = _navdata->GetOptionWithTag<NavdataOptionMagneto>(NavdataTag::Magneto);
		NavdataOptionGPS *gps = _navdata->GetOptionWithTag<NavdataOptionGPS>(NavdataTag::GPS);
		
		if(!demo)
			return false;
		
		
		Location location = gps->location;
		
		double heading = location.GetHeading(waypoint->target);
		double distance = location.GetDistance(waypoint->target);
		
		
		std::cout << "Target heading: " << heading << ", Current heading: " << magneto->heading_unwrapped << std::endl;
		std::cout << "Target: " << waypoint->target.latitude << ", " << waypoint->target.longitude << std::endl;
		std::cout << "Target height: " << waypoint->altitude << ", Current height: " << demo->altitude << std::endl;
		std::cout << "Distance: " << distance << std::endl;
		
		
		
		if(abs(waypoint->altitude - demo->altitude) > 250)
		{
			float direction = (waypoint->altitude > demo->altitude) ? 0.5 : -0.5;
			
			_control->Hover();
			_control->SetDirection(Vector3(0.0, direction, 0.0));
			_control->SetAngularSpeed(0.0f);
			
			return false;
		}
		
		
		double diff = fabs(heading - magneto->heading_unwrapped);
		
		float angularSpeed = 0.0f;
		float direction = 0.0f;
		
		if(diff >= 4.5)
		{
			double n = std::min(180.0, std::max(40.0, diff));
			float angular = n / 180;
			
			angularSpeed = heading > magneto->heading_unwrapped ? angular : -angular;
		}
		
		if(distance > 4.5)
		{
			direction = -0.1f;
		}
		
		_control->SetDirection(Vector3(0.0, 0.0, direction));
		_control->SetAngularSpeed(angularSpeed);
		
		if(fabsf(angularSpeed) > 0.01 || fabsf(direction) > 0.01)
		{
			return false;
		}
		
		_control->Hover();
		return true;
	}
	
	bool AutonomousService::ExecuteCommand(const Command &command)
	{
		switch(command.type)
		{
			case Command::Type::MoveTo:
			{
				Waypoint *waypoint = reinterpret_cast<Waypoint *>(command.data);
				return ExecuteMoveToCommand(command, waypoint);
				break;
			}
				
			case Command::Type::Land:
				_control->Land();
				
				return (_control->GetFlyState() == ControlService::FlyState::Landed);
				break;
				
			case Command::Type::Wait:
			{
			
				break;
			}
		}
		
		return true;
	}
	
	void AutonomousService::Tick(uint32_t reason)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		if(!_wantsRunning && _state != AutonomyState::Stopped)
		{
			_control->Hover(); // Hover and Out!
			_state = AutonomyState::Stopped;
		}
		
		if(!_wantsRunning || !_freshNavdata)
			return;
		
		if(_control->HasEmergency())
		{
			_state = AutonomyState::Stopped;
			_wantsRunning = false;
			_control->Land();
			
			return;
		}
		
		auto time = std::chrono::steady_clock::now();
		if(time <= _cooldown)
			return;
		
		NavdataOptionMagneto *magneto = _navdata->GetOptionWithTag<NavdataOptionMagneto>(NavdataTag::Magneto);
		NavdataOptionGPS *gps = _navdata->GetOptionWithTag<NavdataOptionGPS>(NavdataTag::GPS);
		
		if(!gps || !magneto)
			return;
		
		switch(_state.load())
		{
			case AutonomyState::Stopped:
			{
				std::cout << "Bootstrapping" << std::endl;
				
				ControlService::FlyState flystate = _control->GetFlyState();
				
				_isFlying     = (flystate == ControlService::FlyState::Flying);
				_isTrimmed    = (flystate != ControlService::FlyState::Landed);
				_isCalibrated = false;
				_needsCalibration = true;
				
				_state = AutonomyState::Bootstrapping;
				
				break;
			}
				
			case AutonomyState::Bootstrapping:
			{
				if(_isFlying && (_isCalibrated && !_needsCalibration) && _isTrimmed)
				{
					std::cout << "Finished bootstrapping" << std::endl;
					
					SetCooldown(std::chrono::seconds(5));
					
					_iterator = _commands.begin();
					_state = AutonomyState::WaitingForGPS;
					
					return;
				}
				
				ControlService::FlyState flystate = _control->GetFlyState();
				_isCalibrated = magneto->magneto_calibration_ok;
				
				if(!_isTrimmed)
				{
					_control->Ftrim();
					_isTrimmed = true;
					
					std::cout << "FTRIM" << std::endl;
					
					SetCooldown(std::chrono::milliseconds(500));
					return;
				}
				
				if(!_isFlying)
				{
					std::cout << "Take Off" << std::endl;
					
					_control->TakeOff();
					_isFlying = (flystate == ControlService::FlyState::Flying);
					
					SetCooldown(std::chrono::seconds(2));
					return;
				}
				
				if(!_isCalibrated || _needsCalibration)
				{
					std::cout << "Calibrating" << std::endl;
					
					_control->Calibrate();
					_needsCalibration = false;
					
					SetCooldown(std::chrono::seconds(5));
					return;
				}
				
				break;
			}
				
			case AutonomyState::WaitingForGPS:
			{
				if(gps->nbsat >= 4)
				{
					SetCooldown(std::chrono::seconds(2));
					_state = AutonomyState::Running;
					
					std::cout << "Acquired GPS" << std::endl;
				}
				
				break;
			}
				
			case AutonomyState::Running:
			{
				if(gps->nbsat < 4)
				{
					_state = AutonomyState::WaitingForGPS;
					return;
				}
				
				if(_iterator == _commands.end())
				{
					_state = AutonomyState::Finished;
					return;
				}
				
				if(ExecuteCommand(*_iterator))
					_iterator ++;
				
				break;
			}
				
			case AutonomyState::Finished:
			{
				_control->Hover();
				_wantsRunning = false;
				_state = AutonomyState::Stopped;
				
				break;
			}
				
			default:
				break;
		}
		
		_freshNavdata = false;
	}
}
