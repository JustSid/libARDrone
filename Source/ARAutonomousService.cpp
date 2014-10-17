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
	
	
	
	
	void AutonomousService::AddWaypoint(const Location &location, float direction, uint32_t altitude)
	{
		std::lock_guard<std::mutex> lock(_mutex);
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
		
		switch(_state.load())
		{
			case AutonomyState::Stopped:
			{
				ControlService::FlyState flystate = _control->GetFlyState();
				
				_isFlying     = (flystate == ControlService::FlyState::Flying);
				_isTrimmed    = (flystate != ControlService::FlyState::Landed);
				_isCalibrated = magneto->magneto_calibration_ok;
				
				_state = AutonomyState::Bootstrapping;
				
				break;
			}
				
			case AutonomyState::Bootstrapping:
			{
				if(_isFlying && _isCalibrated && _isTrimmed)
				{
					std::cout << "Finished bootstrapping" << std::endl;
					
					SetCooldown(std::chrono::seconds(5));
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
				
				if(!_isCalibrated)
				{
					std::cout << "Calibrating" << std::endl;
					
					_control->Calibrate();
					
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
				
				_control->Land();
				break;
			}
				
			default:
				break;
		}
		
		_freshNavdata = false;
	}
}
